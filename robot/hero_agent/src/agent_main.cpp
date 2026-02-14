// ==============================
// ROS core headers and messages
// ==============================
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

// ==============================
// Custom message definitions
// ==============================
#include "hero_msgs/hero_agent_state.h"
#include "hero_msgs/hero_agent_vision.h"
#include "hero_msgs/hero_agent_position_result.h"

// ==============================
// Image transport and OpenCV
// ==============================
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>

// ==============================
// C++ standard libraries
// ==============================
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <regex>
#include <atomic>
#include <set>

// ==============================
// System and POSIX headers
// ==============================
#include <unistd.h>
#include <signal.h>
#include <csignal>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <dirent.h>

using namespace std;

// ==============================
// Robot control state variables
// ==============================
float yaw = 0.0f;
float target_yaw = 0.0f;
float depth = 0.0f;
float target_depth = 0.0f;
float gripping_depth = 0.5f;
int throttle = 0;
int valid_yaw = 0;
int move_speed = 0;
int control_state = 0;
int control_flags = 0;
int cont_ver = 1;

// ==============================
// Decoded control flags
// ==============================
int control_yaw_enabled = 0;
int control_depth_enabled = 0;
int relay_enabled = 0;
int laser_enabled = 0;
int recovery_enabled = 0;

// ==============================
// FSM control process variables
// ==============================
int control_process = 0;
int process_count = 0;
int gripper_state = 0;
float start_target_depth = 0.2f;

// ==============================
// Object detection (vision)
// ==============================
const int FRAME_RATE = 10;
const int BUFFER_SIZE = FRAME_RATE * 2;
std::deque<bool> detection_buffer;
int detection_count = 0;
int object_detected = 0;
int object_centered = 0;

// ==============================
// ROS publishers and message objects
// ==============================
ros::Publisher pub_command;
ros::Publisher pub_vision_command;
ros::Publisher pub_cont_ver;

std_msgs::Int8 command_msg;
std_msgs::Int8 vision_command_msg;
std_msgs::Int32 cont_ver_msg;

// ==============================
// Logging and status messages
// ==============================
std::string image_error_msg = "";
std::string image_empty_warn_msg = "";
std::string vision_info_msg = "";
std::string rosbag_file_path = "";
std::string albc_csv_path = "";
std::string rosbag_status_msg = "";
std::string trajectory_file_error_msg = "";
std::string ros_monitor_log = "";
std::string csv_status_msg = "";

// ==============================
// rosbag recording control
// [BUG FIX C4] std::atomic for thread-safe access with AsyncSpinner
// ==============================
std::atomic<int> record_flag(0);
int prev_record_flag = 0;
pid_t rosbag_pid = -1;
std::ofstream fout_csv;

// ==============================
// Constants
// ==============================
const float DEPTH_TOLERANCE = 0.01f;

// Forward declarations
void send_command(char cmd);
std::string get_timestamp_string();
void ensure_directory(const std::string& path);
int get_next_log_index(const std::string& base_path);
void adjust_depth_if_needed();
void start_rosbag_record();
void stop_rosbag_record();
void print_monitor_status();
void handle_signal(int sig);

// [BUG FIX H4] key_input_callback: only handles FSM control for agent_main
// Does NOT forward generic commands - agent_command handles those via its own subscriber
void key_input_callback(const std_msgs::Int8::ConstPtr& msg);

/**
 * Image topic callback for object detection.
 */
void image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    try {
        cv::Mat input_image = cv_bridge::toCvShare(msg, "bgr8")->image;

        if (input_image.empty()) {
            image_empty_warn_msg = "Received image is empty.";
            return;
        } else {
            image_empty_warn_msg.clear();
        }

        const double MIN_CONTOUR_AREA = 500.0;

        int image_center_x = input_image.cols / 2;
        int image_center_y = input_image.rows / 2;

        cv::Mat hsv, sat_mask;
        cv::cvtColor(input_image, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(0, 50, 0), cv::Scalar(255, 200, 255), sat_mask);

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(sat_mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        bool frame_detected = false;
        bool is_centered = false;
        std::vector<cv::Point> largest_contour;
        double max_area = 0.0;

        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area > MIN_CONTOUR_AREA && area > max_area) {
                max_area = area;
                largest_contour = contour;
                frame_detected = true;
            }
        }

        if (frame_detected && !largest_contour.empty()) {
            cv::Rect bbox = cv::boundingRect(largest_contour);
            int center_x = bbox.x + bbox.width / 2;
            int center_y = bbox.y + bbox.height / 2;

            is_centered = (
                std::abs(center_x - image_center_x) < image_center_x * 0.2 &&
                std::abs(center_y - image_center_y) < image_center_y * 0.2
            );
        }

        detection_buffer.push_back(frame_detected);
        if (frame_detected) detection_count++;

        if (detection_buffer.size() > BUFFER_SIZE) {
            if (detection_buffer.front()) detection_count--;
            detection_buffer.pop_front();
        }

        if (detection_buffer.size() == BUFFER_SIZE &&
            static_cast<float>(detection_count) / BUFFER_SIZE >= 0.9f) {
            object_detected = 1;
        } else {
            object_detected = 0;
        }

        object_centered = (object_detected && is_centered) ? 1 : 0;

        if (object_detected) {
            vision_info_msg = object_centered
                ? "Object detected and centered."
                : "Object detected.";
        } else {
            vision_info_msg.clear();
        }

    } catch (cv_bridge::Exception& e) {
        image_error_msg = std::string("cv_bridge exception: ") + e.what();
    }
}

/**
 * Full agent state callback - updates state and runs FSM.
 */
void msg_callback_state(const hero_msgs::hero_agent_state::ConstPtr& msg)
{
    yaw           = msg->Yaw;
    target_yaw    = msg->Target_yaw;
    throttle      = msg->Throttle;
    valid_yaw     = msg->Valid_yaw;
    depth         = msg->Depth;
    target_depth  = msg->Target_depth;
    move_speed    = msg->Move_speed;
    control_state = msg->Cont_state;
    control_flags = msg->State_addit;

    control_yaw_enabled   = (control_flags & 1)  ? 1 : 0;
    control_depth_enabled = (control_flags & 2)  ? 1 : 0;
    relay_enabled         = (control_flags & 4)  ? 1 : 0;
    laser_enabled         = (control_flags & 8)  ? 1 : 0;
    recovery_enabled      = (control_flags & 16) ? 1 : 0;

    const float DEPTH_TOL = 0.01;

    switch (control_process)
    {
        case 0:
            start_target_depth  = 0.1;
            process_count       = 0;
            gripper_state       = 0;

            send_command('h');
            send_command(';');
            send_command('g');

            cont_ver_msg.data = 3;
            pub_cont_ver.publish(cont_ver_msg);
            break;

        case 1:
            cont_ver_msg.data = cont_ver;
            pub_cont_ver.publish(cont_ver_msg);

            send_command('c');

            process_count++;

            if (!control_yaw_enabled) send_command('y');
            if (!control_depth_enabled) send_command('p');

            if (process_count % 100 == 0) {
                adjust_depth_if_needed();
            }
            break;

        case 2:
            process_count++;

            if (move_speed < 10) {
                send_command('z');
            } else if (move_speed > 10) {
                send_command('x');
            }

            if (process_count % 100 == 0) {
                adjust_depth_if_needed();
            }

            if (control_state != 2) {
                send_command('w');
            }

            if (object_centered == 1) {
                process_count = 0;
                control_process = 3;
            }
            break;

        case 3:
            if (control_state != 5) {
                send_command('1');
            }

            process_count++;
            if (process_count > 200) {
                process_count = 0;
                control_process = 4;
            }
            break;

        case 4:
            process_count++;

            if (process_count > 20) {
                process_count = 0;
                send_command('o');

                if (gripping_depth < depth) {
                    control_process = 5;
                    gripper_state = 0;
                    process_count = 0;
                }
            }
            break;

        case 5:
            if (gripper_state == 0) {
                send_command('b');
            }

            process_count++;
            if (process_count > 100 && gripper_state < 2) {
                gripper_state++;
                process_count = 0;
            }

            if (gripper_state > 1) {
                process_count = 0;
                control_process = 6;
            }
            break;

        case 6:
            process_count++;

            if (process_count % 100 == 0) {
                adjust_depth_if_needed();
            }

            if (target_depth <= start_target_depth + DEPTH_TOL) {
                gripper_state = 0;
                process_count = 0;
                send_command('1');
            }
            break;
    }
}

/**
 * ALBC status CSV logging callback.
 */
void albcStatusCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    // [BUG FIX C4] record_flag is std::atomic<int>, safe to read from AsyncSpinner
    if (record_flag.load() == 1 && msg->data.size() >= 8)
    {
        double current_time = ros::Time::now().toSec();

        fout_csv
            << current_time << ","
            << msg->data[0] << "," << msg->data[1] << ","
            << msg->data[2] << "," << msg->data[3] << ","
            << msg->data[4] << "," << msg->data[5] << ","
            << msg->data[6] << "," << msg->data[7] << ","
            << target_depth << "," << depth << "\n";
    }
}

void gripping_depth_callback(const std_msgs::Float64& depth_msg)
{
    gripping_depth = depth_msg.data;
}

/**
 * [BUG FIX H4] key_input_callback: only handles agent_main FSM commands.
 * Does not forward to /hero_agent/command - agent_command has its own subscriber.
 */
void key_input_callback(const std_msgs::Int8::ConstPtr& msg)
{
    int ch = msg->data;

    if (ch == '0') {
        control_process = 0;
    }
    else if (ch == '-' && control_process > 0) {
        control_process--;
    }
    else if (ch == '=' && control_process < 6) {
        control_process++;
    }
    else if (ch == 'R') {
        // Toggle recording
        record_flag.store(record_flag.load() == 0 ? 1 : 0);
    }
    // [BUG FIX H4] Removed generic send_command(ch) forwarding
    // agent_command handles all other keys via its own /hero_agent/key_input subscriber
}

/**
 * Main entry point.
 */
int main(int argc, char** argv)
{
    std::string base_traj_dir = "/home/nvidia/catkin_ws/agent_results/trajectory";
    std::string base_rosbag_dir = "/home/nvidia/catkin_ws/agent_results/rosbags";
    ensure_directory(base_traj_dir);
    ensure_directory(base_rosbag_dir);

    ros::init(argc, argv, "agent_main", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    // ROS Communication
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub = it.subscribe("/image_raw", 1, image_callback);

    ros::Subscriber sub_state  = nh.subscribe("/hero_agent/state", 100, msg_callback_state);
    ros::Subscriber sub_gripping_depth = nh.subscribe("/hero_agent/gripping_target_depth", 100, gripping_depth_callback);
    ros::Subscriber sub_albc_status = nh.subscribe("/albc_status", 10, albcStatusCallback);
    ros::Subscriber sub_key_input = nh.subscribe("/hero_agent/key_input", 10, key_input_callback);

    pub_command        = nh.advertise<std_msgs::Int8>("/hero_agent/command", 100);
    pub_vision_command = nh.advertise<std_msgs::Int8>("/hero_agent/vision_command", 100);
    pub_cont_ver       = nh.advertise<std_msgs::Int32>("/cont_ver", 10);

    ros::Rate loop_rate(100);

    // Startup banner (printed once before dashboard takes over)
    printf("\n  Agent Main Initialized (FSM 0-6, ctrl_ver=%d, grip=%.2fm)\n\n", cont_ver, gripping_depth);

    while (ros::ok())
    {
        // [BUG FIX C4] Atomic load for thread-safe read
        int current_record = record_flag.load();
        if (current_record != prev_record_flag) {
            if (current_record == 1) {
                if (fout_csv.is_open()) fout_csv.close();

                int log_index = get_next_log_index(base_traj_dir);

                albc_csv_path = base_traj_dir + "/albc_status_" + std::to_string(log_index) + ".csv";
                rosbag_file_path = base_rosbag_dir + "/record_" + std::to_string(log_index) + ".bag";

                fout_csv.open(albc_csv_path);
                if (!fout_csv.is_open()) {
                    csv_status_msg = "[ERROR] Failed to open new CSV file.";
                } else {
                    fout_csv << "ros_time,target_roll,current_roll,target_pitch,current_pitch,target_x,target_y,current_x,current_y,target_depth,depth\n";
                    csv_status_msg = "[INFO] Logging started.";
                }

                start_rosbag_record();
                rosbag_status_msg = "[INFO] Rosbag recording started.";
            } else {
                if (fout_csv.is_open()) fout_csv.close();
                csv_status_msg = "[INFO] Logging stopped.";

                stop_rosbag_record();
                rosbag_status_msg = "[INFO] Rosbag recording stopped.";
            }

            prev_record_flag = current_record;
        }

        print_monitor_status();
        loop_rate.sleep();
    }

    spinner.stop();
    stop_rosbag_record();
    if (fout_csv.is_open()) fout_csv.close();
    return 0;
}

void start_rosbag_record()
{
    rosbag_pid = fork();

    if (rosbag_pid < 0) {
        rosbag_status_msg = "[ERROR] Failed to fork rosbag process.";
        return;
    }

    if (rosbag_pid == 0) {
        execlp("rosbag", "rosbag", "record", "-O", rosbag_file_path.c_str(), "-a", NULL);
        perror("rosbag exec failed");
        _exit(1);
    }

    usleep(500000);
    int status = 0;
    pid_t result = waitpid(rosbag_pid, &status, WNOHANG);

    if (result == rosbag_pid && WIFEXITED(status) && WEXITSTATUS(status) != 0) {
        rosbag_status_msg = "[ERROR] Rosbag exec failed (exited immediately).";
        rosbag_pid = -1;
    } else {
        rosbag_status_msg = "[INFO] Rosbag recording process started.";
    }
}

void stop_rosbag_record()
{
    if (rosbag_pid <= 0) return;

    int status = 0;
    pid_t result;

    kill(rosbag_pid, SIGTERM);

    for (int i = 0; i < 30; ++i) {
        result = waitpid(rosbag_pid, &status, WNOHANG);
        if (result == rosbag_pid) break;
        if (result == -1) {
            rosbag_status_msg = "[ERROR] waitpid failed while stopping rosbag.";
            break;
        }
        usleep(100000);
    }

    if (result == 0) {
        kill(rosbag_pid, SIGKILL);
        waitpid(rosbag_pid, &status, 0);
        rosbag_status_msg = "[WARN] Rosbag forcibly killed.";
    } else if (WIFEXITED(status)) {
        rosbag_status_msg = "[INFO] Rosbag exited cleanly.";
    } else {
        rosbag_status_msg = "[WARN] Rosbag exited abnormally.";
    }

    rosbag_pid = -1;

    std::string active_file = rosbag_file_path + ".active";
    std::ifstream infile(active_file.c_str());

    if (infile.good()) {
        infile.close();
        if (std::rename(active_file.c_str(), rosbag_file_path.c_str()) == 0) {
            rosbag_status_msg += " (.bag finalized)";
        } else {
            rosbag_status_msg += " [ERROR] Failed to rename .bag.active: ";
            rosbag_status_msg += strerror(errno);
        }
    } else {
        infile.close();

        std::ifstream check_bag(rosbag_file_path.c_str(), std::ios::binary | std::ios::ate);
        if (check_bag.good()) {
            std::streamsize size = check_bag.tellg();
            check_bag.close();
            if (size > 0) {
                rosbag_status_msg += " [INFO] .bag file already finalized.";
            } else {
                rosbag_status_msg += " [WARN] .bag file is empty.";
            }
        } else {
            rosbag_status_msg += " [ERROR] No .bag.active or .bag file found.";
        }
    }
}

void ensure_directory(const std::string& path)
{
    struct stat st;
    if (stat(path.c_str(), &st) != 0) {
        mkdir(path.c_str(), 0775);
    } else if (!S_ISDIR(st.st_mode)) {
        std::cerr << "[ERROR] Path exists but is not a directory: " << path << std::endl;
    }
}

int get_next_log_index(const std::string& base_path)
{
    int index = 0;
    std::set<int> used_indices;

    DIR* dir = opendir(base_path.c_str());
    if (dir == nullptr) return 0;

    struct dirent* entry;
    std::regex pattern(R"(albc_status_(\d+)\.csv)");

    while ((entry = readdir(dir)) != nullptr) {
        std::cmatch match;
        if (std::regex_match(entry->d_name, match, pattern)) {
            int n = std::stoi(match[1]);
            used_indices.insert(n);
        }
    }

    closedir(dir);

    while (used_indices.count(index)) ++index;
    return index;
}

void adjust_depth_if_needed()
{
    float delta = target_depth - start_target_depth;

    if (delta > DEPTH_TOLERANCE) {
        send_command('l');
    } else if (delta < -DEPTH_TOLERANCE) {
        send_command('o');
    }
}

void send_command(char cmd)
{
    command_msg.data = cmd;
    pub_command.publish(command_msg);
}

void handle_signal(int sig)
{
    if (rosbag_pid > 0) {
        kill(rosbag_pid, SIGTERM);
        rosbag_pid = -1;
    }
    ros::shutdown();
}

void print_monitor_status()
{
    static const char* fsm_names[] = {
        "Init", "CtrlEnable", "Search", "Detected", "Descend", "Grip", "Lift"
    };
    const char* fsm_desc = (control_process >= 0 && control_process <= 6)
        ? fsm_names[control_process] : "Unknown";

    // ANSI clear screen + cursor home for dashboard effect
    printf("\033[2J\033[H");
    printf("╔══════════════════════════════════════════════════════╗\n");
    printf("║            HERO Agent Monitor Dashboard             ║\n");
    printf("╠══════════════════════════════════════════════════════╣\n");
    printf("║ FSM: %d (%s)  cnt=%d                       \n", control_process, fsm_desc, process_count);
    printf("║ Yaw:   %7.1f / %7.1f  [%s]                \n", yaw, target_yaw, control_yaw_enabled ? " ON" : "OFF");
    printf("║ Depth: %7.3f / %7.3f  [%s]                \n", depth, target_depth, control_depth_enabled ? " ON" : "OFF");
    printf("║ Speed: %d   Gripper: %d                    \n", move_speed, gripper_state);
    printf("╠══════════════════════════════════════════════════════╣\n");
    printf("║ Object: %-5s  Record: %-3s                 \n",
        object_detected ? (object_centered ? "CTR" : "DET") : "---",
        record_flag.load() ? "REC" : "---");

    if (!rosbag_status_msg.empty())
        printf("║ Rosbag: %s\n", rosbag_status_msg.c_str());
    if (!csv_status_msg.empty())
        printf("║ CSV:    %s\n", csv_status_msg.c_str());
    if (!image_error_msg.empty())
        printf("║ [WARN]  %s\n", image_error_msg.c_str());
    if (!vision_info_msg.empty())
        printf("║ Vision: %s\n", vision_info_msg.c_str());

    printf("╚══════════════════════════════════════════════════════╝\n");
    fflush(stdout);
}
