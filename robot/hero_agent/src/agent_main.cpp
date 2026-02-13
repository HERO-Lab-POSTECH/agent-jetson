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
const int BUFFER_SIZE = FRAME_RATE * 2;  // Sliding window = 2 seconds
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
// ==============================
int record_flag = 0;
int prev_record_flag = 0;
pid_t rosbag_pid = -1;
std::ofstream fout_csv;

// ==============================
// Constants
// ==============================
const float DEPTH_TOLERANCE = 0.01f;

// Utility: command sender
void send_command(char cmd);

// Utility: timestamp string generator
std::string get_timestamp_string();

// Utility: directory creation
void ensure_directory(const std::string& path);
int get_next_log_index(const std::string& base_path);

// Utility: adjust depth based on FSM reference
void adjust_depth_if_needed();

// Utility: rosbag recording
void start_rosbag_record();
void stop_rosbag_record();

// Utility: monitoring system output
void print_monitor_status();

// Utility: signal interrupt handler
void handle_signal(int sig);

// Callback: keyboard input from key_teleop node
void key_input_callback(const std_msgs::Int8::ConstPtr& msg);

/**
 * @brief Image topic callback for object detection.
 * 
 * This function receives raw RGB images, converts them to HSV,
 * applies filtering and contour analysis to detect an object.
 * It also determines whether the object is approximately centered
 * in the image and updates detection flags accordingly.
 *
 * @param msg ROS image message (sensor_msgs::Image)
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

        // Convert to HSV and apply color filtering
        cv::Mat hsv, sat_mask;
        cv::cvtColor(input_image, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(0, 50, 0), cv::Scalar(255, 200, 255), sat_mask);

        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(sat_mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        bool frame_detected = false;
        bool is_centered = false;
        std::vector<cv::Point> largest_contour;
        double max_area = 0.0;

        // Find the largest contour that exceeds the area threshold
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area > MIN_CONTOUR_AREA && area > max_area) {
                max_area = area;
                largest_contour = contour;
                frame_detected = true;
            }
        }

        // Check if the detected object is centered in the image
        if (frame_detected && !largest_contour.empty()) {
            cv::Rect bbox = cv::boundingRect(largest_contour);
            int center_x = bbox.x + bbox.width / 2;
            int center_y = bbox.y + bbox.height / 2;

            is_centered = (
                std::abs(center_x - image_center_x) < image_center_x * 0.2 &&
                std::abs(center_y - image_center_y) < image_center_y * 0.2
            );
        }

        // Update detection buffer
        detection_buffer.push_back(frame_detected);
        if (frame_detected) detection_count++;

        if (detection_buffer.size() > BUFFER_SIZE) {
            if (detection_buffer.front()) detection_count--;
            detection_buffer.pop_front();
        }

        // Final decision based on detection history
        if (detection_buffer.size() == BUFFER_SIZE &&
            static_cast<float>(detection_count) / BUFFER_SIZE >= 0.9f) {
            object_detected = 1;
        } else {
            object_detected = 0;
        }

        object_centered = (object_detected && is_centered) ? 1 : 0;

        // Update debug message
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
 * @brief Full agent state callback from ROS topic.
 *
 * Updates internal robot state variables and executes the finite state machine (FSM)
 * for controlling robot behavior such as yaw control, depth control, movement, and gripping.
 *
 * @param msg Incoming hero_agent_state message containing sensor and control data.
 */
void msg_callback_state(const hero_msgs::hero_agent_state::ConstPtr& msg)
{
    // Update basic state variables
    yaw           = msg->Yaw;
    target_yaw    = msg->Target_yaw;
    throttle      = msg->Throttle;
    valid_yaw     = msg->Valid_yaw;
    depth         = msg->Depth;
    target_depth  = msg->Target_depth;
    move_speed    = msg->Move_speed;
    control_state = msg->Cont_state;
    control_flags = msg->State_addit;

    // Decode control_flags into boolean flags
    control_yaw_enabled   = (control_flags & 1)  ? 1 : 0;
    control_depth_enabled = (control_flags & 2)  ? 1 : 0;
    relay_enabled         = (control_flags & 4)  ? 1 : 0;
    laser_enabled         = (control_flags & 8)  ? 1 : 0;
    recovery_enabled      = (control_flags & 16) ? 1 : 0;

    const float DEPTH_TOLERANCE = 0.01;

    // FSM control logic
    switch (control_process)
    {
        case 0:
            // Initialization
            start_target_depth  = 0.1;
            process_count       = 0;
            gripper_state       = 0;

            send_command('h');  // Disable yaw controller
            send_command(';');  // Disable depth controller
            send_command('g');  // Stop motors

            cont_ver_msg.data = 3;  // ALBC to position 0.01
            pub_cont_ver.publish(cont_ver_msg);
            break;

        case 1:
            // Controller enable phase
            cont_ver_msg.data = cont_ver;  // TD or PID
            pub_cont_ver.publish(cont_ver_msg);

            send_command('c');  // Open gripper

            process_count++;

            // Enable yaw and depth controllers
            if (!control_yaw_enabled) send_command('y');
            if (!control_depth_enabled) send_command('p');             

            // Adjust depth if necessary
            if (process_count % 100 == 0) {
                adjust_depth_if_needed();
            }
            break;

        case 2:
            // Object search phase (forward movement)
            process_count++;

            if (move_speed < 10) {
                send_command('z');  // Increase speed
            } else if (move_speed > 10) {
                send_command('x');  // Decrease speed
            }

            if (process_count % 100 == 0) {
                adjust_depth_if_needed();
            }

            if (control_state != 2) {
                send_command('w');  // Move forward
            }

            if (object_centered == 1) {
                process_count = 0;
                control_process = 3;
            }
            break;

        case 3:
            // Object detected - hold position
            if (control_state != 5) {
                send_command('1');  // Hold
            }

            process_count++;
            if (process_count > 200) {
                process_count = 0;
                control_process = 4;
            }
            break;

        case 4:
            // Descend toward object
            process_count++;

            if (process_count > 20) {
                process_count = 0;
                send_command('o');  // Descend

                if (gripping_depth < depth) {
                    control_process = 5;
                    gripper_state = 0;
                    process_count = 0;
                }
            }
            break;

        case 5:
            // Gripping phase
            if (gripper_state == 0) {
                send_command('b');  // Close gripper
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
            // Lift object
            process_count++;

            if (process_count % 100 == 0) {
                adjust_depth_if_needed();
            }

            if (target_depth <= start_target_depth + DEPTH_TOLERANCE) {
                gripper_state = 0;
                process_count = 0;
                send_command('1');  // Hold position
            }
            break;
    }
}

/**
 * @brief Callback function to log ALBC status to CSV file.
 * 
 * This function subscribes to the /albc_status topic and writes the received
 * Float64MultiArray data to a CSV file when record_flag is set to 1.
 * 
 * Data layout in CSV:
 *   [0] current_tile (sec)
 *   [1] target_roll (deg)
 *   [2] current_roll (deg)
 *   [3] target_pitch (deg)
 *   [4] current_pitch (deg)
 *   [5] target_x (m)
 *   [6] target_y (m)
 *   [7] current_x (m)
 *   [8] current_y (m)
 */
void albcStatusCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (record_flag == 1 && msg->data.size() >= 8)
    {
        // Get current ROS time as float (in seconds)
        double current_time = ros::Time::now().toSec();

        fout_csv 
            << current_time << ","  // ROS timestamp
            << msg->data[0] << "," << msg->data[1] << ","   // roll
            << msg->data[2] << "," << msg->data[3] << ","   // pitch
            << msg->data[4] << "," << msg->data[5] << ","   // target x, y
            << msg->data[6] << "," << msg->data[7] << ","   // current x, y
            << target_depth << "," << depth << "\n";         // depth
    }
}

/**
 * @brief Callback for desired gripping depth value.
 *
 * Updates the target depth value used for gripping alignment.
 *
 * @param depth_msg ROS Float64 message with gripping depth
 */
void gripping_depth_callback(const std_msgs::Float64& depth_msg)
{
    gripping_depth = depth_msg.data;
}

/**
 * @brief Callback for keyboard input from key_teleop node.
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
    else {
        send_command(ch);
    }
}

/**
 * @brief Main entry point of the ROS agent node.
 *
 * Handles initialization, ROS communication setup, rosbag recording,
 * keyboard input, and FSM execution loop.
 */
int main(int argc, char** argv)
{
    // Setup result file paths
    std::string base_traj_dir = "/home/nvidia/catkin_ws/agent_results/trajectory";
    std::string base_rosbag_dir = "/home/nvidia/catkin_ws/agent_results/rosbags";
    ensure_directory(base_traj_dir);
    ensure_directory(base_rosbag_dir);

    // Initialize ROS (disable default SIGINT handler so we can install our own)
    ros::init(argc, argv, "agent_main", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Install signal handlers AFTER ros::init to prevent override
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

    // Init loop
    ros::Rate loop_rate(100);

    // Startup banner (logged to rosout for records)
    ROS_INFO("Agent Main initialized (FSM 0-6, ctrl_ver=%d, grip=%.2fm)", cont_ver, gripping_depth);

    while (ros::ok())
    {
        // Note: record_flag is written by key_input_callback (via AsyncSpinner) and
        // read here in the main loop. With AsyncSpinner(2), this is a potential race.
        // Using int (not bool) ensures atomic reads/writes on most architectures.
        // For strict correctness, consider std::atomic<int> if issues arise.
        if (record_flag != prev_record_flag) {
            if (record_flag == 1) {
                // Close previous file if open
                if (fout_csv.is_open()) fout_csv.close();
        
                int log_index = get_next_log_index(base_traj_dir);

                albc_csv_path = base_traj_dir + "/albc_status_" + std::to_string(log_index) + ".csv";
                rosbag_file_path = base_rosbag_dir + "/record_" + std::to_string(log_index) + ".bag";                
        
                // Open new CSV file and write header
                fout_csv.open(albc_csv_path);
                if (!fout_csv.is_open()) {
                    csv_status_msg = "[ERROR] Failed to open new CSV file.";
                } else {
                    fout_csv << "ros_time,target_roll,current_roll,target_pitch,current_pitch,target_x,target_y,current_x,current_y,target_depth,depth\n";
                    csv_status_msg = "[INFO] Logging started.";
                }
        
                // Start rosbag recording after CSV is ready
                start_rosbag_record();
                rosbag_status_msg = "[INFO] Rosbag recording started.";
            } else {
                // Stop logging: close CSV file first
                if (fout_csv.is_open()) fout_csv.close();
                csv_status_msg = "[INFO] Logging stopped.";
        
                // Then stop rosbag
                stop_rosbag_record();
                rosbag_status_msg = "[INFO] Rosbag recording stopped.";
            }
        
            // Update previous flag state
            prev_record_flag = record_flag;
        }          

        print_monitor_status();
        loop_rate.sleep();
    }

    spinner.stop();
    stop_rosbag_record();
    if (fout_csv.is_open()) fout_csv.close();
    return 0;
}

/**
 * @brief Start rosbag recording using fork and exec.
 *
 * Spawns a child process to execute `rosbag record -a`. Verifies early failure and updates rosbag_pid.
 */
void start_rosbag_record()
{
    rosbag_pid = fork();

    if (rosbag_pid < 0) {
        rosbag_status_msg = "[ERROR] Failed to fork rosbag process.";
        return;
    }

    if (rosbag_pid == 0) {
        // Child process: replace with rosbag
        execlp("rosbag", "rosbag", "record", "-O", rosbag_file_path.c_str(), "-a", NULL);
        perror("rosbag exec failed");
        _exit(1);  // Use _exit to avoid flushing shared stdio buffers
    }

    // Parent process: check if child exits immediately
    usleep(500000);  // 0.5s
    int status = 0;
    pid_t result = waitpid(rosbag_pid, &status, WNOHANG);

    if (result == rosbag_pid && WIFEXITED(status) && WEXITSTATUS(status) != 0) {
        rosbag_status_msg = "[ERROR] Rosbag exec failed (exited immediately).";
        rosbag_pid = -1;
    } else {
        rosbag_status_msg = "[INFO] Rosbag recording process started.";
    }
} 
 
/**
 * @brief Stop the rosbag recording process and finalize the .bag file.
 *
 * Terminates the rosbag child process, waits for exit, renames .bag.active,
 * and verifies .bag file exists and has content.
 */
void stop_rosbag_record()
{
    if (rosbag_pid <= 0) return;

    int status = 0;
    pid_t result;

    // Attempt graceful termination
    kill(rosbag_pid, SIGTERM);

    for (int i = 0; i < 30; ++i) {
        result = waitpid(rosbag_pid, &status, WNOHANG);
        if (result == rosbag_pid) break;
        if (result == -1) {
            rosbag_status_msg = "[ERROR] waitpid failed while stopping rosbag.";
            break;
        }
        usleep(100000);  // 100ms × 30 = 3s
    }

    // If still running, force kill
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

    // Finalize .bag file
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

        // .active doesn't exist, check if .bag exists and has non-zero size
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
 
/**
 * @brief Ensure a directory exists, and create it if not.
 * 
 * If the path exists and is not a directory, an error is printed.
 *
 * @param path Filesystem path to check or create
 */
void ensure_directory(const std::string& path)
{
    struct stat st;
    if (stat(path.c_str(), &st) != 0) {
        // Directory does not exist
        mkdir(path.c_str(), 0775);
    } else if (!S_ISDIR(st.st_mode)) {
        std::cerr << "[ERROR] Path exists but is not a directory: " << path << std::endl;
    }
}

/**
 * @brief Get the next available log file index by scanning existing files.
 *
 * Scans the base path for files matching "albc_status_<number>.csv"
 * and returns the next available integer index.
 */
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

    // Find the smallest unused index
    while (used_indices.count(index)) ++index;
    return index;
}

/**
 * @brief Adjust vertical depth based on FSM reference (`start_target_depth`).
 * 
 * Sends motor control commands 'l' or 'o' if current target depth deviates.
 */
void adjust_depth_if_needed()
{
    float delta = target_depth - start_target_depth;

    if (delta > DEPTH_TOLERANCE) {
        send_command('l');  // rise
    } else if (delta < -DEPTH_TOLERANCE) {
        send_command('o');  // descend
    }
} 

/**
 * @brief Publish a single character command to the robot.
 * 
 * Wrapper function to simplify command publishing through `/hero_agent/command`.
 * 
 * @param cmd Character command to send
 */
void send_command(char cmd)
{
    command_msg.data = cmd;
    pub_command.publish(command_msg);
}

/**
 * @brief Signal handler for clean shutdown
 */
void handle_signal(int sig)
{
    if (rosbag_pid > 0) {
        kill(rosbag_pid, SIGTERM);
        rosbag_pid = -1;
    }
    ros::shutdown();
}
 
/**
 * @brief Print real-time robot and system status to the terminal.
 * 
 * This function clears the terminal and prints robot control state, FSM status, 
 * sensor flags, and any active warnings or system messages.
 */
void print_monitor_status()
{
    static const char* fsm_names[] = {
        "Init", "CtrlEnable", "Search", "Detected", "Descend", "Grip", "Lift"
    };
    const char* fsm_desc = (control_process >= 0 && control_process <= 6)
        ? fsm_names[control_process] : "Unknown";

    ROS_INFO_THROTTLE(0.5,
        "FSM:%d(%s) cnt=%d | Yaw:%.1f/%.1f[%s] Dep:%.3f/%.3f[%s] | Spd:%d Grip:%d | Obj:%s | Rec:%s",
        control_process, fsm_desc, process_count,
        yaw, target_yaw, control_yaw_enabled ? "ON" : "OFF",
        depth, target_depth, control_depth_enabled ? "ON" : "OFF",
        move_speed, gripper_state,
        object_detected ? (object_centered ? "CTR" : "DET") : "---",
        record_flag ? "REC" : "---");

    if (!rosbag_status_msg.empty())
        ROS_INFO_THROTTLE(1.0, "Rosbag: %s", rosbag_status_msg.c_str());
    if (!csv_status_msg.empty())
        ROS_INFO_THROTTLE(1.0, "CSV: %s", csv_status_msg.c_str());
    if (!image_error_msg.empty())
        ROS_WARN_THROTTLE(1.0, "%s", image_error_msg.c_str());
    if (!vision_info_msg.empty())
        ROS_INFO_THROTTLE(1.0, "Vision: %s", vision_info_msg.c_str());
}
