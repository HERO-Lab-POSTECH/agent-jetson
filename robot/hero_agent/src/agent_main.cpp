// ==============================
// agent_main: Key command forwarding, state monitoring, rosbag recording
// FSM code backed up in agent_main_fsm_backup.cpp
// ==============================

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64MultiArray.h>
#include "hero_msgs/hero_agent_state.h"

#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <regex>
#include <atomic>
#include <set>

#include <unistd.h>
#include <signal.h>
#include <csignal>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <dirent.h>

using namespace std;

// ==============================
// State variables (from Arduino state topic)
// ==============================
float yaw = 0.0f, target_yaw = 0.0f;
float depth = 0.0f, target_depth = 0.0f;
int move_speed = 0;
int control_flags = 0;
int control_yaw_enabled = 0, control_depth_enabled = 0;
int relay_enabled = 0, laser_enabled = 0, recovery_enabled = 0;

// ==============================
// ROS publishers and messages
// ==============================
ros::Publisher pub_command;
std_msgs::Int8 command_msg;

// ==============================
// Rosbag recording
// ==============================
std::atomic<int> record_flag(0);
int prev_record_flag = 0;
pid_t rosbag_pid = -1;
std::ofstream fout_csv;
std::string rosbag_file_path = "";
std::string albc_csv_path = "";
std::string rosbag_status_msg = "";
std::string csv_status_msg = "";

// Forward declarations
void send_command(char cmd);
void ensure_directory(const std::string& path);
int get_next_log_index(const std::string& base_path);
void start_rosbag_record();
void stop_rosbag_record();
void print_monitor_status();
void handle_signal(int sig);

// ==============================
// State callback - just update monitoring variables
// ==============================
void msg_callback_state(const hero_msgs::hero_agent_state::ConstPtr& msg)
{
    yaw           = msg->Yaw;
    target_yaw    = msg->Target_yaw;
    depth         = msg->Depth;
    target_depth  = msg->Target_depth;
    move_speed    = msg->Move_speed;
    control_flags = msg->State_addit;

    control_yaw_enabled   = (control_flags & 1)  ? 1 : 0;
    control_depth_enabled = (control_flags & 2)  ? 1 : 0;
    relay_enabled         = (control_flags & 4)  ? 1 : 0;
    laser_enabled         = (control_flags & 8)  ? 1 : 0;
    recovery_enabled      = (control_flags & 16) ? 1 : 0;
}

// ==============================
// ALBC status CSV logging
// ==============================
void albcStatusCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
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

// ==============================
// Key input: forward ALL keys to Arduino + handle recording toggle
// ==============================
void key_input_callback(const std_msgs::Int8::ConstPtr& msg)
{
    int ch = msg->data;

    if (ch == 'R') {
        record_flag.store(record_flag.load() == 0 ? 1 : 0);
    }

    // Forward all keys to Arduino
    send_command(ch);
}

// ==============================
// Main
// ==============================
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

    // Subscribers
    ros::Subscriber sub_state = nh.subscribe("/hero_agent/state", 100, msg_callback_state);
    ros::Subscriber sub_albc_status = nh.subscribe("/albc_status", 10, albcStatusCallback);
    ros::Subscriber sub_key_input = nh.subscribe("/hero_agent/key_input", 10, key_input_callback);

    // Publishers
    pub_command = nh.advertise<std_msgs::Int8>("/hero_agent/command", 100);

    ros::Rate loop_rate(100);

    printf("\n  Agent Main Initialized (monitor + key forwarding)\n\n");

    while (ros::ok())
    {
        // Recording state change
        int current_record = record_flag.load();
        if (current_record != prev_record_flag) {
            if (current_record == 1) {
                if (fout_csv.is_open()) fout_csv.close();
                int log_index = get_next_log_index(base_traj_dir);
                albc_csv_path = base_traj_dir + "/albc_status_" + std::to_string(log_index) + ".csv";
                rosbag_file_path = base_rosbag_dir + "/record_" + std::to_string(log_index) + ".bag";
                fout_csv.open(albc_csv_path);
                if (fout_csv.is_open()) {
                    fout_csv << "ros_time,target_roll,current_roll,target_pitch,current_pitch,target_x,target_y,current_x,current_y,target_depth,depth\n";
                    csv_status_msg = "Logging started";
                }
                start_rosbag_record();
                rosbag_status_msg = "Recording started";
            } else {
                if (fout_csv.is_open()) fout_csv.close();
                csv_status_msg = "Logging stopped";
                stop_rosbag_record();
                rosbag_status_msg = "Recording stopped";
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

// ==============================
// Utility functions
// ==============================

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

void ensure_directory(const std::string& path)
{
    struct stat st;
    if (stat(path.c_str(), &st) != 0) mkdir(path.c_str(), 0775);
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
        if (std::regex_match(entry->d_name, match, pattern))
            used_indices.insert(std::stoi(match[1]));
    }
    closedir(dir);
    while (used_indices.count(index)) ++index;
    return index;
}

void start_rosbag_record()
{
    rosbag_pid = fork();
    if (rosbag_pid < 0) { rosbag_status_msg = "Fork failed"; return; }
    if (rosbag_pid == 0) {
        execlp("rosbag", "rosbag", "record", "-O", rosbag_file_path.c_str(), "-a", NULL);
        _exit(1);
    }
    usleep(500000);
    int status = 0;
    pid_t result = waitpid(rosbag_pid, &status, WNOHANG);
    if (result == rosbag_pid && WIFEXITED(status) && WEXITSTATUS(status) != 0) rosbag_pid = -1;
}

void stop_rosbag_record()
{
    if (rosbag_pid <= 0) return;
    int status = 0;
    pid_t result;
    kill(rosbag_pid, SIGTERM);
    for (int i = 0; i < 30; ++i) {
        result = waitpid(rosbag_pid, &status, WNOHANG);
        if (result == rosbag_pid || result == -1) break;
        usleep(100000);
    }
    if (result == 0) { kill(rosbag_pid, SIGKILL); waitpid(rosbag_pid, &status, 0); }
    rosbag_pid = -1;

    std::string active_file = rosbag_file_path + ".active";
    std::ifstream infile(active_file.c_str());
    if (infile.good()) {
        infile.close();
        std::rename(active_file.c_str(), rosbag_file_path.c_str());
    }
}

void print_monitor_status()
{
    printf("\033[2J\033[H");
    printf("╔══════════════════════════════════════════════════════╗\n");
    printf("║            HERO Agent Monitor                       ║\n");
    printf("╠══════════════════════════════════════════════════════╣\n");
    printf("║ Yaw:   %7.1f / %7.1f  [%s]                \n", yaw, target_yaw, control_yaw_enabled ? " ON" : "OFF");
    printf("║ Depth: %7.3f / %7.3f  [%s]                \n", depth, target_depth, control_depth_enabled ? " ON" : "OFF");
    printf("║ Relay: %s   Laser: %s   Speed: %d         \n", relay_enabled ? "ON " : "OFF", laser_enabled ? "ON " : "OFF", move_speed);
    printf("║ Record: %-3s                                \n", record_flag.load() ? "REC" : "---");
    if (!rosbag_status_msg.empty()) printf("║ Rosbag: %s\n", rosbag_status_msg.c_str());
    if (!csv_status_msg.empty()) printf("║ CSV:    %s\n", csv_status_msg.c_str());
    printf("╠══════════════════════════════════════════════════════╣\n");
    printf("║ [Jetson]                                            ║\n");
    printf("║ Move: wasd/rf   Target: e=Send q=Reset              ║\n");
    printf("║ TDC: ,=On .=Off  Tune: y/h=Mb u=KKp i=KKv          ║\n");
    printf("║ Winch: 1=Cal 2/3=Meter 4/5=Step                    ║\n");
    printf("║ Recovery: z=Appr x=Close c=Final v=Deploy b=Off    ║\n");
    printf("║           /=ExpHold  ]=ExpClose                     ║\n");
    printf("║ Auto: t=Start g=Stop  Mosaic: p=On o=Off            ║\n");
    printf("║ Darknet: n=On m=Off   Rec: [=Exp R=Rosbag           ║\n");
    printf("╠──────────────────────────────────────────────────────╣\n");
    printf("║ [Arduino]                                           ║\n");
    printf("║ Relay: e=ON t=OFF  Light/Laser: r=ON f=OFF          ║\n");
    printf("║ Speed: z=+10 x=-10  Gripper: c=Open v=Stop b=Close ║\n");
    printf("║ Yaw: y=ON h=OFF n=Reset  i/k=Yaw±0.1               ║\n");
    printf("║ Depth: p=ON ;=OFF  o/l=Depth±0.1                   ║\n");
    printf("║ Throttle: u=+10 j=-10  PWM Init: g                  ║\n");
    printf("╚══════════════════════════════════════════════════════╝\n");
    fflush(stdout);
}
