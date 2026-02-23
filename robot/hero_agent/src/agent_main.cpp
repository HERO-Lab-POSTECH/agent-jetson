// ==============================
// agent_main: V3 key translation layer, state monitoring, rosbag recording
//
// Architecture:
//   key_teleop.py → /hero_agent/key_input (V3 keys)
//                          ↓
//                agent_main.cpp (THIS FILE — TRANSLATION LAYER)
//                 ↓                    ↓
//   /hero_agent/command          /hero_agent/key_translated
//   (Arduino original keys)      (Jetson original keys)
//                                      ↓
//                          agent_command processKey()
//
// FSM code backed up in agent_main_fsm_backup.cpp
// ==============================

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64MultiArray.h>
#include "hero_msgs/hero_agent_state.h"
#include "hero_msgs/hero_agent_sensor.h"

#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <regex>
#include <atomic>
#include <mutex>
#include <set>

#include <unistd.h>
#include <signal.h>
#include <csignal>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <cerrno>
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
int relay_enabled = 0, laser_enabled = 0;

// ==============================
// IMU sensor variables (from /hero_agent/sensors)
// ==============================
float sensor_roll = 0.0f, sensor_pitch = 0.0f, sensor_yaw = 0.0f;
float sensor_depth = 0.0f;

// ==============================
// Jetson-only toggle states (not available from Arduino)
// ==============================
static bool lawnmower_on = false;

// ==============================
// Toggle debounce (500ms)
// ==============================
static ros::Time last_toggle_time[256];
static const double DEBOUNCE_SEC = 0.5;

static bool debounce_ok(int ch)
{
    ros::Time now = ros::Time::now();
    unsigned char idx = (unsigned char)ch;
    if ((now - last_toggle_time[idx]).toSec() < DEBOUNCE_SEC)
        return false;
    last_toggle_time[idx] = now;
    return true;
}

// ==============================
// ROS publishers and messages
// ==============================
ros::Publisher pub_command;
ros::Publisher pub_key_translated;
std_msgs::Int8 command_msg;
std_msgs::Int8 translated_msg;

// ==============================
// Rosbag recording
// ==============================
std::atomic<int> record_flag(0);
int prev_record_flag = 0;
pid_t rosbag_pid = -1;
std::mutex csv_mutex;
std::ofstream fout_csv;
std::string rosbag_file_path = "";
std::string albc_csv_path = "";
std::string rosbag_status_msg = "";
std::string csv_status_msg = "";

// Signal flag (async-signal-safe)
static volatile sig_atomic_t signal_received = 0;

// Forward declarations
void send_command(char cmd);
void send_translated(char cmd);
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
}

// ==============================
// IMU sensor callback
// ==============================
void sensorCallback(const hero_msgs::hero_agent_sensor::ConstPtr& msg)
{
    sensor_roll  = msg->ROLL;
    sensor_pitch = msg->PITCH;
    sensor_yaw   = msg->YAW;
    sensor_depth = msg->DEPTH;
}

// ==============================
// ALBC status CSV logging
// ==============================
void albcStatusCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (record_flag.load() == 1 && msg->data.size() >= 8)
    {
        double current_time = ros::Time::now().toSec();
        std::lock_guard<std::mutex> lock(csv_mutex);
        if (fout_csv.is_open()) {
            fout_csv
                << current_time << ","
                << msg->data[0] << "," << msg->data[1] << ","
                << msg->data[2] << "," << msg->data[3] << ","
                << msg->data[4] << "," << msg->data[5] << ","
                << msg->data[6] << "," << msg->data[7] << ","
                << target_depth << "," << depth << ","
                << sensor_roll << "," << sensor_pitch << "," << sensor_yaw << "\n";
            fout_csv.flush();
        }
    }
}

// ==============================
// V3 Key Translation Layer
//
// Receives V3 key codes from /hero_agent/key_input and routes to:
//   - /hero_agent/command (Arduino) via send_command()
//   - /hero_agent/key_translated (Jetson) via send_translated()
//   - Both, or neither (blocked keys)
// ==============================
void key_input_callback(const std_msgs::Int8::ConstPtr& msg)
{
    int ch = msg->data;

    switch (ch) {

    // ── Number Row: Hardware Toggles ──

    case '1':  // Toggle Relay
        if (!debounce_ok('1')) break;
        send_command(relay_enabled ? 't' : 'e');
        break;

    case '2':  // Toggle Yaw
        if (!debounce_ok('2')) break;
        send_command(control_yaw_enabled ? 'h' : 'y');
        break;

    case '3':  // Toggle Depth
        if (!debounce_ok('3')) break;
        send_command(control_depth_enabled ? ';' : 'p');
        break;

    case '4':  // PWM Init → Arduino only
        send_command('g');
        break;

    case '5':  // Toggle Laser
        if (!debounce_ok('5')) break;
        send_command(laser_enabled ? 'f' : 'r');
        break;

    // ── Letter Keys: Jetson-Only ──

    case 'r':  // Heave Up (target.z)
        send_translated('r');
        break;

    case 'f':  // Heave Down (target.z)
        send_translated('f');
        break;

    case 'p':  // Toggle Lawnmower (Jetson only)
        if (!debounce_ok('p')) break;
        lawnmower_on = !lawnmower_on;
        send_translated(lawnmower_on ? 'p' : 'o');
        break;

    // ── Letter Keys: Arduino-Only ──

    case 'N':  // Yaw Reset → Arduino 'n'
        send_command('n');
        break;

    case 'o':  // Depth -0.1 → Arduino 'o'
        send_command('o');
        break;

    // ── Blocked Keys (freed, no function) ──

    case ';':
    case 'n':
    case 'm':
    case '.':
    case ',':
    case 't':
    case 'g':
    case 'y':
    case 'h':
    case 'e':   // was: Send Target (removed)
    case 'q':   // was: Target Reset (removed)
    case '6':   // was: Winch Calibrate (removed)
    case '7':   // was: Winch Meter + (removed)
    case '8':   // was: Winch Meter - (removed)
    case '9':   // was: Winch Step + (removed)
    case '0':   // was: Winch Step - (removed)
        break;

    // ── Rosbag Toggle (agent_main internal) ──

    case 'R':
        record_flag.fetch_xor(1);
        break;

    // ── Pass-Through: Both Arduino + Jetson ──
    // w/s/a/d, z/x, u/j, i/k, l, c/v/b
    default:
        send_command(ch);
        send_translated(ch);
        break;
    }
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

    // Initialize debounce timestamps
    ros::Time now = ros::Time::now();
    for (int i = 0; i < 256; i++) last_toggle_time[i] = now;

    // Subscribers
    ros::Subscriber sub_state = nh.subscribe("/hero_agent/state", 100, msg_callback_state);
    ros::Subscriber sub_sensor = nh.subscribe("/hero_agent/sensors", 100, sensorCallback);
    ros::Subscriber sub_albc_status = nh.subscribe("/albc_status", 10, albcStatusCallback);
    ros::Subscriber sub_key_input = nh.subscribe("/hero_agent/key_input", 10, key_input_callback);

    // Publishers
    pub_command = nh.advertise<std_msgs::Int8>("/hero_agent/command", 100);
    pub_key_translated = nh.advertise<std_msgs::Int8>("/hero_agent/key_translated", 100);

    ros::Rate loop_rate(100);

    printf("\n  Agent Main Initialized (V3 key translation + monitor)\n\n");

    while (ros::ok() && !signal_received)
    {
        // Recording state change
        int current_record = record_flag.load();
        if (current_record != prev_record_flag) {
            if (current_record == 1) {
                int log_index = get_next_log_index(base_traj_dir);
                albc_csv_path = base_traj_dir + "/albc_status_" + std::to_string(log_index) + ".csv";
                rosbag_file_path = base_rosbag_dir + "/record_" + std::to_string(log_index) + ".bag";
                start_rosbag_record();
                if (rosbag_pid > 0) {
                    std::lock_guard<std::mutex> lock(csv_mutex);
                    if (fout_csv.is_open()) fout_csv.close();
                    fout_csv.open(albc_csv_path);
                    if (fout_csv.is_open()) {
                        fout_csv << "ros_time,target_roll,current_roll,target_pitch,current_pitch,target_x,target_y,current_x,current_y,target_depth,depth,sensor_roll,sensor_pitch,sensor_yaw\n";
                        fout_csv.flush();
                        csv_status_msg = "Logging started";
                    }
                    rosbag_status_msg = "Recording started";
                } else {
                    record_flag.store(0);
                    rosbag_status_msg = "Recording failed (fork error)";
                }
            } else {
                {
                    std::lock_guard<std::mutex> lock(csv_mutex);
                    if (fout_csv.is_open()) fout_csv.close();
                }
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
    {
        std::lock_guard<std::mutex> lock(csv_mutex);
        if (fout_csv.is_open()) fout_csv.close();
    }
    ros::shutdown();
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

void send_translated(char cmd)
{
    translated_msg.data = cmd;
    pub_key_translated.publish(translated_msg);
}

void handle_signal(int sig)
{
    signal_received = 1;  // async-signal-safe: only set flag
}

void ensure_directory(const std::string& path)
{
    struct stat st;
    if (stat(path.c_str(), &st) != 0) {
        if (mkdir(path.c_str(), 0775) != 0)
            ROS_ERROR("Failed to create directory: %s (errno=%d)", path.c_str(), errno);
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
        if (std::rename(active_file.c_str(), rosbag_file_path.c_str()) != 0)
            ROS_ERROR("Failed to rename %s → %s (errno=%d)", active_file.c_str(), rosbag_file_path.c_str(), errno);
    }
}

void print_monitor_status()
{
    printf("\033[2J\033[H");
    printf("═══════════════════════════════════════════════════\n");
    printf("              HERO Agent Monitor\n");
    printf("═══════════════════════════════════════════════════\n");
    printf(" Yaw   %7.1f / %7.1f  [%s]\n",
           yaw, target_yaw, control_yaw_enabled ? " ON" : "OFF");
    printf(" Depth %7.3f / %7.3f  [%s]\n",
           depth, target_depth, control_depth_enabled ? " ON" : "OFF");
    printf(" Roll  %7.2f   Pitch %7.2f\n", sensor_roll, sensor_pitch);
    printf(" Relay %-3s   Laser %-3s   Speed %-3d   Rec %-3s\n",
           relay_enabled ? "ON" : "OFF", laser_enabled ? "ON" : "OFF",
           move_speed, record_flag.load() ? "REC" : "---");
    printf("═══════════════════════════════════════════════════\n");
    printf(" STARTUP: 1=Relay 2=Yaw 3=Depth\n");
    printf("═══════════════════════════════════════════════════\n");
    printf(" Toggle  1=Relay  2=Yaw  3=Depth  5=Laser\n");
    printf(" Init    4=PWM  N=YawReset\n");
    printf(" Move    w/s/a/d  r/f=Heave\n");
    printf(" Speed   z/x=+/-10  u/j=Throttle+/-10\n");
    printf(" Yaw     i/k=+/-0.1\n");
    printf(" Depth   o/l=+/-0.1\n");
    printf(" Grip    c=Open  v=Stop  b=Close\n");
    printf(" Auto    p=Lawnmower\n");
    printf(" Rec     R=Rosbag\n");
    printf("═══════════════════════════════════════════════════\n");
    if (!rosbag_status_msg.empty()) printf(" Rosbag: %s\n", rosbag_status_msg.c_str());
    if (!csv_status_msg.empty())    printf(" CSV:    %s\n", csv_status_msg.c_str());
    fflush(stdout);
}
