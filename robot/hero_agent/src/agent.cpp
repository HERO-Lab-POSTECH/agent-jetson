// ==============================
// agent: single-node V3 key translation + teleop + state monitoring + rosbag
//
// Architecture (single node — no topic round-trip):
//   key_teleop.py → /hero_agent/key_input (V3 keys)
//                          ↓
//                   agent.cpp (THIS FILE)
//                 ↓                    ↓
//   /hero_agent/command          TeleopController.apply() → /hero_agent/dvl
//   (Arduino original keys)      (in-process target update)
//
// 전역 상태·자유함수는 책임별 3 클래스로 분해(무변경 리팩터):
//   StateMonitor   = State/IMU/ALBC 캐시 + 모니터 렌더
//   CsvLogger      = record_flag + CSV 파일 기록
//   RosbagRecorder = rosbag record 서브프로세스 fork/exec/signal
// 콜백 인스턴스는 파일 스코프 전역 — 통합본도 전역이라 거동 동일.
// ==============================

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64MultiArray.h>
#include "hero_msgs/hero_agent_state.h"
#include "hero_msgs/hero_agent_sensor.h"
#include "hero_msgs/hero_agent_dvl.h"

#include "hero_agent/key_translator.h"
#include "hero_agent/teleop_controller.h"
#include "hero_agent/topics.h"
#include "hero_agent/state_monitor.h"
#include "hero_agent/csv_logger.h"
#include "hero_agent/rosbag_recorder.h"

#include <iostream>
#include <string>
#include <cstdlib>
#include <cmath>
#include <regex>
#include <atomic>
#include <set>

#include <unistd.h>
#include <signal.h>
#include <csignal>
#include <sys/types.h>
#include <sys/stat.h>
#include <cerrno>
#include <dirent.h>

using namespace std;
using namespace hero;

// ==============================
// Responsibility classes (file-scope instances — original was all-global, so
// free callbacks referencing them keeps behavior identical).
// ==============================
StateMonitor   state_monitor;
CsvLogger      csv_logger;
RosbagRecorder rosbag_recorder;

// ==============================
// Toggle debounce (500ms) — KeyTranslator-layer concern, kept as free global.
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
ros::Publisher pub_target;
std_msgs::Int8 command_msg;

// ==============================
// Teleop (in-process — replaces the old translated-key topic round-trip)
// xy_step/z_step match original hardcoded teleop defaults (0.05 / 0.01)
// ==============================
TeleopController g_teleop(0.05, 0.01);
std::atomic<bool> g_target_dirty(false);

// Signal flag (async-signal-safe)
static volatile sig_atomic_t signal_received = 0;

// Forward declarations
void send_command(char cmd);
void ensure_directory(const std::string& path);
int get_next_log_index(const std::string& base_path);
void handle_signal(int sig);

// ==============================
// V3 Key Translation Layer (single-node: translate then dispatch in-process)
//
// translate_key (key_translator.h) is the byte-identical oracle for V3 keys.
// debounce stays a callback-layer concern (KeyTranslator is a pure function):
// only toggle keys with debounce flag are 500ms gated — '1'/'2'/'3'/'5'.
// '4' (PWM) is toggle=false so it is NOT gated, preserving prior behavior.
// ==============================
void key_input_callback(const std_msgs::Int8::ConstPtr& msg)
{
    int ch = msg->data;

    // Rosbag toggle (KeyTranslator 범위 밖, agent 내부 플래그)
    if (ch == 'R') {
        csv_logger.toggle();
        return;
    }

    // Current toggle state (built from state-callback cache)
    ToggleState st;
    st.relay_enabled         = state_monitor.relayEnabled();
    st.control_yaw_enabled   = state_monitor.controlYawEnabled();
    st.control_depth_enabled = state_monitor.controlDepthEnabled();
    st.laser_enabled         = state_monitor.laserEnabled();

    // Debounce: only toggle keys carrying the debounce flag are gated.
    // '1'/'2'/'3'/'5' are toggle=true+debounce=true → gated.
    // '4' is toggle=false → excluded, preserving the prior gated key set.
    const KeyDef* kd = lookup_key(ch);
    if (kd && kd->toggle && kd->debounce) {
        if (!debounce_ok(ch)) return;
    }

    KeyXlate out = translate_key(ch, st);
    if (out.cmd != 0)
        send_command(out.cmd);
    if (out.translated != 0) {
        if (g_teleop.apply(out.translated))
            g_target_dirty.store(true);
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

    ros::init(argc, argv, "agent", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Load IMU yaw offset (same parameter as albc_controller)
    double imu_yaw_offset_deg;
    nh.param<double>("/albc_controller/imu_yaw_offset", imu_yaw_offset_deg, 45.0);
    state_monitor.setImuOffset(imu_yaw_offset_deg * M_PI / 180.0);

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    // Initialize debounce timestamps
    ros::Time now = ros::Time::now();
    for (int i = 0; i < 256; i++) last_toggle_time[i] = now;

    // Subscribers
    ros::Subscriber sub_state = nh.subscribe(topics::STATE, 100, &StateMonitor::onState, &state_monitor);
    ros::Subscriber sub_sensor = nh.subscribe(topics::SENSORS, 100, &StateMonitor::onSensor, &state_monitor);
    ros::Subscriber sub_albc_status = nh.subscribe(topics::ALBC_STATUS, 10, &StateMonitor::onAlbc, &state_monitor);
    ros::Subscriber sub_key_input = nh.subscribe(topics::KEY_INPUT, 10, key_input_callback);

    // Publishers
    pub_command = nh.advertise<std_msgs::Int8>(topics::COMMAND, 100);
    pub_target = nh.advertise<hero_msgs::hero_agent_dvl>(topics::DVL, 100);

    ros::Rate loop_rate(100);
    int csv_counter = 0;
    int prev_record_flag = 0;

    printf("\n  Agent Initialized (V3 key translation + teleop + monitor)\n\n");

    while (ros::ok() && !signal_received)
    {
        // Recording state change
        int current_record = csv_logger.flag();
        if (current_record != prev_record_flag) {
            if (current_record == 1) {
                int log_index = get_next_log_index(base_traj_dir);
                std::string albc_csv_path = base_traj_dir + "/albc_status_" + std::to_string(log_index) + ".csv";
                std::string rosbag_file_path = base_rosbag_dir + "/record_" + std::to_string(log_index) + ".bag";
                rosbag_recorder.start(rosbag_file_path);
                rosbag_recorder.setStatus(rosbag_recorder.active() ? "Recording started" : "Rosbag failed, CSV only");
                // Open CSV independently of rosbag
                csv_logger.open(albc_csv_path, state_monitor);
                // Reset ALBC active flag for new recording
                state_monitor.resetAlbc();
            } else {
                csv_logger.close();
                csv_logger.setStopStatus();
                rosbag_recorder.stop();
                rosbag_recorder.setStatus("Recording stopped");
            }
            prev_record_flag = current_record;
        }

        // Publish teleop target when changed (command=0 for incremental updates)
        if (g_target_dirty.exchange(false)) {
            hero_msgs::hero_agent_dvl msg_target;
            msg_target.command = 0;
            msg_target.TARGET_X = g_teleop.x();
            msg_target.TARGET_Y = g_teleop.y();
            msg_target.TARGET_Z = g_teleop.z();
            pub_target.publish(msg_target);
        }

        // Write CSV at 50Hz (every 2nd iteration of 100Hz loop)
        if (++csv_counter >= 2) {
            csv_logger.writeLine(state_monitor);
            csv_counter = 0;
        }

        state_monitor.print(csv_logger.flag(), rosbag_recorder.rosbag_status(), csv_logger.csv_status());
        loop_rate.sleep();
    }

    spinner.stop();
    rosbag_recorder.stop();
    csv_logger.close();
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

void handle_signal(int sig)
{
    signal_received = 1;  // async-signal-safe: only set flag
}

void ensure_directory(const std::string& path)
{
    struct stat st;
    if (stat(path.c_str(), &st) != 0) {
        if (system(("mkdir -p " + path).c_str()) != 0)
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
