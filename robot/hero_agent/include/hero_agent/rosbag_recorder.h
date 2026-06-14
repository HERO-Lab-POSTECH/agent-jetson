#ifndef HERO_AGENT_ROSBAG_RECORDER_H
#define HERO_AGENT_ROSBAG_RECORDER_H

// rosbag record 서브프로세스 fork/exec/signal 캡슐화. agent.cpp rosbag 전역군을 캡슐화.
// 무변경 리팩터: start_rosbag_record/stop_rosbag_record 본문·토픽 인자·폴링 횟수 보존.

#include <ros/ros.h>

#include <cerrno>
#include <cstdio>
#include <fstream>
#include <string>

#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>

namespace hero {

class RosbagRecorder {
public:
    RosbagRecorder() : rosbag_pid(-1) {}

    // start_rosbag_record 본문 그대로. bag_path를 멤버에 저장(stop의 .active rename용).
    void start(const std::string& bag_path)
    {
        rosbag_file_path = bag_path;
        rosbag_pid = fork();
        if (rosbag_pid < 0) { rosbag_status_msg = "Fork failed"; return; }
        if (rosbag_pid == 0) {
            execlp("rosbag", "rosbag", "record", "-O", rosbag_file_path.c_str(),
                   "/albc/status",
                   "/hero_agent/state",
                   "/hero_agent/sensors",
                   "/hero_agent/dvl",
                   "/joint_currents",
                   NULL);
            _exit(1);
        }
        usleep(500000);
        int status = 0;
        pid_t result = waitpid(rosbag_pid, &status, WNOHANG);
        if (result == rosbag_pid && WIFEXITED(status) && WEXITSTATUS(status) != 0) rosbag_pid = -1;
    }

    // stop_rosbag_record 본문 그대로 (SIGTERM→30회 폴링→SIGKILL→.active rename).
    void stop()
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

    bool active() const { return rosbag_pid > 0; }

    void setStatus(const std::string& s) { rosbag_status_msg = s; }
    const std::string& rosbag_status() const { return rosbag_status_msg; }

private:
    pid_t rosbag_pid;
    std::string rosbag_file_path = "";
    std::string rosbag_status_msg = "";
};

}  // namespace hero

#endif  // HERO_AGENT_ROSBAG_RECORDER_H
