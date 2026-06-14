#ifndef HERO_AGENT_CSV_LOGGER_H
#define HERO_AGENT_CSV_LOGGER_H

// record_flag + CSV 파일 기록 담당. agent.cpp record_flag/csv 전역군을 캡슐화.
// 무변경 리팩터: write_csv_line 본문·CSV 헤더 문자열·컬럼 순서·setprecision(6)·flush 보존.

#include <ros/ros.h>

#include <atomic>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <string>

#include "hero_agent/state_monitor.h"

namespace hero {

class CsvLogger {
public:
    CsvLogger() : record_flag(0) {}

    // key_input_callback의 'R' 처리
    void toggle() { record_flag.fetch_xor(1); }

    int flag() const { return record_flag.load(); }

    // 레코딩 시작: fout_csv 열고 헤더 라인 쓰기.
    //
    // CSV 17컬럼 = ros_time(1) + /albc/status data[0:11](11) + StateMonitor 게터(5).
    // 헤더 라벨과 writeLine()의 write 인덱스는 lock-step으로 정합해야 한다
    // (/albc/status 필드 순서가 바뀌면 둘 다 같이 고칠 것 — 안 그러면 침묵 어긋남):
    //   col  1      : ros_time
    //   col  2.. 9  : data[0..7]  target/current roll·pitch, target/current x·y
    //   col 10..12  : data[8..10] angular_vel_roll/pitch/yaw
    //   col 13..17  : targetDepth, currentDepth, sensorRoll, sensorPitch, sensorYaw
    void open(const std::string& path, StateMonitor& /*mon*/)
    {
        albc_csv_path = path;
        std::lock_guard<std::mutex> lock(csv_mutex);
        if (fout_csv.is_open()) fout_csv.close();
        fout_csv.open(albc_csv_path);
        if (fout_csv.is_open()) {
            fout_csv << "ros_time,target_roll,current_roll,target_pitch,current_pitch,target_x,target_y,current_x,current_y,angular_vel_roll,angular_vel_pitch,angular_vel_yaw,target_depth,depth,sensor_roll,sensor_pitch,sensor_yaw\n";
            fout_csv.flush();
            csv_status_msg = "Logging started";
        } else {
            csv_status_msg = "CSV open failed";
        }
    }

    // 레코딩 중지: fout_csv 닫기.
    void close()
    {
        std::lock_guard<std::mutex> lock(csv_mutex);
        if (fout_csv.is_open()) fout_csv.close();
    }

    // 종료 시 상태 메시지 갱신(main의 else 분기 보존)
    void setStopStatus() { csv_status_msg = "Logging stopped"; }

    // CSV line writer (write_csv_line 본문 그대로). 50Hz로 호출.
    void writeLine(StateMonitor& mon)
    {
        if (record_flag.load() != 1) return;

        double current_time = ros::Time::now().toSec();
        double data[11] = {0};
        {
            bool active = false;
            mon.copyAlbc(data, active);
        }

        std::lock_guard<std::mutex> lock(csv_mutex);
        if (fout_csv.is_open()) {
            // write order MUST match the header in open() column-for-column
            // (ros_time, data[0..10], then the 5 StateMonitor getters).
            fout_csv << std::fixed << std::setprecision(6)
                << current_time << ","
                << data[0] << "," << data[1] << ","
                << data[2] << "," << data[3] << ","
                << data[4] << "," << data[5] << ","
                << data[6] << "," << data[7] << ","
                << data[8] << "," << data[9] << "," << data[10] << ","
                << mon.targetDepth() << "," << mon.currentDepth() << ","
                << mon.sensorRoll() << "," << mon.sensorPitch() << "," << mon.sensorYaw() << "\n";
            fout_csv.flush();
        }
    }

    const std::string& csv_status() const { return csv_status_msg; }

private:
    std::atomic<int> record_flag;
    std::mutex csv_mutex;
    std::ofstream fout_csv;
    std::string albc_csv_path = "";
    std::string csv_status_msg = "";
};

}  // namespace hero

#endif  // HERO_AGENT_CSV_LOGGER_H
