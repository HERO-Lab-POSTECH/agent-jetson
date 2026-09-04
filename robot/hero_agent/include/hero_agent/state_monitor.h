#ifndef HERO_AGENT_STATE_MONITOR_H
#define HERO_AGENT_STATE_MONITOR_H

// State/IMU/ALBC 캐시 + 모니터 렌더 담당. agent.cpp 전역 상태군을 책임별로 캡슐화.
// 콜백 클래스라 ROS-free 아님(hero_msgs/std_msgs/ros::Time 의존).
// 무변경 리팩터: 본문 로직·락 순서·연산 순서를 통합본 그대로 보존, 위치만 이동.

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include "hero_msgs/hero_agent_state.h"
#include "hero_msgs/hero_agent_sensor.h"
#include "albc_control/imu_rotation.h"

#include <cmath>
#include <cstdio>
#include <mutex>

namespace hero {

class StateMonitor {
public:
    StateMonitor() {}

    // ==============================
    // State callback - just update monitoring variables
    // ==============================
    void onState(const hero_msgs::hero_agent_state::ConstPtr& msg)
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
    void onSensor(const hero_msgs::hero_agent_sensor::ConstPtr& msg)
    {
        ImuRpy out = rotateImu(msg->ROLL, msg->PITCH, msg->YAW, imu_yaw_offset_rad);
        sensor_roll  = static_cast<float>(out.roll);
        sensor_pitch = static_cast<float>(out.pitch);
        sensor_yaw   = msg->YAW;
    }

    // ==============================
    // ALBC status callback - cache data only
    // ==============================
    // /albc/status ABI = unnamed Float64MultiArray[11] (frozen-by-convention; see
    // csv_logger.h header<->index map). Minimal guards added (size + finiteness):
    // they only REJECT malformed/garbage messages so they aren't silently cached and
    // mirrored into CSV / RL obs. A well-formed 11-field finite message is cached
    // byte-identically to before -- the normal path is unchanged.
    void onAlbc(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {
        if (msg->data.size() != 11)
        {
            ROS_WARN_THROTTLE(5.0,
                "/albc/status size %d != 11 (frozen ABI); message dropped",
                static_cast<int>(msg->data.size()));
            return;
        }
        for (int i = 0; i < 11; i++)
        {
            if (!std::isfinite(msg->data[i]))
            {
                ROS_WARN_THROTTLE(5.0,
                    "/albc/status field %d not finite (%f); message dropped",
                    i, msg->data[i]);
                return;
            }
        }
        std::lock_guard<std::mutex> lock(albc_mutex);
        for (int i = 0; i < 11; i++) albc_data[i] = msg->data[i];
        albc_active = true;
    }

    // main의 imu_yaw_offset 파라미터 로딩용
    void setImuOffset(double rad) { imu_yaw_offset_rad = rad; }

    // ALBC 데이터 읽기(CsvLogger가 호출): write_csv_line의 albc 락+복사 블록 캡슐화.
    // 락 안에서 albc_active면 복사, 아니면 호출자가 채운 0 유지.
    void copyAlbc(double out[11], bool& active_out)
    {
        std::lock_guard<std::mutex> lock(albc_mutex);
        if (albc_active) {
            for (int i = 0; i < 11; i++) out[i] = albc_data[i];
        }
        active_out = albc_active;
    }

    // ALBC 리셋(레코딩 시작 시 main이 호출): main의 albc_active=false; albc_data=0 블록 그대로.
    void resetAlbc()
    {
        std::lock_guard<std::mutex> lock(albc_mutex);
        albc_active = false;
        for (int i = 0; i < 11; i++) albc_data[i] = 0;
    }

    // ==============================
    // 모니터 렌더 (print_monitor_status 본문 그대로).
    // record_flag·status_msg는 CsvLogger/RosbagRecorder 소유라 인자로 받는다.
    // ==============================
    void print(int record_flag, const std::string& rosbag_status_msg,
               const std::string& csv_status_msg)
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
               move_speed, record_flag ? "REC" : "---");
        printf("═══════════════════════════════════════════════════\n");
        printf(" STARTUP: 1=Relay 3=Yaw 4=Depth\n");
        printf("═══════════════════════════════════════════════════\n");
        printf(" Toggle  1=Relay  3=Yaw  4=Depth  5=Laser\n");
        printf(" Init    2=PWM  N=YawReset\n");
        printf(" Move    w/s/a/d/q  r/f=Heave\n");
        printf(" Speed   y/h=+/-10  u/j=Throttle+/-10\n");
        printf(" Yaw     i/k=+/-0.1\n");
        printf(" Depth   o/l=+/-0.1\n");
        printf(" Grip    c=Open  v=Stop  b=Close\n");
        printf(" Rec     R=Rosbag\n");
        printf("═══════════════════════════════════════════════════\n");
        if (!rosbag_status_msg.empty()) printf(" Rosbag: %s\n", rosbag_status_msg.c_str());
        if (!csv_status_msg.empty())    printf(" CSV:    %s\n", csv_status_msg.c_str());
        fflush(stdout);
    }

    // CsvLogger가 CSV 컬럼으로 읽는 게터
    float targetDepth() const { return target_depth; }
    float currentDepth() const { return depth; }
    float sensorRoll() const { return sensor_roll; }
    float sensorPitch() const { return sensor_pitch; }
    float sensorYaw() const { return sensor_yaw; }

    // 모니터 상태 게터(firmware State_addit 비트 미러). firmware f569da4는 self-toggle
    // 이라 key_input_callback이 더 이상 이 게터로 ON/OFF char를 가르지 않는다(그 분기 제거).
    // HUD 표시(print)와 향후 용도로 유지 — backing 필드는 onState가 계속 갱신한다.
    int relayEnabled() const { return relay_enabled; }
    int controlYawEnabled() const { return control_yaw_enabled; }
    int controlDepthEnabled() const { return control_depth_enabled; }
    int laserEnabled() const { return laser_enabled; }

private:
    // State 캐시 (Arduino state 토픽)
    float yaw = 0.0f, target_yaw = 0.0f;
    float depth = 0.0f, target_depth = 0.0f;
    int move_speed = 0;
    int control_flags = 0;
    int control_yaw_enabled = 0, control_depth_enabled = 0;
    int relay_enabled = 0, laser_enabled = 0;

    // IMU 캐시 (/hero_agent/sensors)
    float sensor_roll = 0.0f, sensor_pitch = 0.0f, sensor_yaw = 0.0f;
    // 102.0 deg CONFIRMED 2026-08-12 (was -78.0, itself 180 deg out; before that
    // 45.0). Overwritten at startup from /albc_controller/imu_yaw_offset; this is
    // only the fallback.
    double imu_yaw_offset_rad = 102.0 * M_PI / 180.0;

    // ALBC 캐시
    std::mutex albc_mutex;
    double albc_data[11] = {0};
    bool albc_active = false;
};

}  // namespace hero

#endif  // HERO_AGENT_STATE_MONITOR_H
