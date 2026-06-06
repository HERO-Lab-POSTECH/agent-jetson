#include <ros/ros.h>
#include <cmath>
#include <cstdio>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include "hero_msgs/hero_agent_sensor.h"
#include "albc_control/albc_kinematics.h"
#include "albc_control/inverse_kinematics.h"
#include "albc_control/imu_processor.h"
#include "albc_control/attitude_controller.h"
#include "albc_control/mode_manager.h"
#include "albc_control/dashboard.h"

#include <dynamic_reconfigure/server.h>
#include <albc_control/ALBCControllerConfig.h>

#include <cstdlib>

using namespace albc;

// ==============================
// Control Mode / keyboard
// ==============================
//
// ControlMode / ManualSubMode enums, controlModeName(), the raw-terminal
// keyboard helpers (initKeyboard/closeKeyboard/readKey), the blocking
// selectModeInteractive(), cycleMode(), handleRuntimeKey() and the
// mode-change detection now live in ModeManager (mode_manager.h).

// ==============================
// Constants
// ==============================

// INTEGRAL_MAX / COS_EPSILON / COMMON_FACTOR_MAX / PID_BASE_X / PID_BASE_Y /
// DERIV_LPF_ALPHA / LEVEL_THRESHOLD now live inside AttitudeController (via the
// control_law.h / feedback_filters.h oracle constants). See attitude_controller.h.
// MANUAL_ANGLE_STEP / MANUAL_POS_STEP now live inside ModeManager.

// ==============================
// State Structures
// ==============================
//
// The former ControlGains / ControlState structs and the computeControlOutput()
// function now live inside AttitudeController (attitude_controller.h), which
// owns the feedback pipeline + 4-mode control law.

// ==============================
// Globals
// ==============================

static ModeManager mode_mgr;  // control-mode FSM + key handling (absorbs control_mode/manual_* + key funcs)
static AttitudeController attitude;  // feedback pipeline + 4-mode control law (absorbs ControlState/ControlGains/computeControlOutput)
static InverseKinematics ik;  // DLS IK solver (absorbs the former IKConfig)
static ImuProcessor imu_proc;  // IMU callback wrapper (owns yaw offset + cached RPY)
static float joint_current1_mA = 0.0f;
static float joint_current2_mA = 0.0f;

// ==============================
// Inverse Kinematics (Damped Least Squares)
// ==============================
// IK now lives in the InverseKinematics class (inverse_kinematics.h), which
// delegates each step to dls_oracle::updateJointAnglesOracle (dls_ik.h) — the
// byte-identical transcription of the former global updateJointAngles().

// ==============================
// Callbacks
// ==============================

// imuCallback now lives in the ImuProcessor class (imu_processor.h): onImu()
// delegates the rotation to rotateImu() (imu_rotation.h). state.current_* is
// synced from imu_proc at the top of each control-loop iteration.

void reconfigureCallback(albc_control::ALBCControllerConfig& config, uint32_t /*level*/) {
    mode_mgr.setMode(static_cast<ControlMode>(config.control_mode));

    attitude.setTargets(DEG2RAD(config.target_roll), DEG2RAD(config.target_pitch));

    attitude.setGains(config.M_td, config.Kp_td, config.Kd_td,
                      config.kp_roll, config.ki_roll, config.kd_roll,
                      config.kp_pitch, config.ki_pitch, config.kd_pitch,
                      config.gain_mult);

    ik.setConfig(config.ik_learning_rate, config.ik_lambda_base, config.ik_num_iterations);

    imu_proc.setOffset(DEG2RAD(config.imu_yaw_offset));

    mode_mgr.setManualState(config.manual_theta1, config.manual_theta2,
                            config.manual_x, config.manual_y);

    // Reset integrals on gain change (anti-windup)
    attitude.resetIntegrals();

    ROS_INFO("Reconfigure: mode=%s mult=%.2f M=%.4f Kp=%.3f Kd=%.1f",
             controlModeName(mode_mgr.mode()), attitude.gainMult(),
             attitude.Mtd(), attitude.Kptd(), attitude.Kdtd());
}

void jointCurrentsCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    if (msg->data.size() >= 2) {
        joint_current1_mA = msg->data[0];
        joint_current2_mA = msg->data[1];
    }
}

// ==============================
// Mode Selection / runtime keys / Control Law / Dashboard
// ==============================
//
// selectModeInteractive(), cycleMode(), handleRuntimeKey() now live in
// ModeManager (mode_manager.h). The 4-mode control-law switch (former
// computeControlOutput) lives inside AttitudeController::update ->
// computeControlOutputOracle (control_law.h). printDashboard() now lives in
// Dashboard::render (dashboard.h).

// atexit() needs a free function; forward to mode_mgr.closeKeyboard() so the
// terminal is restored on any exit path (former atexit(closeKeyboard), :376).
static void restoreKeyboardAtExit() { mode_mgr.closeKeyboard(); }

// ==============================
// Main
// ==============================

int main(int argc, char **argv) {
    ros::init(argc, argv, "albc_controller");
    ros::NodeHandle nh("~");

    // Load parameters
    int loop_rate_hz;
    double initial_theta1_deg, initial_theta2_deg;

    nh.param<int>("loop_rate_hz", loop_rate_hz, 50);

    // Gain params (loaded into locals, then handed to AttitudeController::setGains).
    double gain_mult;
    double M_td_base, Kp_td_base, Kd_td_base;
    double kp_roll_base, ki_roll_base, kd_roll_base;
    double kp_pitch_base, ki_pitch_base, kd_pitch_base;
    nh.param<double>("gain_mult", gain_mult, 1.5);

    nh.param<double>("td_control/M_td", M_td_base, 0.004);
    nh.param<double>("td_control/Kp_td", Kp_td_base, 0.04);
    nh.param<double>("td_control/Kd_td", Kd_td_base, 0.55);

    nh.param<double>("pid_roll/kp", kp_roll_base, 0.05);
    nh.param<double>("pid_roll/ki", ki_roll_base, 0.001);
    nh.param<double>("pid_roll/kd", kd_roll_base, 0.0005);

    nh.param<double>("pid_pitch/kp", kp_pitch_base, 0.05);
    nh.param<double>("pid_pitch/ki", ki_pitch_base, 0.001);
    nh.param<double>("pid_pitch/kd", kd_pitch_base, 0.0005);

    int    ik_num_iterations;
    double ik_learning_rate, ik_lambda_base;
    nh.param<int>("ik/num_iterations", ik_num_iterations, 3000);
    nh.param<double>("ik/learning_rate", ik_learning_rate, 0.02);
    nh.param<double>("ik/lambda_base", ik_lambda_base, 0.15);
    ik.setConfig(ik_learning_rate, ik_lambda_base, ik_num_iterations);

    double imu_yaw_offset_deg;
    nh.param<double>("imu_yaw_offset", imu_yaw_offset_deg, 45.0);
    imu_proc.setOffset(DEG2RAD(imu_yaw_offset_deg));

    nh.param<double>("initial_theta1_deg", initial_theta1_deg, 90.0);
    nh.param<double>("initial_theta2_deg", initial_theta2_deg, 90.0);

    double manual_theta1_deg, manual_theta2_deg, manual_x, manual_y;
    nh.param<double>("manual/theta1", manual_theta1_deg, 90.0);
    nh.param<double>("manual/theta2", manual_theta2_deg, 90.0);
    nh.param<double>("manual/x", manual_x, 0.0);
    nh.param<double>("manual/y", manual_y, 0.0);
    mode_mgr.setManualState(manual_theta1_deg, manual_theta2_deg, manual_x, manual_y);

    attitude.setGains(M_td_base, Kp_td_base, Kd_td_base,
                      kp_roll_base, ki_roll_base, kd_roll_base,
                      kp_pitch_base, ki_pitch_base, kd_pitch_base,
                      gain_mult);

    // Interactive mode selection (blocks until user picks 1/2/3/4)
    mode_mgr.selectModeInteractive();
    atexit(restoreKeyboardAtExit);  // restore terminal on any exit path

    // Initial joint angles and end-effector position
    double theta1 = DEG2RAD(initial_theta1_deg);
    double theta2 = DEG2RAD(initial_theta2_deg);

    double current_x, current_y;
    forwardKinematics(theta1, theta2, current_x, current_y);

    attitude.setTarget(current_x, current_y);

    // Dynamic reconfigure server (syncs YAML-loaded values, then enables runtime tuning)
    dynamic_reconfigure::Server<albc_control::ALBCControllerConfig> dr_server(nh);
    {
        albc_control::ALBCControllerConfig cfg;
        cfg.control_mode     = static_cast<int>(mode_mgr.mode());
        cfg.target_roll      = 0.0;
        cfg.target_pitch     = 0.0;
        cfg.gain_mult        = gain_mult;
        cfg.M_td             = M_td_base;
        cfg.Kp_td            = Kp_td_base;
        cfg.Kd_td            = Kd_td_base;
        cfg.kp_roll          = kp_roll_base;
        cfg.ki_roll          = ki_roll_base;
        cfg.kd_roll          = kd_roll_base;
        cfg.kp_pitch         = kp_pitch_base;
        cfg.ki_pitch         = ki_pitch_base;
        cfg.kd_pitch         = kd_pitch_base;
        cfg.ik_learning_rate  = ik_learning_rate;
        cfg.ik_lambda_base    = ik_lambda_base;
        cfg.ik_num_iterations = ik_num_iterations;
        cfg.imu_yaw_offset    = imu_yaw_offset_deg;
        cfg.manual_theta1     = manual_theta1_deg;
        cfg.manual_theta2     = manual_theta2_deg;
        cfg.manual_x          = manual_x;
        cfg.manual_y          = manual_y;
        dr_server.updateConfig(cfg);
    }
    dr_server.setCallback(boost::bind(&reconfigureCallback, _1, _2));

    // Subscribers
    ros::Subscriber imu_sub     = nh.subscribe("/hero_agent/sensors", 50,
                                               &ImuProcessor::onImu, &imu_proc);
    ros::Subscriber current_sub = nh.subscribe("/joint_currents", 10, jointCurrentsCallback);

    // Publishers
    ros::Publisher angle_pub_1 = nh.advertise<std_msgs::Float64>(
        "/hero_agent/active_joint1_position_controller/command", 1000);
    ros::Publisher angle_pub_2 = nh.advertise<std_msgs::Float64>(
        "/hero_agent/active_joint2_position_controller/command", 1000);
    ros::Publisher status_pub = nh.advertise<std_msgs::Float64MultiArray>(
        "/albc_status", 10);

    ros::Rate loop_rate(loop_rate_hz);
    int dashboard_counter = 0;
    const int dashboard_interval = loop_rate_hz / 4;  // ~4 Hz

    // Latch the mode-change baseline to the selected/reconfigured mode so the
    // first tick never fires a reset (former `static ControlMode prev_mode =
    // control_mode;` semantics).
    mode_mgr.syncPrevMode();

    while (ros::ok()) {
        // Runtime key input (non-blocking)
        int key = mode_mgr.readKey();
        if (key >= 0) mode_mgr.handleRuntimeKey(key);

        // Sync cached IMU attitude (ImuProcessor owns the source; updated in
        // the onImu callback during ros::spinOnce() at the loop bottom).
        double current_roll  = imu_proc.roll();
        double current_pitch = imu_proc.pitch();
        double current_yaw   = imu_proc.yaw();

        // Mode change: reset state for clean transition (prevents first-tick D-spike).
        // detectModeChange() returns true only on a transition into TDC/PID
        // (and latches prev_mode on every change, as the source did).
        if (mode_mgr.detectModeChange()) {
            forwardKinematics(theta1, theta2, current_x, current_y);
            // Seed setpoint to current EE, zero integrals, clear derivative /
            // angular-velocity memory so next tick produces derivative = 0.
            attitude.resetForModeChange(current_x, current_y,
                                        current_roll, current_pitch, current_yaw);
        }

        double dt = 1.0 / static_cast<double>(loop_rate_hz);
        double target_length = 0.0;

        // Angular velocity via numerical differentiation + LPF (state display).
        attitude.updateAngularVel(current_roll, current_pitch, current_yaw, dt);

        if (mode_mgr.mode() == ControlMode::MANUAL) {
            // Manual mode: bypass IMU feedback pipeline entirely
            if (mode_mgr.manualSubmode() == ManualSubMode::JOINT) {
                // Direct joint angle assignment — no IK needed
                theta1 = DEG2RAD(mode_mgr.manualTheta1());
                theta2 = DEG2RAD(mode_mgr.manualTheta2());
                forwardKinematics(theta1, theta2, current_x, current_y);
                attitude.setTarget(current_x, current_y);
            } else {
                // Direct EE position — workspace saturation + IK only
                double tx = mode_mgr.manualX(), ty = mode_mgr.manualY();
                ik.solveIK(tx, ty,
                           theta1, theta2, current_x, current_y, target_length);
                attitude.setTarget(tx, ty);   // store the saturated setpoint
            }
        } else {
            // Normal feedback modes: TDC / PID / FIXED
            // Error -> integral (freeze+clamp) -> derivative (gate+LPF) ->
            // 4-mode control law -> prev save. All inside AttitudeController::update.
            attitude.update(static_cast<int>(mode_mgr.mode()),
                            current_roll, current_pitch, dt);

            // Radial workspace saturation + inverse kinematics + final FK
            double tx = attitude.targetX(), ty = attitude.targetY();
            ik.solveIK(tx, ty,
                       theta1, theta2, current_x, current_y, target_length);
            attitude.setTarget(tx, ty);   // store the saturated setpoint
        }

        // Publish joint angles
        std_msgs::Float64 ros_theta1, ros_theta2;
        ros_theta1.data = mapTo2Pi(theta1);
        ros_theta2.data = mapTo2Pi(theta2);
        angle_pub_1.publish(ros_theta1);
        angle_pub_2.publish(ros_theta2);

        // Publish status
        std_msgs::Float64MultiArray status_msg;
        status_msg.data = {
            RAD2DEG(attitude.targetRoll()), RAD2DEG(current_roll),
            RAD2DEG(attitude.targetPitch()), RAD2DEG(current_pitch),
            attitude.targetX(), attitude.targetY(),
            current_x, current_y,
            attitude.angularVelRoll(), attitude.angularVelPitch(), attitude.angularVelYaw()
        };
        status_pub.publish(status_msg);

        // Dashboard (~4 Hz)
        if (++dashboard_counter >= dashboard_interval) {
            dashboard_counter = 0;
            Dashboard::render(mode_mgr, attitude,
                              theta1, theta2, current_x, current_y, target_length,
                              current_roll, current_pitch,
                              joint_current1_mA, joint_current2_mA);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    mode_mgr.closeKeyboard();
    return 0;
}
