#include <ros/ros.h>
#include <cmath>
#include <cstdio>
#include "hero_msgs/hero_agent_sensor.h"
#include "albc_control/albc_kinematics.h"
#include "albc_control/inverse_kinematics.h"
#include "albc_control/imu_processor.h"
#include "albc_control/attitude_controller.h"
#include "albc_control/mode_manager.h"
#include "albc_control/joint_current_monitor.h"
#include "albc_control/status_publisher.h"
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
// keyboard helpers (initKeyboard/closeKeyboard/readKey), initInteractive()
// (param-driven startup, Task-7), cycleMode(), handleRuntimeKey() and the
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
//
// All former file-scope state now lives in classes owned by main():
//   ModeManager / AttitudeController / InverseKinematics / ImuProcessor
//   JointCurrentMonitor (absorbs joint_current1/2_mA + jointCurrentsCallback)
//   StatusPublisher      (absorbs angle_pub_1/2 + status_pub + the publish blocks)
// The only file-scope object left is g_mode_mgr below — a non-owning pointer the
// atexit handler needs, because atexit() takes a zero-arg free function and so
// cannot capture main()'s ModeManager. It is set to main()'s `static` local
// mode_mgr (program lifetime), so the handler is valid even after main returns.
static ModeManager* g_mode_mgr = nullptr;

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

// Dynamic-reconfigure callback. Takes the four configured instances by pointer
// (bound via boost::bind in main) since they are now main() locals, not globals.
// The body order and every operation are byte-identical to the former version.
void reconfigureCallback(albc_control::ALBCControllerConfig& config, uint32_t /*level*/,
                         ModeManager* mode_mgr, AttitudeController* attitude,
                         InverseKinematics* ik, ImuProcessor* imu_proc) {
    mode_mgr->setMode(static_cast<ControlMode>(config.control_mode));

    attitude->setTargets(DEG2RAD(config.target_roll), DEG2RAD(config.target_pitch));

    attitude->setGains(config.M_td, config.Kp_td,
                       config.kp_roll, config.ki_roll, config.kd_roll,
                       config.kp_pitch, config.ki_pitch, config.kd_pitch,
                       config.gain_mult);

    ik->setConfig(config.ik_learning_rate, config.ik_lambda_base, config.ik_num_iterations);

    imu_proc->setOffset(DEG2RAD(config.imu_yaw_offset));

    mode_mgr->setManualState(config.manual_theta1, config.manual_theta2,
                             config.manual_x, config.manual_y);

    // Reset integrals on gain change (anti-windup)
    attitude->resetIntegrals();

    ROS_INFO("Reconfigure: mode=%s mult=%.2f M=%.4f Kp=%.3f",
             controlModeName(mode_mgr->mode()), attitude->gainMult(),
             attitude->Mtd(), attitude->Kptd());
}

// ==============================
// Mode Selection / runtime keys / Control Law / Dashboard
// ==============================
//
// initInteractive(), cycleMode(), handleRuntimeKey() now live in
// ModeManager (mode_manager.h). The 4-mode control-law switch (former
// computeControlOutput) lives inside AttitudeController::update ->
// computeControlOutputOracle (control_law.h). printDashboard() now lives in
// Dashboard::render (dashboard.h).

// atexit() needs a zero-arg free function; forward to the active ModeManager's
// closeKeyboard() (via g_mode_mgr) so the terminal is restored on any exit path
// (former atexit(closeKeyboard), :376).
static void restoreKeyboardAtExit() { if (g_mode_mgr) g_mode_mgr->closeKeyboard(); }

// ==============================
// Main
// ==============================

int main(int argc, char **argv) {
    ros::init(argc, argv, "albc_controller");
    ros::NodeHandle nh("~");

    // Component instances (former file-scope globals, now owned by main).
    // mode_mgr is a `static` local: it is no longer a file-scope global, yet
    // keeps program lifetime so the atexit terminal-restore handler (which runs
    // AFTER main returns and locals are destroyed) never dereferences a dead
    // object — exactly the safety the former file-scope `static ModeManager` had.
    static ModeManager   mode_mgr;     // control-mode FSM + key handling
    AttitudeController   attitude;     // feedback pipeline + 4-mode control law
    InverseKinematics    ik;           // DLS IK solver
    ImuProcessor         imu_proc;     // IMU callback wrapper (yaw offset + cached RPY)
    JointCurrentMonitor  current_mon;  // cached joint motor currents for the dashboard
    StatusPublisher      status_pub;   // joint-angle + /albc_status publishers

    g_mode_mgr = &mode_mgr;  // for the atexit terminal-restore handler

    // Load parameters
    int loop_rate_hz;
    double initial_theta1_deg, initial_theta2_deg;

    nh.param<int>("loop_rate_hz", loop_rate_hz, 50);

    // Gain params (loaded into locals, then handed to AttitudeController::setGains).
    double gain_mult;
    double M_td_base, Kp_td_base;
    double kp_roll_base, ki_roll_base, kd_roll_base;
    double kp_pitch_base, ki_pitch_base, kd_pitch_base;
    nh.param<double>("gain_mult", gain_mult, 3.0);  // fallback matches yaml/cfg (D4 SSOT)

    nh.param<double>("td_control/M_td", M_td_base, 0.004);
    nh.param<double>("td_control/Kp_td", Kp_td_base, 0.04);

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

    // Initial control mode from param (1=TDC,2=PID,3=FIXED,4=MANUAL; default TDC).
    // Replaces the former blocking "press 1/2/3/4" startup picker (Task-7 / D5)
    // so the node boots straight into the configured mode and roslaunch works
    // without an interactive stdin. Runtime switching via keys is unchanged.
    int control_mode_val;
    nh.param<int>("control_mode", control_mode_val, 1);
    mode_mgr.setMode(static_cast<ControlMode>(control_mode_val));

    attitude.setGains(M_td_base, Kp_td_base,
                      kp_roll_base, ki_roll_base, kd_roll_base,
                      kp_pitch_base, ki_pitch_base, kd_pitch_base,
                      gain_mult);

    // Start in the param-selected mode (control_mode); switch STDIN to raw
    // non-blocking mode for runtime keys. No longer blocks on a startup keypress.
    mode_mgr.initInteractive();
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
    dr_server.setCallback(boost::bind(&reconfigureCallback, _1, _2,
                                      &mode_mgr, &attitude, &ik, &imu_proc));

    // Subscribers
    ros::Subscriber imu_sub     = nh.subscribe("/hero_agent/sensors", 50,
                                               &ImuProcessor::onImu, &imu_proc);
    ros::Subscriber current_sub = nh.subscribe("/joint_currents", 10,
                                               &JointCurrentMonitor::onJointCurrents, &current_mon);

    // Publishers
    status_pub.advertise(nh);

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

        // Publish joint angles (mapTo2Pi applied inside StatusPublisher).
        status_pub.publishJointAngles(theta1, theta2);

        // Publish /albc_status — 11 fields in the contracted order/units.
        status_pub.publishStatus(
            attitude.targetRoll(), current_roll,
            attitude.targetPitch(), current_pitch,
            attitude.targetX(), attitude.targetY(),
            current_x, current_y,
            attitude.angularVelRoll(), attitude.angularVelPitch(), attitude.angularVelYaw());

        // Dashboard (~4 Hz)
        if (++dashboard_counter >= dashboard_interval) {
            dashboard_counter = 0;
            Dashboard::render(mode_mgr, attitude,
                              theta1, theta2, current_x, current_y, target_length,
                              current_roll, current_pitch,
                              current_mon.joint1(), current_mon.joint2());
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    mode_mgr.closeKeyboard();
    return 0;
}
