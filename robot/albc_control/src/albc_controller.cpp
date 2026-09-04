#include <ros/ros.h>
#include <cmath>
#include <cstdio>
#include "hero_msgs/hero_agent_sensor.h"
#include <ros/topic.h>                // waitForMessage (measured-angle seed)
#include <sensor_msgs/JointState.h>   // /albc/joint_states (measured arm state)
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

// albc_controller: thin assembly of ModeManager / AttitudeController /
// InverseKinematics / ImuProcessor / JointCurrentMonitor / StatusPublisher /
// Dashboard (albc_control/include). Control math lives in the ROS-free oracle
// headers (control_law.h, feedback_filters.h, dls_ik.h, imu_rotation.h) and is
// pinned by tests/characterization. g_mode_mgr exists only for atexit().
static ModeManager* g_mode_mgr = nullptr;

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
    // -78.0 measured 2026-08-11 (was 45.0, a 122.9 deg frame error). Rationale
    // and the four-tilt solve are in config/albc_controller.yaml next to the value.
    nh.param<double>("imu_yaw_offset", imu_yaw_offset_deg, 102.0);
    imu_proc.setOffset(DEG2RAD(imu_yaw_offset_deg));

    nh.param<double>("initial_theta1_deg", initial_theta1_deg, 90.0);
    nh.param<double>("initial_theta2_deg", initial_theta2_deg, 90.0);

    // Measured-angle seeding (2026-08-24). initial_theta{1,2}_deg above are now
    // only the FALLBACK used when /albc/joint_states never arrives.
    double seed_timeout_s, min_ee_radius_m;
    nh.param<double>("seed_timeout_s", seed_timeout_s, 3.0);
    nh.param<double>("min_ee_radius_m", min_ee_radius_m, 0.05);
    bool allow_yaml_seed;
    nh.param<bool>("allow_yaml_seed", allow_yaml_seed, false);

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

    // ---- Seed the joint state from the MEASURED arm -----------------------
    // Until 2026-08-24 this seeded theta1/theta2 from the yaml constants and
    // published that on the FIRST tick, without ever reading where the arm is.
    // The deployed default (90 deg) against the parked pose (~180 deg) is a
    // 90 deg J2 jump into the "large force x max lever" pose that broke arm1 on
    // 2026-08-22 -- and it fires before any runtime key can be pressed, so no
    // operator action avoided it and no control_mode was exempt.
    //
    // joint_angle_command already publishes the MEASURED angle on
    // /albc/joint_states (position[0]=joint1, position[1]=joint2, rad). Seeding
    // from it makes the first command equal the present position, so the
    // first-tick delta is zero BY CONSTRUCTION rather than by a matched constant.
    //
    // mapTo2Pi() on the way in is deliberate, not cosmetic. That topic is
    // raw-CUMULATIVE (joint_angle_command.cpp: js.position = meas.abs_meas) and
    // J1 has been tens of rad from zero in the field, while this node is in the
    // WRAPPED convention: everything downstream touches theta only through
    // sin/cos (forwardKinematics, calculateJacobian) and StatusPublisher applies
    // the same mapTo2Pi on the way out. Wrapping here puts the seed in the
    // representation this node actually publishes; the math is unchanged either
    // way, but the logged and reconfigured values stay readable.
    //
    // Wrapping here does NOT lose the winding count, and the division of labour
    // is worth stating so nobody re-opens it: the turn counter, the 6pi abort and
    // the cable guard all live in the DRIVER (joint_unwrap.h, 6b85836) and watch
    // the commanded AND measured angle. updateJoint() re-accumulates a wrapped
    // command through unwrapNearest(), so a wrapped command is not a reset.
    //
    // A TIMEOUT IS A MEASURED FAILURE, NOT A NEUTRAL DEFAULT. The 2026-08-24
    // handover records /albc/joint_states going silent on a real Dynamixel bus
    // read failure, so a constant fallback would auto-degrade a dead sensor path
    // into exactly the blind jump this change exists to prevent. Timeout
    // therefore REFUSES unless ~allow_yaml_seed is set, which is the bench opt-in
    // that start_tdc_dryrun.sh passes (it runs with no driver by design).
    double theta1 = DEG2RAD(initial_theta1_deg);
    double theta2 = DEG2RAD(initial_theta2_deg);
    {
        sensor_msgs::JointStateConstPtr js =
            ros::topic::waitForMessage<sensor_msgs::JointState>(
                "/albc/joint_states", nh, ros::Duration(seed_timeout_s));
        // A NaN would pass every ordered comparison downstream (NaN < x is false,
        // so the singularity guard would wave it through), so an unusable message
        // is rejected HERE rather than after the kinematics.
        const bool usable = js && js->position.size() >= 2 &&
                            std::isfinite(js->position[0]) &&
                            std::isfinite(js->position[1]);
        if (usable) {
            theta1 = mapTo2Pi(js->position[0]);
            theta2 = mapTo2Pi(js->position[1]);
            // MANUAL (mode 4) steps from its own baseline, which the yaml leaves
            // at 90/90. Fixing only the TDC seed would leave that path intact:
            // switching to mode 4 at runtime would still command the pose that
            // broke arm1 on 2026-08-22. The manual baseline follows the
            // measurement for the same reason the TDC seed does.
            manual_theta1_deg = RAD2DEG(theta1);
            manual_theta2_deg = RAD2DEG(theta2);
            forwardKinematics(theta1, theta2, manual_x, manual_y);
            mode_mgr.setManualState(manual_theta1_deg, manual_theta2_deg,
                                    manual_x, manual_y);
            ROS_INFO("Seeded from MEASURED arm: theta1 = %.2f deg, theta2 = %.2f deg "
                     "(MANUAL baseline follows the measurement)",
                     RAD2DEG(theta1), RAD2DEG(theta2));
        } else if (allow_yaml_seed) {
            ROS_WARN("No usable /albc/joint_states within %.1f s -- ~allow_yaml_seed "
                     "is set, so seeding from yaml (theta1 = %.1f, theta2 = %.1f deg). "
                     "The FIRST command will move the arm from wherever it is to that "
                     "pose. This is the BENCH path; it must never be set on the robot.",
                     seed_timeout_s, initial_theta1_deg, initial_theta2_deg);
        } else {
            ROS_FATAL("No usable /albc/joint_states within %.1f s -- the arm state is "
                      "UNKNOWN. Seeding from the yaml constants would command "
                      "theta2 = %.1f deg blind, which is the 2026-08-22 arm1 break. "
                      "Start the joint driver (run-joint) and check the Dynamixel bus "
                      "before retrying. Bench sweeps only: _allow_yaml_seed:=true.",
                      seed_timeout_s, initial_theta2_deg);
            return 1;
        }
    }

    double current_x, current_y;
    forwardKinematics(theta1, theta2, current_x, current_y);

    // ---- Refuse to start inside the folded singularity --------------------
    // L1 == L2 (0.233 m), so the EE radius is 2*L1*|cos(theta2/2)| and collapses
    // to 0 at theta2 = pi -- which IS the parked pose. There the Jacobian loses
    // the radial direction and the DLS damping lambda = lambda_base *
    // (1 - sqrt(|sin theta2|)) is simultaneously at its MAXIMUM, so the step is
    // pure damping. Measured on the bench 2026-08-24: seed 180 deg held the
    // command bit-identical for 10 s while a 1.19 deg roll error stood. That is
    // a deadlock, not a divergence -- nothing errors and nothing moves, so a wet
    // test is spent discovering it.
    //
    // The 0.05 m default sits just inside the damping-dominance point: equating
    // the smaller singular value (~L2*|sin theta2|) with lambda gives
    // |sin theta2| ~ 0.29, i.e. theta2 ~ 163 deg and a radius of ~0.068 m. The
    // two bench points bracket that -- 0.0161 m frozen, 0.1252 m responding.
    // Set the param to 0 to disable the guard.
    const double ee_radius = std::sqrt(current_x * current_x + current_y * current_y);
    // !(r >= min), NOT (r < min). The two differ on NaN: `NaN < min` is false, so
    // the naive form waves a NaN radius straight through the guard it exists to
    // enforce. The seed is already checked for finiteness above; this keeps the
    // property when the yaml path or a future caller supplies the angles.
    if (!(ee_radius >= min_ee_radius_m)) {
        // How far the operator has to move J2, computed from the live threshold
        // rather than hard-coded, so a retuned param cannot make the message lie.
        const double clear_deg =
            (min_ee_radius_m < 2.0 * L1)
                ? 180.0 - 2.0 * std::acos(min_ee_radius_m / (2.0 * L1)) * 180.0 / M_PI
                : 180.0;
        ROS_FATAL("Arm is inside the folded singularity: theta2 = %.2f deg gives an "
                  "EE radius of %.4f m (< %.4f m). The IK cannot move from here and "
                  "the controller would hold a frozen command -- it would not error, "
                  "it would simply never move. Move J2 more than %.1f deg away from "
                  "180 deg first; run-joint is the tool for that (do NOT run it "
                  "alongside launch-albc, which starts its own driver). Then restart.",
                  RAD2DEG(theta2), ee_radius, min_ee_radius_m, clear_deg);
        return 1;
    }
    ROS_INFO("EE seed: x = %.4f, y = %.4f (radius %.4f m)",
             current_x, current_y, ee_radius);

    attitude.setTarget(current_x, current_y);

    // Start in the param-selected mode (control_mode); switch STDIN to raw
    // non-blocking mode for runtime keys. No longer blocks on a startup keypress.
    // AFTER the seed/guard above so a refusal never leaves the terminal raw.
    mode_mgr.initInteractive();
    atexit(restoreKeyboardAtExit);  // restore terminal on any exit path

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
