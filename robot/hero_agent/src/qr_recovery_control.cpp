#include "hero_agent/hero_agent_types.h"
#include <map>

// ==============================
// QR Recovery Mode Configuration Table
// ==============================

static std::map<int, RecoveryModeConfig> recovery_configs;

void initRecoveryConfigs()
{
    // mode -1: experiment hold (far, no yaw correction, no transition)
    recovery_configs[-1] = { 0.00f, 0.10f, 1.3f,
        param_saturation_default, false, false, 0, false };

    // mode -2: experiment hold (close, no yaw correction, no transition)
    recovery_configs[-2] = { -0.00f, 0.10f, 0.6f,
        param_saturation_default, false, false, 0, false };

    // mode 1: initial approach (with yaw, reset+calibrate on converge)
    recovery_configs[1] = { 0.00f, 0.00f, 1.0f,
        param_saturation_recovery, true, false, 0, true };

    // mode 2: close approach -> transitions to mode 3
    recovery_configs[2] = { 0.00f, 0.04f, 0.6f,
        param_saturation_recovery, true, true, 3, false };

    // mode 3: final approach -> transitions to mode 4
    recovery_configs[3] = { 0.00f, 0.04f, 0.3f,
        param_saturation_recovery, true, true, 4, false };

    // mode 4 handled separately (sends winch command and resets)
}

// ==============================
// Parameterized QR recovery step
// ==============================

static void executeQrRecoveryStep(const RecoveryModeConfig& cfg)
{
    int flag_next = 0;

    qr.x_target = cfg.x_target;
    qr.y_target = cfg.y_target;
    qr.z_target = cfg.z_target;

    // Compute errors
    qr.x_error = qr.x_target - qr.x;
    qr.y_error = qr.y_target - qr.y;
    qr.z_error = qr.z_target - qr.z;
    qr.yaw_error = qr.yaw_target - qr.yaw;

    // Integral terms
    qr.x_error_sum += qr.x_error;
    qr.z_error_sum += qr.z_error;

    // Derivative terms (sample rate = 30 Hz)
    qr.x_error_d   = (qr.x_error   - qr.x_error_pre)   * 30;
    qr.y_error_d   = (qr.y_error   - qr.y_error_pre)   * 30;
    qr.z_error_d   = (qr.z_error   - qr.z_error_pre)   * 30;
    qr.yaw_error_d = (qr.yaw_error - qr.yaw_error_pre) * 30;

    // Acceleration terms
    qr.x_ax = (qr.x_error_d - qr.x_error_d_pre) * 30;
    qr.z_az = (qr.z_error_d - qr.z_error_d_pre) * 30;

    // [BUG FIX C1] Position-level PD output: use assignment (=) for target_z
    // Previously: target_z += ... caused infinite accumulation
    target.y = -(qr_gains.Kp * qr.x_error + qr_gains.Kd * qr.x_error_d);
    target.z =  (2 * qr_gains.Kp * qr.y_error + qr_gains.Kd * (qr.y_error_d / 30.0));  // [BUG FIX H1] explicit parentheses
    target.x = -(qr_gains.Kp * qr.z_error + qr_gains.Kd * qr.z_error_d);

    // [BUG FIX C2] Accumulate target for close-approach modes
    // Previously: PD terms were applied twice (once above, once in accumulate block)
    // Now: accumulate mode uses += instead of = for target, applied once
    if (cfg.accumulate_target) {
        target.y += -(qr_gains.Kp * qr.x_error + qr_gains.Kd * qr.x_error_d);
        target.z +=  (2 * qr_gains.Kp * qr.y_error + qr_gains.Kd * (qr.y_error_d / 30.0));
        target.x += -(qr_gains.Kp * qr.z_error + qr_gains.Kd * qr.z_error_d);
    }

    // Thrust computation (PID or TDC)
    if (ctrl.qr_tdc == 0) {
        thrust.Tx = -(qr_gains.qr_Kp * qr.z_error + qr_gains.qr_Kd * qr.z_error_d + qr_gains.qr_Ki * qr.z_error_sum);
        thrust.Ty = -(qr_gains.qr_Kp * qr.x_error + qr_gains.qr_Kd * qr.x_error_d + qr_gains.qr_Ki * qr.x_error_sum);
    } else if (ctrl.qr_tdc == 1) {
        thrust.Tx -= qr_gains.Mb * (-qr.z_az_pre + qr_gains.KKv * qr.z_error_d + qr_gains.KKp * qr.z_error);
        thrust.Ty -= qr_gains.Mb * (-qr.x_ax_pre + qr_gains.KKv * qr.x_error_d + qr_gains.KKp * qr.x_error);
    }

    // Saturation
    const double sat = cfg.saturation;
    thrust.Tx = clamp(thrust.Tx, -sat, sat);
    thrust.Ty = clamp(thrust.Ty, -sat, sat);

    // Thrust allocation (X-configuration)
    cont_xy_msg.T0 = -thrust.Tx + thrust.Ty;
    cont_xy_msg.T1 = -thrust.Tx - thrust.Ty;
    cont_xy_msg.T2 =  thrust.Tx - thrust.Ty;
    cont_xy_msg.T3 =  thrust.Tx + thrust.Ty;
    cont_xy_msg.TARGET_DEPTH = 0;

    qr.count++;
    if (qr.count > param_qr_warmup_count) {
        pub_cont_xy.publish(cont_xy_msg);
        // [BUG FIX C3] Removed ros::spinOnce() from callback - causes re-entrant execution
    }

    // Store previous states
    // Sign convention: modes -1, -2 negate acceleration feedback
    if (ctrl.recovery < 0) {
        qr.x_ax_pre = -qr.x_ax;
        qr.z_az_pre = -qr.z_az;
    } else {
        qr.x_ax_pre = qr.x_ax;
        qr.z_az_pre = qr.z_az;
    }

    qr.x_error_pre   = qr.x_error;
    qr.y_error_pre   = qr.y_error;
    qr.z_error_pre   = qr.z_error;
    qr.yaw_error_pre = qr.yaw_error;
    qr.x_error_d_pre = qr.x_error_d;
    qr.z_error_d_pre = qr.z_error_d;

    // Yaw correction (only for modes that enable it)
    if (cfg.yaw_correction) {
        if (qr.yaw > 0.01) {
            command_msg.data = '6';
            pub_command.publish(command_msg);
        } else if (qr.yaw < -0.01) {
            command_msg.data = '5';
            pub_command.publish(command_msg);
        } else {
            flag_next = 1;
        }
    }

    // Convergence check
    float tol = static_cast<float>(param_convergence_tolerance);
    if (std::abs(qr.x_error) < tol && std::abs(qr.y_error) < tol && std::abs(qr.z_error) < tol) {
        qr.flag_count++;
        bool converge_check = cfg.yaw_correction ? (qr.flag_count > param_convergence_count && flag_next == 1)
                                                  : (qr.flag_count > param_convergence_count);
        if (converge_check) {
            if (cfg.reset_on_converge) {
                // Mode 1: reset targets and calibrate winch
                ctrl.recovery = 0;
                target.x = 0; target.y = 0; target.z = 0; target.yaw = 0;
                winch.calib_position = winch.current_position;
                winch.target_meter = 0;

                msg_target.command = 1;
                msg_target.TARGET_X = target.x;
                msg_target.TARGET_Y = target.y;
                msg_target.TARGET_Z = target.z;
                pub_target.publish(msg_target);
                // [BUG FIX C3] Removed ros::spinOnce() from callback

                winch.qr_based_calibration = 1;
                qr.flag_count = 0;
            } else if (cfg.next_mode != 0) {
                // Transition to next mode
                ctrl.recovery = cfg.next_mode;
                qr.flag_count = 0;
            }
        }
    }
}

// ==============================
// Mode 4: Winch deploy and reset
// ==============================

static void executeMode4()
{
    command_msg.data = 'w';
    pub_command.publish(command_msg);

    ctrl.recovery = 0;
    target.x = 0; target.y = 0; target.z = 0; target.yaw = 0;
    winch.calib_position = winch.current_position;
    winch.target_meter = 0;

    msg_target.command = 1;
    msg_target.TARGET_X = target.x;
    msg_target.TARGET_Y = target.y;
    msg_target.TARGET_Z = target.z;
    pub_target.publish(msg_target);
    // [BUG FIX C3] Removed ros::spinOnce() from callback

    winch.qr_based_calibration = 0;
    qr.flag_count = 0;
}

// ==============================
// QR message callback
// ==============================

void msgCallback_ip_qr(const ros_opencv_ipcam_qr::hero_ipcam_qr_msg::ConstPtr &msg)
{
    qr.x = msg->T_X;
    qr.y = msg->T_Y;
    qr.z = msg->T_Z;
    qr.yaw = msg->T_YAW;
    qr.valid = msg->T_valid;

    if (qr.valid != 1) return;

    // Outlier rejection
    float distance = std::sqrt(
        (qr.pre_x - qr.x) * (qr.pre_x - qr.x) +
        (qr.pre_y - qr.y) * (qr.pre_y - qr.y) +
        (qr.pre_z - qr.z) * (qr.pre_z - qr.z));

    if (distance > param_qr_distance_threshold && qr.count > param_qr_warmup_count) {
        qr.x = qr.pre_x;
        qr.y = qr.pre_y;
        qr.z = qr.pre_z;
    } else {
        qr.pre_x = qr.x;
        qr.pre_y = qr.y;
        qr.pre_z = qr.z;
    }

    // Publish QR result for monitoring
    agent_qr_result_msg.TARGET_X = qr.x_target;
    agent_qr_result_msg.TARGET_Y = qr.y_target;
    agent_qr_result_msg.TARGET_Z = qr.z_target;
    agent_qr_result_msg.X = qr.x;
    agent_qr_result_msg.Y = qr.y;
    agent_qr_result_msg.Z = qr.z;
    pub_agent_qr_result.publish(agent_qr_result_msg);

    // Execute recovery step by mode
    if (ctrl.recovery == 4) {
        executeMode4();
    } else {
        auto it = recovery_configs.find(ctrl.recovery);
        if (it != recovery_configs.end()) {
            executeQrRecoveryStep(it->second);
        }
    }
}

// ==============================
// Controller parameter callback
// ==============================

void msgCallback_cont_para(const hero_msgs::hero_agent_cont_para::ConstPtr &msg)
{
    ctrl.qr_tdc   = msg->control_T;
    qr_gains.qr_Kp = msg->Kp;
    qr_gains.qr_Ki = msg->Ki;
    qr_gains.qr_Kd = msg->Kd;
    qr_gains.Mb    = msg->Mb;
    qr_gains.KKp   = msg->KKp;
    qr_gains.KKv   = msg->KKv;
}
