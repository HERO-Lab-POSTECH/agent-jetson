#include "hero_agent/agent_command_types.h"
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

    // mode 2: close approach → transitions to mode 3
    recovery_configs[2] = { 0.00f, 0.04f, 0.6f,
        param_saturation_recovery, true, true, 3, false };

    // mode 3: final approach → transitions to mode 4
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

    qr_x_target = cfg.x_target;
    qr_y_target = cfg.y_target;
    qr_z_target = cfg.z_target;

    // Compute errors
    qr_x_error = qr_x_target - qr_x;
    qr_y_error = qr_y_target - qr_y;
    qr_z_error = qr_z_target - qr_z;
    qr_yaw_error = qr_yaw_target - qr_yaw;

    // Integral terms
    qr_x_error_sum += qr_x_error;
    qr_z_error_sum += qr_z_error;

    // Derivative terms (sample rate = 30 Hz)
    qr_x_error_d   = (qr_x_error   - qr_x_error_pre)   * 30;
    qr_y_error_d   = (qr_y_error   - qr_y_error_pre)   * 30;
    qr_z_error_d   = (qr_z_error   - qr_z_error_pre)   * 30;
    qr_yaw_error_d = (qr_yaw_error - qr_yaw_error_pre) * 30;

    // Acceleration terms
    qr_x_ax = (qr_x_error_d - qr_x_error_d_pre) * 30;
    qr_z_az = (qr_z_error_d - qr_z_error_d_pre) * 30;

    // Position-level PD output
    target_y = -(Kp * qr_x_error + Kd * qr_x_error_d);
    target_z += 2 * Kp * qr_y_error + Kd * qr_y_error_d / 30.0;
    target_x = -(Kp * qr_z_error + Kd * qr_z_error_d);

    // Thrust computation (PID or TDC)
    if (cont_qr_tdc == 0) {
        Tx = -(qr_Kp * qr_z_error + qr_Kd * qr_z_error_d + qr_Ki * qr_z_error_sum);
        Ty = -(qr_Kp * qr_x_error + qr_Kd * qr_x_error_d + qr_Ki * qr_x_error_sum);
    } else if (cont_qr_tdc == 1) {
        Tx -= qr_Mb * (-qr_z_az_pre + qr_KKv * qr_z_error_d + qr_KKp * qr_z_error);
        Ty -= qr_Mb * (-qr_x_ax_pre + qr_KKv * qr_x_error_d + qr_KKp * qr_x_error);
    }

    // Saturation
    const double sat = cfg.saturation;
    Tx = clamp(Tx, -sat, sat);
    Ty = clamp(Ty, -sat, sat);

    // Thrust allocation (X-configuration)
    cont_xy_msg.T0 = -Tx + Ty;
    cont_xy_msg.T1 = -Tx - Ty;
    cont_xy_msg.T2 =  Tx - Ty;
    cont_xy_msg.T3 =  Tx + Ty;
    cont_xy_msg.TARGET_DEPTH = 0;

    qr_count++;
    if (qr_count > param_qr_warmup_count) {
        pub_cont_xy.publish(cont_xy_msg);
        ros::spinOnce();
    }

    // Store previous states
    // Sign convention: modes -1, -2 negate acceleration feedback
    if (cont_recovery < 0) {
        qr_x_ax_pre = -qr_x_ax;
        qr_z_az_pre = -qr_z_az;
    } else {
        qr_x_ax_pre = qr_x_ax;
        qr_z_az_pre = qr_z_az;
    }

    qr_x_error_pre   = qr_x_error;
    qr_y_error_pre   = qr_y_error;
    qr_z_error_pre   = qr_z_error;
    qr_yaw_error_pre = qr_yaw_error;
    qr_x_error_d_pre = qr_x_error_d;
    qr_z_error_d_pre = qr_z_error_d;

    // Yaw correction (only for modes that enable it)
    if (cfg.yaw_correction) {
        if (qr_yaw > 0.01) {
            command_msg.data = '6';
            pub_command.publish(command_msg);
        } else if (qr_yaw < -0.01) {
            command_msg.data = '5';
            pub_command.publish(command_msg);
        } else {
            flag_next = 1;
        }
    }

    // Accumulate target for close-approach modes
    if (cfg.accumulate_target) {
        target_y -= Kp * qr_x_error + Kd * qr_x_error_d;
        target_z += 2 * Kp * qr_y_error + Kd * qr_y_error_d;
        target_x -= Kp * qr_z_error + Kd * qr_z_error_d;
    }

    // Convergence check
    float tol = static_cast<float>(param_convergence_tolerance);
    if (std::abs(qr_x_error) < tol && std::abs(qr_y_error) < tol && std::abs(qr_z_error) < tol) {
        flag_count++;
        bool converge_check = cfg.yaw_correction ? (flag_count > param_convergence_count && flag_next == 1)
                                                  : (flag_count > param_convergence_count);
        if (converge_check) {
            if (cfg.reset_on_converge) {
                // Mode 1: reset targets and calibrate winch
                cont_recovery = 0;
                target_x = 0; target_y = 0; target_z = 0; target_yaw = 0;
                calib_position = current_position;
                target_meter = 0;

                msg_target.command = 1;
                msg_target.TARGET_X = target_x;
                msg_target.TARGET_Y = target_y;
                msg_target.TARGET_Z = target_z;
                pub_target.publish(msg_target);
                ros::spinOnce();

                qr_based_calibration = 1;
                flag_count = 0;
            } else if (cfg.next_mode != 0) {
                // Transition to next mode
                cont_recovery = cfg.next_mode;
                flag_count = 0;
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

    cont_recovery = 0;
    target_x = 0; target_y = 0; target_z = 0; target_yaw = 0;
    calib_position = current_position;
    target_meter = 0;

    msg_target.command = 1;
    msg_target.TARGET_X = target_x;
    msg_target.TARGET_Y = target_y;
    msg_target.TARGET_Z = target_z;
    pub_target.publish(msg_target);
    ros::spinOnce();

    qr_based_calibration = 0;
    flag_count = 0;
}

// ==============================
// QR message callback
// ==============================

void msgCallback_ip_qr(const ros_opencv_ipcam_qr::hero_ipcam_qr_msg::ConstPtr &msg)
{
    qr_x = msg->T_X;
    qr_y = msg->T_Y;
    qr_z = msg->T_Z;
    qr_yaw = msg->T_YAW;
    qr_valid = msg->T_valid;

    if (qr_valid != 1) return;

    // Outlier rejection
    float distance = std::sqrt(
        (pre_qr_x - qr_x) * (pre_qr_x - qr_x) +
        (pre_qr_y - qr_y) * (pre_qr_y - qr_y) +
        (pre_qr_z - qr_z) * (pre_qr_z - qr_z));

    if (distance > param_qr_distance_threshold && qr_count > param_qr_warmup_count) {
        qr_x = pre_qr_x;
        qr_y = pre_qr_y;
        qr_z = pre_qr_z;
    } else {
        pre_qr_x = qr_x;
        pre_qr_y = qr_y;
        pre_qr_z = qr_z;
    }

    // Publish QR result for monitoring
    agent_qr_result_msg.TARGET_X = qr_x_target;
    agent_qr_result_msg.TARGET_Y = qr_y_target;
    agent_qr_result_msg.TARGET_Z = qr_z_target;
    agent_qr_result_msg.X = qr_x;
    agent_qr_result_msg.Y = qr_y;
    agent_qr_result_msg.Z = qr_z;
    pub_agent_qr_result.publish(agent_qr_result_msg);

    // Execute recovery step by mode
    if (cont_recovery == 4) {
        executeMode4();
    } else {
        auto it = recovery_configs.find(cont_recovery);
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
    cont_qr_tdc = msg->control_T;
    qr_Kp  = msg->Kp;
    qr_Ki  = msg->Ki;
    qr_Kd  = msg->Kd;
    qr_Mb  = msg->Mb;
    qr_KKp = msg->KKp;
    qr_KKv = msg->KKv;
}
