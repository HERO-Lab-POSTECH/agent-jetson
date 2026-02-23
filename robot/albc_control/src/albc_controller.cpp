#include <ros/ros.h>
#include <cmath>
#include <cstdio>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include "hero_msgs/hero_agent_sensor.h"
#include "albc_control/albc_kinematics.h"

using namespace albc;

// ==============================
// Control Mode
// ==============================

enum class ControlMode : int { TDC = 1, PID = 2, FIXED = 3 };

static const char* controlModeName(ControlMode m) {
    switch (m) {
        case ControlMode::TDC:   return "TDC";
        case ControlMode::PID:   return "PID";
        case ControlMode::FIXED: return "FIXED";
        default:                 return "???";
    }
}

// ==============================
// Constants
// ==============================

static constexpr double INTEGRAL_MAX        = 100.0;
static constexpr double COS_EPSILON         = 1e-6;
static constexpr double DET_EPSILON         = 1e-6;
static constexpr double UPDATE_ANGLE_EPSILON = 1e-4;
static constexpr double IK_DELTA_THRESHOLD  = 0.01;
static constexpr int    IK_REDUCED_ITERATIONS = 500;

// ==============================
// State Structures
// ==============================

struct ControlGains {
    // Base values (reference for gain_mult)
    double M_td_base, Kp_td_base, Kd_td_base;
    double kp_roll_base, ki_roll_base, kd_roll_base;
    double kp_pitch_base, ki_pitch_base, kd_pitch_base;

    // Active values (= base * gain_mult)
    double M_td, Kp_td, Kd_td;
    double kp_roll, ki_roll, kd_roll;
    double kp_pitch, ki_pitch, kd_pitch;

    double gain_mult;

    void applyMultiplier() {
        M_td  = M_td_base  * gain_mult;
        Kp_td = Kp_td_base * gain_mult;
        Kd_td = Kd_td_base * gain_mult;
        kp_roll  = kp_roll_base  * gain_mult;
        ki_roll  = ki_roll_base  * gain_mult;
        kd_roll  = kd_roll_base  * gain_mult;
        kp_pitch = kp_pitch_base * gain_mult;
        ki_pitch = ki_pitch_base * gain_mult;
        kd_pitch = kd_pitch_base * gain_mult;
    }
};

struct ControlState {
    double current_roll, current_pitch;
    double error_roll, error_pitch;
    double integral_roll, integral_pitch;
    double prev_error_roll, prev_error_pitch;
    double target_roll, target_pitch;
    double target_x, target_y;

    // TODO: w_roll and w_pitch are always 0 because no angular velocity topic is subscribed.
    // This means acceleration feedforward (a_roll, a_pitch) in TDC mode is effectively disabled.
    // To enable it, subscribe to an angular velocity topic and update w_roll/w_pitch in the callback.
    double w_roll, w_pitch;
    double prev_w_roll, prev_w_pitch;
    double a_roll, a_pitch;
};

struct IKConfig {
    double learning_rate;
    double lambda_base;
    int num_iterations;
};

// ==============================
// Globals
// ==============================

static ControlMode control_mode = ControlMode::TDC;
static ControlGains gains = {};
static ControlState state = {};
static IKConfig ik_cfg = {};
static float joint_current1_mA = 0.0f;
static float joint_current2_mA = 0.0f;

// ==============================
// Inverse Kinematics (Damped Least Squares)
// ==============================

void updateJointAngles(double& theta1, double& theta2, double delta_x, double delta_y) {
    double lambda = ik_cfg.lambda_base * (
        1.0 - std::sqrt(std::abs(L1 * L2 * std::sin(theta2))) /
              std::sqrt(std::abs(L1 * L2))
    );

    double j11, j12, j21, j22;
    calculateJacobian(theta1, theta2, j11, j12, j21, j22);

    // J^T * J + lambda^2 * I
    double jtj_11 = j11 * j11 + j21 * j21 + lambda * lambda;
    double jtj_12 = j11 * j12 + j21 * j22;
    double jtj_22 = j12 * j12 + j22 * j22 + lambda * lambda;

    double det = jtj_11 * jtj_22 - jtj_12 * jtj_12;
    if (std::abs(det) < DET_EPSILON) {
        ROS_WARN_THROTTLE(2.0, "Jacobian near-singular (det=%.6f)", det);
        det = (det >= 0.0) ? DET_EPSILON : -DET_EPSILON;
    }

    // Inverse of (J^T * J + lambda^2 * I)
    double inv11 =  jtj_22 / det;
    double inv12 = -jtj_12 / det;
    double inv22 =  jtj_11 / det;

    // Pseudo-inverse: (J^T J + lambda^2 I)^-1 * J^T
    double j11_inv = inv11 * j11 + inv12 * j21;
    double j12_inv = inv11 * j12 + inv12 * j22;
    double j21_inv = inv12 * j11 + inv22 * j21;
    double j22_inv = inv12 * j12 + inv22 * j22;

    double update_angle1 = ik_cfg.learning_rate * (j11_inv * delta_x + j12_inv * delta_y);
    double update_angle2 = ik_cfg.learning_rate * (j21_inv * delta_x + j22_inv * delta_y);

    // Clamp large updates (near singularity)
    if (std::abs(update_angle1) > M_PI) {
        update_angle1 = (update_angle1 > 0) ? UPDATE_ANGLE_EPSILON : -UPDATE_ANGLE_EPSILON;
    }
    if (std::abs(update_angle2) > M_PI) {
        update_angle2 = (update_angle2 > 0) ? UPDATE_ANGLE_EPSILON : -UPDATE_ANGLE_EPSILON;
    }

    theta1 += update_angle1;
    theta2 += update_angle2;
}

// ==============================
// Callbacks
// ==============================

void imuCallback(const hero_msgs::hero_agent_sensor::ConstPtr& msg) {
    state.current_roll  = msg->ROLL;
    state.current_pitch = -(msg->PITCH);
}

void targetRollCallback(const std_msgs::Float64::ConstPtr& msg) {
    state.target_roll = DEG2RAD(msg->data);
}

void targetPitchCallback(const std_msgs::Float64::ConstPtr& msg) {
    state.target_pitch = DEG2RAD(msg->data);
}

void targetGainMultCallback(const std_msgs::Float64::ConstPtr& msg) {
    double new_mult = msg->data;
    if (new_mult <= 0.0 || std::isnan(new_mult)) {
        ROS_WARN("Invalid gain multiplier: %.4f (keeping %.2f)", new_mult, gains.gain_mult);
        return;
    }
    gains.gain_mult = new_mult;
    gains.applyMultiplier();
}

void contVerCallback(const std_msgs::Int32::ConstPtr& msg) {
    int v = msg->data;
    if (v >= 1 && v <= 3)
        control_mode = static_cast<ControlMode>(v);
}

void jointCurrentsCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    if (msg->data.size() >= 2) {
        joint_current1_mA = msg->data[0];
        joint_current2_mA = msg->data[1];
    }
}

// ==============================
// Control Law
// ==============================

void computeControlOutput(double dt, double derivative_roll, double derivative_pitch) {
    switch (control_mode) {
    case ControlMode::TDC: {
        state.a_roll  = (state.w_roll  - state.prev_w_roll)  / dt;
        state.a_pitch = (state.w_pitch - state.prev_w_pitch) / dt;

        double cos_product = cos(state.current_roll) * cos(state.current_pitch);
        double denominator = std::abs(Fb * cos_product);

        if (denominator < COS_EPSILON) {
            ROS_WARN_THROTTLE(2.0, "Denominator too small (Fb*cos=%.6f), capped", denominator);
        }

        double common_factor = Fb / std::max(denominator, COS_EPSILON);

        state.target_y -= common_factor *
            (-1.0 * (gains.M_td * (-state.a_roll + gains.Kd_td * derivative_roll + gains.Kp_td * state.error_roll)));
        state.target_x -= common_factor *
            (gains.M_td * (-state.a_pitch + gains.Kd_td * derivative_pitch + gains.Kp_td * state.error_pitch));
        break;
    }
    case ControlMode::PID:
        state.target_y = gains.kp_roll  * state.error_roll  + gains.ki_roll  * state.integral_roll  + gains.kd_roll  * derivative_roll;
        state.target_x = -1.0 * (gains.kp_pitch * state.error_pitch + gains.ki_pitch * state.integral_pitch + gains.kd_pitch * derivative_pitch);
        break;

    case ControlMode::FIXED:
        state.target_y = 0.01;
        state.target_x = 0.01;
        break;
    }
}

// ==============================
// Dashboard
// ==============================

void printDashboard(double theta1, double theta2,
                    double current_x, double current_y,
                    double target_length) {
    printf("\033[2J\033[H");
    printf("═══════════════════════════════════════════════════\n");
    printf("            ALBC Controller [%s]\n", controlModeName(control_mode));
    printf("═══════════════════════════════════════════════════\n");
    printf(" Roll  %+7.2f / %+7.2f deg  (err %+.2f)\n",
           RAD2DEG(state.current_roll), RAD2DEG(state.target_roll), RAD2DEG(state.error_roll));
    printf(" Pitch %+7.2f / %+7.2f deg  (err %+.2f)\n",
           RAD2DEG(state.current_pitch), RAD2DEG(state.target_pitch), RAD2DEG(state.error_pitch));
    printf("───────────────────────────────────────────────────\n");
    printf(" Target (%+.4f, %+.4f)   FK (%+.4f, %+.4f)\n",
           state.target_x, state.target_y, current_x, current_y);
    printf(" Joints J1=%.1f  J2=%.1f deg   Len=%.4f/%.4f\n",
           RAD2DEG(theta1), RAD2DEG(theta2), target_length, SAFE_ARM_LENGTH);
    printf("───────────────────────────────────────────────────\n");
    printf(" Gains  mult=%.2f  M=%.4f  Kp=%.3f  Kd=%.1f\n",
           gains.gain_mult, gains.M_td, gains.Kp_td, gains.Kd_td);
    printf(" Motor  J1=%+.0f mA  J2=%+.0f mA\n",
           joint_current1_mA, joint_current2_mA);
    printf("═══════════════════════════════════════════════════\n");
    fflush(stdout);
}

// ==============================
// Main
// ==============================

int main(int argc, char **argv) {
    ros::init(argc, argv, "agent_active_roll_pitch_controller", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");

    // Load parameters
    int loop_rate_hz;
    int mode_int;
    double initial_theta1_deg, initial_theta2_deg;

    nh.param<int>("loop_rate_hz", loop_rate_hz, 50);
    nh.param<int>("control_mode", mode_int, 1);
    control_mode = static_cast<ControlMode>(mode_int);
    nh.param<double>("gain_mult", gains.gain_mult, 1.5);

    nh.param<double>("td_control/M_td", gains.M_td_base, 0.0085);
    nh.param<double>("td_control/Kp_td", gains.Kp_td_base, 0.085);
    nh.param<double>("td_control/Kd_td", gains.Kd_td_base, 1.1);

    nh.param<double>("pid_roll/kp", gains.kp_roll_base, 0.05);
    nh.param<double>("pid_roll/ki", gains.ki_roll_base, 0.001);
    nh.param<double>("pid_roll/kd", gains.kd_roll_base, 0.0005);

    nh.param<double>("pid_pitch/kp", gains.kp_pitch_base, 0.05);
    nh.param<double>("pid_pitch/ki", gains.ki_pitch_base, 0.001);
    nh.param<double>("pid_pitch/kd", gains.kd_pitch_base, 0.0005);

    nh.param<int>("ik/num_iterations", ik_cfg.num_iterations, 3000);
    nh.param<double>("ik/learning_rate", ik_cfg.learning_rate, 0.02);
    nh.param<double>("ik/lambda_base", ik_cfg.lambda_base, 0.15);

    nh.param<double>("initial_theta1_deg", initial_theta1_deg, 45.0);
    nh.param<double>("initial_theta2_deg", initial_theta2_deg, 45.0);

    gains.applyMultiplier();

    // Initial joint angles and end-effector position
    double theta1 = DEG2RAD(initial_theta1_deg);
    double theta2 = DEG2RAD(initial_theta2_deg);

    double current_x, current_y;
    forwardKinematics(theta1, theta2, current_x, current_y);

    state.target_x = current_x;
    state.target_y = current_y;

    // Subscribers
    ros::Subscriber imu_sub          = nh.subscribe("/hero_agent/sensors", 50, imuCallback);
    ros::Subscriber target_roll_sub  = nh.subscribe("/target_roll", 10, targetRollCallback);
    ros::Subscriber target_pitch_sub = nh.subscribe("/target_pitch", 10, targetPitchCallback);
    ros::Subscriber gain_mult_sub    = nh.subscribe("/target_gain_mult", 10, targetGainMultCallback);
    ros::Subscriber cont_ver_sub     = nh.subscribe("/cont_ver", 10, contVerCallback);
    ros::Subscriber current_sub      = nh.subscribe("/joint_currents", 10, jointCurrentsCallback);

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

    while (ros::ok()) {
        double dt = 1.0 / static_cast<double>(loop_rate_hz);

        // Error computation
        state.error_roll  = state.target_roll  - state.current_roll;
        state.error_pitch = state.target_pitch - state.current_pitch;

        // Integral with anti-windup
        state.integral_roll  += state.error_roll;
        state.integral_pitch += state.error_pitch;
        state.integral_roll  = std::max(-INTEGRAL_MAX, std::min(INTEGRAL_MAX, state.integral_roll));
        state.integral_pitch = std::max(-INTEGRAL_MAX, std::min(INTEGRAL_MAX, state.integral_pitch));

        // Derivative
        double derivative_roll  = (state.error_roll  - state.prev_error_roll)  / dt;
        double derivative_pitch = (state.error_pitch - state.prev_error_pitch) / dt;

        // Control law
        computeControlOutput(dt, derivative_roll, derivative_pitch);

        // Store previous states
        state.prev_error_roll  = state.error_roll;
        state.prev_error_pitch = state.error_pitch;
        state.prev_w_roll      = state.w_roll;
        state.prev_w_pitch     = state.w_pitch;

        // Radial workspace saturation
        double target_length = std::sqrt(state.target_x * state.target_x + state.target_y * state.target_y);
        if (target_length >= SAFE_ARM_LENGTH) {
            state.target_x = state.target_x / target_length * SAFE_ARM_LENGTH;
            state.target_y = state.target_y / target_length * SAFE_ARM_LENGTH;
        }

        // Inverse kinematics
        double delta_x = state.target_x - current_x;
        double delta_y = state.target_y - current_y;
        double delta_norm = std::sqrt(delta_x * delta_x + delta_y * delta_y);

        int dynamic_iterations = (delta_norm > IK_DELTA_THRESHOLD)
                                 ? ik_cfg.num_iterations : IK_REDUCED_ITERATIONS;

        for (int i = 0; i < dynamic_iterations; i++) {
            forwardKinematics(theta1, theta2, current_x, current_y);
            delta_x = state.target_x - current_x;
            delta_y = state.target_y - current_y;
            updateJointAngles(theta1, theta2, delta_x, delta_y);
        }

        // Final FK for accurate display
        forwardKinematics(theta1, theta2, current_x, current_y);

        // Publish joint angles
        std_msgs::Float64 ros_theta1, ros_theta2;
        ros_theta1.data = mapTo2Pi(theta1);
        ros_theta2.data = mapTo2Pi(theta2);
        angle_pub_1.publish(ros_theta1);
        angle_pub_2.publish(ros_theta2);

        // Publish status
        std_msgs::Float64MultiArray status_msg;
        status_msg.data = {
            RAD2DEG(state.target_roll), RAD2DEG(state.current_roll),
            RAD2DEG(state.target_pitch), RAD2DEG(state.current_pitch),
            state.target_x, state.target_y,
            current_x, current_y
        };
        status_pub.publish(status_msg);

        // Dashboard (~4 Hz)
        if (++dashboard_counter >= dashboard_interval) {
            dashboard_counter = 0;
            printDashboard(theta1, theta2, current_x, current_y, target_length);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
