#include <ros/ros.h>
#include <cmath>
#include <cstdio>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include "hero_msgs/hero_agent_sensor.h"
#include "albc_control/albc_kinematics.h"

#include <dynamic_reconfigure/server.h>
#include <albc_control/ALBCControllerConfig.h>

#include <cstdlib>
#include <termios.h>
#include <unistd.h>

using namespace albc;

// ==============================
// Terminal keyboard (raw mode for key input)
// ==============================

static struct termios initial_term_settings;

static void initKeyboard() {
    // Use already-saved initial_term_settings (saved in selectModeInteractive)
    struct termios raw = initial_term_settings;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;   // non-blocking
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}

static void closeKeyboard() {
    tcsetattr(STDIN_FILENO, TCSANOW, &initial_term_settings);
}

static int readKey() {
    char ch;
    if (read(STDIN_FILENO, &ch, 1) == 1)
        return ch;
    return -1;
}

// ==============================
// Control Mode
// ==============================

enum class ControlMode : int { TDC = 1, PID = 2, FIXED = 3, MANUAL = 4 };
// TDC: Simplified Time-Delay Control (currently incremental PD with buoyancy compensation)
// PID: Standard PID with separate roll/pitch gains
// FIXED: Fixed end-effector position FK(90°,90°) for testing
// MANUAL: Direct joint angle or EE position control for calibration/testing

enum class ManualSubMode { JOINT, POSITION };

static const char* controlModeName(ControlMode m) {
    switch (m) {
        case ControlMode::TDC:    return "TDC";
        case ControlMode::PID:    return "PID";
        case ControlMode::FIXED:  return "FIXED";
        case ControlMode::MANUAL: return "MANUAL";
        default:                  return "???";
    }
}

// ==============================
// Constants
// ==============================

static constexpr double INTEGRAL_MAX        = 0.3;   // was 2.0; ki_eff×0.3 = 0.0225m max
static constexpr double DERIV_FILTER_ALPHA  = 0.1;   // EMA filter for derivative (strong smoothing at 50Hz, ~2s settling)
static constexpr double LAMBDA_DLS           = 0.13;  // DLS damping for buoyancy division (≈Fb×0.01)
static constexpr double DET_EPSILON         = 1e-6;
static constexpr double UPDATE_ANGLE_EPSILON = 1e-4;
static constexpr double IK_DELTA_THRESHOLD  = 0.01;
static constexpr int    IK_REDUCED_ITERATIONS = 500;
static constexpr double MANUAL_ANGLE_STEP     = 5.0;   // deg per keypress
static constexpr double MANUAL_POS_STEP       = 0.01;  // m per keypress

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
    double prev_roll, prev_pitch;           // previous IMU measurements (for derivative of measurement)
    double error_roll, error_pitch;
    double integral_roll, integral_pitch;
    double prev_error_roll, prev_error_pitch;
    double target_roll, target_pitch;
    double target_x, target_y;

    // w_roll/w_pitch are intentionally unused (always 0).
    // Angular velocity feedforward (-M̄·ν̇) was disabled after simulation showed
    // finite-difference amplification exceeds actuator authority at all M̄ values.
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

// Manual mode state
static ManualSubMode manual_submode = ManualSubMode::JOINT;
static double manual_theta1_deg = 90.0, manual_theta2_deg = 90.0;
static double manual_x = 0.0, manual_y = 0.0;

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

    // Pseudo-inverse: (J^T J + lambda^2 I)^{-1} * J^T
    double j11_inv = inv11 * j11 + inv12 * j12;
    double j12_inv = inv11 * j21 + inv12 * j22;
    double j21_inv = inv12 * j11 + inv22 * j12;
    double j22_inv = inv12 * j21 + inv22 * j22;

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
    state.current_pitch = msg->PITCH;
}

void reconfigureCallback(albc_control::ALBCControllerConfig& config, uint32_t /*level*/) {
    control_mode = static_cast<ControlMode>(config.control_mode);

    state.target_roll  = DEG2RAD(config.target_roll);
    state.target_pitch = DEG2RAD(config.target_pitch);

    gains.M_td_base  = config.M_td;
    gains.Kp_td_base = config.Kp_td;
    gains.Kd_td_base = config.Kd_td;

    gains.kp_roll_base  = config.kp_roll;
    gains.ki_roll_base  = config.ki_roll;
    gains.kd_roll_base  = config.kd_roll;

    gains.kp_pitch_base  = config.kp_pitch;
    gains.ki_pitch_base  = config.ki_pitch;
    gains.kd_pitch_base  = config.kd_pitch;

    gains.gain_mult = config.gain_mult;
    gains.applyMultiplier();

    ik_cfg.learning_rate  = config.ik_learning_rate;
    ik_cfg.lambda_base    = config.ik_lambda_base;
    ik_cfg.num_iterations = config.ik_num_iterations;

    manual_theta1_deg = config.manual_theta1;
    manual_theta2_deg = config.manual_theta2;
    manual_x          = config.manual_x;
    manual_y          = config.manual_y;

    // Reset integrals on gain change (anti-windup)
    state.integral_roll  = 0.0;
    state.integral_pitch = 0.0;

    ROS_INFO("Reconfigure: mode=%s mult=%.2f M=%.4f Kp=%.3f Kd=%.1f",
             controlModeName(control_mode), gains.gain_mult,
             gains.M_td, gains.Kp_td, gains.Kd_td);
}

void jointCurrentsCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    if (msg->data.size() >= 2) {
        joint_current1_mA = msg->data[0];
        joint_current2_mA = msg->data[1];
    }
}

// ==============================
// Mode Selection (blocking, before control loop)
// ==============================

static ControlMode selectModeInteractive() {
    // Save original terminal settings before any modification
    tcgetattr(STDIN_FILENO, &initial_term_settings);

    printf("\033[2J\033[H");
    printf("═══════════════════════════════════════════════════\n");
    printf("        ALBC Controller - Mode Selection\n");
    printf("═══════════════════════════════════════════════════\n");
    printf("  [1] TDC    - Time-Delay Control\n");
    printf("  [2] PID    - PID Control\n");
    printf("  [3] FIXED  - Fixed Position (test)\n");
    printf("  [4] MANUAL - Manual Position\n");
    printf("═══════════════════════════════════════════════════\n");
    printf(" Select mode (1/2/3/4): ");
    fflush(stdout);

    // Blocking read for mode selection
    struct termios blocking = initial_term_settings;
    blocking.c_lflag &= ~(ICANON | ECHO);
    blocking.c_cc[VMIN] = 1;   // block until 1 char
    blocking.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &blocking);

    ControlMode selected = ControlMode::TDC;
    while (true) {
        char ch;
        if (read(STDIN_FILENO, &ch, 1) == 1) {
            if (ch >= '1' && ch <= '4') {
                selected = static_cast<ControlMode>(ch - '0');
                printf("%c\n\n Starting [%s] mode...\n", ch, controlModeName(selected));
                fflush(stdout);
                usleep(500000);  // brief pause to show selection
                break;
            }
        }
    }

    // Switch to non-blocking for runtime
    initKeyboard();
    return selected;
}

// ==============================
// Runtime key handling
// ==============================

static void cycleMode() {
    int m = static_cast<int>(control_mode);
    m = (m % 4) + 1;  // 1→2→3→4→1
    control_mode = static_cast<ControlMode>(m);
}

static void handleRuntimeKey(int ch) {
    switch (ch) {
    case '=':   // Cycle mode: TDC→PID→FIXED→TDC
        cycleMode();
        break;
    case '1':   // Direct select: TDC
        control_mode = ControlMode::TDC;
        break;
    case '2':   // Direct select: PID
        control_mode = ControlMode::PID;
        break;
    case '3':   // Direct select: FIXED
        control_mode = ControlMode::FIXED;
        break;
    case '4':   // Direct select: MANUAL
        control_mode = ControlMode::MANUAL;
        break;
    // Manual mode keys (only active in MANUAL mode)
    case 'w':
        if (control_mode == ControlMode::MANUAL) {
            if (manual_submode == ManualSubMode::JOINT)
                manual_theta1_deg += MANUAL_ANGLE_STEP;
            else
                manual_y += MANUAL_POS_STEP;
        }
        break;
    case 's':
        if (control_mode == ControlMode::MANUAL) {
            if (manual_submode == ManualSubMode::JOINT)
                manual_theta1_deg -= MANUAL_ANGLE_STEP;
            else
                manual_y -= MANUAL_POS_STEP;
        }
        break;
    case 'a':
        if (control_mode == ControlMode::MANUAL) {
            if (manual_submode == ManualSubMode::JOINT)
                manual_theta2_deg -= MANUAL_ANGLE_STEP;
            else
                manual_x -= MANUAL_POS_STEP;
        }
        break;
    case 'd':
        if (control_mode == ControlMode::MANUAL) {
            if (manual_submode == ManualSubMode::JOINT)
                manual_theta2_deg += MANUAL_ANGLE_STEP;
            else
                manual_x += MANUAL_POS_STEP;
        }
        break;
    case 'm':
        if (control_mode == ControlMode::MANUAL) {
            manual_submode = (manual_submode == ManualSubMode::JOINT)
                             ? ManualSubMode::POSITION : ManualSubMode::JOINT;
        }
        break;
    default:
        break;
    }
}

// ==============================
// Control Law
// ==============================

void computeControlOutput(double dt, double derivative_roll, double derivative_pitch) {
    switch (control_mode) {
    case ControlMode::TDC: {
        // Simplified TDC: incremental PD with buoyancy compensation.
        // Full TDE terms (Λ_{t-L}·p_{EE}, -M̄·ν̇) are intentionally disabled:
        // simulation experiments showed TDE causes divergence due to
        // (1) Lambda time-variance creating positive feedback,
        // (2) finite-difference ν̇ amplification exceeding actuator authority,
        // (3) H_t ≈ H_{t-L} assumption structurally violated in this system.
        // See: references/docs/tdc-tuning-history.md
        state.a_roll  = (state.w_roll  - state.prev_w_roll)  / dt;
        state.a_pitch = (state.w_pitch - state.prev_w_pitch) / dt;

        // DLS-style division: preserves sign, graceful degradation near singularity
        double l_f = Fb * cos(state.current_roll) * cos(state.current_pitch);
        double common_factor = l_f / (l_f * l_f + LAMBDA_DLS * LAMBDA_DLS);

        state.target_x -= common_factor *
            (gains.M_td * (-state.a_roll + gains.Kd_td * derivative_roll + gains.Kp_td * state.error_roll));
        state.target_y -= common_factor *
            (gains.M_td * (-state.a_pitch + gains.Kd_td * derivative_pitch + gains.Kp_td * state.error_pitch));
        break;
    }
    case ControlMode::PID:
        // Incremental PID: delta applied from current position each cycle.
        // Prevents P-term vanishing (EE snap-back) when error suddenly drops to zero.
        // Anti-windup reset (target=current after IK) ensures no drift accumulation.
        state.target_x -= (gains.kp_roll  * state.error_roll  + gains.ki_roll  * state.integral_roll  + gains.kd_roll  * derivative_roll);
        state.target_y -= (gains.kp_pitch * state.error_pitch + gains.ki_pitch * state.integral_pitch + gains.kd_pitch * derivative_pitch);
        break;

    case ControlMode::FIXED: {
        // FK(90°, 90°): x = L1*cos(90°) + L2*cos(180°) = -L2, y = L1*sin(90°) + L2*sin(180°) = L1
        // Exponential smoothing to avoid sudden jumps (α=0.05 → ~4s settle at 10Hz)
        constexpr double FIXED_ALPHA = 0.05;
        state.target_x += FIXED_ALPHA * (-L2 - state.target_x);
        state.target_y += FIXED_ALPHA * ( L1 - state.target_y);
        break;
    }

    case ControlMode::MANUAL:
        // Handled in main loop before this function is called
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
    if (control_mode == ControlMode::MANUAL) {
        const char* sub = (manual_submode == ManualSubMode::JOINT) ? "JOINT" : "POSITION";
        printf("          ALBC Controller [MANUAL:%s]\n", sub);
    } else {
        printf("            ALBC Controller [%s]\n", controlModeName(control_mode));
    }
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
    printf("───────────────────────────────────────────────────\n");
    if (control_mode == ControlMode::MANUAL)
        printf(" Keys  ==Cycle  1-4=Mode  w/s/a/d=Adjust  m=SubMode\n");
    else
        printf(" Keys  ==Cycle  1=TDC  2=PID  3=FIXED  4=MANUAL\n");
    printf("═══════════════════════════════════════════════════\n");
    fflush(stdout);
}

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
    nh.param<double>("gain_mult", gains.gain_mult, 1.5);

    nh.param<double>("td_control/M_td", gains.M_td_base, 0.0085);
    nh.param<double>("td_control/Kp_td", gains.Kp_td_base, 0.085);
    nh.param<double>("td_control/Kd_td", gains.Kd_td_base, 1.1);

    nh.param<double>("pid_roll/kp", gains.kp_roll_base, 0.1);
    nh.param<double>("pid_roll/ki", gains.ki_roll_base, 0.001);
    nh.param<double>("pid_roll/kd", gains.kd_roll_base, 0.0005);

    nh.param<double>("pid_pitch/kp", gains.kp_pitch_base, 0.1);
    nh.param<double>("pid_pitch/ki", gains.ki_pitch_base, 0.001);
    nh.param<double>("pid_pitch/kd", gains.kd_pitch_base, 0.0005);

    nh.param<int>("ik/num_iterations", ik_cfg.num_iterations, 3000);
    nh.param<double>("ik/learning_rate", ik_cfg.learning_rate, 0.02);
    nh.param<double>("ik/lambda_base", ik_cfg.lambda_base, 0.15);

    nh.param<double>("initial_theta1_deg", initial_theta1_deg, 90.0);
    nh.param<double>("initial_theta2_deg", initial_theta2_deg, 90.0);

    nh.param<double>("manual/theta1", manual_theta1_deg, 90.0);
    nh.param<double>("manual/theta2", manual_theta2_deg, 90.0);
    nh.param<double>("manual/x", manual_x, 0.0);
    nh.param<double>("manual/y", manual_y, 0.0);

    gains.applyMultiplier();

    // Interactive mode selection (blocks until user picks 1/2/3/4)
    control_mode = selectModeInteractive();
    atexit(closeKeyboard);  // restore terminal on any exit path

    // Initial joint angles and end-effector position
    double theta1 = DEG2RAD(initial_theta1_deg);
    double theta2 = DEG2RAD(initial_theta2_deg);

    double current_x, current_y;
    forwardKinematics(theta1, theta2, current_x, current_y);

    state.target_x = current_x;
    state.target_y = current_y;

    // Dynamic reconfigure server (syncs YAML-loaded values, then enables runtime tuning)
    dynamic_reconfigure::Server<albc_control::ALBCControllerConfig> dr_server(nh);
    {
        albc_control::ALBCControllerConfig cfg;
        cfg.control_mode     = static_cast<int>(control_mode);
        cfg.target_roll      = 0.0;
        cfg.target_pitch     = 0.0;
        cfg.gain_mult        = gains.gain_mult;
        cfg.M_td             = gains.M_td_base;
        cfg.Kp_td            = gains.Kp_td_base;
        cfg.Kd_td            = gains.Kd_td_base;
        cfg.kp_roll          = gains.kp_roll_base;
        cfg.ki_roll          = gains.ki_roll_base;
        cfg.kd_roll          = gains.kd_roll_base;
        cfg.kp_pitch         = gains.kp_pitch_base;
        cfg.ki_pitch         = gains.ki_pitch_base;
        cfg.kd_pitch         = gains.kd_pitch_base;
        cfg.ik_learning_rate  = ik_cfg.learning_rate;
        cfg.ik_lambda_base    = ik_cfg.lambda_base;
        cfg.ik_num_iterations = ik_cfg.num_iterations;
        cfg.manual_theta1     = manual_theta1_deg;
        cfg.manual_theta2     = manual_theta2_deg;
        cfg.manual_x          = manual_x;
        cfg.manual_y          = manual_y;
        dr_server.updateConfig(cfg);
    }
    dr_server.setCallback(boost::bind(&reconfigureCallback, _1, _2));

    // Subscribers
    ros::Subscriber imu_sub     = nh.subscribe("/hero_agent/sensors", 50, imuCallback);
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

    while (ros::ok()) {
        // Runtime key input (non-blocking)
        int key = readKey();
        if (key >= 0) handleRuntimeKey(key);

        double dt = 1.0 / static_cast<double>(loop_rate_hz);
        double target_length = 0.0;

        if (control_mode == ControlMode::MANUAL) {
            // Manual mode: bypass IMU feedback pipeline entirely
            if (manual_submode == ManualSubMode::JOINT) {
                // Direct joint angle assignment — no IK needed
                theta1 = DEG2RAD(manual_theta1_deg);
                theta2 = DEG2RAD(manual_theta2_deg);
                forwardKinematics(theta1, theta2, current_x, current_y);
                state.target_x = current_x;
                state.target_y = current_y;
            } else {
                // Direct EE position — workspace saturation + IK only
                state.target_x = manual_x;
                state.target_y = manual_y;

                target_length = std::sqrt(state.target_x * state.target_x + state.target_y * state.target_y);
                if (target_length >= SAFE_ARM_LENGTH) {
                    state.target_x = state.target_x / target_length * SAFE_ARM_LENGTH;
                    state.target_y = state.target_y / target_length * SAFE_ARM_LENGTH;
                }

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

                forwardKinematics(theta1, theta2, current_x, current_y);
            }
        } else {
            // Normal feedback modes: TDC / PID / FIXED

            // 1. Error computation
            state.error_roll  = state.target_roll  - state.current_roll;
            state.error_pitch = state.target_pitch - state.current_pitch;

            // 2. Derivative of measurement (avoids derivative kick on setpoint change)
            //    d(error)/dt ≈ -d(measurement)/dt when target is constant
            static double filtered_deriv_roll = 0.0, filtered_deriv_pitch = 0.0;
            double raw_deriv_roll  = -(state.current_roll  - state.prev_roll)  / dt;
            double raw_deriv_pitch = -(state.current_pitch - state.prev_pitch) / dt;
            filtered_deriv_roll  = DERIV_FILTER_ALPHA * raw_deriv_roll  + (1.0 - DERIV_FILTER_ALPHA) * filtered_deriv_roll;
            filtered_deriv_pitch = DERIV_FILTER_ALPHA * raw_deriv_pitch + (1.0 - DERIV_FILTER_ALPHA) * filtered_deriv_pitch;

            // 3. Control law (uses previous integral — updated AFTER saturation check)
            computeControlOutput(dt, filtered_deriv_roll, filtered_deriv_pitch);

            // Store previous states
            state.prev_roll        = state.current_roll;
            state.prev_pitch       = state.current_pitch;
            state.prev_error_roll  = state.error_roll;
            state.prev_error_pitch = state.error_pitch;
            state.prev_w_roll      = state.w_roll;
            state.prev_w_pitch     = state.w_pitch;

            // 4. Radial workspace saturation
            target_length = std::sqrt(state.target_x * state.target_x + state.target_y * state.target_y);
            bool saturated = (target_length >= SAFE_ARM_LENGTH);
            if (saturated) {
                state.target_x = state.target_x / target_length * SAFE_ARM_LENGTH;
                state.target_y = state.target_y / target_length * SAFE_ARM_LENGTH;
            }

            // 5. Conditional integral update (workspace-aware anti-windup)
            //    When NOT saturated: normal integral accumulation
            //    When saturated: only allow integral to DECREASE (unwind), not increase
            if (!saturated) {
                state.integral_roll  += state.error_roll  * dt;
                state.integral_pitch += state.error_pitch * dt;
            } else {
                double new_ir = state.integral_roll  + state.error_roll  * dt;
                double new_ip = state.integral_pitch + state.error_pitch * dt;
                if (std::abs(new_ir) < std::abs(state.integral_roll))
                    state.integral_roll = new_ir;
                if (std::abs(new_ip) < std::abs(state.integral_pitch))
                    state.integral_pitch = new_ip;
            }
            state.integral_roll  = std::max(-INTEGRAL_MAX, std::min(INTEGRAL_MAX, state.integral_roll));
            state.integral_pitch = std::max(-INTEGRAL_MAX, std::min(INTEGRAL_MAX, state.integral_pitch));

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

            // Anti-windup: reset incremental-mode target to actual FK position.
            // Prevents target drift when IK cannot reach the commanded position
            // (e.g., near singularity). In normal operation target ≈ current.
            // Applies to both TDC and PID (both use incremental form).
            if (control_mode == ControlMode::TDC || control_mode == ControlMode::PID) {
                state.target_x = current_x;
                state.target_y = current_y;
            }
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

    closeKeyboard();
    return 0;
}
