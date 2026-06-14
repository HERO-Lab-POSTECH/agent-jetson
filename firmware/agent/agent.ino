#include <avr/interrupt.h>

#include <Servo.h>
#include <Wire.h>
#include <math.h>
#include "MS5837.h"

#include <ros.h>
#include <std_msgs/Int8.h>
#include "hero_msgs/hero_agent_sensor.h"
#include "hero_msgs/hero_agent_state.h"
#include "ros_opencv_ipcam_qr/hero_ipcam_qr_msg.h"
#include "hero_msgs/hero_xy_cont.h"
#include "hero_msgs/hero_command.h"

#include "hero_msgs/hero_agent_dvl.h"
#include "hero_msgs/hero_agent_dvl_velocity.h"

// 20220613_add for dvl position cont
#include "hero_msgs/hero_agent_cont_xy.h"
#include "hero_msgs/hero_agent_cont_para.h"
#include "hero_msgs/hero_agent_position_result.h"

// for PIN SETTING---------------------
#define F_CPU 16000000UL

#include "config.h"
#include "ahrs.h"
#include "thrusters.h"
#include "pid.h"
#include "dvl_position.h"
#include "io.h"
//--------------------------------------

// 메인 스케치 자체 함수 forward 선언 (정의가 호출보다 뒤 — Arduino auto-prototype 대체)
void Initialization(void);

// for ROS serial libaray for PUBLISH---------------------
ros::NodeHandle nh; // main handle

hero_msgs::hero_agent_state state_msg;            // state
hero_msgs::hero_agent_sensor sensors_msg;         // sensors
hero_msgs::hero_agent_position_result result_msg; // sensors

ros::Publisher pub_state("/hero_agent/state", &state_msg);
ros::Publisher pub_sensors("/hero_agent/sensors", &sensors_msg);
ros::Publisher pub_result("/hero_agent/result", &result_msg);

//--------------------------------------

// for DEPTH sensor---------------------

MS5837 DEPTH_Sensor;

double depth;
//--------------------------------------

// for AHRS_3DM_GX5----------------------
volatile int serial_count = 0;
volatile uint8_t inputString[100];       // a String to hold incoming data
volatile boolean stringComplete = false; // whether the string is complete

volatile uint8_t imu_check = 0;

// u_data union 정의는 ahrs.cpp로 이동 (파싱 전용, ISR은 미사용).

volatile float roll, pitch, yaw;
volatile float yaw_temp;
volatile float acc_roll, acc_pitch, acc_yaw;
volatile float acc_x = 0, acc_y = 0, acc_z = 0;
volatile int acc_count = 0;
volatile uint8_t ahrs_valid1, ahrs_valid2, ahrs_valid3;

volatile uint8_t yaw_calid_command = 0;
volatile char yaw_calib = 0;
volatile float pre_yaw = 0;
volatile float yaw_calib2 = 0;
//--------------------------------------

// for Trusters-------------------------
volatile int pwm_m0 = ESC_NEUTRAL, pid_pwm_m0;
volatile int pwm_m1 = ESC_NEUTRAL, pid_pwm_m1;
volatile int pwm_m2 = ESC_NEUTRAL, pid_pwm_m2;
volatile int pwm_m3 = ESC_NEUTRAL, pid_pwm_m3;
volatile int pwm_m4 = ESC_NEUTRAL, pid_pwm_m4;
volatile int pwm_m5 = ESC_NEUTRAL, pid_pwm_m5;
//--------------------------------------

// for Control yaw and depth---------------------
volatile uint8_t cont_yaw_on = 0;
volatile uint8_t cont_depth_on = 0;

// TODO(4번 flash 조각): T/T_depth를 측정 loop_speed 기반 동적 dt로.
//   현재는 고정 명목값(T=0.004는 실제 루프주기 ~9ms와도 불일치).
//   변경 시 D/I 게인 재튜닝 필수(거동 변경) → flash 검증과 함께 opt-in.
volatile double T = 0.004;      // Loop time.
volatile double T_depth = 0.04; // Loop time.

volatile int throttle = 40;

volatile double P_angle_gain_yaw = 6.5;
volatile double P_gain_yaw = 10;
volatile double I_gain_yaw = 1;
volatile double D_gain_yaw = 0.5;

double P_gain_depth = 1100.0;
double I_gain_depth = 10.0;
double D_gain_depth = 100.0;

volatile double error_yaw;
volatile double error_pid_yaw, error_pid_yaw1;
volatile double P_angle_pid_yaw;
volatile double P_yaw, I_yaw, D_yaw, PID_yaw;
volatile double desired_angle_yaw = 1;

double error_pid_depth, error_pid_depth1;
double P_angle_pid_depth;
double P_depth, I_depth, D_depth, PID_depth;
double desired_angle_depth = 0.5;
//--------------------------------------

volatile uint8_t cont_direc = 0;
volatile int move_speed = 20;

// for STATE addition
volatile uint8_t cont_Yaw = 0;
volatile uint8_t cont_Depth = 0;
volatile uint8_t state_Relay = 0;
volatile uint8_t state_Laser = 0;

volatile int State_all = 0;
//-------------------------------------

volatile int Th_0 = 0;
volatile int Th_1 = 0;
volatile int Th_2 = 0;
volatile int Th_3 = 0;

volatile double X = 0, Y = 0, Z = 0;
volatile double Tx = 0, Ty = 0;
volatile double error_sum_x = 0, error_sum_y = 0;

volatile double Kp = 150, Ki = 0, Kd = 150;
volatile double Mb = 15, KKp = 0.225, KKv = 0.705;

int control_T = 0;

volatile double TARGET_X = 0, TARGET_Y = 0, TARGET_Z = 0;
volatile double temp_depth = 0;

volatile double error_x = 0, error_y = 0;
volatile double error_d_x = 0, error_d_y = 0;
volatile double error_x_pre = 0, error_y_pre = 0;
volatile double a_x = 0, a_y = 0;
volatile double error_d_x_pre = 0, error_d_y_pre = 0;
volatile double pre_a_x = 0, pre_a_y = 0;
volatile double pre_v_x = 0, pre_v_y = 0;

// msgCallback_dvl() / msgCallback_cont_para() 정의는 dvl_position.cpp로 이동 (선언은 dvl_position.h).
// DVL 적분·제어 전역의 *정의*는 agent.ino에 그대로 유지(dvl_position.h가 extern).

volatile float save_acc_x = 0;
volatile float pre_save_acc_x = 0;
volatile float save_v_x = 0;

volatile float save_acc_x_pre[20] = {
    0,
};
volatile int save_acc_count[20] = {
    0,
};
volatile int print_i = 13;

// msgCallback_dvl_velocity() 정의는 dvl_position.cpp로 이동 (선언은 dvl_position.h).

volatile int darknet_Th_0 = 0;
volatile int darknet_Th_1 = 0;
volatile int darknet_Th_2 = 0;
volatile int darknet_Th_3 = 0;

// msgCallback_cont_xy_darknet() 정의는 dvl_position.cpp로 이동 (선언은 dvl_position.h).
//---------------------------------------

int test_cont_set = 0;
unsigned long time_init;
void messageCommand(const std_msgs::Int8 &command_msg)
{
  int Command = command_msg.data;

  if (Command == 'n')
  {
    desired_angle_yaw = 0;
    yaw_calid_command = 1;

    test_cont_set = 0;
  }

  else if (Command == 'q')
  {
    cont_direc = 0;
    save_acc_x = 0;
  }
  else if (Command == 's') // backward
  {
    cont_direc = 1;
  }
  else if (Command == 'w') // forward
  {
    cont_direc = 2;
  }
  else if (Command == 'd') // right
  {
    cont_direc = 3;
  }
  else if (Command == 'a') // left
  {
    cont_direc = 4;
  }
  else if (Command == '1') // concon
  {
    cont_direc = 8;
  }
  else if (Command == '4') // dvl cont
  {
    cont_direc = 7;
  }
  else if (Command == '2') // concon
  {
    print_i++;
  }
  else if (Command == '3') // concon
  {
    print_i--;
  }
  else if (Command == 'z') // left
  {
    move_speed += 10;
  }
  else if (Command == 'x') // right
  {
    move_speed -= 10;
  }
  else if (Command == 'e')
  {
    relay_on();
  }
  else if (Command == 't')
  {
    relay_off();
  }
  else if (Command == 'r')
  {
    laser_on();
  }
  else if (Command == 'f')
  {
    laser_off();
  }
  else if (Command == 'c')
  {
    gripper_set(GRIPPER_OPEN); // open gripper
  }
  else if (Command == 'v')
  {
    gripper_set(GRIPPER_STOP); // stop gripper
  }
  else if (Command == 'b')
  {
    gripper_set(GRIPPER_CLOSE); // close gripper
  }
  else if (Command == 'g')
  {

    pwm_m0 = ESC_NEUTRAL;
    pwm_m1 = ESC_NEUTRAL;
    pwm_m2 = ESC_NEUTRAL;
    pwm_m3 = ESC_NEUTRAL;
    pwm_m4 = ESC_NEUTRAL;
    pwm_m5 = ESC_NEUTRAL;

    esc_input(0x02, pwm_m0, pwm_m1, pwm_m2);
    esc_input(0x03, pwm_m3, pwm_m4, pwm_m5);
  }
  else if (Command == 'y')
  {
    cont_yaw_on = 1;
    cont_Yaw = 1;
  }
  else if (Command == 'h')
  {
    cont_yaw_on = 0;
    cont_Yaw = 0;
  }
  else if (Command == 'u')
  {
    throttle += 10;
  }
  else if (Command == 'j')
  {
    throttle -= 10;
  }
  else if (Command == 'i')
  {
    desired_angle_yaw += 0.1;
  }
  else if (Command == 'k')
  {
    desired_angle_yaw -= 0.1;
  }
  else if (Command == '6')
  {
    desired_angle_yaw += 0.0002;
  }
  else if (Command == '5')
  {
    desired_angle_yaw -= 0.0002;
  }
  else if (Command == '8')
  {
    desired_angle_yaw += 0.01;
  }
  else if (Command == '7')
  {
    desired_angle_yaw -= 0.01;
  }
  else if (Command == 'o')
  {
    desired_angle_depth += 0.1;
  }
  else if (Command == 'l')
  {
    desired_angle_depth -= 0.1;
  }
  else if (Command == 'p')
  {
    cont_depth_on = 1;
    cont_Depth = 1;
  }
  else if (Command == ';')
  {
    cont_depth_on = 0;
    cont_Depth = 0;
  }
  State_all = 0;
  if (cont_Yaw == 1)
  {
    State_all += 1;
  }
  if (cont_Depth == 1)
  {
    State_all += 2;
  }
  if (state_Relay == 1)
  {
    State_all += 4;
  }
  if (state_Laser == 1)
  {
    State_all += 8;
  }
}

ros::Subscriber<std_msgs::Int8> sub_command("/hero_agent/command",
                                            &messageCommand);

ros::Subscriber<hero_msgs::hero_agent_cont_xy> sub_cont_xy_darknet("/hero_agent/cont_xy_darknet",
                                                                   &msgCallback_cont_xy_darknet);

ros::Subscriber<hero_msgs::hero_agent_dvl> sub_dvl("/hero_agent/dvl", &msgCallback_dvl);
ros::Subscriber<hero_msgs::hero_agent_cont_para> sub_cont_para("/hero_agent/cont_para", &msgCallback_cont_para);
ros::Subscriber<hero_msgs::hero_agent_dvl_velocity> sub_dvl_velocity("/hero_agent/dvl_velocity", &msgCallback_dvl_velocity);

//--------------------------------------------
//--------------------------------------------
volatile int check_hz = 0;
// Incoming AHRS data--------------------------------------
ISR(USART1_RX_vect)
{ // USART1에 RX가 들어왔을때 동작하는 interrupt

  char inChar = (char)UDR1;

  if (imu_check == 0 && inChar == (char)MIP_SYNC1)
  {
    imu_check = 1;
    inputString[0] = inChar;
  }
  else if (imu_check == 1 && inChar == (char)MIP_SYNC2)
  {
    imu_check = 2;
    inputString[1] = inChar;
  }
  else if (imu_check == 2 && inChar == (char)MIP_DESC)
  {
    imu_check = 3;
    inputString[2] = inChar;
    serial_count = 3;
  }
  else if (imu_check == 3)
  {
    inputString[serial_count] = inChar;
    serial_count++;
  }
  // if the incoming character is a newline, set a flag so the main loop can
  // do something about it:
  // inputString[19] == 1 &&
  if (serial_count == MIP_PACKET_LEN && inputString[35] == 1) //수신이 완료
  {
    ahrs_parse_packet(); // 패킷 완성 파싱 본문(u_data 추출·yaw unwrap) → ahrs.cpp로 이동
  }
  else if (serial_count > MIP_PACKET_LEN) //데이터가 10개 넘게 들어옴 (데inputString[19]이터 수신 실패)
  {
    inputString[0] = '\0';
    serial_count = 0;
    stringComplete = false;
    imu_check = 0;
  }
}

// UART1_write() 정의는 ahrs.cpp로 이동 (선언은 ahrs.h).
// UART2_write() 정의는 thrusters.cpp로 이동 (선언은 thrusters.h).
//----------------------------------------------------------
unsigned long start_time = 0;
unsigned long current_time = 0;
int loop_count = 0;
int depth_count = 0;
int loop_speed = 0;

void setup()
{
  nh.initNode();

  nh.advertise(pub_state);
  nh.advertise(pub_sensors);
  nh.advertise(pub_result);
  nh.subscribe(sub_command);
  nh.subscribe(sub_cont_xy_darknet);
  nh.subscribe(sub_dvl);
  nh.subscribe(sub_cont_para);
  nh.subscribe(sub_dvl_velocity);
  Initialization(); // init all 1100ms

  start_time = millis();
  loop_count = 0;
}
// loop is for control
void loop()
{
  current_time = millis() - start_time;
  loop_count++;

  if (current_time > 1000)
  {
    start_time = millis();
    loop_speed = loop_count;
    loop_count = 0;
  }

  if (cont_yaw_on == 1)
    PID_control_yaw();
  nh.spinOnce();

  if (depth_count == 0)
  {
    DEPTH_Sensor.read2();
    depth_count = 1;
    delay(9);
  }
  else if (depth_count == 1)
  {
    depth_count = 2;
    delay(9);
  }
  else if (depth_count == 2)
  {
    DEPTH_Sensor.read4();
    depth = DEPTH_Sensor.depth();

    if (cont_depth_on == 1)
      PID_control_depth();

    depth_count = 3;
    delay(9);
  }
  else if (depth_count == 3)
  {
    depth_count = 0;
    delay(9);

    // DEPTH_Sensor.read();
    // depth = DEPTH_Sensor.depth();

    sensors_msg.ROLL = roll;
    sensors_msg.PITCH = pitch;
    sensors_msg.YAW = yaw;
    sensors_msg.DEPTH = loop_speed;

    pub_sensors.publish(&sensors_msg);

    result_msg.TARGET_X = TARGET_X;
    result_msg.TARGET_Y = TARGET_Y;
    result_msg.TARGET_Z = TARGET_Z;
    result_msg.X = X;
    result_msg.Y = Y;
    result_msg.Z = depth - temp_depth;

    pub_result.publish(&result_msg);

    state_msg.Yaw = yaw;
    state_msg.Target_yaw = desired_angle_yaw;
    state_msg.Throttle = throttle;
    state_msg.Valid_yaw = inputString[19];
    state_msg.Depth = depth;
    state_msg.Target_depth = desired_angle_depth;
    state_msg.Move_speed = move_speed;
    state_msg.Cont_state = cont_direc;
    state_msg.State_addit = State_all;
    pub_state.publish(&state_msg);
  }
}

// esc_input() 정의는 thrusters.cpp로 이동 (선언은 thrusters.h).

void Initialization(void)
{
  // Serial setting
  // Serial2.begin(115200); //truster control
  UBRR2H = 0x00;
  UBRR2L = 0x08;
  UCSR2A = 0x00;
  UCSR2B = 0x18;
  UCSR2C = 0x06;
  // Serial1 setting (ahrs)
  UBRR1H = 0x00;
  UBRR1L = 0x08;
  UCSR1A = 0x00;
  UCSR1B = 0x98;
  UCSR1C = 0x06;

  // init pressure sensor
  Wire.begin();

  // Init AHRS

  ahrs_init(); // AHRS MIP init 시퀀스(delay 포함) → ahrs.cpp로 이동

  gripper_init();

  // depth sensor init
  while (!DEPTH_Sensor.init())
  {
    delay(5000);
  }
  DEPTH_Sensor.setModel(MS5837::MS5837_30BA);
  DEPTH_Sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)

  io_init();
}

// PID_control_yaw() / PID_control_depth() 정의는 pid.cpp로 이동 (선언은 pid.h).
// 게인·상태 전역의 *정의*는 위쪽 agent.ino에 그대로 유지(pid.h가 extern).
