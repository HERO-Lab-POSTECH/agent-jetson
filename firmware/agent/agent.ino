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
//--------------------------------------

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

volatile union
{
  float real;
  uint32_t base;
} u_data;

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

void msgCallback_dvl(const hero_msgs::hero_agent_dvl &msg)
{
  if (msg.command == 1)
  {
    X = 0;
    Y = 0;
    temp_depth = depth;
    Tx = 0;
    Ty = 0;
    error_sum_x = 0;
    error_sum_y = 0;

    error_x = 0, error_y = 0;
    error_d_x = 0, error_d_y = 0;
    error_x_pre = 0, error_y_pre = 0;
    a_x = 0, a_y = 0;
    error_d_x_pre = 0, error_d_y_pre = 0;
    pre_a_x = 0, pre_a_y = 0;
    pre_v_x = 0, pre_v_y = 0;

    TARGET_X = 0;
    TARGET_Y = 0;
    TARGET_Z = 0;
  }
  else
  {
    TARGET_X = msg.TARGET_X;
    TARGET_Y = msg.TARGET_Y;
    TARGET_Z = msg.TARGET_Z;
  }
}

void msgCallback_cont_para(const hero_msgs::hero_agent_cont_para &msg)
{
  control_T = msg.control_T;

  Kp = msg.Kp;
  Ki = msg.Ki;
  Kd = msg.Kd;

  Mb = msg.Mb;
  KKp = msg.KKp;
  KKv = msg.KKv;
}

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

void msgCallback_dvl_velocity(const hero_msgs::hero_agent_dvl_velocity &msg)
{

  double time_gap = (double)msg.TIME / 1000.0;

  if (msg.VALID == 'y')
  {

    X += (msg.VX * time_gap) * cos(yaw) + (msg.VY * time_gap) * sin(yaw); //
    Y += (msg.VX * time_gap) * sin(yaw) + (msg.VY * time_gap) * cos(yaw); //

    error_x = TARGET_X - X;
    error_y = TARGET_Y - Y;

    error_d_x = (error_x - error_x_pre) / time_gap;
    error_d_y = (error_y - error_y_pre) / time_gap;
    if (error_d_x > 1 || error_d_x < -1)
      error_d_x = 0;

    error_sum_x += error_x;
    error_sum_y += error_y;

    a_x = (error_d_x - error_d_x_pre) / time_gap;
    a_y = (error_d_y - error_d_y_pre) / time_gap;
    if (a_x > 1 || a_x < -1)
      a_x = 0;

    save_acc_x = -save_acc_x_pre[print_i] / (float)save_acc_count[print_i];

    save_v_x = a_x;
    for (int i = 0; i < 19; i++)
    {
      save_acc_x_pre[i] = save_acc_x_pre[i + 1];
      save_acc_count[i] = save_acc_count[i + 1];
    }
    save_acc_x_pre[19] = acc_x;
    save_acc_count[19] = acc_count;
    acc_x = 0;
    acc_count = 0;

    if (control_T == 0)
    {
      Tx = Kp * error_x + Ki * error_sum_x + Kd * error_d_x;
      Ty = Kp * error_y + Ki * error_sum_y + Kd * error_d_y;
    }
    else if (control_T == 1)
    {
      Tx += Mb * (-pre_a_x + KKv * error_d_x + KKp * error_x);
      Ty += Mb * (-pre_a_y + KKv * error_d_y + KKp * error_y);
    }
    else if (control_T == 3)
    {
      Tx += Mb * (-pre_a_x + KKv * error_d_x + KKp * error_x);
      Ty = Kp * error_y + Ki * error_sum_y + Kd * error_d_y;
    }
    else if (control_T == 4)
    {
      Tx += Mb * (-pre_save_acc_x + KKv * error_d_x + KKp * error_x);
      Ty = Kp * error_y + Ki * error_sum_y + Kd * error_d_y;
    }

    if (Tx > 100)
    {
      Tx = 100;
    }
    else if (Tx < -100)
    {
      Tx = -100;
    }

    if (Ty > 100)
    {
      Ty = 100;
    }
    else if (Ty < -100)
    {
      Ty = -100;
    }

    error_d_x_pre = error_d_x;
    error_d_y_pre = error_d_y;

    pre_v_x = msg.VX;
    pre_v_y = msg.VY;

    pre_save_acc_x = save_acc_x;

    pre_a_x = a_x;
    pre_a_y = a_y;

    error_x_pre = error_x;
    error_y_pre = error_y;

    Th_0 = -Tx + Ty;
    Th_1 = -Tx - Ty;
    Th_2 = Tx - Ty;
    Th_3 = Tx + Ty;

    desired_angle_depth = temp_depth + TARGET_Z;
  }
  else
  {
    Th_0 = 0;
    Th_1 = 0;
    Th_2 = 0;
    Th_3 = 0;
    desired_angle_depth = temp_depth + TARGET_Z;
  }
}

volatile int darknet_Th_0 = 0;
volatile int darknet_Th_1 = 0;
volatile int darknet_Th_2 = 0;
volatile int darknet_Th_3 = 0;

void msgCallback_cont_xy_darknet(const hero_msgs::hero_agent_cont_xy &msg)
{
  darknet_Th_0 = msg.T0;
  darknet_Th_1 = msg.T1;
  darknet_Th_2 = msg.T2;
  darknet_Th_3 = msg.T3;
}
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

  else if (Command == '.') // cont test  yaw
  {
  }
  else if (Command == ',') // cont test depth
  {
  }
  else if (Command == '/') // cont test depth
  {
  }
  else if (Command == '-')
  {
  }
  else if (Command == '0')
  {
  }
  else if (Command == '9')
  {
  }
  else if (Command == '=')
  {
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
    state_Relay = 1;
    digitalWrite(PIN_RELAY, HIGH);
    delay(5000);
  }
  else if (Command == 't')
  {
    state_Relay = 0;
    digitalWrite(PIN_RELAY, LOW);
  }
  else if (Command == 'r')
  {
    state_Laser = 1;
    digitalWrite(PIN_LED_SIG, HIGH);
  }
  else if (Command == 'f')
  {
    state_Laser = 0;
    digitalWrite(PIN_LED_SIG, LOW);
  }
  else if (Command == 'c')
  {
    OCR1A = GRIPPER_OPEN; // open gripper
  }
  else if (Command == 'v')
  {
    OCR1A = GRIPPER_STOP; // stop gripper
  }
  else if (Command == 'b')
  {
    OCR1A = GRIPPER_CLOSE; // close gripper
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
    stringComplete = true;
    serial_count = 0;
    imu_check = 0;
    check_hz++;
    u_data.base = 0;
    u_data.base |= ((uint32_t)inputString[9]) << (8 * 0);
    u_data.base |= ((uint32_t)inputString[8]) << (8 * 1);
    u_data.base |= ((uint32_t)inputString[7]) << (8 * 2);
    u_data.base |= ((uint32_t)inputString[6]) << (8 * 3);
    roll = u_data.real;

    u_data.base = 0;
    u_data.base |= ((uint32_t)inputString[13]) << (8 * 0);
    u_data.base |= ((uint32_t)inputString[12]) << (8 * 1);
    u_data.base |= ((uint32_t)inputString[11]) << (8 * 2);
    u_data.base |= ((uint32_t)inputString[10]) << (8 * 3);
    pitch = u_data.real;

    u_data.base = 0;
    u_data.base |= ((uint32_t)inputString[17]) << (8 * 0);
    u_data.base |= ((uint32_t)inputString[16]) << (8 * 1);
    u_data.base |= ((uint32_t)inputString[15]) << (8 * 2);
    u_data.base |= ((uint32_t)inputString[14]) << (8 * 3);
    yaw = u_data.real;
    yaw_temp = u_data.real;
    u_data.base = 0;
    u_data.base |= ((uint32_t)inputString[25]) << (8 * 0);
    u_data.base |= ((uint32_t)inputString[24]) << (8 * 1);
    u_data.base |= ((uint32_t)inputString[23]) << (8 * 2);
    u_data.base |= ((uint32_t)inputString[22]) << (8 * 3);
    acc_roll = u_data.real;

    u_data.base = 0;
    u_data.base |= ((uint32_t)inputString[29]) << (8 * 0);
    u_data.base |= ((uint32_t)inputString[28]) << (8 * 1);
    u_data.base |= ((uint32_t)inputString[27]) << (8 * 2);
    u_data.base |= ((uint32_t)inputString[26]) << (8 * 3);
    acc_pitch = u_data.real;

    u_data.base = 0;
    u_data.base |= ((uint32_t)inputString[33]) << (8 * 0);
    u_data.base |= ((uint32_t)inputString[32]) << (8 * 1);
    u_data.base |= ((uint32_t)inputString[31]) << (8 * 2);
    u_data.base |= ((uint32_t)inputString[30]) << (8 * 3);
    acc_yaw = u_data.real;

    u_data.base = 0;
    u_data.base |= ((uint32_t)inputString[41]) << (8 * 0);
    u_data.base |= ((uint32_t)inputString[40]) << (8 * 1);
    u_data.base |= ((uint32_t)inputString[39]) << (8 * 2);
    u_data.base |= ((uint32_t)inputString[38]) << (8 * 3);
    // if (u_data.real > -0.01 && u_data.real < 0.01)
    acc_x += u_data.real;

    u_data.base = 0;
    u_data.base |= ((uint32_t)inputString[45]) << (8 * 0);
    u_data.base |= ((uint32_t)inputString[44]) << (8 * 1);
    u_data.base |= ((uint32_t)inputString[43]) << (8 * 2);
    u_data.base |= ((uint32_t)inputString[42]) << (8 * 3);
    if (u_data.real > -0.01 && u_data.real < 0.01)
      acc_y += u_data.real;

    u_data.base = 0;
    u_data.base |= ((uint32_t)inputString[49]) << (8 * 0);
    u_data.base |= ((uint32_t)inputString[48]) << (8 * 1);
    u_data.base |= ((uint32_t)inputString[47]) << (8 * 2);
    u_data.base |= ((uint32_t)inputString[46]) << (8 * 3);
    if (u_data.real > -0.01 && u_data.real < 0.01)
      acc_z += u_data.real;
    acc_count++;

    ahrs_valid1 = inputString[19];
    ahrs_valid2 = inputString[35];
    ahrs_valid3 = inputString[51];

    yaw -= yaw_calib2;

    if (yaw_calid_command == 1)
    {
      yaw_calid_command = 0;
      yaw_calib2 = yaw_temp;
      pre_yaw = 0;
      yaw = 0;
      yaw_calib = 0;
    }

    if (yaw - pre_yaw > YAW_WRAP_THRESH)
      yaw_calib--;
    else if (yaw - pre_yaw < -YAW_WRAP_THRESH)
      yaw_calib++;
    pre_yaw = yaw;
    yaw += YAW_PI * yaw_calib * 2;

    //
    // yaw control loop

    // esc_input(0x01, pwm_m0, pwm_m1, pwm_m2);
    // esc_input(0x02, pwm_m3, pwm_m4, pwm_m5);
  }
  else if (serial_count > MIP_PACKET_LEN) //데이터가 10개 넘게 들어옴 (데inputString[19]이터 수신 실패)
  {
    inputString[0] = '\0';
    serial_count = 0;
    stringComplete = false;
    imu_check = 0;
  }
}

void UART1_write(char ch)
{

  while (!(UCSR1A & 0x20))
    ;

  UDR1 = ch;
}
void UART2_write(char ch)
{

  while (!(UCSR2A & 0x20))
    ;

  UDR2 = ch;
}
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
    // sensors_msg.ROLL = Tx;
    // sensors_msg.PITCH = Ty;
    // sensors_msg.ROLL = save_acc_x;
    // sensors_msg.PITCH = save_v_x;
    // sensors_msg.YAW = acc_x;
    // sensors_msg.DEPTH = acc_y;
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

void esc_input(uint8_t ID, uint16_t pwm0, uint16_t pwm1, uint16_t pwm2)
{
  UART2_write(0xff);
  UART2_write(0xff);
  UART2_write(ID);
  UART2_write(pwm0 / 256);
  UART2_write(pwm0 % 256);
  UART2_write(pwm1 / 256);
  UART2_write(pwm1 % 256);
  UART2_write(pwm2 / 256);
  UART2_write(pwm2 % 256);
  UART2_write(0xfe);
}

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

  delay(100);
  UART1_write(0x75);
  UART1_write(0x65);
  UART1_write(0x0C);
  UART1_write(0x05);
  UART1_write(0x05);
  UART1_write(0x11);
  UART1_write(0x01);
  UART1_write(0x03);
  UART1_write(0x01);
  UART1_write(0x06);
  UART1_write(0x1E);
  delay(1000);

  // init gripper by servo
  // GRIPPER_SERVO.attach(GRIPPER_SIG);
  DDRB |= 0x20;
  TCCR1A = 0x82;
  TCCR1B = 0x1B;
  TCCR1C = 0;
  ICR1 = GRIPPER_ICR1;

  OCR1A = GRIPPER_STOP;

  // depth sensor init
  while (!DEPTH_Sensor.init())
  {
    delay(5000);
  }
  DEPTH_Sensor.setModel(MS5837::MS5837_30BA);
  DEPTH_Sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)

  // basic io init
  pinMode(PIN_RELAY, OUTPUT);
  pinMode(PIN_LED_SIG, OUTPUT);
  pinMode(PIN_RTS, OUTPUT);

  // RTS on (rs485 send)
  digitalWrite(PIN_RTS, HIGH);
}

void PID_control_yaw()
{
  error_yaw = desired_angle_yaw - yaw;            // angle def
  P_angle_pid_yaw = P_angle_gain_yaw * error_yaw; // angle def + outer P control

  error_pid_yaw = P_angle_pid_yaw - acc_yaw; // Pcontrol_angle - angle rate = PID Goal

  P_yaw = error_pid_yaw * P_gain_yaw;                        // Inner P control
  D_yaw = (error_pid_yaw - error_pid_yaw1) / T * D_gain_yaw; // Inner D control
  I_yaw += (error_pid_yaw)*T * I_gain_yaw;                   // Inner I control
  I_yaw = constrain(I_yaw, -100, 100);                       // I control must be limited to prevent being jerk.

  PID_yaw = P_yaw + D_yaw + I_yaw;

  if (cont_direc == 0) // stop
  {
    pwm_m1 = PID_yaw + throttle + ESC_NEUTRAL;
    pwm_m2 = -PID_yaw + throttle + ESC_NEUTRAL;
    pwm_m4 = -PID_yaw - throttle + ESC_NEUTRAL;
    pwm_m5 = -PID_yaw + throttle + ESC_NEUTRAL;
  }
  else if (cont_direc == 1) // backward
  {
    pwm_m1 = PID_yaw + throttle + ESC_NEUTRAL - move_speed;
    pwm_m2 = -PID_yaw + throttle + ESC_NEUTRAL + move_speed;
    pwm_m4 = -PID_yaw - throttle + ESC_NEUTRAL - move_speed;
    pwm_m5 = -PID_yaw + throttle + ESC_NEUTRAL - move_speed;
  }
  else if (cont_direc == 2) // forward
  {
    pwm_m1 = PID_yaw + throttle + ESC_NEUTRAL + move_speed;
    pwm_m2 = -PID_yaw + throttle + ESC_NEUTRAL - move_speed;
    pwm_m4 = -PID_yaw - throttle + ESC_NEUTRAL + move_speed;
    pwm_m5 = -PID_yaw + throttle + ESC_NEUTRAL + move_speed;
  }
  else if (cont_direc == 3) // right
  {
    pwm_m1 = PID_yaw + throttle + ESC_NEUTRAL - move_speed;
    pwm_m2 = -PID_yaw + throttle + ESC_NEUTRAL - move_speed;
    pwm_m4 = -PID_yaw - throttle + ESC_NEUTRAL - move_speed;
    pwm_m5 = -PID_yaw + throttle + ESC_NEUTRAL + move_speed;
  }
  else if (cont_direc == 4) // left
  {
    pwm_m1 = PID_yaw + throttle + ESC_NEUTRAL + move_speed;
    pwm_m2 = -PID_yaw + throttle + ESC_NEUTRAL + move_speed;
    pwm_m4 = -PID_yaw - throttle + ESC_NEUTRAL + move_speed;
    pwm_m5 = -PID_yaw + throttle + ESC_NEUTRAL - move_speed;
  }
  else if (cont_direc == 7) // concon of position
  {
    pwm_m1 = PID_yaw + throttle + ESC_NEUTRAL - Th_0;
    pwm_m2 = -PID_yaw + throttle + ESC_NEUTRAL + Th_1;
    pwm_m4 = -PID_yaw - throttle + ESC_NEUTRAL + Th_2;
    pwm_m5 = -PID_yaw + throttle + ESC_NEUTRAL + Th_3;
  }
  else if (cont_direc == 8) // concon of position
  {
    pwm_m1 = PID_yaw + throttle + ESC_NEUTRAL - darknet_Th_0;
    pwm_m2 = -PID_yaw + throttle + ESC_NEUTRAL + darknet_Th_1;
    pwm_m4 = -PID_yaw - throttle + ESC_NEUTRAL + darknet_Th_2;
    pwm_m5 = -PID_yaw + throttle + ESC_NEUTRAL + darknet_Th_3;
  }

  // pwm_m1 = constrain(pwm_m1, 1100, 1500);
  // pwm_m2 = constrain(pwm_m2, 1500, 1900);
  // pwm_m4 = constrain(pwm_m4, 1100, 1500);
  // pwm_m5 = constrain(pwm_m5, 1500, 1900);

  pwm_m1 = constrain(pwm_m1, ESC_MIN, ESC_MAX);
  pwm_m2 = constrain(pwm_m2, ESC_MIN, ESC_MAX);
  pwm_m4 = constrain(pwm_m4, ESC_MIN, ESC_MAX);
  pwm_m5 = constrain(pwm_m5, ESC_MIN, ESC_MAX);

  esc_input(0x02, pwm_m0, pwm_m1, pwm_m2);
  esc_input(0x03, pwm_m3, pwm_m4, pwm_m5);

  // esc_input(0x02, pwm_m0, pwm_m1, 1500);
  // esc_input(0x03, pwm_m3, 1500, 1500);

  error_pid_yaw1 = error_pid_yaw;
}

void PID_control_depth()
{
  error_pid_depth = desired_angle_depth - depth;
  P_depth = error_pid_depth * P_gain_depth;                                // Inner P control
  D_depth = (error_pid_depth - error_pid_depth1) / T_depth * D_gain_depth; // Inner D control
  I_depth += (error_pid_depth)*T_depth * I_gain_depth;                     // Inner I control
  I_depth = constrain(I_depth, -100, 100);                                 // I control must be limited to prevent being jerk.

  PID_depth = P_depth + D_depth + I_depth;

  pwm_m0 = -PID_depth + ESC_NEUTRAL - DEPTH_BIAS;
  pwm_m3 = -PID_depth + ESC_NEUTRAL - DEPTH_BIAS;

  pwm_m0 = constrain(pwm_m0, DEPTH_PWM_MIN, DEPTH_PWM_MAX);
  pwm_m3 = constrain(pwm_m3, DEPTH_PWM_MIN, DEPTH_PWM_MAX);

  // esc_input(0x02, pwm_m0, pwm_m1, pwm_m2);
  // esc_input(0x03, pwm_m3, pwm_m4, pwm_m5);

  error_pid_depth1 = error_pid_depth;
}
