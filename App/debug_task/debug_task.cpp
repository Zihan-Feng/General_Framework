/**
 * @file debug.cpp
 * @author Keten (2863861004@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-10-07
 *
 * @copyright Copyright (c) 2024
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include "debug_task.h"
#include "air_joy.h"
#include "arm_math.h"
#include "bsp_bitband.h"
#include "bsp_usart.h"
#include "data_type.h"
#include "topics.h"
#include "vofa.h"
#include "xbox.h"
#include "pathplanning.h"
#include "encoder.h"
#ifdef CHASSIS_TO_DEBUG
#include "pid_controller.h"
#endif
// extern Motor_C610 m2006;

#define Simeple_times 15

osThreadId_t Debug_TaskHandle;

uint8_t debug_buffer[9];

Subscriber *sub_debug;
pub_Control_Data debug_pid;

// Subscriber *xbox_data;
// pub_Xbox_Data xbox_data_pub;

// Subscriber *ros_upper_level;
// pub_Upper_level_Control upper_level_data_pub;

// Subscriber *encorder_shoot;
// pub_Encoder_Data encorder_data_pub_shoot;

VOFA_Instance_t *vofa_instance = NULL;
Uart_Instance_t *vofa_uart_instance = NULL;
extern uart_package_t VOFA_uart_package;
Encoder_Instance_t *encoder_instance = NULL;
Uart_Instance_t *encoder_uart_instance = NULL;

extern uart_package_t Encoder_uart_package;

uint8_t Wait_stability(float err);

int speed_2006 = 0;
int mad_speed = 0;
int32_t motor_pos = 0;
bool RESET_M2006_STATE = false;
float vx = 0.0;
float vy = 0.0;
float w = 0.0;
float angle_1 = 0.0;
float angle_2 = 0.0;
float v_1 = 0.0;
float v_2 = 0.0;
float r_ = 0.163;
#ifdef TEST_VESC



CAN_Rx_Instance_t VESC_rx_instance1 = {
    .can_handle = &hcan1,
    .RxHeader = {0},
    .rx_len = 8,
    .can_rx_buff = {0},
};

CAN_Tx_Instance_t VESC_tx_instance1 = {
    .can_handle = &hcan1,
    .isExTid = 1,
    .tx_mailbox = 0,
    .tx_len = 8,
    .can_tx_buff = {0},
};

// CAN_Rx_Instance_t VESC_rx_instance2 = {
//     .can_handle = &hcan2,
//     .RxHeader = {0},
//     .rx_len = 8,
//     .can_rx_buff = {0},
// };

// CAN_Tx_Instance_t VESC_tx_instance2 = {
//     .can_handle = &hcan2,
//     .isExTid = 1,
//     .tx_mailbox = 0,
//     .tx_len = 8,
//     .can_tx_buff = {0},
// };

// CAN_Rx_Instance_t VESC_rx_instance3 = {
//     .can_handle = &hcan2,
//     .RxHeader = {0},
//     .rx_len = 8,
//     .can_rx_buff = {0},
// };

// CAN_Tx_Instance_t VESC_tx_instance3 = {
//     .can_handle = &hcan2,
//     .isExTid = 1,
//     .tx_mailbox = 0,
//     .tx_len = 8,
//     .can_tx_buff = {0},
// };

Motor_Control_Setting_t VESC_motor_ctrl = {0};

  PID_t vesc_pos_shoot_pid={
    .Kp = 34060,
    .Ki = 21000,
    .Kd = 0,
    .MaxOut = 7000,
    .IntegralLimit = 120,
    .DeadBand = 0,
    .CoefA = 0,
    .CoefB = 0,
    .Output_LPF_RC = 0,
    .Derivative_LPF_RC = 0,
    .Improve = Integral_Limit,
  };
    PID_t vesc_pos_reset_pid={
    .Kp = 34060,
    .Ki = 21000,
    .Kd = 0,
    .MaxOut = 7000,
    .IntegralLimit = 120,
    .DeadBand = 0.01,
    .CoefA = 0,
    .CoefB = 0,
    .Output_LPF_RC = 0,
    .Derivative_LPF_RC = 0,
    .Improve = Integral_Limit,
  };
  uint8_t cnt = 0;

VESC vesc[1] = {
    VESC(1, VESC_rx_instance1, VESC_tx_instance1, VESC_motor_ctrl, 0, 1),
    // VESC(2, VESC_rx_instance2, VESC_tx_instance2, VESC_motor_ctrl, 0, 1),
    // VESC(3, VESC_rx_instance3, VESC_tx_instance3, VESC_motor_ctrl, 0, 1)
};

extern Motor_C620 chassis_motor[4];
  double v_now = 0.0;
  double last_v = 0.0;
  double a_now = 0.0;
  double last_a = 0.0;
  double j_now = 0.0;
  uint32_t target__ = 130000; // 目标位置cm

float Target_Pos = 0.012f;
enum state_t {
    WAITE,   // 等待装载或准备就绪
    MOVE,    // 正在移动至装载位置
    READY,   // 装载到位，等待射球指令
    SHOOT,   // 射击中
    RELOAD   // 射击完成后返回原点
} state;
  uint8_t shoot_flag = 0;
  uint8_t shoot_count = 0;
  float debug_band_pos = 0.0f;
#endif

#ifdef TEST_DM
CAN_Rx_Instance_t dm_rx_instance = {
    .can_handle = &hcan1,
    .RxHeader = {0},
    .rx_len = 6,
    .can_rx_buff = {0},
};

CAN_Tx_Instance_t dm_tx_instance = {
    .can_handle = &hcan1,
    .isExTid = 0,
    .tx_mailbox = 0,
    .tx_id = 0x08,
    .tx_len = 8,
    .can_tx_buff = {0},
};

Motor_Control_Setting_t DM_motor_ctrl = {0};

DM_motor dm[1] = {
    DM_motor(8, dm_rx_instance, dm_tx_instance, DM_motor_ctrl,_POS_with_SPEED_CONTROL,0, 10)
};

float dm_pos = 0;
float dm_speed = 10;
#endif

#ifdef TEST_SYSTEM_TURNER
extern Motor_C620 chassis_motor[4];
extern Motor_GM6020 gm6020[1];
#endif

#ifdef TEST_SYSTEM_M2006
extern Motor_C630 m2006[2];
#endif

float wheel_v = 0;
float ref = 0;

#ifdef DEBUG_GO1_MOTOR
extern uint8_t have_start;
uint8_t go1_motor_flag = 0;

CAN_Rx_Instance_t go1_rx_instance = {
    .can_handle = &hcan1,
    .RxHeader = {0},
    .rx_len = 8,
    .can_rx_buff = {0},
};

CAN_Tx_Instance_t go1_tx_instance = {
    .can_handle = &hcan1,
    .isExTid = 1,
    .tx_mailbox = 0,
    .tx_len = 8,
    .can_tx_buff = {0},
};

Motor_Control_Setting_t go1_motor_ctrl = {0};
// 较为特殊的go1电机，有些选项不需要配置！
GO_M8010 go1_motor[1] = {
    GO_M8010(0, go1_rx_instance, go1_tx_instance, go1_motor_ctrl, 0, -1, 3)};
#endif

#ifdef TEST_SYSTEM_TURNER
uint16_t sample_rate = 5000;
uint16_t frequency = 1;
uint16_t num_samples = 100;
float32_t sine = 0;
float32_t phase_increment = 2 * PI * frequency / sample_rate;
float32_t phase = 0.0f;
float ref_temp = 0;
float speed_aps = 0;

#endif

#ifdef DEBUG_GO1_MOTOR
float debug_pos = 0.67;
float debug_kp = 3;
float debug_kd = 0.08;
float debug_spe = 0;
float go1_cur_pos = 0;
float go1_cur_spe = 0;
#endif
double v_test = 0.0;
__attribute((noreturn)) void Debug_Task(void *argument) {
  portTickType currentTime;
  currentTime = xTaskGetTickCount();

#ifdef TEST_SYSTEM_TURNER
  
  SCurvePlanner planner(
    0.0, 600,    // 起始/目标位置 (cm)
    0.0, 0.0,     // 起始/结束速度 (rpm)
    780.0, 10000.0,  // 最大速度/加速度 
    100000.0, 1   // 加加速度/期望时间s
);

  double dt = 0.01; // 采样周期
#endif

#ifdef TEST_VESC
  // state = WAITE;
  // PID_Init(&vesc_pos_shoot_pid);
  // PID_Init(&vesc_pos_reset_pid);
#endif 

#ifdef VOFA_TO_DEBUG
  /* vofa设备创建 */
  vofa_uart_instance = Uart_Register(&VOFA_uart_package);
  if (vofa_uart_instance == NULL) {
    LOGERROR("vofa uart register failed!");
    vTaskDelete(NULL);
  }
  vofa_instance = VOFA_init(vofa_uart_instance, 10);
  if (vofa_instance == NULL) {
    LOGERROR("vofa init failed!");
    vTaskDelete(NULL);
  }
#endif

// encoder_uart_instance = Uart_Register(&Encoder_uart_package);
// if (encoder_uart_instance == NULL) {
//   LOGERROR("encoder uart register failed!");
//   vTaskDelete(NULL);
// } 
// encoder_instance = Encoder_init(encoder_uart_instance, 10);
// if (encoder_instance == NULL) {
//   LOGERROR("encoder init failed!");
//   vTaskDelete(NULL);
// }

#ifdef DEBUG_GO1_MOTOR
  // go1_motor[0].GO_Motor_No_Tarque_Ctrl();
  // if (!have_start)
  //   vTaskDelay(5);
  //   go1_cur_pos = go1_motor[0].real_cur_data.Pos;
  //   go1_cur_spe = go1_motor[0].real_cur_data.W;
  //   debug_pos = go1_cur_pos;
  //   debug_spe = go1_cur_spe;
#endif

  // publish_data xbox_;
  // xbox_data = register_sub("xbox", 1);
  // publish_data ros_upper_level_control_;
  // ros_upper_level = register_sub("ros_upper_level_control", 1);
  // publish_data encorder_data;
  // encorder_shoot = register_sub("Encoder_pub", 1);
  // uint16_t ratio = 8000 / 1024;
  // uint16_t ratio1 = 5000 / 1024;

  for (;;) {
    // encoder_instance->Encoder_task(encoder_instance);
    // LOGINFO("encoder task is running!");
    // xbox_ = xbox_data->getdata(xbox_data);
    // if (xbox_.len != -1) {
    //   xbox_data_pub = *(pub_Xbox_Data *)xbox_.data;
    // }
    // ros_upper_level_control_ = ros_upper_level->getdata(ros_upper_level);
    // if (ros_upper_level_control_.len != -1) {
    //   upper_level_data_pub = *(pub_Upper_level_Control *)ros_upper_level_control_.data;
    // }
    // encorder_data = encorder_shoot->getdata(encorder_shoot);
    // if (encorder_data.len != -1) {
    //   encorder_data_pub_shoot = *(pub_Encoder_Data *)encorder_data.data;
    // }
    // count++;

    // go1_cur_pos = go1_motor[0].real_cur_data.Pos;
    // go1_cur_spe = go1_motor[0].real_cur_data.W;
#ifdef TEST_VESC
#ifdef TEST_SWERVE
    if (ABS(xbox_data_pub.joyLVert-32768) < 4000)vy = 0;
    else vy = -(xbox_data_pub.joyLVert-32768)/32768.0*3;
    if (ABS(xbox_data_pub.joyLHori-32768) < 4000)vx = 0;
    else vx = (xbox_data_pub.joyLHori-32768)/32768.0*3;
    if (ABS(xbox_data_pub.joyRHori-32768) < 4000)w = 0;
    else w = (xbox_data_pub.joyRHori-32768)/32768.0*3;
    angle_1 = atan2(vx,vy+w*r_)*180/PI;//front_wheel_angle = front_wheel_angle > M_PI_2 ? front_wheel_angle - M_PI : front_wheel_angle < -M_PI_2 ? front_wheel_angle + M_PI : front_wheel_angle;
    v_1 = sqrt(vx*vx+(vy+w*r_)*(vy+w*r_));
    // if (ABS(angle_1) > 90)
    // {
    //   angle_1 = angle_1 > 90 ? angle_1 - 180 : angle_1 + 180;
    //   v_1 = -v_1;
    // }
    angle_2 = atan2(vx,vy-w*r_)*180/PI;
    v_2 = sqrt(vx*vx+(vy-w*r_)*(vy-w*r_));
    // if (ABS(angle_2) > 90)
    // {
    //   angle_2 = angle_2 > 90 ? angle_2 - 180 : angle_2 + 180;
    //   v_2 = -v_2;
    // }
    // m2006[0].Motor_Ctrl(angle_1);
    // m2006[1].Motor_Ctrl(angle_2);
    // Motor_SendMsgs(m2006);
    // vesc[0].Rpm_Control(v_1*500);
    // vesc[1].Rpm_Control(v_2*500);
    // COMMON_Motor_SendMsgs(vesc);
#else
    // if (xbox_data_pub.btnY)
    // {
    //   motor_pos += 500;
    // }
    // if (xbox_data_pub.btnA)
    // {
    //   motor_pos -= 500;
    // }
    // if (xbox_data_pub.btnLB)
    // {
    //   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    // }
    // else
    // {
    //   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
    // }
    // if (xbox_data_pub.btnB)
    // {
    //   if(Encoder_count_ < motor_pos)
    //   {
        // v_test = planner.update(dt);
        // vesc[0].Rpm_Control(-v_test);
    //   }
    //   else
    //   {
    //     vesc[0].Rpm_Control(0);
    //     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    //   }
    // }
    // else
    // {
    //   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    //   if (Encoder_count_ > 20000)
    //   {
    //     vesc[0].Rpm_Control(780);
    //   }
    //   else if (Encoder_count_ > 16000)
    //   {
    //     vesc[0].Rpm_Control(130); 
    //   }
    //   else
    //   {
    //     vesc[0].Rpm_Control(0);
    //   }

    // }
    // v_now = (Encoder_count_-Encoder_last_count);
    // v_test = -PID_Calculate(&VESC_pid,Encoder_count_ , 130000);
    // if (xbox_data_pub.btnB)
    // {
    //     v_test = planner.update(dt);
        // vesc[0].Rpm_Control(-v_test);
    // }
    // else if (xbox_data_pub.btnA)
    // {
    //   planner.reset(Encoder_count_);
    // }
    // else if (xbox_data_pub.btnY)
    // {
    //   vesc[0].Rpm_Control(-780);
    // }
    // else if (xbox_data_pub.btnA)
    // {
    //   vesc[0].Rpm_Control(780);
    // }
    // else if(xbox_data_pub.btnB)
    // {
    //   vesc[0].Cur_Control(v_test);
    // }
    // else
    // {
    //   vesc[0].Rpm_Control(0);
    // }
    // last_v = v_now;
    // v_now = vesc[0].speed;
    // last_a = a_now;
    // a_now = (v_now - last_v) / dt;
    // j_now = (a_now - last_a) / dt;
    // v_test = planner.update(dt);
    // vesc[0].Rpm_Control(speed_2006);
    // vesc[1].Rpm_Control(ratio * xbox_data_pub.trigLT);
    // vesc[2].Rpm_Control(ratio1 * xbox_data_pub.trigLT);
    // if (xbox_data_pub.btnLB) upper_level_data_pub.shoot = 1;
    // else upper_level_data_pub.shoot = 0;
    // if (xbox_data_pub.btnY) debug_band_pos += 0.01;
    // else if (xbox_data_pub.btnB) debug_band_pos -= 0.01;
    // if (xbox_data_pub.btnA) upper_level_data_pub.band_pos = debug_band_pos;
    // else upper_level_data_pub.band_pos = 0;



    // if (upper_level_data_pub.cylinder) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    // else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
    // float err = 0.0f;
    // switch(state) {
    //     case RELOAD:
    //       Target_Pos = 0.012f;
    //       // 归零完成后回到等待状态
    //       if (encorder_data_pub_shoot.distance <= 0.016 && encorder_data_pub_shoot.distance >= 0.001) {
    //           state = WAITE;
    //       }
    //       break;
    //     case WAITE:
    //         // 当处于等待状态时，接收新的装载位置并开始移动
    //         if (upper_level_data_pub.band_pos > 0.01 && upper_level_data_pub.band_pos <= 30.0) {
    //             Target_Pos = upper_level_data_pub.band_pos / 100.0f; // cm转m
    //             state = MOVE;
    //         }
    //         break;

    //     case MOVE:
    //         // 确认到达目标位置（误差小于5mm）
    //         err = Target_Pos - encorder_data_pub_shoot.distance;
    //         if( 1 == Wait_stability(err))
    //         {
    //             state = READY;
    //         }
    //         break;

    //     case READY:
    //         // 接收到射球指令时激活射击机构
    //         // if (upper_level_data_pub.shoot == 1) {
    //         //     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    //         //     shoot_flag = 1;
                
    //         // }
    //         // else if (upper_level_data_pub.shoot == 0 && shoot_flag == 1) {
    //         //     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    //         //     shoot_flag = 0;
    //         //     state = SHOOT;
    //         // }
    //         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    //         // shoot_flag = 1;
    //         shoot_count++;
    //         if (shoot_count == 50 /*upper_level_data_pub.shoot == 1 && shoot_flag == 1*/) {
    //             HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    //             // shoot_flag = 0;
    //             shoot_count = 0;
    //             state = SHOOT;
    //         }
    //         break;

    //     case SHOOT:
    //         // 立即释放射击信号并开始返回原点
    //         state = RELOAD;
    //         break;

    // }

    // // 始终执行PID控制
    // if (ABS(Target_Pos - 0.012f) < 0.0001f && state == WAITE) 
    // {
    //   // PID_Calculate(&vesc_pos_reset_pid, encorder_data_pub_shoot.distance, Target_Pos);
    //   // vesc[0].Rpm_Control(vesc_pos_reset_pid.Output);
    //   vesc[0].Rpm_Control(0);
    // }
    // else 
    // {
    //   PID_Calculate(&vesc_pos_shoot_pid, encorder_data_pub_shoot.distance, Target_Pos);
    //   vesc[0].Rpm_Control(vesc_pos_shoot_pid.Output);
    // }
    // COMMON_Motor_SendMsgs(vesc);
#endif

#endif
#ifdef TEST_SYSTEM_M2006
    // RESET_M2006_STATE = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
    // speed_2006 = (int)(xbox_data_pub.joyRHori - 32767) / 32767.0f * 5000;
    // if(ABS(xbox_data_pub.joyRHori - 32767) < 3000)m2006[0].Motor_Ctrl(0);
    // else m2006[0].Motor_Ctrl(speed_2006);
    // Motor_SendMsgs(m2006);
#endif

#ifdef TEST_ONE_SWERVE
    mad_speed = (int)(xbox_data_pub.joyLVert - 32767)/32768.0f * 8000;
    if(ABS(xbox_data_pub.joyLVert - 32767) < 3000)vesc[0].Rpm_Control(0);
    else vesc[0].Rpm_Control(mad_speed);
    COMMON_Motor_SendMsgs(vesc);
#endif // !TEST_ONE_S

#ifdef TEST_DM
    if (xbox_data_pub.btnDirLeft)
    {
      dm_pos += 0.05;
      dm[0].POS_with_SPEED_CONTROL(dm_pos,dm_speed);
    } else if (xbox_data_pub.btnDirRight) {
      dm_pos -= 0.05;
      dm[0].POS_with_SPEED_CONTROL(dm_pos,dm_speed);
    } else if (xbox_data_pub.btnB) {
      dm[0].DM_MOTOR_ZERO_POSITION();
    } else if (xbox_data_pub.btnY) {
      dm[0].DM_MOTOR_ENABLE();
    }
    COMMON_Motor_SendMsgs(dm);
#endif

#ifdef VOFA_TO_DEBUG
    vofa_instance->vofa_task(vofa_instance);
    LOGINFO("debug task is running!");
#endif


#ifdef TEST_SYSTEM_TURNER
    float32_t sine_value = arm_sin_f32(phase);
    sine = sine_value * 2000;
    /* 更新相位 */
    phase += phase_increment;
    if (phase >= 2 * PI) {
      phase -= 2 * PI;
    }

    
#ifdef TEST_SYSTEM_M3508
    count++;
    if (count <= 3000) {
      chassis_motor[3].Motor_Ctrl(0);
      Motor_SendMsgs(chassis_motor);
    } else if (count > 3000 && count <= 6000) {
      chassis_motor[3].Motor_Ctrl(2000);
      Motor_SendMsgs(chassis_motor);
    } else if (count > 6000 && count <= 9000) {
      chassis_motor[3].Motor_Ctrl(0);
      Motor_SendMsgs(chassis_motor);
    } else {
      count = 0;
    }
    speed_aps = chassis_motor[3].speed_aps;

#endif

#ifdef TEST_SYSTEM_M2006
    // m2006[0].Motor_Ctrl(sine);
    // Motor_SendMsgs(m2006);
    // count++;
    // if (count <= 400) {
    //   planner.reset();
    //   m2006[0].Motor_Ctrl(0);
    //   Motor_SendMsgs(m2006);
    // } else if (count > 400 && count <= 800) {
    //   v_test = planner.update(dt);
    //   m2006[0].Motor_Ctrl(v_test);
    //   Motor_SendMsgs(m2006);
    // } else if (count > 800 && count <= 1200) {
    //   m2006[0].Motor_Ctrl(0);
    //   Motor_SendMsgs(m2006);
    // } else {
    //   count = 0;
    // }
    // m2006[0].Motor_Ctrl(speed_2006);
    // Motor_SendMsgs(m2006);
#endif

#ifdef TEST_SYSTEM_GM6020
    count++;
    if (count <= 3000) {
      ref = 0;
      gm6020[0].Motor_Ctrl(0);
      Motor_SendMsgs(gm6020);
    } else if (count > 3000 && count <= 6000) {
      ref = 180;
      gm6020[0].Motor_Ctrl(180);
      Motor_SendMsgs(gm6020);
    } else if (count > 6000 && count <= 9000) {
      ref = 0;
      gm6020[0].Motor_Ctrl(0);
      Motor_SendMsgs(gm6020);
    } else {
      count = 0;
    }

#endif

#endif

#ifdef DEBUG_GO1_MOTOR
    // if (xbox_data_pub.btnY) {
    //   debug_pos += 0.01;
    //   // debug_pos = go1_cur_pos + 1.1;
    // } else if (xbox_data_pub.btnA) {
    //   debug_pos -= 0.01;
    // }
    // if (xbox_data_pub.btnDirUp) upper_level_data_pub.go1_pos = 1;
    // else if (xbox_data_pub.btnDirDown) upper_level_data_pub.go1_pos = 2;
    // else if (xbox_data_pub.btnDirLeft) upper_level_data_pub.go1_pos = 3;
    // else if (xbox_data_pub.btnDirRight) upper_level_data_pub.go1_pos = 4;
    // else upper_level_data_pub.go1_pos = 0;

    
    // if (upper_level_data_pub.go1_pos == 1 || upper_level_data_pub.go1_pos == 2) go1_motor_flag = 1;
    // else if (upper_level_data_pub.go1_pos == 3 || upper_level_data_pub.go1_pos == 4) go1_motor_flag = 2;
    // if (go1_motor_flag == 1)
    // {
    //   if (upper_level_data_pub.go1_pos == 1)
    //   {
    //     debug_pos = go1_cur_pos + 2.83;
    //   }
    //   else if (upper_level_data_pub.go1_pos == 2)
    //   {
    //     debug_pos = go1_cur_pos + 0.1;
    //   }
    //   if (debug_pos > 4.2) debug_pos = 4.2;
    //   // if (xbox_data_pub.btnB) {
    //   //   go1_motor->stop_the_motor();
    //   // }
    //   if (ABS(debug_pos - go1_motor[0].real_cur_data.Pos) > 0.3)
    //   {
    //      debug_kp = 0.2;
    //      debug_kd = 0.021;
    //      go1_motor[0].GO_Motor_Pos_Ctrl(debug_pos, debug_kp, debug_kd);
    //   }
    //   else{
    //     debug_kp = 3;
    //     debug_kd = 0.08;
    //     go1_motor[0].GO_Motor_Pos_Ctrl(debug_pos, debug_kp, debug_kd);
    //   }
    // }
    // else if (go1_motor_flag == 2)
    // {
    //   debug_kp = 3;
    //   debug_kd = 0.08;
    //   if (upper_level_data_pub.go1_pos == 3)
    //   {
    //     debug_pos -= 0.005;
    //   }
    //   else if (upper_level_data_pub.go1_pos == 4)
    //   {
    //     debug_pos += 0.005;
    //   }
    //   if (debug_pos > 4.2) debug_pos = 4.2;
    //   go1_motor[0].GO_Motor_Pos_Ctrl(debug_pos, debug_kp, debug_kd);
    // }
    

    // if (debug <= 5000 || debug >= 10000) {
    // go1_motor[0].GO_Motor_Speed_Ctrl(5, 0.05);
    //   go1_motor[0].GO_Motor_Pos_Ctrl(debug_pos, debug_kp, debug_kd);
    //   // go1_motor[0].GO_Motor_No_Tarque_Ctrl();
    //   debug++;
    // } else {
    //   go1_motor[0].GO_Motor_No_Tarque_Ctrl();
    //   debug++;
    // }
#endif
    vTaskDelayUntil(&currentTime, 5);
    // vTaskDelay(5);
  }
}

// enum Wait_stability_state{
//     Wait_Reach = 0,
//     Sampling = 1,
//     Check = 2,
// }wait_status;

// uint8_t Wait_stability(float err)
// {
//   static uint8_t cnt = 0;
//   static uint8_t acceptable_cnt = 0;
//   switch (wait_status)
//   {
//     case Wait_Reach:
//       if(err < 0.005 && err > -0.005)
//       {
//         LOGINFO("Reach at %d",HAL_GetTick());
//         wait_status = Sampling;
//         cnt = 0;
//         acceptable_cnt = 0;
//       }
//     break;
//     case Sampling:      
//       cnt++;
//       if(err < 0.005 && err > -0.005)
//       {
//         acceptable_cnt++;
//       }
//       if(cnt >= Simeple_times)
//       {
//         wait_status = Check;
//       }
//     break;
//     case Check:
//       wait_status = Wait_Reach;
//       float Ratio_valid_values = (float)acceptable_cnt / cnt;
//       if(Ratio_valid_values>0.8)
//       {
//         LOGINFO("Check passe at %d",HAL_GetTick());
//         return 1;
//       }
//       else
//       {
//         LOGINFO("Check failed at %d",HAL_GetTick());
//       }
//     break;
//   }
//   return 0;
// }