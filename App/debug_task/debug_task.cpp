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

osThreadId_t Debug_TaskHandle;

uint8_t debug_buffer[9];

Subscriber *sub_debug;
pub_Control_Data debug_pid;

Subscriber *xbox_data;
pub_Xbox_Data xbox_data_pub;

VOFA_Instance_t *vofa_instance = NULL;
Uart_Instance_t *vofa_uart_instance = NULL;
extern uart_package_t VOFA_uart_package;
Encoder_Instance_t *encoder_instance = NULL;
Uart_Instance_t *encoder_uart_instance = NULL;

extern uart_package_t Encoder_uart_package;

int speed_2006 = 0;
int mad_speed = 0;
int32_t motor_pos = 0;
bool RESET_M2006_STATE = false;
#ifdef TEST_VESC

int count = 0;
CAN_Rx_Instance_t VESC_rx_instance1 = {
    .can_handle = &hcan2,
    .RxHeader = {0},
    .rx_len = 8,
    .can_rx_buff = {0},
};

CAN_Tx_Instance_t VESC_tx_instance1 = {
    .can_handle = &hcan2,
    .isExTid = 1,
    .tx_mailbox = 0,
    .tx_len = 8,
    .can_tx_buff = {0},
};

CAN_Rx_Instance_t VESC_rx_instance2 = {
    .can_handle = &hcan2,
    .RxHeader = {0},
    .rx_len = 8,
    .can_rx_buff = {0},
};

CAN_Tx_Instance_t VESC_tx_instance2 = {
    .can_handle = &hcan2,
    .isExTid = 1,
    .tx_mailbox = 0,
    .tx_len = 8,
    .can_tx_buff = {0},
};

CAN_Rx_Instance_t VESC_rx_instance3 = {
    .can_handle = &hcan2,
    .RxHeader = {0},
    .rx_len = 8,
    .can_rx_buff = {0},
};

CAN_Tx_Instance_t VESC_tx_instance3 = {
    .can_handle = &hcan2,
    .isExTid = 1,
    .tx_mailbox = 0,
    .tx_len = 8,
    .can_tx_buff = {0},
};

Motor_Control_Setting_t VESC_motor_ctrl = {0};

PID_t VESC_pid = {
  .Kp = 1,
  .Ki = 0,
  .Kd = 0,
  .MaxOut = 10000,
  .IntegralLimit = 3000,
  .DeadBand = 100,
  .CoefA = 0,
  .CoefB = 0,
  .Output_LPF_RC = 0,
  .Derivative_LPF_RC = 0,
  .OLS_Order = 0,
  .Improve = OutputFilter | Trapezoid_Intergral | Integral_Limit | Derivative_On_Measurement, 
};

VESC vesc[1] = {
    VESC(1, VESC_rx_instance1, VESC_tx_instance1, VESC_motor_ctrl, 0, 1)};

extern Motor_C620 chassis_motor[4];
  double v_now = 0.0;
  double last_v = 0.0;
  double a_now = 0.0;
  double last_a = 0.0;
  double j_now = 0.0;
  uint32_t target__ = 130000; // 目标位置cm
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
extern Motor_C630 m2006[1];
#endif

float wheel_v = 0;
float ref = 0;

#ifdef DEBUG_GO1_MOTOR
extern uint8_t have_start;

CAN_Rx_Instance_t go1_rx_instance = {
    .can_handle = &hcan2,
    .RxHeader = {0},
    .rx_len = 8,
    .can_rx_buff = {0},
};

CAN_Tx_Instance_t go1_tx_instance = {
    .can_handle = &hcan2,
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
float debug_pos = 0.5;
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
  int count = 0;
  
  SCurvePlanner planner(
    0.0, 600,    // 起始/目标位置 (cm)
    0.0, 0.0,     // 起始/结束速度 (rpm)
    780.0, 10000.0,  // 最大速度/加速度 
    100000.0, 1   // 加加速度/期望时间s
);

  double dt = 0.01; // 采样周期
#endif

#ifdef TEST_VESC
  PID_Init(&VESC_pid);
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

encoder_uart_instance = Uart_Register(&Encoder_uart_package);
if (encoder_uart_instance == NULL) {
  LOGERROR("encoder uart register failed!");
  vTaskDelete(NULL);
} 
encoder_instance = Encoder_init(encoder_uart_instance, 10);
if (encoder_instance == NULL) {
  LOGERROR("encoder init failed!");
  vTaskDelete(NULL);
}

#ifdef DEBUG_GO1_MOTOR
  go1_motor[0].GO_Motor_No_Tarque_Ctrl();
  if (!have_start)
    vTaskDelay(5);
    go1_cur_pos = go1_motor[0].real_cur_data.Pos;
    go1_cur_spe = go1_motor[0].real_cur_data.W;
    debug_pos = go1_cur_pos;
    debug_spe = go1_cur_spe;
#endif

  publish_data xbox_;
  xbox_data = register_sub("xbox", 1);
  uint16_t ratio = 8000 / 1024;
  uint16_t ratio1 = 5000 / 1024;

  for (;;) {
    xbox_ = xbox_data->getdata(xbox_data);
    if (xbox_.len != -1) {
      xbox_data_pub = *(pub_Xbox_Data *)xbox_.data;
    }
    // go1_cur_pos = go1_motor[0].real_cur_data.Pos;
    // go1_cur_spe = go1_motor[0].real_cur_data.W;
#ifdef TEST_VESC
    count++;
    // if (xbox_data_pub.btnY)
    // {
    //   motor_pos += 500;
    // }
    // if (xbox_data_pub.btnA)
    // {
    //   motor_pos -= 500;
    // }
    if (xbox_data_pub.btnLB)
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    }
    else
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
    }
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
    if (xbox_data_pub.btnB)
    {
        v_test = planner.update(dt);
        // vesc[0].Rpm_Control(-v_test);
    }
    else if (xbox_data_pub.btnA)
    {
      planner.reset(Encoder_count_);
    }
    else if (xbox_data_pub.btnY)
    {
      vesc[0].Rpm_Control(-780);
    }
    // else if (xbox_data_pub.btnA)
    // {
    //   vesc[0].Rpm_Control(780);
    // }
    // else if(xbox_data_pub.btnB)
    // {
    //   vesc[0].Cur_Control(v_test);
    // }
    else
    {
      vesc[0].Rpm_Control(0);
    }
    // last_v = v_now;
    // v_now = vesc[0].speed;
    // last_a = a_now;
    // a_now = (v_now - last_v) / dt;
    // j_now = (a_now - last_a) / dt;
    // v_test = planner.update(dt);
    // vesc[0].Rpm_Control(speed_2006);
    // vesc[1].Rpm_Control(ratio * xbox_data_pub.trigLT);
    // vesc[2].Rpm_Control(ratio1 * xbox_data_pub.trigLT);
    COMMON_Motor_SendMsgs(vesc);
    Encoder_last_count = Encoder_count_;

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
encoder_instance->Encoder_task(encoder_instance);
LOGINFO("encoder task is running!");

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
    m2006[0].Motor_Ctrl(speed_2006);
    Motor_SendMsgs(m2006);
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
    if (xbox_data_pub.btnDirUp)
    {
      debug_pos = go1_cur_pos + 2.91;
    }
    if (xbox_data_pub.btnDirDown)
    {
      debug_pos = go1_cur_pos + 0.05;
    }
    if (xbox_data_pub.btnB) {
      go1_motor->stop_the_motor();
    }
    if (ABS(debug_pos - go1_motor[0].real_cur_data.Pos) > 0.3)
      {
         debug_kp = 0.15;
         debug_kd =0.02;
         go1_motor[0].GO_Motor_Pos_Ctrl(debug_pos, debug_kp, debug_kd);
      }
      else{
      debug_kp = 3;
      debug_kd =0.08;
      go1_motor[0].GO_Motor_Pos_Ctrl(debug_pos, debug_kp, debug_kd);
      }
    

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
    vTaskDelayUntil(&currentTime, 10);
    // vTaskDelay(5);
  }
}
