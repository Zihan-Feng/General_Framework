/**
 * @file shoot_task.cpp
 * @author STHY
 * @brief
 * @version 0.1
 * @date 2025-5-13
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include "shoot_task.h"
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
#include "pid_controller.h"

#define Simeple_times 15
#define SAMPLE_TIMES 15

osThreadId_t Shoot_TaskHandle;

Subscriber *xbox_data_shoot;
pub_Xbox_Data xbox_data_pub_shoot;

Subscriber *encorder_shoot;
pub_Encoder_Data encorder_data_pub_shoot;
#ifdef VOFA_TO_DEBUG
extern VOFA_Instance_t *vofa_instance;
extern Uart_Instance_t *vofa_uart_instance;
#endif

Subscriber *ros_upper_level;
pub_Upper_level_Control upper_level_data_pub;

Encoder_Instance_t *encoder_instance_shoot = NULL;
Uart_Instance_t *encoder_uart_instance_shoot = NULL;

extern uart_package_t Encoder_uart_package;

Flash_Data_t Shoot_Flash;

//函数声明
uint8_t Wait_stability(float err);
void Encoder_INIT(void);
void Send_Debug_Data_To_Vofa(float target, float distance, float cur_pos);

uint8_t a_66666 = 0;

#ifdef SHOOT_VESC
uint8_t cnt = 0;
float Target_Pos = 0; //发射电机的目标位置(或者说拉伸距离)
float shoot_err = 0;
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

VESC vesc[1] = {
    VESC(1, VESC_rx_instance1, VESC_tx_instance1, VESC_motor_ctrl, 0, 1)};
    
float cur_pos =0;
#endif

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
  

#ifdef SHOOT_GO1_MOTOR

extern uint8_t have_start;
float go_target_pos = 0.5;
float Go_kp = 3;
float Go_kd = 0.08;
float debug_spe = 0;
float go1_initial_pos = 0;
float go1_cur_spe = 0;


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

#ifdef SHOOT_M3508
CAN_Tx_Instance_t m3508_left_pole_tx_instance = {
  .can_handle = &hcan2,
  .isExTid = 0,
  .tx_mailbox = 0,
  .tx_id = 0x200,
  .tx_len = 8,
  .can_tx_buff = {0},
};

CAN_Rx_Instance_t m3508_left_pole_rx_instance = {
  .can_handle = &hcan2,
  .RxHeader = {0},
  .rx_len = 8,
  .rx_id = 0x201,
  .can_rx_buff = {0},
};

Motor_Control_Setting_t m3508_left_pole_control_instance = {
    .motor_controller_setting = {
        .speed_PID = {
            .Kp = 22.5,
            .Ki = 18.9,
            .Kd = 0.065,
            .MaxOut = 10000,
            // 积分项的最大限制 IntegralLimit
            .IntegralLimit = 4000,
            // 死区带宽 DeadBand，用于消除控制误差
            .DeadBand = 5,
            // 输出低通滤波器的 RC 常数 Output_LPF_RC
            .Output_LPF_RC = 0,
            // 微分项低通滤波器的 RC 常数 Derivative_LPF_RC
            .Derivative_LPF_RC = 0,
            // 预测模型阶数 OLS_Order
            .OLS_Order = 0,
            // PID 控制器的改进选项 Improve，包括输出滤波、梯形积分、微分在测量值上、积分限制等
            // .Improve = Feedforward_CONTROLL | OutputFilter | Trapezoid_Intergral | Derivative_On_Measurement | Integral_Limit
            .Improve = OutputFilter | Trapezoid_Intergral | Derivative_On_Measurement | Integral_Limit
        },  
        .pid_ref = 0,
    },
    .outer_loop_type = SPEED_LOOP,// 外环控制为速度环
    .inner_loop_type = SPEED_LOOP,// 内环控制为速度环
    .motor_is_reverse_flag = MOTOR_DIRECTION_REVERSE,// 反转
    .motor_working_status = MOTOR_ENABLED,// 使能电机
};

CAN_Tx_Instance_t m3508_right_pole_tx_instance = {
  .can_handle = &hcan2,
  .isExTid = 0,
  .tx_mailbox = 0,
  .tx_id = 0x200,
  .tx_len = 8,
  .can_tx_buff = {0},
};

CAN_Rx_Instance_t m3508_right_pole_rx_instance = {
  .can_handle = &hcan2,
  .RxHeader = {0},
  .rx_len = 8,
  .rx_id = 0x202,
  .can_rx_buff = {0},
};

Motor_Control_Setting_t m3508_right_pole_control_instance = {
    .motor_controller_setting = {
        .speed_PID = {
            .Kp = 22.5,
            .Ki = 18.9,
            .Kd = 0.065,
            .MaxOut = 10000,
            // 积分项的最大限制 IntegralLimit
            .IntegralLimit = 4000,
            // 死区带宽 DeadBand，用于消除控制误差
            .DeadBand = 5,
            // 输出低通滤波器的 RC 常数 Output_LPF_RC
            .Output_LPF_RC = 0,
            // 微分项低通滤波器的 RC 常数 Derivative_LPF_RC
            .Derivative_LPF_RC = 0,
            // 预测模型阶数 OLS_Order
            .OLS_Order = 0,
            // PID 控制器的改进选项 Improve，包括输出滤波、梯形积分、微分在测量值上、积分限制等
            // .Improve = Feedforward_CONTROLL | OutputFilter | Trapezoid_Intergral | Derivative_On_Measurement | Integral_Limit
            .Improve = OutputFilter | Trapezoid_Intergral | Derivative_On_Measurement | Integral_Limit
        },  
        .pid_ref = 0,
    },
    .outer_loop_type = SPEED_LOOP,// 外环控制为速度环
    .inner_loop_type = SPEED_LOOP,// 内环控制为速度环
    .motor_is_reverse_flag = MOTOR_DIRECTION_NORMAL,// 正转
    .motor_working_status = MOTOR_ENABLED,// 使能电机
};

CCMRAM Motor_C630 pole_motor[2] = {
    Motor_C630(1, m3508_left_pole_rx_instance, m3508_left_pole_tx_instance, 
               m3508_left_pole_control_instance, 16384, -1),
    Motor_C630(2, m3508_right_pole_rx_instance, m3508_right_pole_tx_instance, 
               m3508_right_pole_control_instance, 16384, -1)};

double SPEED_TEST = 0.0;
#endif
uint32_t get_time;
//Shoot_Task
__attribute((noreturn)) void Shoot_Task(void *argument) {

  portTickType currentTime;
  currentTime = xTaskGetTickCount();

  Encoder_INIT();

#ifdef SHOOT_M3508

SCurvePlanner pole1_planner(
    0.0, 520,    // 起始/目标位置 (cm)
    0.0, 0.0,     // 起始/结束速度 (rpm)
    4000.0, 15000.0,  // 最大速度/加速度 
    100000.0, 0.8   // 加加速度/期望时间s
);

SCurvePlanner pole2_planner(
    0.0, 480,    // 起始/目标位置 (cm)
    0.0, 0.0,     // 起始/结束速度 (rpm)
    4000.0, 15000.0,  // 最大速度/加速度 
    100000.0, 0.8   // 加加速度/期望时间s
);

double dt = 0.005; // 采样周期

enum pole_t {
  UPPERING_AUTO,
  LOWERING_AUTO,
  UPPERING_MANUAL,
  LOWERING_MANUAL,
  ARRIVE,
  ARRIVE_2
}pole1 = ARRIVE;
enum pole_t pole2 = ARRIVE;
#endif

#ifdef SHOOT_GO1_MOTOR
  go1_motor[0].GO_Motor_No_Tarque_Ctrl();
  if (!have_start)
  {
    vTaskDelay(5);
  }    
  go1_initial_pos = go1_motor[0].real_cur_data.Pos;
  go1_cur_spe = go1_motor[0].real_cur_data.W;
  go_target_pos = go1_initial_pos+0.35 ;
  debug_spe = go1_cur_spe;
#endif

  publish_data xbox_;
  xbox_data_shoot= register_sub("xbox", 1);

  publish_data ros_upper_level_control_;
  ros_upper_level = register_sub("ros_upper_level_control", 1);

  publish_data encorder_data;
  encorder_shoot = register_sub("Encoder_pub", 1);

  enum state_t {
    RESET,
    MOVE,
    STOP,
    SHOOT,
    WAIT_RESET
  }state = RESET;
  uint8_t shoot_flag = 0;

  PID_Init(&vesc_pos_shoot_pid);
  #ifdef SHOOT_Flash_SAVE

  Flash_LoadArray(Shoot_Flash.shoot_flash_buf, ARRAY_SIZE);
  
  
  Flash_SaveArray(Shoot_Flash.shoot_flash_buf,ARRAY_SIZE);
  #endif
 for (;;) {
   #ifdef VOFA_TO_DEBUG
   if(vofa_instance == NULL)
   {
     vTaskDelay(5);
    }
    #endif
    
    encoder_instance_shoot->Encoder_task(encoder_instance_shoot);
    LOGINFO("encoder task is running!");
    xbox_ = xbox_data_shoot->getdata(xbox_data_shoot);
    if (xbox_.len != -1) {
      xbox_data_pub_shoot = *(pub_Xbox_Data *)xbox_.data;
    }
    encorder_data = encorder_shoot->getdata(encorder_shoot);
    if (encorder_data.len != -1) {
      encorder_data_pub_shoot = *(pub_Encoder_Data *)encorder_data.data;
    }
    
    ros_upper_level_control_ = ros_upper_level->getdata(ros_upper_level);
    if (ros_upper_level_control_.len != -1) {
      upper_level_data_pub = *(pub_Upper_level_Control *)ros_upper_level_control_.data;
    }
    
    #ifdef SHOOT_M3508
    // if (xbox_data_pub_shoot.btnDirUp) upper_level_data_pub.go1_pos = 1;
    // else if (xbox_data_pub_shoot.btnDirDown) upper_level_data_pub.go1_pos = 2;
    // else if (xbox_data_pub_shoot.btnDirLeft) upper_level_data_pub.go1_pos = 4;
    // else if (xbox_data_pub_shoot.btnDirRight) upper_level_data_pub.go1_pos = 3;
    // else upper_level_data_pub.go1_pos = 0;
    if (upper_level_data_pub.go1_pos == 1) {pole1 = UPPERING_AUTO; pole2 = UPPERING_AUTO;}
    else if (upper_level_data_pub.go1_pos == 2) {pole1 = LOWERING_AUTO; pole2 = LOWERING_AUTO;}
    else if (upper_level_data_pub.go1_pos == 3) {pole1 = UPPERING_MANUAL; pole2 = UPPERING_MANUAL;}
    else if (upper_level_data_pub.go1_pos == 4) {pole1 = LOWERING_MANUAL; pole2 = LOWERING_MANUAL;}
    switch (pole1)
    {
      case UPPERING_AUTO:
        SPEED_TEST = pole1_planner.update(dt);
        pole_motor[0].Motor_Ctrl(SPEED_TEST);
        if (pole1_planner.arrived()) pole1 = UPPERING_MANUAL;
        break;
      case LOWERING_AUTO:
        SPEED_TEST = -pole1_planner.update(dt);
        pole_motor[0].Motor_Ctrl(SPEED_TEST);
        if (pole1_planner.arrived()) pole1 = LOWERING_MANUAL;
        break;
      case UPPERING_MANUAL:
        pole_motor[0].Motor_Ctrl(120);
        if (ABS(pole_motor[0].motor_current) > 2000) 
        {
          pole1_planner.reset(0.0); // 重置规划器
          pole1 = ARRIVE;
        }
        break;
      case LOWERING_MANUAL:
        pole_motor[0].Motor_Ctrl(-120);
        if (ABS(pole_motor[0].motor_current) > 2000) 
        {
          pole1_planner.reset(0.0); // 重置规划器
          pole1 = ARRIVE_2;
        }
        break;
      case ARRIVE:
        pole_motor[0].Motor_Ctrl(50);
        break;
      case ARRIVE_2:
        pole_motor[0].Motor_Ctrl(0);
        break;
      default:
        break;
    }
    switch (pole2)
    {
      case UPPERING_AUTO:
        SPEED_TEST = pole2_planner.update(dt);
        pole_motor[1].Motor_Ctrl(SPEED_TEST);
        if (pole2_planner.arrived()) pole2 = UPPERING_MANUAL;
        break;
      case LOWERING_AUTO:
        SPEED_TEST = -pole2_planner.update(dt);
        pole_motor[1].Motor_Ctrl(SPEED_TEST);
        if (pole2_planner.arrived()) pole2 = LOWERING_MANUAL;
        break;
      case UPPERING_MANUAL:
        pole_motor[1].Motor_Ctrl(120);
        if (ABS(pole_motor[1].motor_current) > 4000)
        {
          pole2_planner.reset(0.0); // 重置规划器
          pole2 = ARRIVE;
        }
        break;
      case LOWERING_MANUAL:
        pole_motor[1].Motor_Ctrl(-120);
        if (ABS(pole_motor[1].motor_current) > 4000)
        {
          pole2_planner.reset(0.0); // 重置规划器
          pole2 = ARRIVE_2;
        }
        break;
      case ARRIVE:
        pole_motor[1].Motor_Ctrl(20);
        break;
      case ARRIVE_2:
        pole_motor[1].Motor_Ctrl(0);
        break;
      default:
        break;
    }
    // if (upper_level_data_pub.go1_pos == 1) pole_motor[0].Motor_Ctrl(120);
    // else if (upper_level_data_pub.go1_pos == 2) pole_motor[0].Motor_Ctrl(-120);
    // else pole_motor[0].Motor_Ctrl(0);
    Motor_SendMsgs(pole_motor);
#endif

#ifdef SHOOT_VESC
      
#ifdef VOFA_TO_DEBUG
      vesc_pos_shoot_pid.Kp = vofa_instance->pub_data.Kp;
      vesc_pos_shoot_pid.Ki = vofa_instance->pub_data.Ki;
      vesc_pos_shoot_pid.IntegralLimit = vofa_instance->pub_data.i_limit;
#endif
      switch (state)
      {
        case RESET://发射机构回位,Targer的单位为m
        //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
        //注意这重置零位之后需要重新校准
        Target_Pos = -0.008f;//发射机构零位置
        if(fabs(encorder_data_pub_shoot.distance - Target_Pos) < 0.002
          && encorder_data_pub_shoot.distance != 0)
          {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);//气缸立起，卡主发射机构
            Target_Pos = encorder_data_pub_shoot.distance;
        //cylinder是气缸的意思，但是修改了逻辑之后，这里已经没有控制气缸的意思了
        if(upper_level_data_pub.band_pos)
        {
          state = MOVE;
          Target_Pos = upper_level_data_pub.band_pos/100.0f;
          upper_level_data_pub.cylinder = 0;
        }  
      }
      break;
    case MOVE:
#ifdef VOFA_TO_DEBUG
      Target_Pos = vofa_instance->pub_data.ref;
#endif
      //  Target_Pos = 0.053;
      
      if(Target_Pos>0.32501||Target_Pos<0.001)
      {
        Target_Pos = 0;
        LOGERROR("Target_Pos error!");
        break;
      }
      shoot_err = Target_Pos - encorder_data_pub_shoot.distance;
      if( 1 == Wait_stability(shoot_err))
      {
          state = SHOOT;
      }
      break;
    
    case SHOOT:
      if(1)
      {
      //  Target_Pos = encorder_data_pub_shoot.distance;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);//气缸回缩，发射机构释放
        state = WAIT_RESET;
        get_time = HAL_GetTick();
        //只是作为记录，用于查看发送瞬间编码器的值（因为只需在这个时刻记录，所以单开了一个变量）
        cur_pos = encorder_data_pub_shoot.distance;
        //vofa_data_pub.shoot_flag = 0;
      }
      break;
    case WAIT_RESET:
      if(HAL_GetTick()-get_time>100)
      {
        #ifdef SHOOT_Flash_SAVE
        uint8_t index = (uint8_t)((Target_Pos*100-10)/2.5);
        index = (index > 7) ? 7 : index;
        Shoot_Flash.shoot_flash_buf[index]++;
        Shoot_Flash.Flash_Mark++;
        if(Shoot_Flash.Flash_Mark>=3)
        {
          Shoot_Flash.Flash_Mark = 0;
          Flash_SaveArray(Shoot_Flash.shoot_flash_buf,ARRAY_SIZE);
          LOGINFO("shoot num:\r\n");
          for(int i=0;i<ARRAY_SIZE;i++)
          {
            LOGINFO("%d\r\n",Shoot_Flash.shoot_flash_buf[i]);
          }
          LOGINFO("\r\n");
        }
        #endif
        state = RESET;
      }
    break;
    default:
      break;
    }
    PID_Calculate(&vesc_pos_shoot_pid,encorder_data_pub_shoot.distance, Target_Pos);
    vesc[0].Rpm_Control(vesc_pos_shoot_pid.Output);
    COMMON_Motor_SendMsgs(vesc);   

    Send_Debug_Data_To_Vofa(Target_Pos,
      encorder_data_pub_shoot.distance, 
      cur_pos);

#endif  
    
#ifdef SHOOT_GO1_MOTOR
    // if (xbox_data_pub_shoot.btnDirUp) {
    //   go_target_pos += 0.005;
    //   go1_motor[0].GO_Motor_Pos_Ctrl(go_target_pos, Go_kp, Go_kd);
    // } else if (xbox_data_pub_shoot.btnDirDown) {
    //   go_target_pos -= 0.005;
    //   go1_motor[0].GO_Motor_Pos_Ctrl(go_target_pos, Go_kp, Go_kd);
    // } else if (xbox_data_pub_shoot.btnB) {
    //   go1_motor->stop_the_motor();
    // }

    //设置位置
    if (upper_level_data_pub.go1_pos == 1)
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
      go_target_pos = go1_initial_pos + 3.43;//这里的2.83是弧度制
      upper_level_data_pub.go1_pos = 0;
    }
    else if (upper_level_data_pub.go1_pos == 2)
    { 
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
      go_target_pos = go1_initial_pos + 0.35;//回到初始点（因为是相对坐标）
      upper_level_data_pub.go1_pos = 0;
    }
    else if (upper_level_data_pub.go1_pos == 3)
    {
        go_target_pos -= 0.01;
        upper_level_data_pub.go1_pos = 0;
    }
    else if (upper_level_data_pub.go1_pos == 4)
    {
        go_target_pos += 0.01;
        upper_level_data_pub.go1_pos = 0;
    }
    //控制电机
    // if (xbox_data_pub_shoot.btnB) {
    //   go1_motor->stop_the_motor();
    // }
    if (go_target_pos > 3.5) 
    {
      go_target_pos = 3.5;
    }
    if (ABS(go_target_pos - go1_motor[0].real_cur_data.Pos) > 0.5)
    {
       Go_kp = 0.15;
       Go_kd =0.02;
       go1_motor[0].GO_Motor_Pos_Ctrl(go_target_pos, Go_kp, Go_kd);
    }
    else{
       Go_kp = 2;
       Go_kd =0.18;
       go1_motor[0].GO_Motor_Pos_Ctrl(go_target_pos, Go_kp, Go_kd);
    }
    // if (xbox_data_pub_shoot.btnDirUp)
    // {
    //   go_target_pos = go1_initial_pos + 2.83;
    // }
    // if (xbox_data_pub_shoot.btnDirDown)
    // { 
    //   go_target_pos = go1_initial_pos + 0.05;
    // }
    // if (xbox_data_pub_shoot.btnB) {
    //   go1_motor->stop_the_motor();
    // }
    // if (ABS(go_target_pos - go1_motor[0].real_cur_data.Pos) > 0.3)
    //   {
    //      Go_kp = 0.15;
    //      Go_kd =0.02;
    //      go1_motor[0].GO_Motor_Pos_Ctrl(go_target_pos, Go_kp, Go_kd);
    //   }
    //   else{
    //      Go_kp = 3;
    //      Go_kd =0.08;
    //      go1_motor[0].GO_Motor_Pos_Ctrl(go_target_pos, Go_kp, Go_kd);
    //   }
#endif
    vTaskDelayUntil(&currentTime, 5);
}

}
#define Sampling_Width 0.005
enum Wait_stability_state{
    Wait_Reach = 0,
    Sampling = 1,
    Check = 2,
}wait_status;

uint8_t Wait_stability(float err)
{
  static uint8_t cnt = 0;
  static uint8_t acceptable_cnt = 0;
  switch (wait_status)
  {
    case Wait_Reach:
      if(err < Sampling_Width && err > -Sampling_Width)
      {
        LOGINFO("Reach at %d",HAL_GetTick());
        a_66666 = 1;
        wait_status = Sampling;
        cnt = 0;
        acceptable_cnt = 0;
      }
    break;
    case Sampling:      
      cnt++;
      if(err < Sampling_Width && err > -Sampling_Width)
      {
        acceptable_cnt++;
      }
      if(cnt >= SAMPLE_TIMES)
      {
        wait_status = Check;
      }
    break;
    case Check:
      wait_status = Wait_Reach;
      float Ratio_valid_values = (float)acceptable_cnt / cnt;
      if(Ratio_valid_values>0.8)
      {
        LOGINFO("Check passe at %d",HAL_GetTick());
        a_66666 = 0;
        return 1;
      }
      else
      {
        LOGINFO("Check failed at %d",HAL_GetTick());
      }
    break;
  }
  return 0;
}

void Encoder_INIT(void)
{
   encoder_uart_instance_shoot = Uart_Register(&Encoder_uart_package);
  if (encoder_uart_instance_shoot == NULL) 
  {
    LOGERROR("encoder uart register failed!");
    vTaskDelete(NULL);
  } 
  encoder_instance_shoot = Encoder_init(encoder_uart_instance_shoot, 10);
  if (encoder_instance_shoot == NULL) 
  {
    LOGERROR("encoder init failed!");
    vTaskDelete(NULL);
  }
  LOGINFO("Encoder init success!");
}

void Send_Debug_Data_To_Vofa(float target, float distance, float cur_pos) {
#ifdef VOFA_TO_DEBUG
    if (vofa_instance == NULL) return;

    taskENTER_CRITICAL();
    vofa_instance->vofa_send_float(target);
    vofa_instance->vofa_send_float(distance);
    vofa_instance->vofa_send_float(cur_pos);
    vofa_instance->vofa_send_end();
    taskEXIT_CRITICAL();
#endif
}

