/**
 * @file referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "referee_task.h"
#include "robot_def.h"
#include "rm_referee.h"
#include "referee_UI.h"
#include "string.h"
#include "cmsis_os.h"
#include "custom_controller.h"
#include "message_center.h"

static Referee_Interactive_info_t *Interactive_data; // UI绘制需要的机器人状态数据
static referee_info_t *referee_recv_info;            // 接收到的裁判系统数据
uint8_t UI_Seq;                                      // 包序号，供整个referee文件使用
static Subscriber_t* controller_sub;


// @todo 不应该使用全局变量

/**
 * @brief  判断各种ID，选择客户端ID
 * @param  referee_info_t *referee_recv_info
 * @retval none
 * @attention
 */
static void DeterminRobotID()
{
    // id小于7是红色,大于7是蓝色,0为红色，1为蓝色   #define Robot_Red 0    #define Robot_Blue 1
    referee_recv_info->referee_id.Robot_Color = referee_recv_info->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
    referee_recv_info->referee_id.Robot_ID = referee_recv_info->GameRobotState.robot_id;
    referee_recv_info->referee_id.Cilent_ID = 0x0100 + referee_recv_info->referee_id.Robot_ID; // 计算客户端ID
    referee_recv_info->referee_id.Receiver_Robot_ID = 0;
}

static void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data);
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data); // 模式切换检测
static void RobotModeTest(Referee_Interactive_info_t *_Interactive_data); // 测试用函数，实现模式自动变化

referee_info_t *UITaskInit(UART_HandleTypeDef *referee_usart_handle, Referee_Interactive_info_t *UI_data)
{
    referee_recv_info = RefereeInit(referee_usart_handle); // 初始化裁判系统的串口,并返回裁判系统反馈数据指针
    Interactive_data = UI_data;                            // 获取UI绘制需要的机器人状态数据
    referee_recv_info->init_flag = 1;
    return referee_recv_info;
}

void UITask()
{
    RobotModeTest(Interactive_data); // 测试用函数，实现模式自动变化,用于检查该任务和裁判系统是否连接正常
    MyUIRefresh(referee_recv_info, Interactive_data);
}

static Graph_Data_t UI_shoot_line[10]; // 射击准线
static Graph_Data_t UI_Energy[3];      // 电容能量条
static String_Data_t UI_State_sta[6];  // 机器人状态,静态只需画一次
static String_Data_t UI_State_dyn[6];  // 机器人状态,动态先add才能change
static uint32_t shoot_line_location[10] = {540, 960, 490, 515, 565};

void MyUIInit()
{
    /*自定义控制器图传通信*/
    controller_sub = SubRegister("controller_cmd",sizeof(Controller_cmd_s));
}

// 测试用函数，实现模式自动变化,用于检查该任务和裁判系统是否连接正常
static uint8_t count = 0;
static uint16_t count1 = 0;
static void RobotModeTest(Referee_Interactive_info_t *_Interactive_data) // 测试用函数，实现模式自动变化
{
    count++;
    if (count >= 50)
    {
        count = 0;
        count1++;
    }
    switch (count1 % 4)
    {
    case 0:
    {
        _Interactive_data->chassis_mode = CHASSIS_ZERO_FORCE;
        _Interactive_data->gimbal_mode = GIMBAL_ZERO_FORCE;
        _Interactive_data->shoot_mode = SHOOT_ON;
        _Interactive_data->friction_mode = FRICTION_ON;
        _Interactive_data->lid_mode = LID_OPEN;
        _Interactive_data->Chassis_Power_Data.chassis_power_mx += 3.5;
        if (_Interactive_data->Chassis_Power_Data.chassis_power_mx >= 18)
            _Interactive_data->Chassis_Power_Data.chassis_power_mx = 0;
        break;
    }
    case 1:
    {
        _Interactive_data->chassis_mode = CHASSIS_ROTATE;
        _Interactive_data->gimbal_mode = GIMBAL_FREE_MODE;
        _Interactive_data->shoot_mode = SHOOT_OFF;
        _Interactive_data->friction_mode = FRICTION_OFF;
        _Interactive_data->lid_mode = LID_CLOSE;
        break;
    }
    case 2:
    {
        _Interactive_data->chassis_mode = CHASSIS_NO_FOLLOW;
        _Interactive_data->gimbal_mode = GIMBAL_GYRO_MODE;
        _Interactive_data->shoot_mode = SHOOT_ON;
        _Interactive_data->friction_mode = FRICTION_ON;
        _Interactive_data->lid_mode = LID_OPEN;
        break;
    }
    case 3:
    {
        _Interactive_data->chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        _Interactive_data->gimbal_mode = GIMBAL_ZERO_FORCE;
        _Interactive_data->shoot_mode = SHOOT_OFF;
        _Interactive_data->friction_mode = FRICTION_OFF;
        _Interactive_data->lid_mode = LID_CLOSE;
        break;
    }
    default:
        break;
    }
}

static void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data)
{
    Controller_cmd_s controller_data;
    if(SubGetMessage(controller_sub,(void*)&controller_data)){
        CostomControllerRefresh(&referee_recv_info->referee_id,controller_data);
    }
}
