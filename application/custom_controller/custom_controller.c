#include <string.h>
#include "custom_controller.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "message_center.h"
#include "can_comm.h"
#include "rm_referee.h"
#include "SEGGER_RTT.h"
#include "FreeRTOS.h"
#include "task.h"
//目前版本必须以复位姿态启动！！！目前只写了yaw和大小pitch相关部分，剩下的后面加

/*-----角度读取----- */
#define OFFSET_PITCH_BIG (3.0f)
#define OFFSET_PITCH_SMALL (18.66f)
#define M2006_TOTALANGLE_JOINTANGLE_RATIO (34.768f) //两个关节的减速比是否一致？
/*-----力反馈----- */
#define ANGLE_DEADZONE (5.0f)

#define BUFFER_LENGTH (8)

static DJIMotorInstance *yaw_motor,*pitch_motor_big,*pitch_motor_small,*roll_motor_big,*differencial_motor_pitch;
static Joint_state_s yaw_state,pitch_big_state,pitch_small_state,roll_state,diff_pitch_state;
static float yaw_angle,pitch_angle_big,pitch_angle_small,roll_angle,end_pitch_angle;
static float yaw_ecd_offset,pitch_big_ecd_offset,pitch_small_ecd_offset,roll_ecd_offset,end_pitch_ecd_offset;
static Button_judge_s button_1,button_2,button_3,button_4;
static Publisher_t *controller_pub;
static Subscriber_t *arm_sub;
static uint8_t controller_idx;
static Controller_cmd_s controller_cmd_buff[BUFFER_LENGTH];
static Controller_cmd_s controller_cmd;//用于通讯发送
static Arm_feed_s arm_feedback;//机械臂角度反馈
static CANCommInstance* controller_can_comm;
static controller_state_e controller_state;//是否复位，启动力反馈和重力补偿
static TickType_t last_rx_time;
static TickType_t interval = 100;
static Joint_pid_s* yaw_pid;
static Joint_pid_s* pitch_small_pid;
static Joint_pid_s* pitch_big_pid;
static Joint_pid_s* roll_pid;
static Joint_pid_s* diff_pitch_pid;


void CustomControllerInit()
{
    /*先随便配置*/
    Motor_Init_Config_s yaw_config =
    {
        .can_init_config = 
        {
            .can_handle = &hcan1,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 0,
                .Ki = 0,
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 0,
                .MaxOut = 0,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP| SPEED_LOOP, 
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    };
    yaw_config.can_init_config.tx_id = 2;
    yaw_motor = DJIMotorInit(&yaw_config);

    Motor_Init_Config_s _2006_config =
    {
        .can_init_config = 
        {
            .can_handle = &hcan1,
        },
        .controller_param_init_config = {
            .current_PID = {
                .Kp = 1,
                .Ki = 0,
                .Kd = 0.0,
                .MaxOut = 8000,
            }
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = CURRENT_LOOP,
            .close_loop_type = CURRENT_LOOP, 
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M2006,
    };
    _2006_config.can_init_config.tx_id = 1;
    pitch_motor_big = DJIMotorInit(&_2006_config);
    _2006_config.can_init_config.tx_id = 2;
    pitch_motor_small = DJIMotorInit(&_2006_config);
    _2006_config.can_init_config.tx_id = 3;
    roll_motor_big = DJIMotorInit(&_2006_config);
    _2006_config.can_init_config.tx_id = 4;
    differencial_motor_pitch = DJIMotorInit(&_2006_config);
    PID_Init_Config_s yaw_inner_pid_cfg = {
        .Kp = 0,
        .Ki = 0,
        .Kd = 0,

        .Improve = PID_Integral_Limit,
        .IntegralLimit = 3000,
        .MaxOut = 9000,
    };
    PID_Init_Config_s yaw_outer_pid_cfg = {
        .Kp = 0,
        .Ki = 0,
        .Kd = 0,

        .Improve = PID_Integral_Limit,
        .IntegralLimit = 3000,
        .MaxOut = 9000,
    };
    PID_Init_Config_s small_inner_pid_cfg = {
        .Kp = 8,
        .Ki = 10,
        .Kd = 0,

        .Improve = PID_Integral_Limit,
        .IntegralLimit = 3000,
        .MaxOut = 9000,
    };
    PID_Init_Config_s small_outer_pid_cfg = {
        .Kp = 50,
        .Ki = 0,
        .Kd = 2,

        .MaxOut = 4500,
    };
    PID_Init_Config_s big_inner_pid_cfg = {
        .Kp = 8,
        .Ki = 10,
        .Kd = 0,

        .Improve = PID_Integral_Limit,
        .IntegralLimit = 3000,
        .MaxOut = 9000,
    };
    PID_Init_Config_s big_outer_pid_cfg = {
        .Kp = 50,
        .Ki = 0,
        .Kd = 2,

        .MaxOut = 4500,
    };
    PID_Init_Config_s roll_inner_pid_cfg = {
        .Kp = 0,
        .Ki = 0,
        .Kd = 0,

        .Improve = PID_Integral_Limit,
        .IntegralLimit = 3000,
        .MaxOut = 9000,
    };
    PID_Init_Config_s roll_outer_pid_cfg = {
        .Kp = 0,
        .Ki = 0,
        .Kd = 0,

        .Improve = PID_Integral_Limit,
        .IntegralLimit = 3000,
        .MaxOut = 9000,
    };
    PID_Init_Config_s diff_pitch_inner_pid_cfg = {
        .Kp = 0,
        .Ki = 0,
        .Kd = 0,

        .Improve = PID_Integral_Limit,
        .IntegralLimit = 3000,
        .MaxOut = 9000,
    };
    PID_Init_Config_s diff_pitch_outer_pid_cfg = {
        .Kp = 0,
        .Ki = 0,
        .Kd = 0,

        .Improve = PID_Integral_Limit,
        .IntegralLimit = 3000,
        .MaxOut = 9000,
    };
    PIDInit(&yaw_pid->inner_loop,&yaw_inner_pid_cfg);
    PIDInit(&yaw_pid->outer_loop,&yaw_outer_pid_cfg);
    PIDInit(&pitch_small_pid->inner_loop,&small_inner_pid_cfg);
    PIDInit(&pitch_small_pid->outer_loop,&small_outer_pid_cfg);
    PIDInit(&pitch_big_pid->inner_loop,&big_inner_pid_cfg);
    PIDInit(&pitch_big_pid->outer_loop,&big_outer_pid_cfg);
    PIDInit(&roll_pid->inner_loop,&roll_inner_pid_cfg);
    PIDInit(&roll_pid->outer_loop,&roll_outer_pid_cfg);
    PIDInit(&diff_pitch_pid->inner_loop,&diff_pitch_inner_pid_cfg);
    PIDInit(&diff_pitch_pid->outer_loop,&diff_pitch_outer_pid_cfg);
    CANComm_Init_Config_s controller_can_cfg = 
    {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x313,
            .rx_id = 0x314,
        },
        .send_data_len = sizeof(Controller_cmd_s),
        .recv_data_len = sizeof(Arm_feed_s),//需要反馈什么？后续加入
    };

    

    RefereeVtInit(&huart1);
    controller_can_comm = CANCommInit(&controller_can_cfg);
    controller_pub = PubRegister("controller_cmd",sizeof(Controller_cmd_s));
    arm_sub = SubRegister("arm_feedback",sizeof(Arm_feed_s)); 
}

static void CalculateJointAngle(){
    yaw_angle = yaw_motor->measure.angle_single_round - yaw_ecd_offset;
    pitch_angle_big = (pitch_motor_big->measure.total_angle - pitch_big_ecd_offset) / M2006_TOTALANGLE_JOINTANGLE_RATIO + OFFSET_PITCH_BIG;
    pitch_angle_small = -((pitch_motor_small->measure.total_angle - pitch_small_ecd_offset) / M2006_TOTALANGLE_JOINTANGLE_RATIO - OFFSET_PITCH_SMALL);
    end_pitch_angle = (differencial_motor_pitch->measure.total_angle - end_pitch_ecd_offset) / M2006_TOTALANGLE_JOINTANGLE_RATIO;
    roll_angle = (roll_motor_big->measure.total_angle - roll_ecd_offset) / M2006_TOTALANGLE_JOINTANGLE_RATIO;
    if(roll_angle > 178) roll_angle = 178;
    else if(roll_angle < -178) roll_angle = -178;
    if(yaw_angle > 178) yaw_angle = 178;
    else if(yaw_angle < -178) yaw_angle = -178;

    controller_cmd_buff[controller_idx].yaw_angle = yaw_angle;
    controller_cmd_buff[controller_idx].pitch_big_angle = pitch_angle_big;
    controller_cmd_buff[controller_idx].pitch_small_angle = pitch_angle_small;
    controller_cmd_buff[controller_idx].roll_angle = roll_angle;
    controller_cmd_buff[controller_idx].diff_pitch = end_pitch_angle;

    controller_idx++;

}
static void CalculateCMD(){
    Controller_cmd_s buff = {0};
    for(int i = 0;i < BUFFER_LENGTH;i++)
    {
        buff.yaw_angle += controller_cmd_buff[i].yaw_angle;
        buff.pitch_big_angle += controller_cmd_buff[i].pitch_big_angle;
        buff.pitch_small_angle += controller_cmd_buff[i].pitch_small_angle;
        buff.roll_angle += controller_cmd_buff[i].roll_angle;
        buff.diff_pitch += controller_cmd_buff[i].diff_pitch;
    }
    controller_cmd.yaw_angle = buff.yaw_angle / (float)(BUFFER_LENGTH);
    controller_cmd.pitch_big_angle = buff.pitch_big_angle / (float)(BUFFER_LENGTH);
    controller_cmd.pitch_small_angle = buff.pitch_small_angle / (float)(BUFFER_LENGTH);
    controller_cmd.roll_angle = buff.roll_angle / (float)(BUFFER_LENGTH);
    controller_cmd.diff_pitch = buff.diff_pitch / (float)(BUFFER_LENGTH);
}
static void MotorAngleInit(){
    yaw_motor->measure.total_round = 0;
    yaw_ecd_offset = yaw_motor->measure.angle_single_round; 

    pitch_motor_big->measure.total_round = 0;
    pitch_big_ecd_offset = pitch_motor_big->measure.angle_single_round;

    pitch_motor_small->measure.total_round = 0;
    pitch_small_ecd_offset = pitch_motor_small->measure.angle_single_round;

    roll_motor_big->measure.total_round = 0;
    roll_ecd_offset = roll_motor_big->measure.angle_single_round;

    differencial_motor_pitch->measure.total_round = 0;
    end_pitch_ecd_offset = differencial_motor_pitch->measure.angle_single_round;
}
static void MotorAngleLimit(){
    /*若总角度小于标记零点，则记录新零点*/
    if(pitch_motor_big->measure.total_angle < pitch_big_ecd_offset)
    {
        pitch_motor_big->measure.total_round = 0;
        pitch_big_ecd_offset = pitch_motor_big->measure.angle_single_round;
    }
    if(pitch_motor_small->measure.total_angle > pitch_small_ecd_offset)
    {
        pitch_motor_small->measure.total_round = 0;
        pitch_small_ecd_offset = pitch_motor_small->measure.angle_single_round;
    }
}
static void ButtonCheck(Button_judge_s* button,uint8_t current_status)
{
    if(current_status == 0){
        button->timer++;
    }
    else{
        if(button->last_status == 0){
            if(button->timer > 3 && button->timer < 200){
                button->button_state = BUTTON_PRESS;
            }
            else if (button->timer >= 200)
            {
                button->button_state = BUTTON_LONG_PRESS;
            }
        }
        else{
            button->button_state = BUTTON_RELEASE;
        }
        button->timer = 0;
    }
    button->last_status = current_status;
}
static void ButtonRealTimeCheck(Button_judge_s* button,uint8_t current_status)
{
    if(current_status == 0)
    {
        button->button_state = BUTTON_PRESS;
    }
    else
    {
        button->button_state = BUTTON_RELEASE;
    }
}
static void ButtonTask()
{
    ButtonCheck(&button_1,HAL_GPIO_ReadPin(BUTTON_1_GPIO_Port,BUTTON_1_Pin));
    ButtonCheck(&button_2,HAL_GPIO_ReadPin(BUTTON_2_GPIO_Port,BUTTON_2_Pin));
    ButtonRealTimeCheck(&button_3,HAL_GPIO_ReadPin(BUTTON_3_GPIO_Port,BUTTON_3_Pin));
    ButtonRealTimeCheck(&button_4,HAL_GPIO_ReadPin(BUTTON_4_GPIO_Port,BUTTON_4_Pin));
    /*----------短按控制夹爪开合------------*/
    if(button_1.button_state == BUTTON_PRESS){
        if(controller_cmd.gripper_state == GRIPPER_CLOSE){
            controller_cmd.gripper_state = GRIPPER_OPEN;
        }
        else{
            controller_cmd.gripper_state = GRIPPER_CLOSE;
        }
    }
    /*----------长按角度复位-------------*/
    if(button_2.button_state == BUTTON_LONG_PRESS)
    {
        MotorAngleInit();
        controller_state = CONTROLLER_READY;
    }
    /*----------左右按键控制小roll逆顺时针旋转--------------*/
    if(button_3.button_state == BUTTON_PRESS && button_4.button_state == BUTTON_RELEASE)
    {
        controller_cmd.diff_roll_state = ROLL_CCW;
    }
    else if(button_4.button_state == BUTTON_PRESS && button_3.button_state == BUTTON_RELEASE)
    {
        controller_cmd.diff_roll_state = ROLL_CW;
    }
    else
    {
        controller_cmd.diff_roll_state = ROLL_STAY;
    }
}


float JointPIDCal(Joint_pid_s* pid,float diff,float speed)
{
    float ref = diff;
    ref = PIDCalculate(&(pid->outer_loop),diff,0);
    ref = PIDCalculate(&(pid->inner_loop),speed,ref);
    return ref;
}

static void LinearInterpolation()
{
    TickType_t time = xTaskGetTickCount();
    TickType_t dt = time - last_rx_time;
    if(dt <= 300)
    {
        float dtheta_pitch_small = pitch_small_state.now_angle - pitch_small_state.last_angle;
        float speed_pitch_small = dtheta_pitch_small / (float)interval;
        float predict_pitch_small_angle = arm_feedback.pitch_small_angle + speed_pitch_small * (float)dt;
        pitch_small_state.smooth_diff = controller_cmd.pitch_small_angle - predict_pitch_small_angle;

        float dtheta_pitch_big = pitch_big_state.now_angle - pitch_big_state.last_angle;
        float speed_pitch_big = dtheta_pitch_big / (float)interval;
        float predict_pitch_big_angle = arm_feedback.pitch_big_angle + speed_pitch_big * (float)dt;
        pitch_big_state.smooth_diff = predict_pitch_big_angle - controller_cmd.pitch_big_angle;
    }
    else
    {
        pitch_small_state.smooth_diff = pitch_small_state.diff;
        pitch_big_state.smooth_diff = pitch_big_state.diff;
    }
}
//开启线性差值时，修改计算式中diff为smooth_diff
static void KeepBalance()
{
    /*一般重力补偿相关参数*/
    static float coe_small = 0.8f;
    static float coe_big = 0.8f;
    static float kv_small = 0.0f; // 虚拟阻尼系数，根据发热情况调整
    static float kv_big = 0.0f;
    static float filter_alpha = 0.2f; // 滤波系数，越小越平滑
    
    //存储滤波状态的静态变量
    static float last_comp_yaw = 0;
    static float last_comp_small = 0;
    static float last_comp_big = 0;
    static float last_comp_roll = 0;
    static float last_comp_diff_pitch = 0;
    //原始输出
    static float raw_comp_yaw;
    static float raw_comp_small;
    static float raw_comp_big;
    static float raw_comp_roll;
    static float raw_comp_diff_pitch;
    //角度差-电流映射参数
    static float Kp_small = 30.0f;
    static float Kp_big = 30.0f;

    float A = 0;
    float B = 0;
    float C = 0;
    switch(pitch_small_state.state)
    {
        case JOINT_BALANCE:
        {
            A = -2420.15;
            C = -426.26;
            raw_comp_small = A * cosf((pitch_angle_small - pitch_angle_big) * DEG_TO_RAD) + C;
            break;
        }
        case JOINT_FORCE_FEEDBACK:
        {

            // //向上补偿摩擦力并加额外力矩
            // if(pitch_small_state.diff < 0)
            // {
            //     A = -3002.8;
            //     C = -949.5;
                
            // }
            // //向下补偿
            // else
            // {
            //     A = -1837.5;
            //     C = 97.0;

            // }
            A = -2420.15;
            C = -426.26;
            float external_force = JointPIDCal(pitch_small_pid,-pitch_small_state.diff,pitch_motor_small->measure.speed_aps);
            //运动过程中不触发反馈
            if (pitch_small_state.diff < 0 && pitch_motor_small->measure.speed_aps < -200)
                external_force = 0;
            if (pitch_small_state.diff > 0 && pitch_motor_small->measure.speed_aps > 200)
                external_force = 0;
            raw_comp_small = (A * cosf((pitch_angle_small - pitch_angle_big) * DEG_TO_RAD)) + C + external_force;
            break;
        }
    }
    raw_comp_small *= coe_small;
    float filtered_small = filter_alpha * raw_comp_small + (1.0f - filter_alpha) * last_comp_small;
    last_comp_small = filtered_small;
    float damping_small = pitch_motor_small->measure.speed_aps * kv_small;
    DJIMotorSetRef(pitch_motor_small, filtered_small - damping_small);

    float K = -400;
    switch(pitch_big_state.state)
    {
        case JOINT_BALANCE:
        {
            A = 2543.6;
            B = 3429.2;
            C = 415.4;
            raw_comp_big = (A * cosf(pitch_angle_big * DEG_TO_RAD)) + 
                            (B * cosf((180.0f - pitch_angle_small + pitch_angle_big) * DEG_TO_RAD)) + 
                            C;
            break;
        }
        case JOINT_FORCE_FEEDBACK:
        {
            // if(pitch_big_state.diff > 0)
            // {
            //     A = 1753.2;
            //     B = 2997.3;
            //     C = 1264.9;
            // }
            // else
            // {
            //     A = 3334.0;
            //     B = 3861.1;
            //     C = -434.1;
            // }
            A = 2543.6;
            B = 3429.2;
            C = 415.4;
            float external_force = JointPIDCal(pitch_big_pid,-pitch_big_state.diff,pitch_motor_big->measure.speed_aps);
            if (pitch_big_state.diff < 0 && pitch_motor_big->measure.speed_aps < -200)
                external_force = 0;
            if (pitch_big_state.diff > 0 && pitch_motor_big->measure.speed_aps > 200)
                external_force = 0;
            raw_comp_big = (A * cosf(pitch_angle_big * DEG_TO_RAD)) + 
                            (B * cosf((180.0f - pitch_angle_small + pitch_angle_big) * DEG_TO_RAD)) + 
                            C + external_force;
            break;
        }
    }
    float cable_tension = K * cosf(pitch_angle_big * DEG_TO_RAD);
    raw_comp_big = (raw_comp_big + cable_tension) * coe_big;
    float filtered_big = filter_alpha * raw_comp_big + (1.0f - filter_alpha) * last_comp_big;
    last_comp_big = filtered_big;
    float damping_big = pitch_motor_big->measure.speed_aps * kv_big;
    DJIMotorSetRef(pitch_motor_big, filtered_big - damping_big);

    switch(yaw_state.state)
    {
        case JOINT_BALANCE:
        {
            raw_comp_yaw = 0;
            break;
        }
        case JOINT_FORCE_FEEDBACK:
        {
            raw_comp_yaw = JointPIDCal(yaw_pid,yaw_state.diff,yaw_motor->measure.speed_aps);
            if(yaw_state.diff < 0 && yaw_motor->measure.speed_aps < -10)
                raw_comp_yaw = 0;
            if(yaw_state.diff > 0 && yaw_motor->measure.speed_aps > 10)
                raw_comp_yaw = 0;
            break;
        }
    }
    float filtered_yaw = filter_alpha * raw_comp_yaw + (1.0f - filter_alpha) * last_comp_yaw;
    last_comp_yaw = filtered_yaw;
    DJIMotorSetRef(yaw_motor,filtered_yaw);

    switch(roll_state.state)
    {
        case JOINT_BALANCE:
        {
            raw_comp_roll = 0;
            break;
        }
        case JOINT_FORCE_FEEDBACK:
        {
            raw_comp_roll = JointPIDCal(roll_pid,roll_state.diff,roll_motor_big->measure.speed_aps);
            if(roll_state.diff < 0 && roll_motor_big-> measure.speed_aps < -10)
                raw_comp_roll = 0;
            if(roll_state.diff > 0 && roll_motor_big-> measure.speed_aps > 10)
                raw_comp_roll = 0;
            break;
        }
    }
    float filtered_roll = filter_alpha * raw_comp_roll + (1.0f - filter_alpha) * last_comp_roll;
    last_comp_roll = filtered_roll;
    DJIMotorSetRef(roll_motor_big,filtered_roll);

    switch(diff_pitch_state.state)
    {
        case JOINT_BALANCE:
        {
            raw_comp_diff_pitch = 0;
            break;
        }
        case JOINT_FORCE_FEEDBACK:
        {
            raw_comp_diff_pitch = JointPIDCal(diff_pitch_pid,diff_pitch_state.diff,differencial_motor_pitch->measure.speed_aps);
            if(diff_pitch_state.diff < 0 && differencial_motor_pitch-> measure.speed_aps < -10)
                raw_comp_diff_pitch = 0;
            if(diff_pitch_state.diff > 0 && differencial_motor_pitch-> measure.speed_aps > 10)
                raw_comp_diff_pitch = 0;
            break;
        }
    }
    float filtered_diff_pitch = filter_alpha * raw_comp_diff_pitch + (1.0f - filter_alpha) * last_comp_diff_pitch;
    last_comp_diff_pitch = filtered_diff_pitch;
    DJIMotorSetRef(differencial_motor_pitch,filtered_diff_pitch);
}

static void GetFeedBackInfo()
{
    // if(SubGetMessage(arm_sub,(void*)&arm_feedback))
    // {
    //     TickType_t now = xTaskGetTickCount();
    //     if(now - last_rx_time > 0)
    //     {
    //         interval = now - last_rx_time;
    //     }
    //     pitch_small_state.last_angle = pitch_small_state.now_angle;
    //     pitch_big_state.last_angle = pitch_big_state.now_angle;
    //     pitch_small_state.now_angle = arm_feedback.pitch_small_angle;
    //     pitch_big_state.now_angle = arm_feedback.pitch_big_angle;
    //     last_rx_time = now;
    // }

    Arm_feed_s* buff = (Arm_feed_s*)CANCommGet(controller_can_comm);
    if(buff == NULL){
        return;
    }
    arm_feedback = *buff;
}

static void ForceFeedBack()
{
    /*角度差以与电流补偿方向一致为正*/
    yaw_state.diff = controller_cmd.yaw_angle - arm_feedback.yaw_angle;//符号可能需要反一下，未验证
    pitch_small_state.diff = controller_cmd.pitch_small_angle - arm_feedback.pitch_small_angle;
    pitch_big_state.diff = arm_feedback.pitch_big_angle - controller_cmd.pitch_big_angle;
    roll_state.diff = controller_cmd.roll_angle - arm_feedback.roll_angle;
    diff_pitch_state.diff = controller_cmd.diff_pitch - arm_feedback.diff_pitch;
    if(fabs(yaw_state.diff) >= ANGLE_DEADZONE && arm_feedback.feedback_flag_yaw == 1)
    {
        yaw_state.state = JOINT_FORCE_FEEDBACK;
    }
    else
    {
        yaw_state.state = JOINT_BALANCE;
    }
    if(fabs(pitch_big_state.diff) >= ANGLE_DEADZONE && arm_feedback.feedback_flag_pitch_big == 1)
    {
        pitch_big_state.state = JOINT_FORCE_FEEDBACK;
    }
    else
    {
        pitch_big_state.state = JOINT_BALANCE;
    }
    if(fabs(pitch_small_state.diff) >= ANGLE_DEADZONE && arm_feedback.feedback_flag_pitch_small == 1)
    {
        pitch_small_state.state = JOINT_FORCE_FEEDBACK;
    }
    else
    {
        pitch_small_state.state = JOINT_BALANCE;
    }
    if(fabs(roll_state.diff) >= ANGLE_DEADZONE && arm_feedback.feedback_flag_roll == 1)
    {
        roll_state.state = JOINT_FORCE_FEEDBACK;
    }
    else
    {
        roll_state.state = JOINT_BALANCE;
    }
    if(fabs(diff_pitch_state.diff) >= ANGLE_DEADZONE && arm_feedback.feedback_flag_diff_pitch == 1)
    {
        roll_state.state = JOINT_FORCE_FEEDBACK;
    }
    else
    {
        roll_state.state = JOINT_BALANCE;
    }    
    //LinearInterpolation();
}

/*加一些安全检查，给反馈加点插值平滑*/
void CustomControllerTask()
{
    ButtonTask();
    MotorAngleLimit();

    if(controller_state == CONTROLLER_READY)
    {
        /*发送数据平滑处理，可以通过BUFFER_LENGTH控制发送频率？*/
        CalculateJointAngle();
        if(controller_idx >= BUFFER_LENGTH)
        {
            CalculateCMD();
            CANCommSend(controller_can_comm,(uint8_t*)&controller_cmd);
            //PubPushMessage(controller_pub,(void*)&controller_cmd);//给图传链路的
            memset(controller_cmd_buff,0,sizeof(controller_cmd_buff));
            controller_idx = 0;
        }
        GetFeedBackInfo();
        KeepBalance();
        if(arm_feedback.feedback_state == FEEDBACK_ON)
        {
            ForceFeedBack();
        }
    }
}











/*自动复位，暂时弃用*/
// static void PitchMotorSmallInit(){
//     static uint8_t setflag = 0;
//     DJIMotorStop(pitch_motor_big);
//     if(pitch_motor_small->motor_state == MOTOR_INIT && setflag == 0)
//     {
//         setflag = 1;
//         DJIMotorSetRef(pitch_motor_small,1000);
//     }
//     if(abs(pitch_motor_small->measure.real_current) >= 6000 && pitch_motor_small->motor_state == MOTOR_INIT)
//     {
//         pitch_motor_small->motor_state = MOTOR_READY;
//         pitch_motor_small->measure.last_ecd = pitch_motor_small->measure.ecd;
//         // pitch_motor_small->measure.total_angle = 0;
//         // pitch_motor_small->measure.total_round = 0;
//         DJIMotorSetRef(pitch_motor_small,0);
//         DJIMotorEnable(pitch_motor_big);
//     }
// }

// static void PitchMotorBigInit(){
//     static uint8_t setflag = 0;
//     if(pitch_motor_big->motor_state == MOTOR_INIT && setflag == 0)
//     {
//         setflag = 1;
//         DJIMotorSetRef(pitch_motor_big,-1000);
//     }
//     if(abs(pitch_motor_big->measure.real_current) >= 6000 && pitch_motor_big->motor_state == MOTOR_INIT)
//     {
//         pitch_motor_big->motor_state = MOTOR_READY;
//         pitch_motor_big->measure.last_ecd = pitch_motor_big->measure.ecd;
//         // pitch_motor_small->measure.total_angle = 0;
//         // pitch_motor_small->measure.total_round = 0;
//         DJIMotorSetRef(pitch_motor_big,0);
//     }
// }

