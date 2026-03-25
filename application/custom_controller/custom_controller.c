#include <string.h>
#include "custom_controller.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "message_center.h"
#include "can_comm.h"
#include "rm_referee.h"
//目前版本必须以复位姿态启动！！！目前只写了yaw和大小pitch相关部分，剩下的后面加

#define M_ARM_BIG (0.0f)
#define M_M2006 (0.0f)
#define M_ARM_SMALL (0.0f)  //暂时认为小臂与末端线密度一致
#define L_ARM_BIG (0.0f)
#define L_ARM_SMALL (0.0f)
#define OFFSET_PITCH_BIG (3.0f)
#define OFFSET_PITCH_SMALL (18.66f)
#define M2006_TOTALANGLE_JOINTANGLE_RATIO (34.768f) //两个关节的减速比是否一致？

#define BUFFER_LENGTH (8)

static DJIMotorInstance *yaw_motor,*pitch_motor_big,*pitch_motor_small,*roll_motor_big,*differencial_motor_pitch;
static float yaw_angle,pitch_angle_big,pitch_angle_small,roll_angle,end_pitch_angle;
static float yaw_ecd_offset,pitch_big_ecd_offset,pitch_small_ecd_offset,roll_ecd_offset,end_pitch_ecd_offset;
static Button_judge_s button_1,button_2,button_3,button_4;
static Publisher_t *controller_pub;
static uint8_t controller_idx;
static Controller_cmd_s controller_cmd_buff[BUFFER_LENGTH];
static Controller_cmd_s controller_cmd;//用于通讯发送
static CANCommInstance* controller_can_comm;
static controller_state_e controller_state;

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
                .Kd = 0,
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

    CANComm_Init_Config_s controller_can_cfg = 
    {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x313,//随便写的
            .rx_id = 0x314,
        },
        .send_data_len = sizeof(Controller_cmd_s),
        .recv_data_len = 0,//需要反馈什么？后续加入
    };

    

    RefereeVtInit(&huart1);
    controller_can_comm = CANCommInit(&controller_can_cfg);
    controller_pub = PubRegister("controller_cmd",sizeof(Controller_cmd_s));

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
/*位号C5,C6为输入口,松开上拉，按下接地*/
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

static void DiffRollControll()
{
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
static void KeepBalance()
{
    static float coe_small = 0.8f;
    float A_pitch_small = -2420.15;
    float C_pitch_small = -426.26;
    static float coe_big = 0.8f;
    float A_pitch_big = 2543.6;
    float B_pitch_big = 3429.2;
    float C_pitch_big = 415.4;
    float K = -400;
    static float kv_small = 0.0f; // 虚拟阻尼系数，根据发热情况调整
    static float kv_big = 0.0f;
    static float filter_alpha = 0.2f; // 滤波系数，越小越平滑
    

    //存储滤波状态的静态变量
    static float last_comp_small = 0;
    static float last_comp_big = 0;


    float raw_comp_small = ((A_pitch_small * cosf((pitch_angle_small - pitch_angle_big) * DEG_TO_RAD)) + C_pitch_small) * coe_small;
    
    float cable_tension = K * cosf(pitch_angle_big * DEG_TO_RAD);
    float raw_comp_big = ((A_pitch_big * cosf(pitch_angle_big * DEG_TO_RAD)) + 
                          (B_pitch_big * cosf((180.0f - pitch_angle_small + pitch_angle_big) * DEG_TO_RAD)) + 
                          C_pitch_big + cable_tension) * coe_big;

    //低通滤波
    float filtered_small = filter_alpha * raw_comp_small + (1.0f - filter_alpha) * last_comp_small;
    float filtered_big = filter_alpha * raw_comp_big + (1.0f - filter_alpha) * last_comp_big;
    
    last_comp_small = filtered_small;
    last_comp_big = filtered_big;

    //虚拟阻尼
    float damping_small = pitch_motor_small->measure.speed_aps * kv_small;
    float damping_big = pitch_motor_big->measure.speed_aps * kv_big;


    DJIMotorSetRef(pitch_motor_small, filtered_small - damping_small);
    DJIMotorSetRef(pitch_motor_big, filtered_big - damping_big);
}

static int16_t pitch_big_current;

void SetCurrent(){
    DJIMotorSetRef(pitch_motor_big,pitch_big_current);

}

void CustomControllerTask()
{
    ButtonCheck(&button_1,HAL_GPIO_ReadPin(BUTTON_1_GPIO_Port,BUTTON_1_Pin));
    ButtonCheck(&button_2,HAL_GPIO_ReadPin(BUTTON_2_GPIO_Port,BUTTON_2_Pin));
    ButtonRealTimeCheck(&button_3,HAL_GPIO_ReadPin(BUTTON_3_GPIO_Port,BUTTON_3_Pin));
    ButtonRealTimeCheck(&button_4,HAL_GPIO_ReadPin(BUTTON_4_GPIO_Port,BUTTON_4_Pin));
    if(button_1.button_state == BUTTON_PRESS){
        if(controller_cmd.gripper_state == GRIPPER_CLOSE){
            controller_cmd.gripper_state = GRIPPER_OPEN;
        }
        else{
            controller_cmd.gripper_state = GRIPPER_CLOSE;
        }
    }
    if(button_2.button_state == BUTTON_LONG_PRESS)
    {
        MotorAngleInit();
        controller_state = CONTROLLER_READY;
    }
    DiffRollControll();
    MotorAngleLimit();

    /*发送数据平滑处理，可以通过BUFFER_LENGTH控制发送频率？*/
    CalculateJointAngle();
    if(controller_idx >= BUFFER_LENGTH)
    {
        CalculateCMD();
        CANCommSend(controller_can_comm,(uint8_t*)&controller_cmd);
        PubPushMessage(controller_pub,(void*)&controller_cmd);//给图传链路的
        memset(controller_cmd_buff,0,sizeof(controller_cmd_buff));
        controller_idx = 0;
    }

    if(controller_state == CONTROLLER_READY)
    {
        //SetCurrent();
        KeepBalance();
    }

}











/*自动复位，暂时弃用*/
static void PitchMotorSmallInit(){
    static uint8_t setflag = 0;
    DJIMotorStop(pitch_motor_big);
    if(pitch_motor_small->motor_state == MOTOR_INIT && setflag == 0)
    {
        setflag = 1;
        DJIMotorSetRef(pitch_motor_small,1000);
    }
    if(abs(pitch_motor_small->measure.real_current) >= 6000 && pitch_motor_small->motor_state == MOTOR_INIT)
    {
        pitch_motor_small->motor_state = MOTOR_READY;
        pitch_motor_small->measure.last_ecd = pitch_motor_small->measure.ecd;
        // pitch_motor_small->measure.total_angle = 0;
        // pitch_motor_small->measure.total_round = 0;
        DJIMotorSetRef(pitch_motor_small,0);
        DJIMotorEnable(pitch_motor_big);
    }
}

static void PitchMotorBigInit(){
    static uint8_t setflag = 0;
    if(pitch_motor_big->motor_state == MOTOR_INIT && setflag == 0)
    {
        setflag = 1;
        DJIMotorSetRef(pitch_motor_big,-1000);
    }
    if(abs(pitch_motor_big->measure.real_current) >= 6000 && pitch_motor_big->motor_state == MOTOR_INIT)
    {
        pitch_motor_big->motor_state = MOTOR_READY;
        pitch_motor_big->measure.last_ecd = pitch_motor_big->measure.ecd;
        // pitch_motor_small->measure.total_angle = 0;
        // pitch_motor_small->measure.total_round = 0;
        DJIMotorSetRef(pitch_motor_big,0);
    }
}

