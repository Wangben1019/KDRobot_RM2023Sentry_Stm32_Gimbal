#include "WatchDog_Task.h"

uint8_t Block_Num_Right;    //!<@brief 右侧拨弹盘卡弹检测误差积累
uint8_t Block_Num_Left;     //!<@brief 左侧拨弹盘卡弹检测误差积累

uint16_t Resume_Num_Right;   //!<@brief 右侧拨弹盘反转时间
uint16_t Resume_Num_Left;    //!<@brief 右侧拨弹盘反转时间

uint16_t PC_Control_Flag_Num;

uint16_t Switch_Time;

WatchDog_TypeDef WGimbal_Yaw_Motor;
WatchDog_TypeDef WGimbal_Pitch_Left_Motor;
WatchDog_TypeDef WGimbal_Pitch_Right_Motor;
WatchDog_TypeDef WFrictionwheel_Right_Motor;
WatchDog_TypeDef WFrictionwheel_Centre_Motor;
WatchDog_TypeDef WFrictionwheel_Left_Motor;
WatchDog_TypeDef WPluck_Right_Motor;
WatchDog_TypeDef WPluck_Left_Motor;

WatchDog_TypeDef WRemote_Communication;
WatchDog_TypeDef WCan1_Communication;
WatchDog_TypeDef WCan2_Communication;
WatchDog_TypeDef WPC_Communication;
WatchDog_TypeDef WIMU_Communication;
WatchDog_TypeDef WReferee_Communication_1;
WatchDog_TypeDef WReferee_Communication_2;

Device_Status SGimbal_Yaw_Motor = Device_Error;
Device_Status SGimbal_Pitch_Left_Motor = Device_Error;
Device_Status SGimbal_Pitch_Right_Motor = Device_Error;
Device_Status SFrictionwheel_Right_Motor = Device_Error;
Device_Status SFrictionwheel_Centre_Motor = Device_Error;
Device_Status SFrictionwheel_Left_Motor = Device_Error;
Device_Status SPluck_Right_Motor = Device_Error;
Device_Status SPluck_Left_Motor = Device_Error;
Device_Status SRemote_Communication = Device_Error;
Device_Status SCan1_Communication = Device_Error;
Device_Status SCan2_Communication = Device_Error;
Device_Status SPC_Communication = Device_Error;
Device_Status SIMU_Communication = Device_Error;
Device_Status SReferee_Communication_1 = Device_Error;
Device_Status SReferee_Communication_2 = Device_Error;

eTaskState Init_Task_Status;

TaskHandle_t WatchDog_Task_Handle;
void WatchDog_Task(void *pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    for(;;)
    {
        WatchDog_Polling();
        vTaskDelayUntil(&xLastWakeTime,2);
    }
}
int16_t rotate_expect;
uint8_t expect_flag = 0;
TaskHandle_t Monitor_Device_Status_Task_Handle;
void Monitor_Device_Status_Task(void *pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    for(;;)
    {
        /* 需要重新初始化检测 */
        /****************************************Start****************************************/
        Init_Task_Status = eTaskGetState(Robot_Init_Task_Handle);
        if(Robot_Status == Doing_Init && Init_Task_Status == eSuspended)
        {
            vTaskResume(Robot_Init_Task_Handle);
        }
        /****************************************End******************************************/
        
        /* 卡弹检测 */
        /****************************************Start****************************************/       
        /* 基于误差时间上的累积判断是否卡弹 */
        if(Robot_Shoot_Status == Start_Shoot_Right && abs(Pluck_Right_Motor.Speed) < 300)
        {
            /* 速度不足，开始积累 */
            Block_Num_Right++;
            /* 未判断出卡弹，不需要反转时间积累 */
            Resume_Num_Right = 0;
            /* 速度不足时间过长，可以确定为卡弹 */
            if(Block_Num_Right == 100)
            {
                /* 右侧拨弹盘卡弹 */
                Robot_Shoot_Status = Pluck_Block_Right;
                /* 积累时间重置 */
                Block_Num_Right = 0;  
            }
        }
        else if(Robot_Shoot_Status == Start_Shoot_Right && abs(Pluck_Right_Motor.Speed) > 1000)
        {
            /* 拨弹状态且速度正常,不需要累计时间 */
            Block_Num_Right = 0;
        }
        else if(Robot_Shoot_Status == Pluck_Block_Right)
        {
//            Block_Num_Right = 0;
            /* 此时拨弹盘已经开始反转，进行时间积累 */
            Resume_Num_Right++;
            /* 经过一段时间之后假设已经退弹完成，回归发射状态：如果仍卡弹，则还会继续积累 */
            if(Resume_Num_Right == 200)
            {
                Robot_Shoot_Status = Start_Shoot_Right;
                Resume_Num_Right = 0;
            }
        }
        else if(Robot_Shoot_Status == Start_Shoot_Left && abs(Pluck_Left_Motor.Speed) < 300)
        {
            Block_Num_Left++;
            Resume_Num_Left = 0;
            if(Block_Num_Left == 100)
            {
                Robot_Shoot_Status = Pluck_Block_Left;
                Block_Num_Left = 0;
            }
        }
        else if(Robot_Shoot_Status == Start_Shoot_Left && abs(Pluck_Left_Motor.Speed) > 1000)
        {
            Block_Num_Left = 0;
        }
        else if(Robot_Shoot_Status == Pluck_Block_Left)
        {
            Block_Num_Left = 0;
            Resume_Num_Left++;
            if(Resume_Num_Left == 200)
            {
                Robot_Shoot_Status = Start_Shoot_Left;
                Resume_Num_Left = 0;
            }
        }
        /****************************************End******************************************/
        
        /* 开枪状态下枪口切换检测 */
        /****************************************Start****************************************/ 
        // 左侧发弹直接切换右侧发弹
        if(Robot_Shoot_Status == Start_Shoot_Left && Last_Robot_Shoot_Status == Start_Shoot_Right)
        {
            // 更新状态为切换枪口
            Robot_Shoot_Status = Switch_RightToLeft;
            Switch_Time = 0;
        }
        else if(Robot_Shoot_Status == Start_Shoot_Right && Last_Robot_Shoot_Status == Start_Shoot_Left)
        {
            Robot_Shoot_Status = Switch_LeftToRight;
            Switch_Time = 0;
        }
        // 此时枪口在切换的中间态，开始积累时间(延时打开拨弹)
        else if(Robot_Shoot_Status == Switch_RightToLeft)
        {
            Switch_Time++;
            if(Switch_Time == 200)
            {
                // 延时一定时间后更改状态为发弹(打开拨弹)
                Robot_Shoot_Status = Start_Shoot_Left;
                Switch_Time = 0;
            }
        }
        else if(Robot_Shoot_Status == Switch_LeftToRight)
        {
            Switch_Time++;
            if(Switch_Time == 200)
            {
                Robot_Shoot_Status = Start_Shoot_Right;
                Switch_Time = 0;
            }
        }
        /****************************************End******************************************/
        
        /* 自瞄云台状态改变 */
        /****************************************Start****************************************/ 
        if(Lock_Robot_Rx == 0 && Shoot_Rx == 0)
        {
            Aim_Shoot_Status = UnLock_Robot;
            Robot_Gimbal_Status = Patro_Mode;
        }
        else if(Shoot_Rx == 1)
        {
            Aim_Shoot_Status = Shoot;
            Robot_Gimbal_Status = Vision_Control_Mode;
        }
        else if(Lock_Robot_Rx == 1)
        {
            Aim_Shoot_Status = Lock_Robot;
            Robot_Gimbal_Status = Vision_Control_Mode;
        }
        
        
        if(Robot_Status == Finish_Init)
        {
            if(Aim_Shoot_Status == UnLock_Robot)
            {
                Robot_Shoot_Status = Stop_Shoot;
            }
            else if(Robot_Shoot_Status != Pluck_Block_Right && Aim_Shoot_Status == Lock_Robot && Robot_Shoot_Direction == Shoot_Right)
            {
                Robot_Shoot_Status = Prepare_Shoot_Right;
            }
            else if(Robot_Shoot_Status != Pluck_Block_Right && Robot_Shoot_Status != Switch_LeftToRight && Aim_Shoot_Status == Shoot && Robot_Shoot_Direction == Shoot_Right)
            {
                Robot_Shoot_Status = Start_Shoot_Right;
            }
            else if(Robot_Shoot_Status != Pluck_Block_Left && Aim_Shoot_Status == Lock_Robot && Robot_Shoot_Direction == Shoot_Left)
            {
                Robot_Shoot_Status = Prepare_Shoot_Left;
            }
            else if(Robot_Shoot_Status != Pluck_Block_Left && Robot_Shoot_Status != Switch_RightToLeft && Aim_Shoot_Status == Shoot && Robot_Shoot_Direction == Shoot_Left)
            {
                Robot_Shoot_Status = Start_Shoot_Left;
            }
        }
        /****************************************End******************************************/
        if(SReferee_Communication_1 == Device_Right)
        {
            if(Robot_Status == Finish_Init && The_RX_Buffer_1.RX_Buffer_1.game_progress == 4)
            {
                if(The_RX_Buffer_1.RX_Buffer_1.Outpost_status == 0)
                {
//                    dPatro_EulerAngler_Yaw = 0;
//                    dPatro_Angle_Pitch = 0;
                    if(The_RX_Buffer_1.RX_Buffer_1.chassis_power == 0 && ChassisSpeed_Ref.rotate_ref > 3000)
                    {
                        ChassisSpeed_Ref.rotate_ref = 3000;
                    }
                    if(The_RX_Buffer_1.RX_Buffer_1.chassis_power < 100)
                    {
                        expect_flag = 0;
                    }
                    if(The_RX_Buffer_1.RX_Buffer_1.chassis_power < 140)
                    {
                        if(expect_flag == 0)
                        {
                            ChassisSpeed_Ref.rotate_ref += 10;
                        }                        
                    }
                    else if(The_RX_Buffer_1.RX_Buffer_1.chassis_power >= 140)
                    {
                        expect_flag = 1;
                    }
                }
                else if(The_RX_Buffer_1.RX_Buffer_1.Outpost_status == 1)
                {
                    dPatro_EulerAngler_Yaw = 1.0;
                    dPatro_Angle_Pitch = 1;
                    ChassisSpeed_Ref.rotate_ref = 0;
                }    
            }
        }
        else
        {
            ChassisSpeed_Ref.rotate_ref = 3000;
        }

        if(The_RX_Buffer_2.RX_Buffer_2.shooter_id2_17mm_cooling_heat >= 200)  // 240  id 2  right
        {
            Robot_Shoot_Direction = Shoot_Left;
        }
        else if(The_RX_Buffer_2.RX_Buffer_2.shooter_id2_17mm_cooling_heat <= 120)
        {
            Robot_Shoot_Direction = Shoot_Right;
        }
        
        if(SFrictionwheel_Right_Motor == Device_Error)
        {
            Robot_Shoot_Direction = Shoot_Left;
        }
        else if(SFrictionwheel_Left_Motor == Device_Error)
        {
            Robot_Shoot_Direction = Shoot_Right;
        }
        vTaskDelayUntil(&xLastWakeTime,2);
    }
}

void WatchDog_Init_Gather()
{
    WatchDog_Init(&WGimbal_Yaw_Motor, 20);
    WatchDog_Init(&WGimbal_Pitch_Left_Motor, 20);
    WatchDog_Init(&WGimbal_Pitch_Right_Motor, 20);
    WatchDog_Init(&WFrictionwheel_Right_Motor, 20);
    WatchDog_Init(&WFrictionwheel_Centre_Motor, 20);
    WatchDog_Init(&WFrictionwheel_Left_Motor, 20);
    WatchDog_Init(&WPluck_Right_Motor, 20);
    WatchDog_Init(&WPluck_Left_Motor, 20);
    
    WatchDog_Init(&WRemote_Communication, 20);
    WatchDog_Init(&WCan1_Communication, 20);
    WatchDog_Init(&WCan2_Communication, 20);
    WatchDog_Init(&WPC_Communication, 20);
    WatchDog_Init(&WIMU_Communication, 20);
    WatchDog_Init(&WReferee_Communication_1, 20);
    WatchDog_Init(&WReferee_Communication_2, 20);
}

void FeedDog_CallBack(WatchDogp handle)
{
    if(IS_Dog(handle, WGimbal_Yaw_Motor))
    {
        SGimbal_Yaw_Motor = Device_Right;
    }
    else if(IS_Dog(handle, WGimbal_Pitch_Left_Motor))
    {
        SGimbal_Pitch_Left_Motor = Device_Right;
    }
    else if(IS_Dog(handle, WGimbal_Pitch_Right_Motor))
    {
        SGimbal_Pitch_Right_Motor = Device_Right;
    }
    else if(IS_Dog(handle, WFrictionwheel_Right_Motor))
    {
        SFrictionwheel_Right_Motor = Device_Right;
    }
    else if(IS_Dog(handle, WFrictionwheel_Centre_Motor))
    {
        SFrictionwheel_Centre_Motor = Device_Right;
    }
    else if(IS_Dog(handle, WFrictionwheel_Left_Motor))
    {
        SFrictionwheel_Left_Motor = Device_Right;
    }
    else if(IS_Dog(handle, WPluck_Right_Motor))
    {
        SPluck_Right_Motor = Device_Right;
    }
    else if(IS_Dog(handle, WPluck_Left_Motor))
    {
        SPluck_Left_Motor = Device_Right;
    }
    else if(IS_Dog(handle, WRemote_Communication))
    {
        SRemote_Communication = Device_Right;
    }
    else if(IS_Dog(handle, WCan1_Communication))
    {
        SCan1_Communication = Device_Right;
    }
    else if(IS_Dog(handle, WCan2_Communication))
    {
        SCan2_Communication = Device_Right;
    }
    else if(IS_Dog(handle, WPC_Communication))
    {
        SPC_Communication = Device_Right;
    }
    else if(IS_Dog(handle, WIMU_Communication))
    {
        SIMU_Communication = Device_Right;
    }
    else if(IS_Dog(handle, WReferee_Communication_1))
    {
        SReferee_Communication_1 = Device_Right;
    }
    else if(IS_Dog(handle, WReferee_Communication_2))
    {
        SReferee_Communication_2 = Device_Right;
    }
}

void WatchDog_CallBack(WatchDogp handle)
{
    if(IS_Dog(handle, WGimbal_Yaw_Motor))
    {
        SGimbal_Yaw_Motor = Device_Error;
    }
    else if(IS_Dog(handle, WGimbal_Pitch_Left_Motor))
    {
        SGimbal_Pitch_Left_Motor = Device_Error;
    }
    else if(IS_Dog(handle, WGimbal_Pitch_Right_Motor))
    {
        SGimbal_Pitch_Right_Motor = Device_Error;
    }
    else if(IS_Dog(handle, WFrictionwheel_Right_Motor))
    {
        SFrictionwheel_Right_Motor = Device_Error;
    }
    else if(IS_Dog(handle, WFrictionwheel_Centre_Motor))
    {
        SFrictionwheel_Centre_Motor = Device_Error;
    }
    else if(IS_Dog(handle, WFrictionwheel_Left_Motor))
    {
        SFrictionwheel_Left_Motor = Device_Error;
    }
    else if(IS_Dog(handle, WPluck_Right_Motor))
    {
        SPluck_Right_Motor = Device_Error;
    }
    else if(IS_Dog(handle, WPluck_Left_Motor))
    {
        SPluck_Left_Motor = Device_Error;
    }
    else if(IS_Dog(handle, WRemote_Communication))
    {
        SRemote_Communication = Device_Error;
    }
    else if(IS_Dog(handle, WCan1_Communication))
    {
        SCan1_Communication = Device_Error;
    }
    else if(IS_Dog(handle, WCan2_Communication))
    {
        SCan2_Communication = Device_Error;
    }
    else if(IS_Dog(handle, WPC_Communication))
    {
        SPC_Communication = Device_Error;
        Lock_Robot_Rx = 0;
        Shoot_Rx = 0;
        Aim_rx.aim_shoot = 0;
        Aim_rx.dis_r = 0;
        Aim_rx.p_angle_r = 0;
        Aim_rx.dis_rx = 0;
        Aim_rx.p_angle_rx = 0;
        Aim_rx.p_angle_rx_off = 0;
        Aim_rx.y_angle_r = 0;
        Aim_rx.y_angle_rx = 0;
        Aim_rx.y_angle_rx_off = 0;
    }
    else if(IS_Dog(handle, WIMU_Communication))
    {
        SIMU_Communication = Device_Error;
    }
    else if(IS_Dog(handle, WReferee_Communication_1))
    {
        SReferee_Communication_1 = Device_Error;
    }
    else if(IS_Dog(handle, WReferee_Communication_2))
    {
        SReferee_Communication_2 = Device_Error;
    }
}
