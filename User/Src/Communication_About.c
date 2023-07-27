#include "Communication_About.h"
#include "Gimbal_Task.h"
#include "Shoot_Task.h"
#include "WatchDog_Task.h"

uint8_t usart1_dma_buf[USART1_LEN];

The_RX_Buffer_1_ The_RX_Buffer_1;
The_RX_Buffer_2_ The_RX_Buffer_2;

SemaphoreHandle_t Remote_DMA_Finish_Semaphore;
TaskHandle_t Remote_Receive_Task_Handle;
void Remote_Receive_Task(void *pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    BaseType_t Usart_return_data = pdFALSE;
    for(;;)
    {
        Usart_return_data = xSemaphoreTake(Remote_DMA_Finish_Semaphore, portMAX_DELAY);
        if(Usart_return_data == pdTRUE)
        {
            Remote_Rx(usart1_dma_buf);
        }
        else if(Usart_return_data == pdFALSE)
        {
            vTaskDelay(10);
        }  
        vTaskDelayUntil(&xLastWakeTime,2);
    }
}

void RemoteControlProcess(Remote *rc) 
{
    if(Stop_After_Flag == 1)
    {
        PID_Param_Init_Gimbal();
        PID_Param_Init_Shoot();
        Robot_Shoot_Status = Stop_Shoot;
        Stop_After_Flag = 0;
        Robot_Status = Doing_Init;
    }
}

void MouseKeyControlProcess(Mouse *mouse, Key_t key, Key_t Lastkey) {
    UNUSED(mouse);
    UNUSED(key);
}

void STOPControlProcess(void)
{
    Shoot_Expect.Frictionwheel_Right_Speed_Exp = 0;
    Shoot_Expect.Frictionwheel_Centre_Speed_Exp = 0;
    Shoot_Expect.Frictionwheel_Left_Speed_Exp = 0;

    Shoot_Expect.Pluck_Right_Speed_Exp = 0;
    Shoot_Expect.Pluck_Left_Speed_Exp = 0;
    
    Expect_Speed.Expect_Speed_X = 0;
    Expect_Speed.Expect_Speed_Y = 0;
    Expect_Speed.Expect_Speed_Yaw = 0;
    
    Tx_Chassis_Exp.Chassis_Speed_Exp.speed_1 = 0;
    Tx_Chassis_Exp.Chassis_Speed_Exp.speed_2 = 0;
    Tx_Chassis_Exp.Chassis_Speed_Exp.speed_3 = 0;
    Tx_Chassis_Exp.Chassis_Speed_Exp.speed_4 = 0;
    ChassisSpeed_Ref.rotate_ref = 0;
    
    Robot_Status = Finish_Init;
    
    PID_Param_Clear_Gimbal();
    PID_Param_Clear_Shoot();
    
    Stop_After_Flag = 1;
    return;
}

// CAN1 云台 Pitch Yaw 轴电机 两个拨弹
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t ID = CAN_Receive_DataFrame(&hcan1, CAN1_buff);
    if (hcan->Instance == CAN1)
    {
        switch(ID)
        {
            case 0x205:
                GM6020_Receive(&Gimbal_Pitch_Right_Motor, CAN1_buff);  // pitch left -- ID = 2  pitch right -- ID = 1
                Feed_Dog(&WGimbal_Pitch_Right_Motor);
                break;
            case 0x206:
                GM6020_Receive(&Gimbal_Pitch_Left_Motor, CAN1_buff);  // pitch left -- ID = 2  pitch right -- ID = 1
                Feed_Dog(&WGimbal_Pitch_Left_Motor);
                break;
            case 0x207:
                GM6020_Receive(&Gimbal_Yaw_Motor, CAN1_buff); // yaw -- ID = 3
                Feed_Dog(&WGimbal_Yaw_Motor);
                break;
            case 0x203:
                M2006_Receive(&Pluck_Right_Motor, CAN1_buff);
                Feed_Dog(&WPluck_Right_Motor);
                break;
            case 0x204:
                M2006_Receive(&Pluck_Left_Motor, CAN1_buff);
                Feed_Dog(&WPluck_Left_Motor);
                break;
            case 0x111:
                memcpy(&The_RX_Buffer_1, CAN1_buff, sizeof(The_RX_Buffer_1_));
                Feed_Dog(&WReferee_Communication_1);
                break;
            case 0x222:
                memcpy(&The_RX_Buffer_2, CAN1_buff, sizeof(The_RX_Buffer_2_));
                Feed_Dog(&WReferee_Communication_2);
                break;
        }
        Feed_Dog(&WCan1_Communication);
    }
}

// CAN2 三个摩擦轮 与下板通信
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t ID = CAN_Receive_DataFrame(&hcan2, CAN2_buff);
    if (hcan->Instance == CAN2)
    {
        switch(ID)
        {
            case 0x201:
                RM3508_Receive(&Frictionwheel_Right_Motor, CAN2_buff);
                Feed_Dog(&WFrictionwheel_Right_Motor);
                break;
            case 0x202:
                RM3508_Receive(&Frictionwheel_Centre_Motor, CAN2_buff);
                Feed_Dog(&WFrictionwheel_Centre_Motor);
                break;
            case 0x203:
                RM3508_Receive(&Frictionwheel_Left_Motor, CAN2_buff);
                Feed_Dog(&WFrictionwheel_Left_Motor);
                break;
        }
        Feed_Dog(&WCan2_Communication);
    }
}
