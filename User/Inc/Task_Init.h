#ifndef _TASK_INIT_H_
#define _TASK_INIT_H_

#include "stm32f4xx_hal.h"
#include "can.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"
#include "dma.h"	

#include "FreeRTOS.h"
#include "semphr.h"
#include "cmsis_os.h"
#include "task.h"
#include "queue.h"

#include "CANDrive.h"
#include "Chassis.h"
#include "motor.h"
#include "PID.h"
#include "ramp.h"
#include "remote.h"
#include "WatchDog.h"
#include "kalman.h"

#include "IMU.h"

#include "PC_Task.h"
#include "Gimbal_Task.h"
#include "Shoot_Task.h"
#include "Communication_About.h"
#include "WatchDog_Task.h"
#include "Chassis_Task.h"
#include "IMU_Task.h"

/**
 * @brief 机器人运行状态枚举
 */
typedef enum
{
    Doing_Init = 0,         //!<@brief 开机时状态，需要进行归中初始化
    Finish_Init,            //!<@brief 归中初始化完成状态
    
    Robot_Error             //!<@brief 机器人错误
} Robot_Status_;
extern Robot_Status_ Robot_Status;

extern uint8_t dPatro_Angle_Pitch;        //!<@brief 巡逻状态下P轴的周期变化量
extern float dPatro_EulerAngler_Yaw;

/**
 * @brief 读取机器人当前状态
 * @param Robot_Status_：机器人运行状态枚举
 * @return 返回机器人当前状态
 */
inline Robot_Status_ Read_Status(Robot_Status_ status)
{
    return status;
}

extern uint8_t Stop_After_Flag;                 //!<@brief 急停恢复标志位

extern TaskHandle_t Robot_Init_Task_Handle;     //!<@brief 机器人归中初始化任务句柄

extern SemaphoreHandle_t Weak_Up_Administrator_Semaphore;
extern uint8_t Administrator_Work_Flag;
extern TaskHandle_t Task_Administrator_Handle;
void Task_Administrator(void *pvParameters);

/**
 * @brief 任务初始化函数
 */
void Start_Task(void);

#endif 
