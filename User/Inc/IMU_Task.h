#ifndef _IMU_TASK_H_
#define _IMU_TASK_H_

#include "Task_Init.h"

#define USART6_LEN 30

extern uint8_t usart6_dma_buf[USART6_LEN];

extern IMU_Typedef IMU_Data_User;   //!<@brief IMU数据接收结构体

extern SemaphoreHandle_t IMU_DMA_Finish_Semaphore;
extern TaskHandle_t Imu_Data_Task_Handle;
void Imu_Data_Task(void *pvParameters);

typedef struct {
    float Pitch;
    float Roll;
    float Yaw;
    int16_t r;
    float LsatAngle;
    float ContinuousYaw;
    float Yawoffset;
} User_EulerAngler_Typedef;
extern User_EulerAngler_Typedef IMU_Start_EulerAngler_Data;

void Get_Imu_User_EulerAngler(User_EulerAngler_Typedef *user_eulerangler, IMU_Typedef *imu_data);

#endif 
