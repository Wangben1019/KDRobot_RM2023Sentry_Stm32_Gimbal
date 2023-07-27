#include "IMU_Task.h"

IMU_Typedef IMU_Data_User;
User_EulerAngler_Typedef IMU_Start_EulerAngler_Data;
uint8_t usart6_dma_buf[USART6_LEN];

float a[100];
int i;

SemaphoreHandle_t IMU_DMA_Finish_Semaphore;
TaskHandle_t Imu_Data_Task_Handle;
void Imu_Data_Task(void *pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    BaseType_t IMU_return_data = pdFALSE;
    for(;;)
    {
        IMU_return_data = xSemaphoreTake(IMU_DMA_Finish_Semaphore, portMAX_DELAY);
        if(IMU_return_data == pdTRUE)
        {
            IMU_Receive(&IMU_Data_User, usart6_dma_buf);
        }
        vTaskDelayUntil(&xLastWakeTime,2);
    }
}

void Get_Imu_User_EulerAngler(User_EulerAngler_Typedef *user_eulerangler, IMU_Typedef *imu_data)
{
    user_eulerangler->Pitch = imu_data->EulerAngler.Pitch;
    user_eulerangler->Roll = imu_data->EulerAngler.Roll;
    user_eulerangler->Yaw = imu_data->EulerAngler.Yaw;
    user_eulerangler->r = imu_data->EulerAngler.r;
    user_eulerangler->LsatAngle = imu_data->EulerAngler.LsatAngle;
    user_eulerangler->ContinuousYaw = imu_data->EulerAngler.ContinuousYaw;
    user_eulerangler->Yawoffset = imu_data->EulerAngler.Yawoffset;
}

