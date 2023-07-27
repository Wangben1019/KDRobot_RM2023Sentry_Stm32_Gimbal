#ifndef _WATCHDOG_TASK_H_
#define _WATCHDOG_TASK_H_

#include "Task_Init.h"

extern WatchDog_TypeDef WGimbal_Yaw_Motor;
extern WatchDog_TypeDef WGimbal_Pitch_Left_Motor;
extern WatchDog_TypeDef WGimbal_Pitch_Right_Motor;
extern WatchDog_TypeDef WFrictionwheel_Right_Motor;
extern WatchDog_TypeDef WFrictionwheel_Centre_Motor;
extern WatchDog_TypeDef WFrictionwheel_Left_Motor;
extern WatchDog_TypeDef WPluck_Right_Motor;
extern WatchDog_TypeDef WPluck_Left_Motor;

extern WatchDog_TypeDef WRemote_Communication;
extern WatchDog_TypeDef WCan1_Communication;
extern WatchDog_TypeDef WCan2_Communication;
extern WatchDog_TypeDef WPC_Communication;
extern WatchDog_TypeDef WIMU_Communication;
extern WatchDog_TypeDef WReferee_Communication_1;
extern WatchDog_TypeDef WReferee_Communication_2;

extern TaskHandle_t WatchDog_Task_Handle;
void WatchDog_Task(void *pvParameters);

void WatchDog_Init_Gather(void);

extern TaskHandle_t Monitor_Device_Status_Task_Handle;
void Monitor_Device_Status_Task(void *pvParameters);

typedef enum
{
    Device_Error = 0,
    Device_Right = 1
} Device_Status;

#endif 
