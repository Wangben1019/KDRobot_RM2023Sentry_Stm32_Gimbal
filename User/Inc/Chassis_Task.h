#ifndef _CHASSIS_TASK_H_
#define _CHASSIS_TASK_H_

#include "Task_Init.h"

#define SPEED_LEVEL 2100
#define RAD2ANGLE 57.295779
#define Planner_Pub_Hz 100

typedef struct
{
    float Current_Speed_X;
    float Current_Speed_Y;
    float Current_Speed_Yaw;
} Current_Speed_Typedef;
extern Current_Speed_Typedef Current_Speed;

typedef struct
{
    double Expect_Speed_X;
    double Expect_Speed_Y;
    double Expect_Speed_Yaw; 
} Expect_Speed_Typedef;
extern Expect_Speed_Typedef Expect_Speed;

typedef union
{
    uint8_t can_buff[8];
    Chassis_Motor_Speed Chassis_Speed_Exp;
} Tx_Chassis_Exp_;
extern Tx_Chassis_Exp_ Tx_Chassis_Exp;

extern double chassis_expect[4];

#if DEBUG 
extern ChassisSpeed_Ref_t ChassisSpeed_Ref;
#endif

//typedef struct
//{
//    Current_Speed_Typedef Current_Speed;
//    Expect_Speed_Typedef  Expect_Speed;
//} Robot_Chassis_Speed_Typedef;

extern TaskHandle_t Chassis_Task_Handle;
void Chassis_Control_Task(void *pvParameters);

#endif 
