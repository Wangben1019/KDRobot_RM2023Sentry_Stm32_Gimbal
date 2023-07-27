#include "Chassis_Task.h"

Current_Speed_Typedef Current_Speed;
Expect_Speed_Typedef Expect_Speed;
Tx_Chassis_Exp_ Tx_Chassis_Exp;
double chassis_expect[4] = {0, 0, 0, 0};
float chassis_offset;
float exp_chassis_body_x;
float exp_chassis_body_y;

//float Chassis_Angle_Compensate = 0;

ChassisSpeed_Ref_t ChassisSpeed_Ref;
void ChassisMotorSpeed_get(Chassis_Motor_Speed *motor, ChassisSpeed_Ref_t *ref) 
{
    motor->speed_1 = ref->forward_back_ref - ref->left_right_ref + ref->rotate_ref;
    motor->speed_2 = ref->forward_back_ref + ref->left_right_ref + ref->rotate_ref;
    motor->speed_3 = -ref->forward_back_ref - ref->left_right_ref + ref->rotate_ref;
    motor->speed_4 = -ref->forward_back_ref + ref->left_right_ref + ref->rotate_ref;
}

TaskHandle_t Chassis_Task_Handle;
void Chassis_Control_Task(void *pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    for(;;)
    {
        ChassisMotorSpeed_get(&(Tx_Chassis_Exp.Chassis_Speed_Exp), &ChassisSpeed_Ref);
//        CAN_Send_StdDataFrame(&hcan1, 0x147, (uint8_t * )&Tx_Chassis_Exp);    
        vTaskDelayUntil(&xLastWakeTime,2);
    }
}
