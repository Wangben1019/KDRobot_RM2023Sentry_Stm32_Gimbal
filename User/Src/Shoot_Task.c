#include "Shoot_Task.h"

Robot_Shoot_Direction_ Robot_Shoot_Direction = Shoot_Right;

Robot_Shoot_Status_ Robot_Shoot_Status = Stop_Shoot;
Robot_Shoot_Status_ Last_Robot_Shoot_Status;

RM3508_TypeDef Frictionwheel_Right_Motor;
RM3508_TypeDef Frictionwheel_Centre_Motor;
RM3508_TypeDef Frictionwheel_Left_Motor;

M2006_TypeDef Pluck_Right_Motor;
M2006_TypeDef Pluck_Left_Motor;

PID Frictionwheel_Right_Speed_PID;
PID Frictionwheel_Centre_Speed_PID;
PID Frictionwheel_Left_Speed_PID;

PID Pluck_Right_Speed_PID;
PID Pluck_Left_Speed_PID;

Shoot_Expect_ Shoot_Expect;

int16_t msg_tx_1[4];
int16_t msg_tx_2[4];

void Get_Shoot_Exp(void);

TaskHandle_t Shoot_Control_Task_Handle;
void Shoot_Control_Task(void *pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    for(;;)
    {
        Get_Shoot_Exp();
        
        PID_Control(Frictionwheel_Right_Motor.Speed, Shoot_Expect.Frictionwheel_Right_Speed_Exp, &Frictionwheel_Right_Speed_PID);
        PID_Control(Frictionwheel_Centre_Motor.Speed, Shoot_Expect.Frictionwheel_Centre_Speed_Exp, &Frictionwheel_Centre_Speed_PID);
        PID_Control(Frictionwheel_Left_Motor.Speed, Shoot_Expect.Frictionwheel_Left_Speed_Exp, &Frictionwheel_Left_Speed_PID);
        
        PID_Control(Pluck_Right_Motor.Speed, Shoot_Expect.Pluck_Right_Speed_Exp, &Pluck_Right_Speed_PID);
        PID_Control(Pluck_Left_Motor.Speed, Shoot_Expect.Pluck_Left_Speed_Exp, &Pluck_Left_Speed_PID);
        
        limit(Frictionwheel_Right_Speed_PID.pid_out, RM3510_LIMIT, -RM3510_LIMIT);
        limit(Frictionwheel_Centre_Speed_PID.pid_out, RM3510_LIMIT, -RM3510_LIMIT);
        limit(Frictionwheel_Left_Speed_PID.pid_out, RM3510_LIMIT, -RM3510_LIMIT);
        
        limit(Pluck_Right_Speed_PID.pid_out, M2006_LIMIT, -M2006_LIMIT);
        limit(Pluck_Left_Speed_PID.pid_out, M2006_LIMIT, -M2006_LIMIT);
        
        msg_tx_1[2] = Pluck_Right_Speed_PID.pid_out;
        msg_tx_1[3] = Pluck_Left_Speed_PID.pid_out;
        
        msg_tx_2[0] = Frictionwheel_Right_Speed_PID.pid_out;
        msg_tx_2[1] = Frictionwheel_Centre_Speed_PID.pid_out;
        msg_tx_2[2] = Frictionwheel_Left_Speed_PID.pid_out;
        
        MotorSend(&hcan1, 0x200, msg_tx_1);
        MotorSend(&hcan2, 0x200, msg_tx_2);
        
        vTaskDelayUntil(&xLastWakeTime,2);
    }
}

void Get_Shoot_Exp()
{
    if(Robot_Shoot_Status == Stop_Shoot)
    {
        Shoot_Expect.Frictionwheel_Right_Speed_Exp = 0;
        Shoot_Expect.Frictionwheel_Centre_Speed_Exp = 0;
        Shoot_Expect.Frictionwheel_Left_Speed_Exp = 0;

        Shoot_Expect.Pluck_Right_Speed_Exp = 0;
        Shoot_Expect.Pluck_Left_Speed_Exp = 0;
        
        Last_Robot_Shoot_Status = Stop_Shoot;
    }
    else if(Robot_Shoot_Status == Prepare_Shoot_Right)
    {
        Shoot_Expect.Frictionwheel_Right_Speed_Exp = FRICTIONWHELL_SPEED_EXP;
        Shoot_Expect.Frictionwheel_Centre_Speed_Exp = -FRICTIONWHELL_SPEED_EXP;
        Shoot_Expect.Frictionwheel_Left_Speed_Exp = 0;
        
        Shoot_Expect.Pluck_Right_Speed_Exp = 0;
        Shoot_Expect.Pluck_Left_Speed_Exp = 0;
        
        Last_Robot_Shoot_Status = Prepare_Shoot_Right;
    }
    else if(Robot_Shoot_Status == Start_Shoot_Right)
    {
        Shoot_Expect.Frictionwheel_Right_Speed_Exp = FRICTIONWHELL_SPEED_EXP;
        Shoot_Expect.Frictionwheel_Centre_Speed_Exp = -FRICTIONWHELL_SPEED_EXP;
        Shoot_Expect.Frictionwheel_Left_Speed_Exp = 0;
        
        Shoot_Expect.Pluck_Right_Speed_Exp = -PLUCK_SPEED_EXP;
        Shoot_Expect.Pluck_Left_Speed_Exp = 0;
        
        Last_Robot_Shoot_Status = Start_Shoot_Right;
    }
    else if(Robot_Shoot_Status == Pluck_Block_Right)
    {
        Shoot_Expect.Frictionwheel_Right_Speed_Exp = FRICTIONWHELL_SPEED_EXP;
        Shoot_Expect.Frictionwheel_Centre_Speed_Exp = -FRICTIONWHELL_SPEED_EXP;
        Shoot_Expect.Frictionwheel_Left_Speed_Exp = 0;
        
        Shoot_Expect.Pluck_Right_Speed_Exp = PLUCK_SPEED_EXP;
        Shoot_Expect.Pluck_Left_Speed_Exp = 0;
        
        Last_Robot_Shoot_Status = Pluck_Block_Right;
    }
    else if(Robot_Shoot_Status == Switch_LeftToRight)
    {
        Shoot_Expect.Frictionwheel_Right_Speed_Exp = FRICTIONWHELL_SPEED_EXP;
        Shoot_Expect.Frictionwheel_Centre_Speed_Exp = -FRICTIONWHELL_SPEED_EXP;
        Shoot_Expect.Frictionwheel_Left_Speed_Exp = 0;
        
        Shoot_Expect.Pluck_Right_Speed_Exp = 0;
        Shoot_Expect.Pluck_Left_Speed_Exp = 0;
        
        Last_Robot_Shoot_Status = Switch_LeftToRight;
    }
    else if(Robot_Shoot_Status == Prepare_Shoot_Left)
    {
        Shoot_Expect.Frictionwheel_Right_Speed_Exp = 0;
        Shoot_Expect.Frictionwheel_Centre_Speed_Exp = FRICTIONWHELL_SPEED_EXP;
        Shoot_Expect.Frictionwheel_Left_Speed_Exp = -FRICTIONWHELL_SPEED_EXP;
        
        Shoot_Expect.Pluck_Right_Speed_Exp = 0;
        Shoot_Expect.Pluck_Left_Speed_Exp = 0;
        
        Last_Robot_Shoot_Status = Prepare_Shoot_Left;
    }
    else if(Robot_Shoot_Status == Start_Shoot_Left)
    {
        Shoot_Expect.Frictionwheel_Right_Speed_Exp = 0;
        Shoot_Expect.Frictionwheel_Centre_Speed_Exp = FRICTIONWHELL_SPEED_EXP;
        Shoot_Expect.Frictionwheel_Left_Speed_Exp = -FRICTIONWHELL_SPEED_EXP;
        
        Shoot_Expect.Pluck_Right_Speed_Exp = 0;
        Shoot_Expect.Pluck_Left_Speed_Exp =  PLUCK_SPEED_EXP;
        
        Last_Robot_Shoot_Status = Start_Shoot_Left;
    }
    else if(Robot_Shoot_Status == Pluck_Block_Left)
    {
        Shoot_Expect.Frictionwheel_Right_Speed_Exp = 0;
        Shoot_Expect.Frictionwheel_Centre_Speed_Exp = FRICTIONWHELL_SPEED_EXP;
        Shoot_Expect.Frictionwheel_Left_Speed_Exp = -FRICTIONWHELL_SPEED_EXP;
        
        Shoot_Expect.Pluck_Right_Speed_Exp = 0;
        Shoot_Expect.Pluck_Left_Speed_Exp =  -PLUCK_SPEED_EXP;
        
        Last_Robot_Shoot_Status = Pluck_Block_Left;
    }
    else if(Robot_Shoot_Status == Switch_RightToLeft)
    {
        Shoot_Expect.Frictionwheel_Right_Speed_Exp = 0;
        Shoot_Expect.Frictionwheel_Centre_Speed_Exp = FRICTIONWHELL_SPEED_EXP;
        Shoot_Expect.Frictionwheel_Left_Speed_Exp = -FRICTIONWHELL_SPEED_EXP;
        
        Shoot_Expect.Pluck_Right_Speed_Exp = 0;
        Shoot_Expect.Pluck_Left_Speed_Exp = 0;
        
        Last_Robot_Shoot_Status = Switch_RightToLeft;
    }
}

void PID_Param_Init_Shoot()
{
    Frictionwheel_Right_Speed_PID.Kp = 5;
    Frictionwheel_Right_Speed_PID.Ki = 0;
    Frictionwheel_Right_Speed_PID.Kd = 0;
    Frictionwheel_Right_Speed_PID.error_inter = 0;
    Frictionwheel_Right_Speed_PID.error_last = 0;
    Frictionwheel_Right_Speed_PID.error_now = 0;
    Frictionwheel_Right_Speed_PID.limit = 5000;
    Frictionwheel_Right_Speed_PID.pid_out = 0;
    
    Frictionwheel_Centre_Speed_PID.Kp = 5;
    Frictionwheel_Centre_Speed_PID.Ki = 0;
    Frictionwheel_Centre_Speed_PID.Kd = 0;
    Frictionwheel_Centre_Speed_PID.error_inter = 0;
    Frictionwheel_Centre_Speed_PID.error_last = 0;
    Frictionwheel_Centre_Speed_PID.error_now = 0;
    Frictionwheel_Centre_Speed_PID.limit = 5000;
    Frictionwheel_Centre_Speed_PID.pid_out = 0;
    
    Frictionwheel_Left_Speed_PID.Kp = 5;
    Frictionwheel_Left_Speed_PID.Ki = 0;
    Frictionwheel_Left_Speed_PID.Kd = 0;
    Frictionwheel_Left_Speed_PID.error_inter = 0;
    Frictionwheel_Left_Speed_PID.error_last = 0;
    Frictionwheel_Left_Speed_PID.error_now = 0;
    Frictionwheel_Left_Speed_PID.limit = 5000;
    Frictionwheel_Left_Speed_PID.pid_out = 0;
    
    Pluck_Right_Speed_PID.Kp = 5;
    Pluck_Right_Speed_PID.Ki = 0;
    Pluck_Right_Speed_PID.Kd = 0;
    Pluck_Right_Speed_PID.error_inter = 0;
    Pluck_Right_Speed_PID.error_last = 0;
    Pluck_Right_Speed_PID.error_now = 0;
    Pluck_Right_Speed_PID.limit = 5000;
    Pluck_Right_Speed_PID.pid_out = 0;
    
    Pluck_Left_Speed_PID.Kp = 5;
    Pluck_Left_Speed_PID.Ki = 0;
    Pluck_Left_Speed_PID.Kd = 0;
    Pluck_Left_Speed_PID.error_inter = 0;
    Pluck_Left_Speed_PID.error_last = 0;
    Pluck_Left_Speed_PID.error_now = 0;
    Pluck_Left_Speed_PID.limit = 5000;
    Pluck_Left_Speed_PID.pid_out = 0;
}

void PID_Param_Clear_Shoot()
{
    Frictionwheel_Right_Speed_PID.Kp = 0;
    Frictionwheel_Right_Speed_PID.Ki = 0;
    Frictionwheel_Right_Speed_PID.Kd = 0;
    Frictionwheel_Right_Speed_PID.error_inter = 0;
    Frictionwheel_Right_Speed_PID.error_last = 0;
    Frictionwheel_Right_Speed_PID.error_now = 0;
    Frictionwheel_Right_Speed_PID.limit = 5000;
    Frictionwheel_Right_Speed_PID.pid_out = 0;
    
    Frictionwheel_Centre_Speed_PID.Kp = 0;
    Frictionwheel_Centre_Speed_PID.Ki = 0;
    Frictionwheel_Centre_Speed_PID.Kd = 0;
    Frictionwheel_Centre_Speed_PID.error_inter = 0;
    Frictionwheel_Centre_Speed_PID.error_last = 0;
    Frictionwheel_Centre_Speed_PID.error_now = 0;
    Frictionwheel_Centre_Speed_PID.limit = 5000;
    Frictionwheel_Centre_Speed_PID.pid_out = 0;
    
    Frictionwheel_Left_Speed_PID.Kp = 0;
    Frictionwheel_Left_Speed_PID.Ki = 0;
    Frictionwheel_Left_Speed_PID.Kd = 0;
    Frictionwheel_Left_Speed_PID.error_inter = 0;
    Frictionwheel_Left_Speed_PID.error_last = 0;
    Frictionwheel_Left_Speed_PID.error_now = 0;
    Frictionwheel_Left_Speed_PID.limit = 5000;
    Frictionwheel_Left_Speed_PID.pid_out = 0;
    
    Pluck_Right_Speed_PID.Kp = 0;
    Pluck_Right_Speed_PID.Ki = 0;
    Pluck_Right_Speed_PID.Kd = 0;
    Pluck_Right_Speed_PID.error_inter = 0;
    Pluck_Right_Speed_PID.error_last = 0;
    Pluck_Right_Speed_PID.error_now = 0;
    Pluck_Right_Speed_PID.limit = 5000;
    Pluck_Right_Speed_PID.pid_out = 0;
    
    Pluck_Left_Speed_PID.Kp = 0;
    Pluck_Left_Speed_PID.Ki = 0;
    Pluck_Left_Speed_PID.Kd = 0;
    Pluck_Left_Speed_PID.error_inter = 0;
    Pluck_Left_Speed_PID.error_last = 0;
    Pluck_Left_Speed_PID.error_now = 0;
    Pluck_Left_Speed_PID.limit = 5000;
    Pluck_Left_Speed_PID.pid_out = 0;
}
