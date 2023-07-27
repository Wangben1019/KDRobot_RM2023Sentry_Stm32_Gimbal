#include "Gimbal_Task.h"

Robot_Gimbal_Status_ Robot_Gimbal_Status = Patro_Mode;

GM6020_TypeDef Gimbal_Yaw_Motor;
GM6020_TypeDef Gimbal_Pitch_Left_Motor;     // 抬头时，左侧电机正转
GM6020_TypeDef Gimbal_Pitch_Right_Motor;    // 抬头时，右侧电机反转

PID_Smis Gimbal_Yaw_Place_PID;
PID Gimbal_Yaw_Speed_PID;

PID Gimbal_Yaw_Speed_Center_PID;

PID_Smis Gimbal_Pitch_Right_Place_PID;
PID Gimbal_Pitch_Right_Speed_PID;

PID Gimbal_Pitch_Right_Speed_Center_PID;
PID Gimbal_Pitch_Left_Speed_Center_PID;

PID Chassis_Follow_PID;

Gimbal_Expect_ Gimbal_Expect;

float P_LIMIT = 0;           //p轴限制
float P_LIMIT_Last = 0;

float Y_expect_lv = 0;  	 //y轴滤波值
float Y_expect_lv_last;

float Pitch_Init_EulerAngler;
float Yaw_Init_EulerAngler;

int16_t msg_ptz[4];
float ki_error = 0;
float DOut = 0;
float PIDOut_None_D;

float Chassis_Yaw_Exp_Angle = 0;

TaskHandle_t Gimbal_Control_Task_Handle;
void Gimbal_Control_Task(void *pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    for(;;)
    {
        if(Robot_Status == Finish_Init)
        {      
            limit(Gimbal_Expect.Gimbal_Pitch_Right_Exp, PITCH_RIGHT_BOTTOM_ANGLE - GIMBAL_PITCH_RIGHT_CENTRE, PITCH_RIGHT_TOP_ANGLE - GIMBAL_PITCH_RIGHT_CENTRE);
            Chassis_Yaw_Exp_Angle += (((1.0f / Planner_Pub_Hz) * Expect_Speed.Expect_Speed_Yaw) * RAD2ANGLE);
                
            P_LIMIT = GIMBAL_PITCH_RIGHT_CENTRE + Pitch_Init_EulerAngler;
//            P_LIMIT_Last = P_LIMIT;
//            limit(P_LIMIT, GIMBAL_PITCH_RIGHT_CENTRE + 500, GIMBAL_PITCH_RIGHT_CENTRE - 350);
            /* P 的 PID */
            PID_Control_Smis(Gimbal_Pitch_Right_Motor.MchanicalAngle , GIMBAL_PITCH_RIGHT_CENTRE + Gimbal_Expect.Gimbal_Pitch_Right_Exp, &Gimbal_Pitch_Right_Place_PID, Gimbal_Pitch_Right_Motor.Speed);
            PID_Control(Gimbal_Pitch_Right_Motor.Speed, Gimbal_Pitch_Right_Place_PID.pid_out,&Gimbal_Pitch_Right_Speed_PID);
            limit(Gimbal_Pitch_Right_Speed_PID.pid_out, GM6020_LIMIT, -GM6020_LIMIT);
            
            /* Y 的 PID */        
            Y_expect_lv = Kalman_Filter(&Yaw_control, Yaw_Init_EulerAngler);
            Y_expect_lv_last = Y_expect_lv;
//            PID_Control_Smis(IMU_Data_User.EulerAngler.ContinuousYaw, Y_expect_lv + Gimbal_Expect.Yaw_Exp_EulerAngler + Chassis_Yaw_Exp_Angle, &Gimbal_Yaw_Place_PID, IMU_Data_User.AngularVelocity.Z);
            PID_Control_Smis(IMU_Data_User.EulerAngler.ContinuousYaw, Y_expect_lv + Gimbal_Expect.Yaw_Exp_EulerAngler, &Gimbal_Yaw_Place_PID, IMU_Data_User.AngularVelocity.Z);
            PID_Control(IMU_Data_User.AngularVelocity.Z, Gimbal_Yaw_Place_PID.pid_out, &Gimbal_Yaw_Speed_PID);
            limit(Gimbal_Yaw_Speed_PID.pid_out, GM6020_LIMIT, -GM6020_LIMIT);
            
            Motor_Angle_Feedback.Pitch_EulerAngler_Feedback = IMU_Data_User.EulerAngler.Pitch;
            Motor_Angle_Feedback.Yaw_EulerAngler_Feedback = IMU_Data_User.EulerAngler.Yaw;
            
            msg_ptz[0] = Gimbal_Pitch_Right_Speed_PID.pid_out;
            msg_ptz[1] = -Gimbal_Pitch_Right_Speed_PID.pid_out;
            msg_ptz[2] = Gimbal_Yaw_Speed_PID.pid_out;
            
            PID_Control(Gimbal_Yaw_Motor.MchanicalAngle, GIMBAL_YAW_CENTRE, &Chassis_Follow_PID);
            
            MotorSend(&hcan1, 0x1FF, msg_ptz);
            if(Administrator_Work_Flag == 1)
            {
                vTaskResume(Chassis_Task_Handle);
                vTaskResume(Shoot_Control_Task_Handle);
                vTaskResume(Remote_Receive_Task_Handle);
                vTaskResume(PC_Task_Handle);
                vTaskResume(Monitor_Device_Status_Task_Handle);
                vTaskResume(Task_Administrator_Handle);
                Administrator_Work_Flag = 0;
            }
        }
        vTaskDelayUntil(&xLastWakeTime,2);
    }
}

void PID_Param_Init_Center()
{
    Gimbal_Yaw_Speed_Center_PID.Kp = 20;
    Gimbal_Yaw_Speed_Center_PID.Ki = 0;
    Gimbal_Yaw_Speed_Center_PID.Kd = 0;
    Gimbal_Yaw_Speed_Center_PID.error_inter = 0;
    Gimbal_Yaw_Speed_Center_PID.error_last = 0;
    Gimbal_Yaw_Speed_Center_PID.error_now = 0;
    Gimbal_Yaw_Speed_Center_PID.limit = 5000;
    Gimbal_Yaw_Speed_Center_PID.pid_out = 0;
    
    Gimbal_Pitch_Left_Speed_Center_PID.Kp = 180;
    Gimbal_Pitch_Left_Speed_Center_PID.Ki = 0.02;
    Gimbal_Pitch_Left_Speed_Center_PID.Kd = 0;
    Gimbal_Pitch_Left_Speed_Center_PID.error_inter = 0;
    Gimbal_Pitch_Left_Speed_Center_PID.error_last = 0;
    Gimbal_Pitch_Left_Speed_Center_PID.error_now = 0;
    Gimbal_Pitch_Left_Speed_Center_PID.limit = 1000;
    Gimbal_Pitch_Left_Speed_Center_PID.pid_out = 0;
    
    Gimbal_Pitch_Right_Speed_Center_PID.Kp = 180; 
    Gimbal_Pitch_Right_Speed_Center_PID.Ki = 0.02; 
    Gimbal_Pitch_Right_Speed_Center_PID.Kd = 0;   
    Gimbal_Pitch_Right_Speed_Center_PID.error_inter = 0;
    Gimbal_Pitch_Right_Speed_Center_PID.error_last = 0;
    Gimbal_Pitch_Right_Speed_Center_PID.error_now = 0;
    Gimbal_Pitch_Right_Speed_Center_PID.limit = 1000;
    Gimbal_Pitch_Right_Speed_Center_PID.pid_out = 0;
}

void PID_Param_Init_Gimbal()
{
    Gimbal_Yaw_Place_PID.Kp = 25;
    Gimbal_Yaw_Place_PID.Ki = 0.0295;
    Gimbal_Yaw_Place_PID.Kd = -3.5;
    Gimbal_Yaw_Place_PID.error_inter = 0;
    Gimbal_Yaw_Place_PID.error_now = 0;
    Gimbal_Yaw_Place_PID.error_thre = 8.789062;
    Gimbal_Yaw_Place_PID.limit = 3000;
    Gimbal_Yaw_Place_PID.DeadBand = 0.219726;
    Gimbal_Yaw_Place_PID.pid_out = 0;
    
    Gimbal_Yaw_Speed_PID.Kp = 60;
    Gimbal_Yaw_Speed_PID.Ki = 0;
    Gimbal_Yaw_Speed_PID.Kd = 15;
    Gimbal_Yaw_Speed_PID.error_inter = 0;
    Gimbal_Yaw_Speed_PID.error_last = 0;
    Gimbal_Yaw_Speed_PID.error_now = 0;
    Gimbal_Yaw_Speed_PID.error_thre = 8.789062;
    Gimbal_Yaw_Speed_PID.limit = 219.726562;
    Gimbal_Yaw_Speed_PID.DeadBand = 0.219726;
    Gimbal_Yaw_Speed_PID.pid_out = 0;

    Gimbal_Pitch_Right_Place_PID.Kp = 13;   // 13
    Gimbal_Pitch_Right_Place_PID.Ki = 0.1;  // 0.1
    Gimbal_Pitch_Right_Place_PID.Kd = -5;    // -5
    Gimbal_Pitch_Right_Place_PID.error_inter = 0;
    Gimbal_Pitch_Right_Place_PID.error_now = 0;
    Gimbal_Pitch_Right_Place_PID.error_thre = 200;
    Gimbal_Pitch_Right_Place_PID.limit = 5000;
    Gimbal_Pitch_Right_Place_PID.DeadBand = 5;
    Gimbal_Pitch_Right_Place_PID.pid_out = 0;
    
    Gimbal_Pitch_Right_Speed_PID.Kp = 8.5;  // 8.5
    Gimbal_Pitch_Right_Speed_PID.Ki = 0;    // 0
    Gimbal_Pitch_Right_Speed_PID.Kd = 3;    // 3
    Gimbal_Pitch_Right_Speed_PID.error_inter = 0;
    Gimbal_Pitch_Right_Speed_PID.error_last = 0;
    Gimbal_Pitch_Right_Speed_PID.error_now = 0;
    Gimbal_Pitch_Right_Speed_PID.error_thre = 200;
    Gimbal_Pitch_Right_Speed_PID.limit = 5000;
    Gimbal_Pitch_Right_Speed_PID.DeadBand = 5;
    Gimbal_Pitch_Right_Speed_PID.pid_out = 0;
    
    Chassis_Follow_PID.Kp = 3;  
    Chassis_Follow_PID.Ki = 0;    
    Chassis_Follow_PID.Kd = 15;    
    Chassis_Follow_PID.error_inter = 0;
    Chassis_Follow_PID.error_last = 0;
    Chassis_Follow_PID.error_now = 0;
    Chassis_Follow_PID.error_thre = 200;
    Chassis_Follow_PID.limit = 5000;
    Chassis_Follow_PID.DeadBand = 5;
    Chassis_Follow_PID.pid_out = 0;
}                           
void PID_Param_Clear_Gimbal()
{
    Gimbal_Yaw_Place_PID.Kp = 0;
    Gimbal_Yaw_Place_PID.Ki = 0;
    Gimbal_Yaw_Place_PID.Kd = 0;
    Gimbal_Yaw_Place_PID.error_inter = 0;
    Gimbal_Yaw_Place_PID.error_now = 0;
    Gimbal_Yaw_Place_PID.limit = 5000;
    Gimbal_Yaw_Place_PID.pid_out = 0;
    
    Gimbal_Yaw_Speed_PID.Kp = 0;
    Gimbal_Yaw_Speed_PID.Ki = 0;
    Gimbal_Yaw_Speed_PID.Kd = 0;
    Gimbal_Yaw_Speed_PID.error_inter = 0;
    Gimbal_Yaw_Speed_PID.error_last = 0;
    Gimbal_Yaw_Speed_PID.error_now = 0;
    Gimbal_Yaw_Speed_PID.limit = 0;
    Gimbal_Yaw_Speed_PID.pid_out = 0;
    
    Gimbal_Pitch_Right_Place_PID.Kp = 0;  
    Gimbal_Pitch_Right_Place_PID.Ki = 0;  
    Gimbal_Pitch_Right_Place_PID.Kd = 0;  
    Gimbal_Pitch_Right_Place_PID.error_inter = 0;
    Gimbal_Pitch_Right_Place_PID.error_now = 0;
    Gimbal_Pitch_Right_Place_PID.limit = 0;
    Gimbal_Pitch_Right_Place_PID.pid_out = 0;
                 
    Gimbal_Pitch_Right_Speed_PID.Kp = 0;    
    Gimbal_Pitch_Right_Speed_PID.Ki = 0;    
    Gimbal_Pitch_Right_Speed_PID.Kd = 0;    
    Gimbal_Pitch_Right_Speed_PID.error_inter = 0;
    Gimbal_Pitch_Right_Speed_PID.error_last = 0;
    Gimbal_Pitch_Right_Speed_PID.error_now = 0;
    Gimbal_Pitch_Right_Speed_PID.limit = 0;
    Gimbal_Pitch_Right_Speed_PID.pid_out = 0;
}
