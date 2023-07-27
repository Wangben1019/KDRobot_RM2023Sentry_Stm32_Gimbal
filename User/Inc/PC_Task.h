#ifndef _PC_TASK_H_
#define _PC_TASK_H_

#include "Task_Init.h"
#include "kalmanII.h"

extern SemaphoreHandle_t PC_Receive_Finish_Semaphore;

extern TaskHandle_t PC_Task_Handle;
void PC_Task(void *pvParameters);

typedef enum {
    UnLock_Robot = 0,   //!<@brief 未锁定敌方机器人
    Lock_Robot = 1,         //!<@brief 已锁定敌方机器人
    Shoot = 2               //!<@brief 发射
} Aim_Status_;
extern Aim_Status_ Aim_Shoot_Status;
extern int Lock_Robot_Rx;
extern int Shoot_Rx;

typedef struct {
    float Aim_Pitch;           //!<@brief Pitch轴转动角度
    float Aim_Yaw;             //!<@brief Yaw轴转动角度
    int Lock_Robot;
    int Shoot;
    float Lock_Robot_Distance;  //!<@brief 距离敌方机器人的距离
//    int Aim_Status_Flag;
} PC_Gimbal_Control_Typedef;
extern PC_Gimbal_Control_Typedef PC_Gimbal_Control;

extern kalman_filter_t Yaw_control;
extern uint32_t time_aim;

typedef struct {
    float Pitch_EulerAngler_Feedback;
    float Yaw_EulerAngler_Feedback;
} Motor_Angle_Feedback_Typedef;
extern Motor_Angle_Feedback_Typedef Motor_Angle_Feedback;

typedef struct{
    float y_angle_r;
    float p_angle_r;
    float dis_r;
    
	float y_angle_rx;
	float p_angle_rx;	
	float dis_rx;
	float y_angle_rx_off;
	float p_angle_rx_off;
    uint8_t aim_shoot;
}aim_rx;
extern aim_rx Aim_rx;

typedef struct{
	uint8_t time;
	float IMU_data_delay[200];
	uint16_t IMU_need;
}IMU_st_delay;
extern IMU_st_delay IMU_Pitch_delay;
extern IMU_st_delay IMU_Yaw_delay;

typedef struct{
	uint32_t last_time;
	uint16_t count;
	float last_position;
	float speed;
	float speed_proce;
}aim_speed;


extern float Y_delay_off;    
extern float P_delay_off;

extern float Y_angle_def;
extern float P_angle_def;

extern kalman_filterII_t Y_krl_init;
extern kalman_filterII_t P_krl_init;

extern TaskHandle_t IMU_Delay_Task_Handle;
extern uint8_t delay_time_y;
void IMU_Delay_Task(void *pvParameters);

extern float IMU_delay_gim(IMU_st_delay *delay,float IMU_now,uint8_t delay_time);

extern TaskHandle_t PC2Gimbal_Data_Task_Handle;
void PC2Gimbal_Data_Task(void *pvParameters);

#endif 
