#ifndef _GIMBAL_TASK_H_
#define _GIMBAL_TASK_H_

#include "Task_Init.h"

#define GIMBAL_YAW_CENTRE 167           //!<@brief Yaw轴电机归中值

#define GIMBAL_PITCH_LEFT_CENTRE 7222   //!<@brief Pitch轴电机归中值
#define GIMBAL_PITCH_RIGHT_CENTRE 7671  //!<@brief Pitch轴电机归中值

#define PITCH_LEFT_TOP_ANGLE 7322       //!<@brief Pitch轴电机极限角度
#define PITCH_LEFT_BOTTOM_ANGLE 7135    //!<@brief Pitch轴电机极限角度
#define PITCH_RIGHT_TOP_ANGLE 7570      //!<@brief Pitch轴电机极限角度
#define PITCH_RIGHT_BOTTOM_ANGLE 7760   //!<@brief Pitch轴电机极限角度

/**
 * @brief 云台状态枚举体
 */
typedef enum
{
    Patro_Mode,             //!<@brief 巡逻模式
    Vision_Control_Mode,    //!<@brief 视觉锁定模式
} Robot_Gimbal_Status_;
extern Robot_Gimbal_Status_ Robot_Gimbal_Status;

/**
* @brief 电机期望值结构体
 */
typedef struct
{
    int16_t Gimbal_Yaw_Exp;         //!<@brief Yaw轴电机角度期望值
    int16_t Gimbal_Pitch_Left_Exp;  //!<@brief Pitch轴电机角度期望值
    int16_t Gimbal_Pitch_Right_Exp; //!<@brief Pitch轴电机角度期望值
    
    float Pitch_Exp_EulerAngler;
    float Yaw_Exp_EulerAngler;
    float Yaw_expect_lv;
} Gimbal_Expect_;
extern Gimbal_Expect_ Gimbal_Expect;

extern GM6020_TypeDef Gimbal_Yaw_Motor;         //!<@brief Yaw轴电机结构体
extern GM6020_TypeDef Gimbal_Pitch_Left_Motor;  //!<@brief Pitch轴电机结构体
extern GM6020_TypeDef Gimbal_Pitch_Right_Motor; //!<@brief Pitch轴电机结构体

extern PID_Smis Gimbal_Yaw_Place_PID;   //!<@brief Yaw轴电机位置环PID
extern PID Gimbal_Yaw_Speed_PID;        //!<@brief Yaw轴电机速度环PID
extern PID Gimbal_Yaw_Speed_Center_PID; //!<@brief Yaw轴电机归中时PID

extern PID_Smis Gimbal_Pitch_Right_Place_PID;   //!<@brief Pitch轴电机位置环PID
extern PID Gimbal_Pitch_Right_Speed_PID;        //!<@brief Pitch轴电机速度环PID
extern PID Gimbal_Pitch_Right_Speed_Center_PID; //!<@brief Pitch轴电机归中时PID
extern PID Gimbal_Pitch_Left_Speed_Center_PID;  //!<@brief Pitch轴电机归中时PID

extern PID Chassis_Follow_PID;

extern float P_LIMIT;           //p轴限制
extern float P_LIMIT_Last;

extern float Y_expect_lv;  	 //y轴滤波值
extern float Y_expect_lv_last;
extern float Pitch_Init_EulerAngler;
extern float Yaw_Init_EulerAngler;

extern float Chassis_Yaw_Exp_Angle;

extern TaskHandle_t Gimbal_Control_Task_Handle; //!<@brief 云台控制任务句柄

/**
 * @brief 云台控制任务
 */
void Gimbal_Control_Task(void *pvParameters);

/**
 * @brief 云台归中PID初始化函数
 */
void PID_Param_Init_Center(void);

/**
 * @brief 云台控制PID初始化函数
 */
void PID_Param_Init_Gimbal(void);

/**
 * @brief 云台PID清空函数
 */
void PID_Param_Clear_Gimbal(void);

#endif 
