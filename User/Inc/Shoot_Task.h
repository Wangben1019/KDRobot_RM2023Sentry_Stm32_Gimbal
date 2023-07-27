#ifndef _SHOOT_TASK_H_
#define _SHOOT_TASK_H_

#include "Task_Init.h"

#define FRICTIONWHELL_SPEED_EXP 7000
#define PLUCK_SPEED_EXP 4000

extern RM3508_TypeDef Frictionwheel_Right_Motor;
extern RM3508_TypeDef Frictionwheel_Centre_Motor;
extern RM3508_TypeDef Frictionwheel_Left_Motor;

extern M2006_TypeDef Pluck_Right_Motor;
extern M2006_TypeDef Pluck_Left_Motor;

extern PID Frictionwheel_Right_Speed_PID;
extern PID Frictionwheel_Centre_Speed_PID;
extern PID Frictionwheel_Left_Speed_PID;

extern PID Pluck_Right_Speed_PID;
extern PID Pluck_Left_Speed_PID;

typedef enum
{
    Shoot_Right = 0,
    Shoot_Left
} Robot_Shoot_Direction_;
extern Robot_Shoot_Direction_ Robot_Shoot_Direction;

typedef enum
{
    Prepare_Shoot_Right = 0,//!<@brief 右侧枪口准备发射：开摩擦轮
    Start_Shoot_Right,      //!<@brief 右侧枪口进行发射：开摩擦轮，开拨弹
    Pluck_Block_Right,      //!<@brief 右侧拨弹盘卡弹
    
    Prepare_Shoot_Left,     //!<@brief 左侧枪口准备发射：开摩擦轮
    Start_Shoot_Left,       //!<@brief 左侧枪口进行发射：开摩擦轮，开拨弹
    Pluck_Block_Left,       //!<@brief 左侧拨弹盘卡弹
    
    Switch_LeftToRight,     //!<@brief 切换左发弹到右发弹
    Switch_RightToLeft,     //!<@brief 切换右发弹到左发弹
    
    Stop_Shoot
} Robot_Shoot_Status_;
extern Robot_Shoot_Status_ Robot_Shoot_Status;
extern Robot_Shoot_Status_ Last_Robot_Shoot_Status;

typedef struct
{
    int16_t Frictionwheel_Right_Speed_Exp;  // + 
    int16_t Frictionwheel_Centre_Speed_Exp; // right -  left + 
    int16_t Frictionwheel_Left_Speed_Exp; // -

    int16_t Pluck_Right_Speed_Exp; // +
    int16_t Pluck_Left_Speed_Exp; // -
} Shoot_Expect_;
extern Shoot_Expect_ Shoot_Expect;

extern TaskHandle_t Shoot_Control_Task_Handle;
void Shoot_Control_Task(void *pvParameters);
void PID_Param_Init_Shoot(void);
void PID_Param_Clear_Shoot(void);

#endif 
