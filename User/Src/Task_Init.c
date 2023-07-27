#include "Task_Init.h"

Robot_Status_ Robot_Status = Doing_Init;
//Robot_Status_ Robot_Status = Finish_Init;

uint8_t Stop_After_Flag = 0;

int16_t msg_init[4];

uint8_t dPatro_Angle_Pitch;
float dPatro_EulerAngler_Yaw;

uint8_t Administrator_Work_Flag = 0;

static uint8_t Flag_Get_Exp = 0;                                    //!<@brief 需要获取归中期望值标志位
static Ramp_Typedef Ramp_Yaw_Init = {.RampTime = 1000};             //!<@brief Yaw轴初始化斜坡参数初始化
static Ramp_Typedef Ramp_Pitch_Left_Init = {.RampTime = 1000};      //!<@brief Pitch轴初始化斜坡参数初始化
static Ramp_Typedef Ramp_Pitch_Right_Init = {.RampTime = 1000};     //!<@brief Pitch轴初始化斜坡参数初始化

static int16_t dMchanicalAngle_Yaw;         //!<@brief Yaw轴电机归中时需要转过的机械角度
static int16_t dMchanicalAngle_Pitch_Left;  //!<@brief Pitch轴电机归中时需要转过的机械角度
static int16_t dMchanicalAngle_Pitch_Right; //!<@brief Pitch轴电机归中时需要转过的机械角度

/* 机器人归中初始化任务 */
TaskHandle_t Robot_Init_Task_Handle;    //!<@brief 
static void Robot_Init_Task(void *pvParameters);

SemaphoreHandle_t Weak_Up_Administrator_Semaphore;
TaskHandle_t Task_Administrator_Handle;
void Task_Administrator(void *pvParameters);

void Start_Task()
{
    /* 创建遥控器接受数据完成信号量 */
    Remote_DMA_Finish_Semaphore = xSemaphoreCreateBinary();
    IMU_DMA_Finish_Semaphore = xSemaphoreCreateBinary();
    PC_Receive_Finish_Semaphore = xSemaphoreCreateBinary();
    Weak_Up_Administrator_Semaphore = xSemaphoreCreateBinary();
    
    /* 初始化CAN滤波器 */
    CanFilter_Init(&hcan1);
    CanFilter_Init(&hcan2);
    
    /* 启动CAN模块 */
    HAL_CAN_Start(&hcan1);
    HAL_CAN_Start(&hcan2);
    
    /* 启动CAN中断 */
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

    /* 失能DMA中断 */
    HAL_NVIC_DisableIRQ(DMA2_Stream1_IRQn);
    HAL_NVIC_DisableIRQ(DMA1_Stream1_IRQn);
    
    /* 启动串口中断中断源为IDLE */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
    
    /* 以DMA模式接收数据 */
    HAL_UART_Receive_DMA(&huart1, usart1_dma_buf, USART1_LEN);
    HAL_UART_Receive_DMA(&huart6, usart6_dma_buf, USART6_LEN);
  
    /* 定时器以及PWM初始化 */
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start(&htim5);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
    
    /* PID参数初始化 */
    PID_Param_Init_Center();
    PID_Param_Init_Shoot();
    
    WatchDog_Init_Gather();
    kalmanII_Init(&Y_krl_init);
	kalmanII_Init(&P_krl_init);
    kalman_Init(&Yaw_control, 20, 200);
    
    Stop_After_Flag = 0;
    dPatro_Angle_Pitch = 1;
    dPatro_EulerAngler_Yaw = 1.0;

    xTaskCreate(Robot_Init_Task, "Robot_Init_Task", 128, NULL, osPriorityNormal, &Robot_Init_Task_Handle);
    xTaskCreate(Remote_Receive_Task, "Remote_Receive_Task", 128, NULL, osPriorityNormal, &Remote_Receive_Task_Handle);
    xTaskCreate(WatchDog_Task, "WatchDog_Task", 128, NULL, osPriorityNormal, &WatchDog_Task_Handle);
    xTaskCreate(Monitor_Device_Status_Task, "Monitor_Device_Status_Task", 128, NULL, osPriorityNormal, &Monitor_Device_Status_Task_Handle);
    xTaskCreate(PC_Task, "PC_Task", 256 * 4, NULL, osPriorityNormal, &PC_Task_Handle);
    xTaskCreate(Imu_Data_Task, "Imu_Data_Task", 128, NULL, osPriorityNormal, &Imu_Data_Task_Handle);
    xTaskCreate(IMU_Delay_Task, "IMU_Delay_Task", 128, NULL, osPriorityNormal, &IMU_Delay_Task_Handle);
    xTaskCreate(PC2Gimbal_Data_Task, "PC2Gimbal_Data_Task", 256 * 4, NULL, osPriorityNormal, &PC2Gimbal_Data_Task_Handle);
    xTaskCreate(Task_Administrator, "Task_Administrator", 256 * 4, NULL, osPriorityNormal, &Task_Administrator_Handle);
}

static void Robot_Init_Task(void *pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    for(;;)
    {
        if(Robot_Status == Doing_Init)
        {
            if(Gimbal_Yaw_Motor.MchanicalAngle == 0 && Gimbal_Pitch_Left_Motor.MchanicalAngle == 0 && Gimbal_Pitch_Right_Motor.MchanicalAngle == 0)
            {
                continue;
            }
            /* 获取归中Yaw，Pitch轴期望值 */
            Gimbal_Expect.Gimbal_Yaw_Exp = QuickCentering(Gimbal_Yaw_Motor.MchanicalAngle, GIMBAL_YAW_CENTRE);
            if(Flag_Get_Exp == 0) 
            {
                Gimbal_Expect.Gimbal_Pitch_Left_Exp = GIMBAL_PITCH_LEFT_CENTRE;
                Gimbal_Expect.Gimbal_Pitch_Right_Exp = GIMBAL_PITCH_RIGHT_CENTRE;
                
                ResetSlope(&Ramp_Yaw_Init);
                ResetSlope(&Ramp_Pitch_Left_Init);
                ResetSlope(&Ramp_Pitch_Right_Init);
                
                dMchanicalAngle_Yaw = Gimbal_Expect.Gimbal_Yaw_Exp - Gimbal_Yaw_Motor.MchanicalAngle;
                dMchanicalAngle_Pitch_Left = Gimbal_Expect.Gimbal_Pitch_Left_Exp - Gimbal_Pitch_Left_Motor.MchanicalAngle;
                dMchanicalAngle_Pitch_Right = Gimbal_Expect.Gimbal_Pitch_Right_Exp - Gimbal_Pitch_Right_Motor.MchanicalAngle;
                
                Flag_Get_Exp = 1;
            }
            
            limit(Gimbal_Expect.Gimbal_Pitch_Left_Exp, PITCH_LEFT_TOP_ANGLE, PITCH_LEFT_BOTTOM_ANGLE);
            limit(Gimbal_Expect.Gimbal_Pitch_Right_Exp, PITCH_RIGHT_BOTTOM_ANGLE, PITCH_RIGHT_TOP_ANGLE);
            
            PID_Control(Gimbal_Yaw_Motor.Angle, Gimbal_Expect.Gimbal_Yaw_Exp - (dMchanicalAngle_Yaw) * (1.0f - Slope(&Ramp_Yaw_Init)), &Gimbal_Yaw_Speed_Center_PID);
            
            PID_Control(Gimbal_Pitch_Left_Motor.Angle, Gimbal_Expect.Gimbal_Pitch_Left_Exp - (dMchanicalAngle_Pitch_Left) * (1.0f - Slope(&Ramp_Pitch_Left_Init)), &Gimbal_Pitch_Left_Speed_Center_PID);
            PID_Control(Gimbal_Pitch_Right_Motor.Angle, Gimbal_Expect.Gimbal_Pitch_Right_Exp - (dMchanicalAngle_Pitch_Right) * (1.0f - Slope(&Ramp_Pitch_Right_Init)), &Gimbal_Pitch_Right_Speed_Center_PID);
            
            limit(Gimbal_Yaw_Speed_Center_PID.pid_out, GM6020_LIMIT, -GM6020_LIMIT);
            limit(Gimbal_Pitch_Left_Speed_Center_PID.pid_out, GM6020_LIMIT, -GM6020_LIMIT);
            limit(Gimbal_Pitch_Right_Speed_Center_PID.pid_out, GM6020_LIMIT, -GM6020_LIMIT);
            
            msg_init[0] = Gimbal_Pitch_Right_Speed_Center_PID.pid_out;
            msg_init[1] = Gimbal_Pitch_Left_Speed_Center_PID.pid_out;
            msg_init[2] = Gimbal_Yaw_Speed_Center_PID.pid_out * Slope(&Ramp_Yaw_Init);
            MotorSend(&hcan1, 0x1FF, msg_init);
            
            /* 利用斜坡的进度对归中角度进行判断 */
            if(Slope(&Ramp_Yaw_Init) == 1 && Slope(&Ramp_Pitch_Left_Init) == 1 && Slope(&Ramp_Pitch_Right_Init) == 1)
            {
                /* 此时机器人的状态为完成归中初始化 */
                Gimbal_Yaw_Motor.r = 0;
                Robot_Status = Finish_Init;
                Get_Imu_User_EulerAngler(&IMU_Start_EulerAngler_Data, &IMU_Data_User);
                
                IMU_Data_User.EulerAngler.Yawoffset = IMU_Data_User.EulerAngler.ContinuousYaw;
                Pitch_Init_EulerAngler = Gimbal_Pitch_Right_Motor.Angle;
                Yaw_Init_EulerAngler = IMU_Data_User.EulerAngler.Yawoffset;
//                Pitch_Init_EulerAngler = IMU_Data_User.EulerAngler.Pitch;
                Gimbal_Expect.Gimbal_Pitch_Left_Exp = 0;
                Gimbal_Expect.Gimbal_Pitch_Right_Exp = 0;
                IMU_Data_User.EulerAngler.ContinuousYaw = IMU_Data_User.EulerAngler.Yaw;
                
                ResetSlope(&Ramp_Yaw_Init);
                ResetSlope(&Ramp_Pitch_Left_Init);
                ResetSlope(&Ramp_Pitch_Right_Init);
                Flag_Get_Exp = 0;
            }
        }
        else if(Robot_Status == Finish_Init)
        {
            /* 进入临界区，创建后续控制任务 */
            taskENTER_CRITICAL();
            xTaskCreate(Gimbal_Control_Task, "Gimbal_Control_Task", 128, NULL, osPriorityNormal, &Gimbal_Control_Task_Handle);
            xTaskCreate(Shoot_Control_Task, "Shoot_Control_Task", 128, NULL, osPriorityNormal, &Shoot_Control_Task_Handle);
            xTaskCreate(Chassis_Control_Task, "Chassis_Control_Task", 128, NULL, osPriorityNormal, &Chassis_Task_Handle);
            PID_Param_Init_Gimbal();
            taskEXIT_CRITICAL(); 
            vTaskSuspend(NULL);
        }
        vTaskDelayUntil(&xLastWakeTime,2);
    }
}

void Task_Administrator(void *pvParameters)
{
    BaseType_t Administrator_Work = pdFALSE;
    for(;;)
    {
        Administrator_Work = xSemaphoreTake(Weak_Up_Administrator_Semaphore, portMAX_DELAY);
        if(Administrator_Work == pdTRUE)
        {
            vTaskSuspend(Chassis_Task_Handle);
            vTaskSuspend(Shoot_Control_Task_Handle);
            vTaskSuspend(Remote_Receive_Task_Handle);
            vTaskSuspend(PC_Task_Handle);
            vTaskSuspend(Monitor_Device_Status_Task_Handle);
            vTaskSuspend(NULL);
            portYIELD();
            Administrator_Work_Flag = 1;
        }
    }
}

