#include "PC_Task.h"
#include "VCOMCOMM.h"

PC_Gimbal_Control_Typedef PC_Gimbal_Control;
//Motor_Angle_Feedback_Typedef *Motor_Angle_Feedback;
Motor_Angle_Feedback_Typedef Motor_Angle_Feedback;
Aim_Status_ Aim_Shoot_Status = UnLock_Robot;

SemaphoreHandle_t PC_Receive_Finish_Semaphore;

uint8_t Yaw_Error_DeadBand = 10;
uint8_t Pitch_Error_DeadBand = 10;

kalman_filter_t Yaw_control;
uint32_t time_aim = 0;

IMU_st_delay IMU_Pitch_delay;
IMU_st_delay IMU_Yaw_delay;

aim_rx Aim_rx;

aim_speed Aim_speed_Y;
aim_speed Aim_speed_P;

float Y_delay_off = 0;     //最终延迟角度
float P_delay_off = 0;

float Y_angle_def = 0;     //绝对角度‘’
float P_angle_def = 0;

int Lock_Robot_Rx;
int Shoot_Rx;

/* 二阶卡尔曼初始化 */
kalman_filterII_t Y_krl_init={
	.P_data = {2,0,0,2},              //协方差矩阵    
    .A_data = {1, 0.001, 0, 1},       //预测矩阵（采样时间）
    .H_data = {1, 0, 0, 1}, 	      //传感器测量数据矩阵																					
    .Q_data = {1, 0, 0, 1},           //外部的不确定性（过程噪声协方差）       
    .R_data = {17,0,0,4000},            //传感器测量方差（采集数据方差）
};
kalman_filterII_t P_krl_init={
	.P_data = {2,0,0,2},
    .A_data = {1, 0.001, 0, 1},
    .H_data = {1, 0, 0, 1},
    .Q_data = {1,0 , 0, 0.1},
    .R_data = {200,0,0,2000},
};

uint8_t Receive_Flag = 0;

RX_Buffer_1_Typedef TX_PC_Buffer;

void VCOMM_CallBack(uint8_t fun_code, uint16_t id, uint8_t* data, uint8_t len) {
    BaseType_t xHigherPriorityTaskWoken;
//	printf("fun_code=%02X, id=%04x, len=%d\r\n", fun_code, id, len);
    if(Stop_After_Flag == 0)
    {
        if(fun_code == 0x00 && id == 0x00) 
        {           
            memcpy(&PC_Gimbal_Control, data, sizeof(PC_Gimbal_Control_Typedef));
            Aim_rx.p_angle_r = PC_Gimbal_Control.Aim_Pitch;
            Aim_rx.y_angle_r = PC_Gimbal_Control.Aim_Yaw;
            Aim_rx.dis_r = PC_Gimbal_Control.Lock_Robot_Distance;
            Lock_Robot_Rx = PC_Gimbal_Control.Lock_Robot;
            Shoot_Rx = PC_Gimbal_Control.Shoot;
            xSemaphoreGiveFromISR(PC_Receive_Finish_Semaphore,&xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        } 
    }
    
    Feed_Dog(&WPC_Communication);
}

float IMU_delay_gim(IMU_st_delay *delay,float IMU_now,uint8_t delay_time){
	if (delay->time > 200){                                               
		delay->time = 0;                                  
    }                                                                                           
	delay->IMU_data_delay[delay->time] = IMU_now;           
	if(delay->time >= delay_time){	
		delay->IMU_need = delay->time - delay_time;
	}
	else{
		delay->IMU_need = delay->time + 200- delay_time;            
	}	
	delay->time++;
  return delay->IMU_need;	
}
/* 自瞄速度判断 */
float aim_speed_judge(aim_speed *S,float position){
	float speed_limit = 10.0f;

	S->count++;

	if(time_aim != S->last_time){
		S->speed = (position - S->last_position) / (time_aim - S->last_time) * 1000;
		
		if(S->speed - S->speed_proce > speed_limit)
			S->speed_proce = S->speed_proce + speed_limit;
		else if(S->speed - S->speed_proce < -speed_limit)
			S->speed_proce = S->speed_proce - speed_limit;
		else
			S->speed_proce = S->speed;
		
		S->last_time = time_aim;
		S->last_position = position;
		S->count = 0;
	}
	if(S->count > 200)
		S->speed_proce = 0;
	return S->speed_proce;
}

TaskHandle_t IMU_Delay_Task_Handle;
uint8_t delay_time_y = 70;
void IMU_Delay_Task(void *pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    for(;;)
    {
        IMU_delay_gim(&IMU_Yaw_delay, IMU_Data_User.EulerAngler.ContinuousYaw, delay_time_y);
        IMU_delay_gim(&IMU_Pitch_delay, (Gimbal_Pitch_Right_Motor.MchanicalAngle - GIMBAL_PITCH_RIGHT_CENTRE) / 22.75, 55);
        time_aim++;
        vTaskDelayUntil(&xLastWakeTime, 1);
    }
}

TaskHandle_t PC_Task_Handle;
void PC_Task(void *pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    for(;;)
    {
        VCOMM_Transmit(0x00, 0x00, (uint8_t *)(&Motor_Angle_Feedback), sizeof(Motor_Angle_Feedback_Typedef));
        vTaskDelayUntil(&xLastWakeTime,10);
    }
}

TaskHandle_t PC2Gimbal_Data_Task_Handle;
void PC2Gimbal_Data_Task(void *pvParameters)
{
//    portTickType xLastWakeTime;
//    xLastWakeTime = xTaskGetTickCount();
    BaseType_t PC_Receive_data = pdFALSE;
    for(;;)
    {
        PC_Receive_data = xSemaphoreTake(PC_Receive_Finish_Semaphore, portMAX_DELAY);
        if(PC_Receive_data == pdTRUE)
        {
            Aim_rx.p_angle_rx = Aim_rx.p_angle_r;
            Aim_rx.y_angle_rx = -Aim_rx.y_angle_r;
            Aim_rx.dis_rx = Aim_rx.dis_r;
                
            Aim_rx.p_angle_rx_off = Aim_rx.p_angle_rx/1.0f;
            P_delay_off = IMU_Pitch_delay.IMU_data_delay[IMU_Pitch_delay.IMU_need];
            P_angle_def = Aim_rx.p_angle_rx_off + P_delay_off;
            
            KalmanII_Filter(&P_krl_init, P_angle_def, aim_speed_judge(&Aim_speed_P,P_angle_def));
            Gimbal_Expect.Gimbal_Pitch_Right_Exp = P_krl_init.kalman.filtered_value[0]*22.75;            
        
            Aim_rx.y_angle_rx_off = Aim_rx.y_angle_rx;
            Y_delay_off = IMU_Yaw_delay.IMU_data_delay[IMU_Yaw_delay.IMU_need];
            Y_angle_def = Aim_rx.y_angle_rx_off + Y_delay_off;

            KalmanII_Filter(&Y_krl_init, Y_angle_def, aim_speed_judge(&Aim_speed_Y,Y_angle_def));
            Gimbal_Expect.Yaw_Exp_EulerAngler = Y_krl_init.kalman.filtered_value[0] - Y_expect_lv_last;
            
            xSemaphoreGive(Weak_Up_Administrator_Semaphore);
            portYIELD();
        }
    }
}
