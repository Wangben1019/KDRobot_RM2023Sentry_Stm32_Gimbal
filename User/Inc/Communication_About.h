#ifndef _COMMUNICATION_ABOUT_H_
#define _COMMUNICATION_ABOUT_H_

#include "Task_Init.h"

#define USART1_LEN 30

typedef __packed struct {
    uint8_t game_progress;
    uint8_t Outpost_status;
    float chassis_power;
} RX_Buffer_1_Typedef;

typedef union
{
    uint8_t can_buff[8];
    RX_Buffer_1_Typedef RX_Buffer_1;
} The_RX_Buffer_1_;
extern The_RX_Buffer_1_ The_RX_Buffer_1;

typedef __packed struct {
    uint16_t shooter_id1_17mm_cooling_heat;
    uint16_t shooter_id2_17mm_cooling_heat;
    float bullet_speed;
} RX_Buffer_2_Typedef;

typedef union
{
    uint8_t can_buff[8];
    RX_Buffer_2_Typedef RX_Buffer_2;
} The_RX_Buffer_2_;
extern The_RX_Buffer_2_ The_RX_Buffer_2;

extern uint8_t usart1_dma_buf[USART1_LEN];

extern SemaphoreHandle_t Remote_DMA_Finish_Semaphore;  //!<@brief 遥控器接收完成信号量
extern TaskHandle_t Remote_Receive_Task_Handle;
void Remote_Receive_Task(void *pvParameters);


#endif 
