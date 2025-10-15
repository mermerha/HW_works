

#include "stm32f103xb.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "math.h"
#include "tim.h"
#include "stdint.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "HW_can.hpp"
#include "main_task.hpp"
#include "string.h"

void UART_Encode_Data(uint8_t *tx_data);
void CAN_Decode_Data(const uint8_t *rx_data);


uint32_t tick = 0;   // 运行计数
uint8_t g_uart_tx_byte = 0;  
uint8_t g_uart_rx_byte = 0;



// 串口收发
struct UartCommData g_uart_tx_data;
struct UartCommData g_uart_rx_data;


// CAN收发
struct CANCommData g_can_tx_data;
struct CANCommData g_can_rx_data;


void MainInit(void)
{
    tick = 0;
    //can 初始化
    CanFilter_Init(&hcan);// 初始化CAN滤波器
    HAL_CAN_Start(&hcan); 
    HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING);

    HAL_UART_Receive_IT(&huart1,&g_uart_rx_byte,1);

    HAL_TIM_Base_Start_IT(&htim3); // 定时器初始化：启动定时器3，配置为1ms中断一次
}



void MainTask(void)
{
    tick++;

    if(tick%1000 == 0){
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);// 每秒翻转一次LED
    }
    if(tick%100 == 5){// 小偏移量
        g_uart_tx_data.tick = tick;
        g_uart_tx_data.value = sinf((float)tick / 1000.0f);//value的sin计算
        // store
        uint8_t uart_tx_buf[9];
        UART_Encode_Data(uart_tx_buf);

        HAL_UART_Transmit(&huart1, uart_tx_buf, 9, 0xFFFF);

    }


    

    if(tick%100 == 0){
        g_can_tx_data.tick = tick;
        g_can_tx_data.value1 = sinf((float)tick / 1000.0f);
        g_can_tx_data.value2 = (uint8_t)(tick / 100);
        // 定义can的含义进行通信
        g_can_tx_data.flag1 = (tick / 100) % 2 == 0;
        g_can_tx_data.flag2 = (tick / 200) % 2 == 0;
        g_can_tx_data.flag3 = (tick / 400) % 2 == 0;
        g_can_tx_data.flag4 = (tick / 800) % 2 == 0;
        // 存放数据
        uint8_t can_tx_buf[8];
        CAN_Encode_Data(can_tx_buf);
        // 发送数据
        CAN_Send_Msg(&hcan,can_tx_buf,0x100,8);
    }
}


// 串口数据编码函数

void UART_Encode_Data(uint8_t *tx_data)
{    
    int16_t value_encoded = (int16_t)(g_uart_tx_data.value * 30000.0f);

    // 协议帧头 
    tx_data[0] = 0xAA;
    tx_data[1] = 0xBB;
    tx_data[2] = 0xCC;
    // 位运算
    // tick 运行刻记录
    tx_data[3] = (uint8_t)(g_uart_tx_data.tick >> 24);
    tx_data[4] = (uint8_t)(g_uart_tx_data.tick >> 16);
    tx_data[5] = (uint8_t)(g_uart_tx_data.tick >> 8);
    tx_data[6] = (uint8_t)(g_uart_tx_data.tick);
    
    // 打包 value 
    tx_data[7] = (uint8_t)(value_encoded >> 8);
    tx_data[8] = (uint8_t)(value_encoded);
}


void UART_Decode_Data(const uint8_t *rx_buffer)
{
    // 解包 tick

    g_uart_rx_data.tick = (uint32_t)(rx_buffer[3] << 24 | rx_buffer[4] << 16 | rx_buffer[5] << 8 | rx_buffer[6]);

    // 解包 value
    int16_t value_encoded = (int16_t)(rx_buffer[7] << 8 | rx_buffer[8]);
    g_uart_rx_data.value = (float)value_encoded / 30000.0f;
}


// void UART_Receive_StateMachine(uint8_t byte)

void UART_Receive_StateMachine(uint8_t byte)
{
    // 定义状态机的各种状态
    typedef enum {
        STATE_WAIT_HEADER_1, // 等待帧头1 (0xAA)
        STATE_WAIT_HEADER_2, // 等待帧头2 (0xBB)
        STATE_WAIT_HEADER_3, // 等待帧头3 (0xCC)
        STATE_RECEIVE_DATA   // 接收数据
    } ReceiveState;

    static ReceiveState state = STATE_WAIT_HEADER_1; // 静态变量，保存当前状态
    static uint8_t data_buffer[9];                   // 静态缓冲区，保存接收的数据
    static uint8_t data_count = 0;                   // 静态计数器，记录已接收的数据字节数

    switch (state)
    {
        case STATE_WAIT_HEADER_1:
            if (byte == 0xAA) {
                data_buffer[0] = byte;
                state = STATE_WAIT_HEADER_2;
            }
            break;

        case STATE_WAIT_HEADER_2:
            if (byte == 0xBB) {
                data_buffer[1] = byte;
                state = STATE_WAIT_HEADER_3;
            } else {
                state = STATE_WAIT_HEADER_1; // 帧头错误，重置状态机
            }
            break;

        case STATE_WAIT_HEADER_3:
            if (byte == 0xCC) {
                data_buffer[2] = byte;
                data_count = 3; // 帧头接收完毕，准备接收数据
                state = STATE_RECEIVE_DATA;
            } else {
                state = STATE_WAIT_HEADER_1; // 帧头错误，重置状态机
            }
            break;

        case STATE_RECEIVE_DATA:
            data_buffer[data_count++] = byte;
            if (data_count >= 9) { // 接收完一整帧 
                UART_Decode_Data(data_buffer);
                state = STATE_WAIT_HEADER_1; // 重置状态机，准备接收下一帧
            }
            break;

        default:
            state = STATE_WAIT_HEADER_1; // 异常状态，重置
            break;
    }
}

void CAN_Encode_Data(uint8_t *tx_data)
{   // can数据编码
    int16_t value1_encoded = (int16_t)(g_can_tx_data.value1 * 30000.0f);
    tx_data[0] = (uint8_t)(g_can_tx_data.tick >> 24);
    tx_data[1] = (uint8_t)(g_can_tx_data.tick >> 16);
    tx_data[2] = (uint8_t)(g_can_tx_data.tick >> 8);
    tx_data[3] = (uint8_t)(g_can_tx_data.tick);
    tx_data[4] = (uint8_t)(value1_encoded >> 8);
    tx_data[5] = (uint8_t)(value1_encoded);
    tx_data[6] = g_can_tx_data.value2;
    tx_data[7] = (g_can_tx_data.flag1 << 0) | (g_can_tx_data.flag2 << 1) | (g_can_tx_data.flag3 << 2) | (g_can_tx_data.flag4 << 3);
}

// can数据解码
void CAN_Decode_Data(const uint8_t *rx_data)
{
    g_can_rx_data.tick = (uint32_t)(rx_data[0] << 24 | rx_data[1] << 16 | rx_data[2] << 8 | rx_data[3]);
    int16_t value1_encoded = (int16_t)(rx_data[4] << 8 | rx_data[5]);
    g_can_rx_data.value1 = (float)value1_encoded / 30000.0f;
    g_can_rx_data.value2 = rx_data[6];
    g_can_rx_data.flag1 = (rx_data[7] >> 0) & 0x01;
    g_can_rx_data.flag2 = (rx_data[7] >> 1) & 0x01;
    g_can_rx_data.flag3 = (rx_data[7] >> 2) & 0x01;
    g_can_rx_data.flag4 = (rx_data[7] >> 3) & 0x01;
}


// 定时器中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim ==&htim3)
    {

        MainTask();
        
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart1)
    {
         // 将接收到的字节送入状态机进行处理
        UART_Receive_StateMachine(g_uart_rx_byte);

        //重新使能单字节接收中断
        HAL_UART_Receive_IT(&huart1,&g_uart_rx_byte,1);
    }
}