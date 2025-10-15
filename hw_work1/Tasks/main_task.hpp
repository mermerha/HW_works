#ifndef __MAIN_TASK_HPP__
#define __MAIN_TASK_HPP__


#include "stdint.h"   //引入标准的uint8_t等类型
#include "stdbool.h"  //引入标准的bool类型

#ifdef __cplusplus  
extern "C" {
#endif


//定义数据结构，全局变量声明和函数原型
// 串口通信的结构体定义
struct UartCommData
{
    uint32_t tick;
    float value;
};

extern struct UartCommData  g_uart_tx_data;  //  transmit data
extern struct UartCommData  g_uart_rx_data;  //  receive data

// 串口编码函数,tx_data作为一个指向9字节数组的指针
void UART_Encode_Data(uint8_t *tx_data);

// 通过状态机处理串口接收到的单个字节
void UART_Receive_StateMachine(uint8_t byte);

// CAN通信数据结构体定义 
struct CANCommData
{
    uint32_t tick;   // 记录运行数
    float value1;    
    uint8_t value2;  
    bool flag1;      
    bool flag2;    
    bool flag3;      
    bool flag4;     
};


// 存储CAN发送和接收的数据
extern struct CANCommData g_can_tx_data;
extern struct CANCommData g_can_rx_data; 


// CAN编码和解码函数
void CAN_Encode_Data(uint8_t *tx_data);

void CAN_Decode_Data(const uint8_t *rx_data);





// 主函数任务声明
void MainInit(void);

void MainTask(void);

#ifdef __cplusplus  
}
#endif //cplusplus
 
#endif //__MAIN_TASK_HPP__


