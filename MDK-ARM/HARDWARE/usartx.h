#ifndef __USRATX_H
#define __USRATX_H 

#ifndef ESP32_BRIDGE_ENABLE
#define ESP32_BRIDGE_ENABLE 1
#endif


#define ESP32_BRIDGE_TIMEOUT_MS 500U
#define ESP32_BRIDGE_RX_RING_SIZE 256U
#define ESP32_BRIDGE_FRAME_MAX 64U

#include "stdio.h"
#include "sys.h"
#include "system.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define DATA_STK_SIZE   512 
#define DATA_TASK_PRIO  4

#define FRAME_HEADER      0X7B //Frame_header //帧头
#define FRAME_TAIL        0X7D //Frame_tail   //帧尾
#define SEND_DATA_SIZE    24
#define RECEIVE_DATA_SIZE 11

/*****A structure for storing triaxial data of a gyroscope accelerometer*****/
/*****用于存放陀螺仪加速度计三轴数据的结构体*********************************/
typedef struct __Mpu6050_Data_ 
{
	short X_data; //2 bytes //2个字节
	short Y_data; //2 bytes //2个字节
	short Z_data; //2 bytes //2个字节
}Mpu6050_Data;

/*******The structure of the serial port sending data************/
/*******串口发送数据的结构体*************************************/
typedef struct _SEND_DATA_  
{
	unsigned char buffer[SEND_DATA_SIZE];
	struct _Sensor_Str_
	{
		unsigned char Frame_Header; //1个字节
		short X_speed;	            //2 bytes //2个字节
		short Y_speed;              //2 bytes //2个字节
		short Z_speed;              //2 bytes //2个字节
		short Power_Voltage;        //2 bytes //2个字节
		Mpu6050_Data Accelerometer; //6 bytes //6个字节
		Mpu6050_Data Gyroscope;     //6 bytes //6个字节	
		unsigned char Frame_Tail;   //1 bytes //1个字节
	}Sensor_Str;
}SEND_DATA;

typedef struct _RECEIVE_DATA_  
{
	unsigned char buffer[RECEIVE_DATA_SIZE];
	struct _Control_Str_
	{
		unsigned char Frame_Header; //1 bytes //1个字节
		float X_speed;	            //4 bytes //4个字节
		float Y_speed;              //4 bytes //4个字节
		float Z_speed;              //4 bytes //4个字节
		unsigned char Frame_Tail;   //1 bytes //1个字节
	}Control_Str;
}RECEIVE_DATA;


/*UWB解析相关变量*/
// UWB数据结构体
typedef struct {
    char role[4];           // 设备角色，如"T0"
    char filter_mode[3];    // 滤波模式，"K"或"NK"
    float distances[4];     // 到四个基站的距离，NULL表示为NAN
    float position[3];      // 定位坐标[x, y, z]
    uint8_t valid_distances;// 有效距离数量
    uint8_t valid_position; // 坐标是否有效
} UWB_Data;

extern UWB_Data uwb_data;
extern char rx_buffer[256];
extern uint16_t rx_index;

extern char rx6_buffer[256];
extern uint16_t rx6_index;


extern char rx4_buffer[256];
extern uint16_t rx4_index;

extern uint8_t esp32_rx_byte;

void usart1_send(u8 data);
void usart1_send_cstring(const char *str);
void usart3_send(u8 data);
void usart3_send_cstring(const char *str);
void Parse_UWB_Data(char* data);

void ESP32_Bridge_Init(void);
void ESP32_Bridge_RxByte(uint8_t byte);
void ESP32_Bridge_Task(void);
uint8_t ESP32_Bridge_GetTargets(float *vx, float *vy, float *vz, uint32_t *last_tick);
void ESP32_SendStatus(float x, float y, float heading, float vx, float vy, float vz);
#endif


