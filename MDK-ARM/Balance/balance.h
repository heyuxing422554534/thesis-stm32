#ifndef __BALANCE_H
#define __BALANCE_H			  	 
#include "sys.h"
#include "system.h"

#define BALANCE_TASK_PRIO		3     //Task priority //任务优先级
#define BALANCE_STK_SIZE 		512   //Task stack size //任务堆栈大小

//Parameter of kinematics analysis of omnidirectional trolley
//全向轮小车运动学分析参数
#define X_PARAMETER    (sqrt(3)/2.f)               
#define Y_PARAMETER    (0.5f)    
#define L_PARAMETER    (1.0f)
#define PWMA1   TIM10->CCR1 
#define PWMA2   TIM11->CCR1 

#define PWMB1   TIM9->CCR1 
#define PWMB2   TIM9->CCR2

#define PWMC1   TIM1->CCR1  
#define PWMC2   TIM1->CCR2 

#define PWMD1   TIM1->CCR3 
#define PWMD2   TIM1->CCR4

#define EN     PDin(3)  

extern short test_num;
extern int robot_mode_check_flag;

// 小车速度变量
extern float Current_Vx;      // X轴方向速度 (m/s)
extern float Current_Vy;      // Y轴方向速度 (m/s) 
extern float Current_Vz;      // Z轴旋转角速度 (rad/s)

void Balance_task(void *pvParameters);
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo);
void Limit_Pwm(int amplitude);
float target_limit_float(float insert,float low,float high);
int target_limit_int(int insert,int low,int high);
u8 Turn_Off( int voltage);
u32 myabs(long int a);
int Incremental_PI_A (float Encoder,float Target);
int Incremental_PI_B (float Encoder,float Target);
int Incremental_PI_C (float Encoder,float Target);
int Incremental_PI_D (float Encoder,float Target);
void Drive_Motor(float Vx,float Vy,float Vz);
void Get_Velocity_Form_Encoder(void);
void Smooth_control(float vx,float vy,float vz);
float float_abs(float insert);
void robot_mode_check(void);
// 速度计算函数
void Calculate_Car_Velocity(void);

#endif  

