#include "balance.h"
#include "eskf.h"
int Time_count=0; //Time variable //计时变量
Encoder OriginalEncoder; //Encoder raw data //编码器原始数据

float Current_Vx = 0.0f;      // X轴方向速度 (m/s) - 前向为正
float Current_Vy = 0.0f;      // Y轴方向速度 (m/s) - 左向为正  
float Current_Vz = 0.0f;      // Z轴旋转角速度 (rad/s) - 逆时针为正

// ESKF global instance and UWB update flag
eskf_t my_eskf;
volatile uint8_t uwb_new_data_flag = 0;



/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X and Y, Z axis direction of the target movement speed
Output  : none
函数功能：运动学逆解，根据三轴目标速度计算各车轮目标转速
入口参数：X和Y、Z轴方向的目标运动速度
返回  值：无
**************************************************************************/
void Drive_Motor(float Vx,float Vy,float Vz)
{
		float amplitude=3.5; //Wheel target speed limit //车轮目标速度限幅
        
	        //Speed smoothing is enabled when moving the omnidirectional trolley
	        //全向移动小车才开启速度平滑处理
			Smooth_control(Vx,Vy,Vz); //Smoothing the input speed //对输入速度进行平滑处理
  
            //Get the smoothed data 
			//获取平滑处理后的数据			
			Vx=smooth_control.VX;     
			Vy=smooth_control.VY;
			Vz=smooth_control.VZ;
		
			//Mecanum wheel car
			//麦克纳姆轮小车
			//Inverse kinematics //运动学逆解
			MOTOR_A.Target   = +Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_B.Target   = -Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_C.Target   = +Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_D.Target   = -Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
		
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude); 
			MOTOR_D.Target=target_limit_float(MOTOR_D.Target,-amplitude,amplitude); 

}



/**************************************************************************
Function: 计算小车整体速度（运动学正解）
Input   : 无
Output  : 无
函数功能：根据四个轮子的速度计算小车的整体运动速度
**************************************************************************/
void Calculate_Car_Velocity(void)
{
    float v1 = MOTOR_A.Encoder;  // 后左
    float v2 = MOTOR_B.Encoder;  // 前左  
    float v3 = MOTOR_C.Encoder;  // 前右
    float v4 = MOTOR_D.Encoder;  // 后右
    
    float L = (Axle_spacing + Wheel_spacing) / 2.0f;
    
    // 从逆运动学推导正运动学：
    // 逆运动学矩阵：
    // [v1]   [ 1  1 -L] [Vx]
    // [v2] = [ 1 -1 -L] [Vy]
    // [v3]   [ 1  1  L] [Vz]
    // [v4]   [ 1 -1  L]
    
    // 正运动学（解这个线性方程组）：
    Current_Vx = (v1 + v2 + v3 + v4) / 4.0f;
    Current_Vy = (v1 - v2 + v3 - v4) / 4.0f;
    
    if (L > 0.01f) {
        Current_Vz = (-v1 - v2 + v3 + v4) / (4.0f * L);
    } else {
        Current_Vz = 0.0f;
    }
    
    // 滤波处理
    static float filtered_Vx = 0.0f, filtered_Vy = 0.0f, filtered_Vz = 0.0f;
    float filter_factor = 0.7f;
    
    filtered_Vx = filtered_Vx * filter_factor + Current_Vx * (1.0f - filter_factor);
    filtered_Vy = filtered_Vy * filter_factor + Current_Vy * (1.0f - filter_factor);  
    filtered_Vz = filtered_Vz * filter_factor + Current_Vz * (1.0f - filter_factor);
    
    Current_Vx = filtered_Vx; 
    Current_Vy = filtered_Vy;
    Current_Vz = filtered_Vz;
    
}
/**************************************************************************
Function: FreerTOS task, core motion control task
Input   : none
Output  : none
函数功能：FreeRTOS任务，核心运动控制任务
入口参数：无
返回  值：无
**************************************************************************/
// 在 Balance_task 函数中修改模式切换逻辑
void Balance_task(void *pvParameters)
{ 
    u32 lastWakeTime = getSysTickCnt();
    uint32_t last_status_send_tick = 0;
    ESP32_Bridge_Init();
	eskf_init(&my_eskf);
    
    while(1)
    {	
        // 此任务以100Hz的频率运行（10ms控制一次）
        vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ)); 

        // 时间计数
        if(Time_count<40000) Time_count++;

        // 获取编码器数据
        Get_Velocity_Form_Encoder();           		

		// 1. 获取最新传感器数据并统一单位
        {
            float dt = 0.01f;
            float yaw_rad = Yaw * (3.14159265f / 180.0f);

            // ==========================================
            // 【新增：速度死区截断 (Deadband Filter)】
            // 剔除绝对值小于 0.04 m/s 的底噪，防止静止时的幽灵积分漂移
            float eskf_vx = Current_Vx;
            float eskf_vy = Current_Vy;
            
            if (eskf_vx > -ESKF_VEL_DEADBAND && eskf_vx < ESKF_VEL_DEADBAND) {
                eskf_vx = 0.0f;
            }
            if (eskf_vy > -ESKF_VEL_DEADBAND && eskf_vy < ESKF_VEL_DEADBAND) {
                eskf_vy = 0.0f;
            }
            // ==========================================

            // 2. 执行 ESKF 高频预测步 (注意这里传入的是处理过的 eskf_vx 和 eskf_vy)
            eskf_predict(&my_eskf, eskf_vx, eskf_vy, yaw_rad, dt);

            // 3. 检查 UWB 标志位，执行低频更新步
            if (uwb_new_data_flag == 1) {
                eskf_update(&my_eskf, position[0], position[1], eskf_vx, eskf_vy);
                uwb_new_data_flag = 0;
            }
        }

        ESP32_Bridge_Task();

        float cmd_vx = 0.0f;
        float cmd_vy = 0.0f;
        float cmd_vz = 0.0f;
        uint32_t last_cmd_tick = 0;
        uint32_t now = HAL_GetTick();
        uint8_t has_cmd = ESP32_Bridge_GetTargets(&cmd_vx, &cmd_vy, &cmd_vz, &last_cmd_tick);

        if (!has_cmd || (now - last_cmd_tick) > ESP32_BRIDGE_TIMEOUT_MS) {
            cmd_vx = 0.0f;
            cmd_vy = 0.0f;
            cmd_vz = 0.0f;
        }

        Drive_Motor(cmd_vx, cmd_vy, cmd_vz);

        if (now - last_status_send_tick >= 100U) {
			float fused_yaw_deg;

			// 将 ESKF 融合出的最优坐标写回
			position[0] = (float)my_eskf.nominal.x;
			position[1] = (float)my_eskf.nominal.y;

			// 将融合后的航向角转回角度用于上报
			fused_yaw_deg = (float)(my_eskf.nominal.yaw * (180.0f / 3.14159265f));

			ESP32_SendStatus(position[0], position[1], fused_yaw_deg, Current_Vx, Current_Vy, Current_Vz);
            last_status_send_tick = now;
        }
        
        
        if((Voltage>10)&&(EN==1)) 
        { 			
            // 速度闭环控制计算各电机PWM值
            MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
            MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
            MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
            MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);
            Limit_Pwm(1500);
            
            // 设置PWM
            switch(Car_Mode)
            {
                case Mec_Car: Set_Pwm(MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm, MOTOR_C.Motor_Pwm, -MOTOR_D.Motor_Pwm, 0); break;
            }
        }
        else
        {
            Set_Pwm(0,0,0,0,0); 
        }
        
    }
}
/**************************************************************************
Function: Assign a value to the PWM register to control wheel speed and direction
Input   : PWM
Output  : none
函数功能：赋值给PWM寄存器，控制车轮转速与方向
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo)
{
	//Forward and reverse control of motor
	//电机正反转控制
	if(motor_a<0)		PWMA1=1599,PWMA2=1599+motor_a;
	else 	            PWMA2=1599,PWMA1=1599-motor_a;
	
	//Forward and reverse control of motor
	//电机正反转控制	
	if(motor_b<0)		PWMB1=1599,PWMB2=1599+motor_b;
	else 	            PWMB2=1599,PWMB1=1599-motor_b;
//  PWMB1=10000,PWMB2=5000;

	//Forward and reverse control of motor
	//电机正反转控制	
	if(motor_c<0)		PWMC1=1599,PWMC2=1599+motor_c;
	else 	            PWMC2=1599,PWMC1=1599-motor_c;
	
	//Forward and reverse control of motor
	//电机正反转控制
	if(motor_d<0)		PWMD1=1599,PWMD2=1599+motor_d;
	else 	            PWMD2=1599,PWMD1=1599-motor_d;

	
}

/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : none
函数功能：限制PWM值 
入口参数：幅值
返回  值：无
**************************************************************************/
void Limit_Pwm(int amplitude)
{	
	    MOTOR_A.Motor_Pwm=target_limit_float(MOTOR_A.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_B.Motor_Pwm=target_limit_float(MOTOR_B.Motor_Pwm,-amplitude,amplitude);
		MOTOR_C.Motor_Pwm=target_limit_float(MOTOR_C.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_D.Motor_Pwm=target_limit_float(MOTOR_D.Motor_Pwm,-amplitude,amplitude);
}	    
/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
函数功能：限幅函数
入口参数：幅值
返回  值：无
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
		return insert;
}
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
/**************************************************************************
Function: Check the battery voltage, enable switch status, software failure flag status
Input   : Voltage
Output  : Whether control is allowed, 1: not allowed, 0 allowed
函数功能：检查电池电压、使能开关状态、软件失能标志位状态
入口参数：电压
返回  值：是否允许控制，1：不允许，0允许
          目前没有用上该函数
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(voltage<10||EN==0||Flag_Stop==1)
			{	                                                
				temp=1;      
				PWMA1=0;PWMA2=0;
				PWMB1=0;PWMB2=0;		
				PWMC1=0;PWMC2=0;	
				PWMD1=0;PWMD2=0;					
      }
			else
			temp=0;
			return temp;			
}
/**************************************************************************
Function: Calculate absolute value
Input   : long int
Output  : unsigned int
函数功能：求绝对值
入口参数：long int
返回  值：unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
Function: Incremental PI controller
Input   : Encoder measured value (actual speed), target speed
Output  : Motor PWM
According to the incremental discrete PID formula
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) is the last deviation and so on
PWM stands for incremental output
In our speed control closed loop system, only PI control is used
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)

函数功能：增量式PI控制器
入口参数：编码器测量值(实际速度)，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>1500)Pwm=1500;
	 if(Pwm<-1500)Pwm=-1500;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm;    
}
int Incremental_PI_B (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>1500)Pwm=1500;
	 if(Pwm<-1500)Pwm=-1500;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm;
}
int Incremental_PI_C (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>1500)Pwm=1500;
	 if(Pwm<-1500)Pwm=-1500;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm; 
}
int Incremental_PI_D (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>1500)Pwm=1500;
	 if(Pwm<-1500)Pwm=-1500;
	 Last_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return Pwm; 
}



/**************************************************************************
Function: Read the encoder value and calculate the wheel speed, unit m/s
Input   : none
Output  : none
函数功能：读取编码器数值并计算车轮速度，单位m/s
入口参数：无
返回  值：无
**************************************************************************/
void Get_Velocity_Form_Encoder(void)
{
	  //Retrieves the original data of the encoder
	  //获取编码器的原始数据

		float Encoder_A_pr,Encoder_B_pr,Encoder_C_pr,Encoder_D_pr; 
		OriginalEncoder.A=Read_Encoder(2);	
		OriginalEncoder.B=Read_Encoder(3);	
		OriginalEncoder.C=Read_Encoder(4);	
		OriginalEncoder.D=Read_Encoder(5);	
	
	  //Decide the encoder numerical polarity according to different car models
		//根据不同小车型号决定编码器数值极性
		switch(Car_Mode)
		{
			case Mec_Car:       Encoder_A_pr= OriginalEncoder.A; Encoder_B_pr= OriginalEncoder.B; Encoder_C_pr=-OriginalEncoder.C;  Encoder_D_pr=-OriginalEncoder.D; break; 
			case Omni_Car:      Encoder_A_pr=-OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr=-OriginalEncoder.C;  Encoder_D_pr=-OriginalEncoder.D; break;
			case Akm_Car:       Encoder_A_pr= OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break;
			case Diff_Car:      Encoder_A_pr= OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break; 
			case FourWheel_Car: Encoder_A_pr= OriginalEncoder.A; Encoder_B_pr= OriginalEncoder.B; Encoder_C_pr=-OriginalEncoder.C;  Encoder_D_pr=-OriginalEncoder.D; break; 
			case Tank_Car:      Encoder_A_pr= OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break; 
		}
		
		//The encoder converts the raw data to wheel speed in m/s
		//编码器原始数据转换为车轮速度，单位m/s
		MOTOR_A.Encoder= Encoder_A_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_B.Encoder= Encoder_B_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_C.Encoder= Encoder_C_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
		MOTOR_D.Encoder= Encoder_D_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
	
        // 新增：计算小车整体速度
        Calculate_Car_Velocity();
}
/**************************************************************************
Function: Smoothing the three axis target velocity
Input   : Three-axis target velocity
Output  : none
函数功能：对三轴目标速度做平滑处理
入口参数：三轴目标速度
返回  值：无
**************************************************************************/
void Smooth_control(float vx,float vy,float vz)
{
	float step=0.01;

	if	   (vx>0) 	smooth_control.VX+=step;
	else if(vx<0)		smooth_control.VX-=step;
	else if(vx==0)	smooth_control.VX=smooth_control.VX*0.9f;
	
	if	   (vy>0)   smooth_control.VY+=step;
	else if(vy<0)		smooth_control.VY-=step;
	else if(vy==0)	smooth_control.VY=smooth_control.VY*0.9f;
	
	if	   (vz>0) 	smooth_control.VZ+=step;
	else if(vz<0)		smooth_control.VZ-=step;
	else if(vz==0)	smooth_control.VZ=smooth_control.VZ*0.9f;
	
	smooth_control.VX=target_limit_float(smooth_control.VX,-float_abs(vx),float_abs(vx));
	smooth_control.VY=target_limit_float(smooth_control.VY,-float_abs(vy),float_abs(vy));
	smooth_control.VZ=target_limit_float(smooth_control.VZ,-float_abs(vz),float_abs(vz));
}
/**************************************************************************
Function: Floating-point data calculates the absolute value
Input   : float
Output  : The absolute value of the input number
函数功能：浮点型数据计算绝对值
入口参数：浮点数
返回  值：输入数的绝对值
**************************************************************************/
float float_abs(float insert)
{
	if(insert>=0) return insert;
	else return -insert;
}


