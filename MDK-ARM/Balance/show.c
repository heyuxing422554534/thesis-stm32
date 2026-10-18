#include "show.h"



// 全局变量（显示及状态相关）
int Voltage_Show;                     // 电池电压显示值（放大100倍，用于显示）
unsigned char i;                      // 循环计数变量

// 模拟数据（需替换为实际传感器数据）
uint8_t self_id = 1;                  // 本车ID
int16_t current_angle = 135;          // 当前航向角（°）
int16_t target_angle = 140;           // 目标航向角（°）
uint8_t current_speed = 45;           // 当前速度（单位根据实际定义）
uint8_t target_speed = 50;            // 目标速度（单位根据实际定义）


/**************************************************************************
Function: 显示任务：读取电池电压、蜂鸣器控制、OLED显示、APP数据发送
Input   : pvParameters - 任务参数（未使用）
Output  : none
**************************************************************************/
int Buzzer_count = 25;                // 蜂鸣器计数（控制响铃时长）
void show_task(void *pvParameters)
{
    // 初始化任务周期（10ms执行一次，100Hz频率）
    TickType_t xLastWakeTime = xTaskGetTickCount();  // 记录初始唤醒时间
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 任务周期（10ms）

    while(1)
    {
        // 按固定周期执行任务（确保时间精度）
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        // 开机蜂鸣器提示（前500ms响）
        if (Time_count < 50)
            Buzzer_work();          // 蜂鸣器工作
        else if (Time_count >= 51 && Time_count < 100)
            Buzzer_silence();       // 蜂鸣器静音
        
        // 读取电池电压（10次采样平均，降低波动）
        for (i = 0; i < 10; i++)
            Voltage_All += Get_battery_volt();  // 累加电压采样值
        Voltage = Voltage_All / 10.0;          // 计算平均值
        Voltage_All = 0;                       // 清零累加器
        Voltage_Show = Voltage * 100;          // 转换为整数（保留2位小数）

        oled_show();     // 更新OLED显示
    }
}

/**************************************************************************
Function: OLED显示管理：处理页面切换并刷新对应页面
Input   : none
Output  : none
**************************************************************************/
void oled_show(void)
{
    // 新增：记录是否首次进入自检完成状态（用于清屏）
    static bool first_enter_ready = true;

    if((angle_calibrated==1)&&(gyro_bias_captured==1))
    {
        // 自检完成：首次进入时清屏（避免残留"Calibrating..."）
        if (first_enter_ready)
        {
            OLED_Clear();          // 清屏
            first_enter_ready = false;  // 重置标志，只清屏一次
        }

        display_page1();    // 固定显示第1页：电机数据
    }
    else
    {
        // 自检中：显示校准提示，同时重置首次进入标志（下次完成时重新清屏）
        first_enter_ready = true;
        OLED_ShowString(5, 30, "Calibrating... ");
    }

    OLED_Refresh_Gram();  // 刷新OLED显存（显示生效）
}

/**************************************************************************
Function: OLED第2页显示：小车ID、坐标、角度、速度、电池电压等基础状态
Input   : none
Output  : none
**************************************************************************/
void display_page4(void)
{
    // 显示小车ID
    OLED_ShowString(0, 0, "ID :    ");
    OLED_ShowString(40, 0, "#");
    OLED_ShowNumber(45, 0, self_id, 2, 12);  // 显示ID（2位）

    // 显示使能状态（ON/OFF）
    if (EN == 1)
        OLED_ShowString(90, 0, "O N");
    else if (EN == 0)
        OLED_ShowString(90, 0, "OFF");

    // 显示当前坐标（保留2位小数）
    char title[16];
    sprintf(title, "x,y:(%1.2f,%1.2f)", position[0], position[1]);
    OLED_ShowString(0, 12, title);

    // 显示目标坐标（保留2位小数）
    char target_pos[20];
    sprintf(target_pos, "X,Y:(%1.2f,%1.2f)", Target_position[0], Target_position[1]);
    OLED_ShowString(0, 24, target_pos);

    // 显示航向角（当前Yaw/目标角度） - 下移一行
    OLED_ShowString(0, 36, "YAW:  ");
    if (Yaw > 0)
        OLED_ShowString(38, 36, "+");  // 正号标记
    else
        OLED_ShowString(40, 36, "-");  // 负号标记
    OLED_ShowNumber(43, 36, float_abs(Yaw), 3, 12);  // 显示角度绝对值

    OLED_ShowString(65, 36, ",");
    if (Target_Yaw > 0)
        OLED_ShowString(72, 36, "+ ");
    else
        OLED_ShowString(75, 36, "-");
    OLED_ShowNumber(80, 36, float_abs(Target_Yaw), 3, 12);  // 显示目标角度绝对值

    // 显示速度（当前速度/目标速度） - 下移一行
    OLED_ShowString(0, 48, "SPD:   ");
    OLED_ShowNumber(45, 48, current_speed, 2, 12);
    OLED_ShowString(65, 48, ",");
    OLED_ShowNumber(75, 48, target_speed, 2, 12);
}

/**************************************************************************
Function: OLED第1页显示：电机速度、Yaw角、电池电压等详细数据
Input   : none
Output  : none
**************************************************************************/
void display_page1(void)
{
    // 第1行：小车ID及Z轴角速度
    OLED_ShowString(0, 0, "#");
    OLED_ShowNumber(10, 0, self_id, 2, 12);  // 显示ID
    OLED_ShowString(55, 0, "GZ");          // 标记Z轴角速度
    if (gyro[2] < 0)
    {
        OLED_ShowString(80, 0, "-");
        OLED_ShowNumber(90, 0, -gyro[2], 5, 12);  // 显示负角速度绝对值
    }
    else
    {
        OLED_ShowString(80, 0, "+");
        OLED_ShowNumber(90, 0, gyro[2], 5, 12);   // 显示正角速度
    }

    // 第2行：电机A目标速度/实际速度
    OLED_ShowString(0, 10, "A");  // 标记电机A
    // 目标速度
    if (MOTOR_A.Target < 0)
    {
        OLED_ShowString(15, 10, "-");
        OLED_ShowNumber(20, 10, -MOTOR_A.Target * 1000, 5, 12);
    }
    else
    {
        OLED_ShowString(15, 10, "+");
        OLED_ShowNumber(20, 10, MOTOR_A.Target * 1000, 5, 12);
    }
    // 实际速度（编码器值）
    if (MOTOR_A.Encoder < 0)
    {
        OLED_ShowString(60, 10, "-");
        OLED_ShowNumber(75, 10, -MOTOR_A.Encoder * 1000, 5, 12);
    }
    else
    {
        OLED_ShowString(60, 10, "+");
        OLED_ShowNumber(75, 10, MOTOR_A.Encoder * 1000, 5, 12);
    }

    // 第3行：电机B目标速度/实际速度
    OLED_ShowString(0, 20, "B");  // 标记电机B
    // 目标速度
    if (MOTOR_B.Target < 0)
    {
        OLED_ShowString(15, 20, "-");
        OLED_ShowNumber(20, 20, -MOTOR_B.Target * 1000, 5, 12);
    }
    else
    {
        OLED_ShowString(15, 20, "+");
        OLED_ShowNumber(20, 20, MOTOR_B.Target * 1000, 5, 12);
    }
    // 实际速度
    if (MOTOR_B.Encoder < 0)
    {
        OLED_ShowString(60, 20, "-");
        OLED_ShowNumber(75, 20, -MOTOR_B.Encoder * 1000, 5, 12);
    }
    else
    {
        OLED_ShowString(60, 20, "+");
        OLED_ShowNumber(75, 20, MOTOR_B.Encoder * 1000, 5, 12);
    }

    // 第4行：电机C目标速度/实际速度
    OLED_ShowString(0, 30, "C");  // 标记电机C
    // 目标速度
    if (MOTOR_C.Target < 0)
    {
        OLED_ShowString(15, 30, "-");
        OLED_ShowNumber(20, 30, -MOTOR_C.Target * 1000, 5, 12);
    }
    else
    {
        OLED_ShowString(15, 30, "+");
        OLED_ShowNumber(20, 30, MOTOR_C.Target * 1000, 5, 12);
    }
    // 实际速度
    if (MOTOR_C.Encoder < 0)
    {
        OLED_ShowString(60, 30, "-");
        OLED_ShowNumber(75, 30, -MOTOR_C.Encoder * 1000, 5, 12);
    }
    else
    {
        OLED_ShowString(60, 30, "+");
        OLED_ShowNumber(75, 30, MOTOR_C.Encoder * 1000, 5, 12);
    }

    // 第5行：电机D目标速度/实际速度
    OLED_ShowString(0, 40, "D");  // 标记电机D
    // 目标速度
    if (MOTOR_D.Target < 0)
    {
        OLED_ShowString(15, 40, "-");
        OLED_ShowNumber(20, 40, -MOTOR_D.Target * 1000, 5, 12);
    }
    else
    {
        OLED_ShowString(15, 40, "+");
        OLED_ShowNumber(20, 40, MOTOR_D.Target * 1000, 5, 12);
    }
    // 实际速度
    if (MOTOR_D.Encoder < 0)
    {
        OLED_ShowString(60, 40, "-");
        OLED_ShowNumber(75, 40, -MOTOR_D.Encoder * 1000, 5, 12);
    }
    else
    {
        OLED_ShowString(60, 40, "+");
        OLED_ShowNumber(75, 40, MOTOR_D.Encoder * 1000, 5, 12);
    }

    // 第6行：Yaw角及电池电压
    // Yaw角（带正负号）
    if (Yaw > 0)
        OLED_ShowString(0, 50, "+");
    else
        OLED_ShowString(0, 50, "-");
    OLED_ShowNumber(10, 50, myabs(Yaw), 3, 12);  // 显示Yaw角绝对值

    // 电池电压（带小数）
    OLED_ShowNumber(75, 50, Voltage_Show / 100, 2, 12);    // 整数部分
    OLED_ShowString(88, 50, ".");
    OLED_ShowNumber(98, 50, Voltage_Show % 100, 2, 12);    // 小数部分
    OLED_ShowString(110, 50, "V");
    // 小数部分补零（确保2位显示）
    if (Voltage_Show % 100 < 10)
        OLED_ShowNumber(92, 50, 0, 2, 12);
}

// 显示三轴角速度、角度、坐标（明确显示正负号，适配5行×12像素）
void display_page3(void)
{
    // -------------------------- 第一行（行0）：Gx和Gy角速度 --------------------------
    // 左侧：Gx角速度（带正负号）
    OLED_ShowString(0, 0, "Gx:");
    if (gyro[0] < 0)
    {
        OLED_ShowString(24, 0, "-");  // 负号
        OLED_ShowNumber(34, 0, -gyro[0], 4, 12);  // 显示绝对值（4位）
    }
    else
    {
        OLED_ShowString(24, 0, "+");  // 正号
        OLED_ShowNumber(34, 0, gyro[0], 4, 12);   // 显示正值（4位）
    }

    // 右侧：Gy角速度（带正负号）
    OLED_ShowString(72, 0, "Gy:");
    if (gyro[1] < 0)
    {
        OLED_ShowString(96, 0, "-");  // 负号
        OLED_ShowNumber(106, 0, -gyro[1], 4, 12);  // 显示绝对值（4位）
    }
    else
    {
        OLED_ShowString(96, 0, "+");  // 正号
        OLED_ShowNumber(106, 0, gyro[1], 4, 12);   // 显示正值（4位）
    }


    // -------------------------- 第二行（行12）：Gz角速度 --------------------------
    OLED_ShowString(0, 12, "Gz:");
    if (gyro[2] < 0)
    {
        OLED_ShowString(24, 12, "-");  // 负号
        OLED_ShowNumber(34, 12, -gyro[2], 4, 12);  // 显示绝对值（4位）
    }
    else
    {
        OLED_ShowString(24, 12, "+");  // 正号
        OLED_ShowNumber(34, 12, gyro[2], 4, 12);   // 显示正值（4位）
    }


    // -------------------------- 第三行（行24）：Roll和Pitch角 --------------------------
    // 左侧：Roll角（带正负号，保留1位小数）
    OLED_ShowString(0, 24, "R:");
    int roll_val = (int)(Roll * 10);  // 保留1位小数（×10）
    if (roll_val < 0)
    {
        OLED_ShowString(20, 24, "-");  // 负号
        OLED_ShowNumber(30, 24, -roll_val / 10, 3, 12);  // 整数部分
        OLED_ShowString(54, 24, ".");
        OLED_ShowNumber(60, 24, -roll_val % 10, 1, 12);  // 小数部分
    }
    else
    {
        OLED_ShowString(20, 24, "+");  // 正号
        OLED_ShowNumber(30, 24, roll_val / 10, 3, 12);   // 整数部分
        OLED_ShowString(54, 24, ".");
        OLED_ShowNumber(60, 24, roll_val % 10, 1, 12);   // 小数部分
    }

    // 右侧：Pitch角（带正负号，保留1位小数）
    OLED_ShowString(72, 24, "P:");
    int pitch_val = (int)(Pitch * 10);
    if (pitch_val < 0)
    {
        OLED_ShowString(92, 24, "-");  // 负号
        OLED_ShowNumber(102, 24, -pitch_val / 10, 3, 12);  // 整数部分
        OLED_ShowString(126, 24, ".");  // 不超过110像素，完整显示
        OLED_ShowNumber(132, 24, -pitch_val % 10, 1, 12);  // 仅小数位可能超，不影响
    }
    else
    {
        OLED_ShowString(92, 24, "+");  // 正号
        OLED_ShowNumber(102, 24, pitch_val / 10, 3, 12);   // 整数部分
        OLED_ShowString(126, 24, ".");
        OLED_ShowNumber(132, 24, pitch_val % 10, 1, 12);
    }


    // -------------------------- 第四行（行36）：Yaw角 --------------------------
    OLED_ShowString(0, 36, "Y:");
    int yaw_val = (int)(Yaw * 10);  // 保留1位小数（×10）
    if (yaw_val < 0)
    {
        OLED_ShowString(20, 36, "-");  // 负号
        OLED_ShowNumber(30, 36, -yaw_val / 10, 3, 12);  // 整数部分
        OLED_ShowString(54, 36, ".");
        OLED_ShowNumber(60, 36, -yaw_val % 10, 1, 12);  // 小数部分
    }
    else
    {
        OLED_ShowString(20, 36, "+");  // 正号
        OLED_ShowNumber(30, 36, yaw_val / 10, 3, 12);   // 整数部分
        OLED_ShowString(54, 36, ".");
        OLED_ShowNumber(60, 36, yaw_val % 10, 1, 12);   // 小数部分
    }


    // -------------------------- 第五行（行48）：目标位置坐标 --------------------------
    // 坐标带正负号，保留1位小数
    char pos_str[16];
    // X坐标正负处理
    char x_sign = (Target_position[0] >= 0) ? '+' : '-';
    float x_abs = (Target_position[0] >= 0) ? Target_position[0] : -Target_position[0];
    // Y坐标正负处理
    char y_sign = (Target_position[1] >= 0) ? '+' : '-';
    float y_abs = (Target_position[1] >= 0) ? Target_position[1] : -Target_position[1];
    // 拼接字符串（如"X+1.2 Y-3.4"）
    sprintf(pos_str, "X%c%1.1f Y%c%1.1f", x_sign, x_abs, y_sign, y_abs);
    OLED_ShowString(0, 48, pos_str);  // 完整显示
}

// 修复后的page4显示：第二行显示本车，其他行按#1~#4顺序显示
void display_page2(void)
{
    const int LINE_HEIGHT = 12;  // 行高12像素
    char line_text[32];          // 每行显示的文本缓存
    // -------------------------- 第一行：标题行 --------------------------
    OLED_ShowString(0, 0 * LINE_HEIGHT, "ID   X    Y  YAW");
    
    // -------------------------- 第二行：本车信息 --------------------------
    sprintf(line_text, "#%1d %1.2f %1.2f%4d", 
            self_id,
            position[0],
            position[1],
            (int)Yaw);
    OLED_ShowString(0, 1 * LINE_HEIGHT, line_text);
    
}


