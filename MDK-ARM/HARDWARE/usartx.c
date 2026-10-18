#include "usartx.h"
#include "stm32f4xx.h"
SEND_DATA Send_Data;
RECEIVE_DATA Receive_Data;
extern int Time_count;
extern volatile uint8_t uwb_new_data_flag;
UWB_Data uwb_data;
char rx_buffer[256];
uint16_t rx_index = 0;

char rx6_buffer[256];
uint16_t rx6_index = 0;

char rx4_buffer[256];
uint16_t rx4_index = 0;

uint8_t esp32_rx_byte;

#if ESP32_BRIDGE_ENABLE
static uint8_t esp32_rx_ring[ESP32_BRIDGE_RX_RING_SIZE];
static volatile uint16_t esp32_rx_head = 0;
static volatile uint16_t esp32_rx_tail = 0;
static uint8_t esp32_frame_buf[ESP32_BRIDGE_FRAME_MAX];
static uint16_t esp32_frame_len = 0;
static uint8_t esp32_in_frame = 0;
static float esp32_target_vx = 0.0f;
static float esp32_target_vy = 0.0f;
static float esp32_target_vz = 0.0f;
static uint32_t esp32_last_cmd_tick = 0;
static uint8_t esp32_has_cmd = 0;
#endif

/**************************************************************************
Function: Refresh the OLED screen
Input   : none
Output  : none
函数功能：串口接收中断
入口参数：无
返回  值：无
**************************************************************************/

#if ESP32_BRIDGE_ENABLE
static uint8_t ESP32_IsFrameChar(uint8_t ch)
{
    if ((ch >= '0' && ch <= '9') || ch == '+' || ch == '-' || ch == '.' ||
        ch == ',' || ch == ' ' || ch == '\t' || ch == 'e' || ch == 'E') {
        return 1;
    }
    return 0;
}

static uint8_t ESP32_RingPop(uint8_t *out)
{
    uint16_t tail = esp32_rx_tail;
    if (esp32_rx_head == tail) {
        return 0;
    }
    *out = esp32_rx_ring[tail];
    esp32_rx_tail = (uint16_t)((tail + 1U) % ESP32_BRIDGE_RX_RING_SIZE);
    return 1;
}

static uint8_t ESP32_Parse_Frame(const uint8_t *buf, uint16_t len, float *vx, float *vy, float *vz)
{
    char tmp[ESP32_BRIDGE_FRAME_MAX];
    char *p = NULL;
    char *end = NULL;

    if (len == 0U || len >= sizeof(tmp)) {
        return 0;
    }

    memcpy(tmp, buf, len);
    tmp[len] = '\0';

    p = tmp;
    *vx = strtof(p, &end);
    if (end == p) {
        return 0;
    }
    p = end;
    while (*p == ' ' || *p == '\t') {
        p++;
    }
    if (*p != ',') {
        return 0;
    }
    p++;

    *vy = strtof(p, &end);
    if (end == p) {
        return 0;
    }
    p = end;
    while (*p == ' ' || *p == '\t') {
        p++;
    }
    if (*p != ',') {
        return 0;
    }
    p++;

    *vz = strtof(p, &end);
    if (end == p) {
        return 0;
    }
    p = end;
    while (*p == ' ' || *p == '\t') {
        p++;
    }
    if (*p != '\0') {
        return 0;
    }

    return 1;
}

void ESP32_Bridge_Init(void)
{
    esp32_rx_head = 0;
    esp32_rx_tail = 0;
    esp32_frame_len = 0;
    esp32_in_frame = 0;
    esp32_target_vx = 0.0f;
    esp32_target_vy = 0.0f;
    esp32_target_vz = 0.0f;
    esp32_last_cmd_tick = 0;
    esp32_has_cmd = 0;
}

void ESP32_Bridge_RxByte(uint8_t byte)
{
    uint16_t next = (uint16_t)((esp32_rx_head + 1U) % ESP32_BRIDGE_RX_RING_SIZE);
    if (next != esp32_rx_tail) {
        esp32_rx_ring[esp32_rx_head] = byte;
        esp32_rx_head = next;
    }
}

void ESP32_Bridge_Task(void)
{
    uint8_t ch = 0;
    float vx = 0.0f;
    float vy = 0.0f;
    float vz = 0.0f;

    while (ESP32_RingPop(&ch)) {
        if (ch == '[') {
            esp32_in_frame = 1;
            esp32_frame_len = 0;
            continue;
        }
        if (!esp32_in_frame) {
            continue;
        }
        if (ch == ']') {
            if (ESP32_Parse_Frame(esp32_frame_buf, esp32_frame_len, &vx, &vy, &vz)) {
                esp32_target_vx = vx;
                esp32_target_vy = vy;
                esp32_target_vz = vz;
                esp32_last_cmd_tick = HAL_GetTick();
                esp32_has_cmd = 1;
            }
            esp32_in_frame = 0;
            esp32_frame_len = 0;
            continue;
        }
        if (!ESP32_IsFrameChar(ch)) {
            esp32_in_frame = 0;
            esp32_frame_len = 0;
            continue;
        }
        if (esp32_frame_len < (ESP32_BRIDGE_FRAME_MAX - 1U)) {
            esp32_frame_buf[esp32_frame_len++] = ch;
        } else {
            esp32_in_frame = 0;
            esp32_frame_len = 0;
        }
    }
}

uint8_t ESP32_Bridge_GetTargets(float *vx, float *vy, float *vz, uint32_t *last_tick)
{
    if (vx == NULL || vy == NULL || vz == NULL || last_tick == NULL) {
        return 0;
    }
    if (!esp32_has_cmd) {
        *vx = 0.0f;
        *vy = 0.0f;
        *vz = 0.0f;
        *last_tick = 0;
        return 0;
    }
    *vx = esp32_target_vx;
    *vy = esp32_target_vy;
    *vz = esp32_target_vz;
    *last_tick = esp32_last_cmd_tick;
    return 1;
}
#else
void ESP32_Bridge_Init(void) {}
void ESP32_Bridge_RxByte(uint8_t byte) { (void)byte; }
void ESP32_Bridge_Task(void) {}
uint8_t ESP32_Bridge_GetTargets(float *vx, float *vy, float *vz, uint32_t *last_tick)
{
    if (vx) { *vx = 0.0f; }
    if (vy) { *vy = 0.0f; }
    if (vz) { *vz = 0.0f; }
    if (last_tick) { *last_tick = 0; }
    return 0;
}
#endif

void ESP32_SendStatus(float x, float y, float heading, float vx, float vy, float vz)
{
    char msg[96];
    float safe_x = x;
    float safe_y = y;
    float safe_heading = heading;
    float safe_vx = vx;
    float safe_vy = vy;
    float safe_vz = vz;

    if (safe_x != safe_x) { safe_x = 0.0f; }
    if (safe_y != safe_y) { safe_y = 0.0f; }
    if (safe_heading != safe_heading) { safe_heading = 0.0f; }
    if (safe_vx != safe_vx) { safe_vx = 0.0f; }
    if (safe_vy != safe_vy) { safe_vy = 0.0f; }
    if (safe_vz != safe_vz) { safe_vz = 0.0f; }

    while (safe_heading > 180.0f) { safe_heading -= 360.0f; }
    while (safe_heading < -180.0f) { safe_heading += 360.0f; }

    int len = snprintf(msg, sizeof(msg), "[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]\r\n",
                       safe_x, safe_y, safe_heading, safe_vx, safe_vy, safe_vz);
    if (len > 0) {
        (void)HAL_UART_Transmit(&huart6, (uint8_t *)msg, (uint16_t)len, 10);
    }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{ 
    if (huart->Instance == UART5) {  // 假设发送：$KT1,1.69,2.93,4.98,NULL,LO=[-2.45,5.44,1.43]
      // 检查是否收到完整的一行（以换行符结束）
        if (rx_buffer[rx_index] == '\n') {
            // 处理完整的数据行
            rx_buffer[rx_index + 1] = '\0'; // 添加字符串结束符
            Parse_UWB_Data(rx_buffer);
            rx_index = 0;
        } else {
            rx_index++;
            if (rx_index >= sizeof(rx_buffer)) {
                rx_index = 0; // 防止缓冲区溢出
            }
        }
        // 重新启动接收
        HAL_UART_Receive_IT(&huart5, (uint8_t*)&rx_buffer[rx_index], 1);    
    } else if(huart->Instance == USART6) {
        ESP32_Bridge_RxByte(esp32_rx_byte);
        HAL_UART_Receive_IT(&huart6, &esp32_rx_byte, 1);
    }

	
}


/**************************************************************************
Function: Serial port 1 sends data
Input   : The data to send
Output  : none
函数功能：串口1发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart1_send(u8 data)
{
	USART1->DR = data;
	while((USART1->SR&0x40)==0);	
}
/**
 * Send a string of arbitrary length via USART1
 * @param str Pointer to the string to be sent
 * @param length Number of bytes to send
 */
void usart1_send_cstring(const char *str)
{
    // Check for null pointer
    if (str == NULL) return;
    
    // Send characters until null terminator is encountered
    while (*str != '\0') {
        usart1_send((u8)*str);
        str++;
    }
}


/**************************************************************************
Function: Serial port 3 sends data
Input   : The data to send
Output  : none
函数功能：串口3发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart5_send(u8 data)
{
	USART3->DR = data;
	while((UART5->SR&0x40)==0);	
}
/**
 * Send a string of arbitrary length via USART1
 * @param str Pointer to the string to be sent
 * @param length Number of bytes to send
 */
void usart5_send_cstring(const char *str)
{
    // Check for null pointer
    if (str == NULL) return;
    
    // Send characters until null terminator is encountered
    while (*str != '\0') {
        usart5_send((u8)*str);
        str++;
    }
}

void Parse_UWB_Data(char* data) {
    // 初始化UWB数据结构
    memset(&uwb_data, 0, sizeof(uwb_data));
    for (int i = 0; i < 4; i++) {
        uwb_data.distances[i] = NAN;
    }
    uwb_data.valid_position = 0;
    
    // 检查数据起始符
    if (data[0] != '$') {
        return; // 无效数据
    }
    
    // 提取角色和滤波模式 (如KT0中的K和T0)
    if (strlen(data) >= 3 && data[1] == 'K') {
        strncpy(uwb_data.filter_mode, "K", 1);
        // 提取角色 (T0, T1等)
        if (sscanf(data + 2, "%2s", uwb_data.role) == 1) {
            uwb_data.role[2] = '\0'; // 确保字符串结束
        }
    }
    
    // 提取距离信息
    char* distances_start = strchr(data, ',') + 1; // 找到第一个逗号后的位置
    if (distances_start) {
        int count = sscanf(distances_start, "%f,%f,%f,%f",
                         &uwb_data.distances[0],
                         &uwb_data.distances[1],
                         &uwb_data.distances[2],
                         &uwb_data.distances[3]);
        
        // 计算有效距离数量
        uwb_data.valid_distances = 0;
        for (int i = 0; i < 4; i++) {
            if (!isnan(uwb_data.distances[i])) {
                uwb_data.valid_distances++;
            }
        }
    }
    
    // 提取坐标信息
    char* lo_start = strstr(data, "LO=[");
    if (lo_start) {
        lo_start += 4; // 跳过"LO=["
        char* lo_end = strchr(lo_start, ']');
        if (lo_end) {
            // 临时存储坐标字符串
            char coord_str[50];
            int len = lo_end - lo_start;
            if (len < sizeof(coord_str) - 1) {
                strncpy(coord_str, lo_start, len);
                coord_str[len] = '\0';
                
                // 解析坐标值
                if (sscanf(coord_str, "%f,%f,%f",
                         &uwb_data.position[0],
                         &uwb_data.position[1],
                         &uwb_data.position[2]) == 3) { // 解析到了坐标值
                    uwb_data.valid_position = 1;
                    
                    if (uwb_data.valid_position == 1) {
            
                        // ====================================================
                        // UWB 数据清洗 (中值 + 智能低通)
                        // ====================================================
                        #define UWB_WINDOW_SIZE 5
                        static float x_buffer[UWB_WINDOW_SIZE] = {0};
                        static float y_buffer[UWB_WINDOW_SIZE] = {0};
                        static uint8_t buf_index = 0;
                        static uint8_t is_buffer_full = 0;
                        static float last_clean_x = 0.0f;
                        static float last_clean_y = 0.0f;

                        // 1. 将新数据推入滑动窗口
                        x_buffer[buf_index] = uwb_data.position[0];
                        y_buffer[buf_index] = uwb_data.position[1];
                        buf_index++;
                        if (buf_index >= UWB_WINDOW_SIZE) {
                            buf_index = 0;
                            is_buffer_full = 1;
                        }

                        // 如果缓冲区还没填满（开机前几帧），先直接用原始数据
                        float clean_x = uwb_data.position[0];
                        float clean_y = uwb_data.position[1];

                        if (is_buffer_full) {
                            // 2. 提取中值 (Median Filter) - 彻底剔除突变飞点
                            float temp_x[UWB_WINDOW_SIZE];
                            float temp_y[UWB_WINDOW_SIZE];
                            memcpy(temp_x, x_buffer, sizeof(x_buffer));
                            memcpy(temp_y, y_buffer, sizeof(y_buffer));

                            // 冒泡排序 (N=5 非常快，无性能负担)
                            for (int i = 0; i < UWB_WINDOW_SIZE - 1; i++) {
                                for (int j = 0; j < UWB_WINDOW_SIZE - i - 1; j++) {
                                    if (temp_x[j] > temp_x[j+1]) {
                                        float t = temp_x[j]; temp_x[j] = temp_x[j+1]; temp_x[j+1] = t;
                                    }
                                    if (temp_y[j] > temp_y[j+1]) {
                                        float t = temp_y[j]; temp_y[j] = temp_y[j+1]; temp_y[j+1] = t;
                                    }
                                }
                            }
                            
                            // 取中间值
                            float median_x = temp_x[UWB_WINDOW_SIZE / 2];
                            float median_y = temp_y[UWB_WINDOW_SIZE / 2];

                            // 3. 智能放行检查 (Smart Bypass)
                            float dx = median_x - last_clean_x;
                            float dy = median_y - last_clean_y;
                            float dist = sqrt(dx * dx + dy * dy); 

                            if (dist > 1.0f) {
                                // 【情况 A：被搬走】距离发生剧烈跳变 (>1.0m)
                                // 中值都这么大，说明连续多帧都在新位置。这不是飞点，是真实位移！
                                // 直接击穿低通滤波器，把最新位置立刻传给后端 ESKF！
                                clean_x = median_x;
                                clean_y = median_y;
                            } else {
                                // 【情况 B：正常行驶】微小的正常移动
                                // 使用一阶低通滤波器让轨迹丝滑
                                // Alpha = 0.3：相信 30% 新中值，70% 维持历史惯性
                                float alpha = 0.3f; 
                                clean_x = last_clean_x * (1.0f - alpha) + median_x * alpha;
                                clean_y = last_clean_y * (1.0f - alpha) + median_y * alpha;
                            }
                        }

                        // 更新历史值
                        last_clean_x = clean_x;
                        last_clean_y = clean_y;
                        // ====================================================
                        
                        // 存储洗干净的位置坐标
                        position[0] = clean_x; // 存储X坐标
                        position[1] = clean_y; // 存储Y坐标
                    }
                    position[2] = uwb_data.position[2]; // 存储Z坐标
                    uwb_new_data_flag = 1;
                }
            }
        }
    }
}





