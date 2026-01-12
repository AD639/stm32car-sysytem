/*
 * 功能说明：
 * 1. 电机测试模式 - 测试四轮全向运动
 * 2. 红外巡线模式 - 基于四路红外传感器的黑线跟踪
 * 3. 超声波跟随模式 - 基于HC-SR04的目标跟随
 * 4. WiFi远程控制 - 通过ESP8266实现无线控制
 */

#include "stm32f10x.h"
#include "car_config.h"
#include "motor_control.h"
#include "sensor.h"
#include "wifi_control.h"
#include "car_modes.h"
#include "delay.h"
#include "led_key.h"
#include <string.h>

/* 函数声明 */
void WiFi_Mode_Switch_Handle(uint8_t cmd);

/* 外部变量声明 */
extern volatile uint8_t motor_test_running;            // 电机测试运行标志

/* 全局变量定义 */
Car_WorkMode g_CurMode = MODE_STOP;                    // 当前工作模式
volatile CarState_t Car_State = CAR_STATE_STOP;       // 小车运动状态

/* 红外巡线相关变量 */
uint8_t SensorState[4] = {1,1,1,1};                   // 传感器状态数组

/* 超声波测距相关变量 */
uint32_t TimeCount = 0;                               // 定时器计数值
float CurrentDistance = 0;                            // 当前测距值
uint8_t MeasureFlag = 0;                              // 测距完成标志

/* WiFi控制相关变量 */
volatile uint8_t WIFI_Connected = 0;                  // WiFi连接状态
volatile uint8_t Key_Pressed_Flag = 0;                // 按键按下标志
volatile uint8_t Rx_Cmd_Flag = 0;                     // 接收命令标志
volatile uint8_t Rx_Cmd_Data = 0;                     // 接收命令数据
volatile uint8_t wifi_remote_active = 0;              // WiFi远程控制激活标志
volatile uint8_t current_wifi_cmd = 0;                // 当前WiFi控制指令
volatile uint8_t serial_cmd_enabled = 0;              // 串口命令使能标志，防止初始化时误触发
volatile uint8_t motor_test_running = 0;              // 电机测试运行标志

/* ESP8266数据接收缓冲区 */
volatile char Esp_RxBuf[ESP_RX_BUF_LEN];
volatile uint8_t Esp_RxIdx = 0;

/* ESP8266 TCP数据解析状态机 */
static uint8_t ipd_state = 0;                        // IPD解析状态
static uint8_t ipd_char_count = 0;                   // IPD字符计数

/* 串口接收缓冲区 */
volatile uint8_t serialRxBuf[SERIAL_RX_BUF_LEN];
volatile uint16_t serialRxHead = 0, serialRxTail = 0;

/*
 * TIM3中断服务函数 - 超声波测距计时
 */
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
    {
        TimeCount++;
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    }
}


/*
 * USART1中断服务函数 - WiFi数据接收处理
 * 
 * 功能：
 * 1. 解析ESP8266的+IPD格式数据
 * 2. 检测"CONNECT"连接字符串
 * 3. 提取TCP数据中的控制指令
 */
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
    {
        char c = (char)USART_ReceiveData(USART1);

        /* 记录到缓冲区用于检测"CONNECT" */
        Esp_RxBuf[Esp_RxIdx++] = c;
        if (Esp_RxIdx >= ESP_RX_BUF_LEN)
        {
            Esp_RxIdx = 0;
        }

        /* 检测"CONNECT"字符串 */
        if (WIFI_Connected == 0 && Esp_RxIdx >= 7)
        {
            char tmp[32];
            uint8_t i, start = (Esp_RxIdx >= 32) ? (Esp_RxIdx - 32) : 0;
            for (i = 0; i < Esp_RxIdx - start && i < 31; i++)
            {
                tmp[i] = Esp_RxBuf[start + i];
            }
            tmp[i] = '\0';
            if (strstr(tmp, "CONNECT") != NULL)
            {
                WIFI_Connected = 1;
            }
        }

        /* 解析+IPD格式数据 */
        switch (ipd_state)
        {
            case 0:  // 等待'+'
                if (c == '+')
                {
                    ipd_state = 1;
                    ipd_char_count = 1;
                }
                else
                {
                    // 只有在WiFi连接成功后，才将非+IPD数据放入串口缓冲区
                    // 避免ESP8266连接过程中的状态信息被误认为用户命令
                    if (WIFI_Connected)
                    {
                        uint16_t next = (serialRxHead + 1) % SERIAL_RX_BUF_LEN;
                        if (next != serialRxTail) 
                        { 
                            serialRxBuf[serialRxHead] = (uint8_t)c; 
                            serialRxHead = next; 
                        }
                    }
                }
                break;
            case 1:  // 等待'I'
                if (c == 'I')
                {
                    ipd_state = 2;
                    ipd_char_count = 2;
                }
                else
                {
                    ipd_state = 0;
                }
                break;
            case 2:  // 等待'P'和'D'
                if (ipd_char_count == 2 && c == 'P')
                {
                    ipd_char_count = 3;
                }
                else if (ipd_char_count == 3 && c == 'D')
                {
                    ipd_state = 3;
                    ipd_char_count = 0;
                }
                else
                {
                    ipd_state = 0;
                }
                break;
            case 3:  // 等待冒号':'
                if (c == ':')
                {
                    ipd_state = 4;
                    ipd_char_count = 0;
                }
                else if (c == '\n' || c == '\r')
                {
                    ipd_state = 0;
                }
                break;
            case 4:  // 读取控制指令
                if (c >= '0' && c <= '9')
                {
                    Rx_Cmd_Data = (uint8_t)c;
                    Rx_Cmd_Flag = 1;
                }
                ipd_state = 0;
                break;
        }

        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}
void System_Init(void)
{
    SystemInit();
    
    /* 硬件初始化 */
    LED_Init();
    Mode_Indicator_Init();  // 初始化模式切换指示灯
    KEY_K2_Init();
    TIM2_PWM_Init();
    TIM4_PWM_Init();
    USART1_Init(115200);
    
    /* 传感器初始化 */
    Sensor_Init();
    HC_SR04_GPIO_Init();
    HC_SR04_TIM3_Init();
    
    /* 初始状态设置 */
    Car_Stop();
    
    Usart1_SendString("STM32 WIFI Car start...\r\n");
    
    /* ESP8266初始化 */
    if (ESP8266_Init())
    {
        Usart1_SendString("ESP8266 Init OK\r\n");
    }
    else
    {
        Usart1_SendString("ESP8266 Init FAIL\r\n");
    }
    
    USART1_SendString("Car System Init Done!\r\n");
    USART1_SendString("WiFi AP: ESP8266AP_lisixian, Password: 12345678, TCP Port: 8080\r\n");
    USART1_SendString("Please connect to WiFi and send any command from mobile app to activate control.\r\n");
    USART1_SendString("WiFi Commands: 0(Stop),1(Test),2(IR),3(US),8(Forward),5(Back),4(Left),6(Right),7(TransLeft),9(TransRight)\r\n");
}
/*
 * WiFi连接状态处理（已禁用LED指示）
 */
void WiFi_Status_Handle(void)
{
    // WiFi连接状态不再通过LED显示
}

/*
 * 按键处理（已禁用WiFi验证功能）
 */
void Key_Handle(void)
{
    if (Key_Pressed_Flag)
    {
        /* 消抖处理 */
        Delay_ms(20);
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == Bit_RESET)
        {
            // K2按键功能已禁用，不再用于WiFi验证
        }
        Key_Pressed_Flag = 0;
    }
}

/*
 * WiFi模式切换处理
 */
void WiFi_Mode_Switch_Handle(uint8_t cmd)
{
    switch(cmd)
    {
        case '0':
            g_CurMode = MODE_STOP;
            Car_Stop();
            Reset_IR_Tracking_State();
            wifi_remote_active = 0;
            WiFi_SendResponse("WiFi CMD: Switched to Stop Mode\r\n");
            break;
        case '1':
            g_CurMode = MODE_MOTOR_TEST;
            Reset_Motor_Test_State();  
            wifi_remote_active = 0;
            WiFi_SendResponse("WiFi CMD: Switched to Motor Test Mode\r\n");
            break;
        case '2':
            g_CurMode = MODE_IR_TRACKING;
            Reset_IR_Tracking_State();
            wifi_remote_active = 0;
            WiFi_SendResponse("WiFi CMD: Switched to IR Tracking Mode\r\n");
            WiFi_SendResponse("Waiting for black line...\r\n");
            break;
        case '3':
            g_CurMode = MODE_US_FOLLOW;
            wifi_remote_active = 0;
            WiFi_SendResponse("WiFi CMD: Switched to Ultrasonic Following Mode\r\n");
            WiFi_SendResponse("Start following target...\r\n");
            break;
        default:
            break;
    }
}

/*
 * WiFi远程控制处理
 */
void WiFi_Remote_Handle(void)
{
    // 只有在WiFi连接成功且电机测试未运行时才处理远程命令
    if (!WIFI_Connected || motor_test_running) return;
    
    if (Rx_Cmd_Flag)
    {
        uint8_t cmd = Rx_Cmd_Data;
        Rx_Cmd_Flag = 0;  // 立即清除标志
        
        // 收到第一个WiFi命令时，启用串口命令处理
        if (!serial_cmd_enabled)
        {
            serial_cmd_enabled = 1;
            WiFi_SendResponse("Serial command enabled. You can now use mobile app commands.\r\n");
        }
        
        // 处理命令前清空其他可能的标志
        wifi_remote_active = 0;
        
        // 检查是否是模式切换命令
        if (cmd == '0' || cmd == '1' || cmd == '2' || cmd == '3')
        {
            // 处理模式切换命令
            WiFi_Mode_Switch_Handle(cmd);
        }
        else
        {
            // 处理移动控制命令
            current_wifi_cmd = cmd;
            wifi_remote_active = 1;
            
            switch (cmd)
            {
                case '8':
                    WiFi_SendResponse("CMD: FORWARD\r\n");
                    break;
                case '5':  // 修改后退命令为5，避免与模式2冲突
                    WiFi_SendResponse("CMD: BACKWARD\r\n");
                    break;
                case '4':
                    WiFi_SendResponse("CMD: TURN LEFT\r\n");
                    break;
                case '6':
                    WiFi_SendResponse("CMD: TURN RIGHT\r\n");
                    break;
                case '7':  // 修改左平移命令为7，避免与模式1冲突
                    WiFi_SendResponse("CMD: TRANS LEFT\r\n");
                    break;
                case '9':  // 修改右平移命令为9，避免与模式3冲突
                    WiFi_SendResponse("CMD: TRANS RIGHT\r\n");
                    break;
                default:
                    WiFi_SendResponse("CMD: STOP\r\n");
                    wifi_remote_active = 0;
                    break;
            }
        }
        
        // 处理完命令后清空串口缓冲区
        SerialClearBuffer();
    }
}

/*
 * WiFi远程控制执行
 */
void WiFi_Remote_Execute(void)
{
    // 只有在停止模式下才执行移动控制命令
    if (wifi_remote_active && g_CurMode == MODE_STOP)
    {
        switch (current_wifi_cmd)
        {
            case '8':
                Car_Forward(PWM_VAL);
                break;
            case '5':  // 后退
                Car_Backward(PWM_VAL);
                break;
            case '4':
                Car_TurnLeft(PWM_VAL);
                break;
            case '6':
                Car_TurnRight(PWM_VAL);
                break;
            case '7':  // 左平移
                Car_TransLeft(PWM_VAL);
                break;
            case '9':  // 右平移
                Car_TransRight(PWM_VAL);
                break;
            default:
                Car_Stop();
                wifi_remote_active = 0;
                break;
        }
    }
}

/*
 * 工作模式处理 
 */
void Work_Mode_Handle(void)
{
    static Car_WorkMode last_mode = MODE_STOP;
    static uint8_t mode_executed = 0;
    
    // 检测模式变化（不输出调试信息）
    if (g_CurMode != last_mode)
    {
        last_mode = g_CurMode;
        mode_executed = 0;  // 新模式，重置执行标志
    }
    
    switch (g_CurMode)
    {
        case MODE_STOP:
            if (!wifi_remote_active)
            {
                Car_Stop();
            }
            mode_executed = 0;  // 停止模式可以重复执行
            break;
            
        case MODE_MOTOR_TEST:
            wifi_remote_active = 0;
            if (!mode_executed)
            {
                mode_executed = 1;
                Mode1_MotorTest();
                // 电机测试会自动切换回停止模式
            }
            break;
            
        case MODE_IR_TRACKING:
            wifi_remote_active = 0;
            // 红外巡线需要持续执行
            Mode2_IRTracking();
            break;
            
        case MODE_US_FOLLOW:
            wifi_remote_active = 0;
            // 超声波跟随需要持续执行
            Mode3_US_Follow();
            break;
            
        default:
            // 无效模式时重置为停止模式（不输出调试信息）
            g_CurMode = MODE_STOP;
            mode_executed = 0;
            break;
    }
}

/*
 * 主函数
 */
int main(void)
{
    /* 系统初始化 */
    System_Init();
    
    /* 主循环 */
    while (1)
    {
        /* WiFi连接状态处理 */
        WiFi_Status_Handle();
        
        /* 按键处理 */
        Key_Handle();
        
        /* WiFi远程控制命令处理 */
        WiFi_Remote_Handle();
        
        /* 串口命令解析 */
        Car_ParseUSART_CMD();
        
        /* WiFi远程控制执行 */
        WiFi_Remote_Execute();
        
        /* 工作模式处理 */
        Work_Mode_Handle();
    }
}