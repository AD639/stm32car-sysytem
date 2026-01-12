#include "car_modes.h"
#include "motor_control.h"
#include "sensor.h"
#include "wifi_control.h"
#include "delay.h"
#include "led_key.h"

/* 外部变量声明 */
extern Car_WorkMode g_CurMode;
extern uint8_t SensorState[4];
extern float CurrentDistance;
extern volatile uint8_t WIFI_Connected;               // WiFi连接状态
extern volatile uint8_t serial_cmd_enabled;           // 串口命令使能标志
extern volatile uint8_t motor_test_running;           // 电机测试运行标志
extern volatile uint8_t Rx_Cmd_Flag;                  // 接收命令标志
extern volatile uint8_t wifi_remote_active;           // WiFi远程控制激活标志

/* 红外巡线相关变量 */
static int Last_Bias_Direction = 0;                   // 上次偏移方向
static uint8_t hasFoundBlack = 0;                     // 是否找到黑线标志
uint8_t motor_test_executed = 0;                      // 电机测试是否已执行标志（全局变量）

/*
 * 模式1：电机测试
 */
void Mode1_MotorTest(void)
{
    // 如果已经执行过测试，直接返回
    if (motor_test_executed) return;
    
    motor_test_executed = 1;  // 标记为已执行
    motor_test_running = 1;   // 标记测试正在运行，禁用命令处理
    
    WiFi_SendResponse("Enter Mode 1: Motor Test\r\n");
    WiFi_SendResponse("Start: Forward & Backward\r\n");

    Car_Forward(500);
    delay_ms(2000);
    Car_Backward(500);
    delay_ms(2000);
    Car_Stop();
    delay_ms(500);

    WiFi_SendResponse("Start: Left & Right Move\r\n");
    Car_TransLeft(400);
    delay_ms(1000);
    Car_TransRight(400);
    delay_ms(1000);
    Car_Stop();
    delay_ms(500);

    WiFi_SendResponse("Start: Rotate\r\n");
    Car_TurnLeft(300);
    delay_ms(1000);
    Car_TurnRight(300);
    delay_ms(1000);
    Car_Stop();
    delay_ms(500);

    WiFi_SendResponse("Start: Omnidirectional Move\r\n");
    Motor1_SetSpeed(1, 450);
    Motor2_SetSpeed(1, 0);
    Motor3_SetSpeed(1, 450);
    Motor4_SetSpeed(1, 0);
    delay_ms(1500);
    Car_Stop();
    delay_ms(500);

    WiFi_SendResponse("Mode 1 Done: Back to Stop Mode\r\n\r\n");
    g_CurMode = MODE_STOP;
    motor_test_running = 0;   // 测试完成，重新启用命令处理
    SerialClearBuffer();      // 清空串口缓冲区，防止积累的数据被处理
}

/*
 * 模式2：红外巡线
 */
void Mode2_IRTracking(void)
{
    SensorState[0] = Sensor1_Get_State();
    SensorState[1] = Sensor2_Get_State();
    SensorState[2] = Sensor3_Get_State();
    SensorState[3] = Sensor4_Get_State();

    uint8_t hasBlack = 0;
    int i;
    for (i=0;i<4;i++) if (SensorState[i]==0) { hasBlack=1; break; }

    if (!hasFoundBlack) {
        if (hasBlack) {
            hasFoundBlack = 1;
            WiFi_SendResponse("Black line found! Start tracking...\r\n");
        } else {
            Car_Stop();
            return;
        }
    }

    if (SensorState[0] && !SensorState[1] && !SensorState[2] && SensorState[3]) {
        Last_Bias_Direction = 0; Car_Forward(SPEED_RUN);
    } else if (SensorState[0] && SensorState[1] && !SensorState[2] && SensorState[3]) {
        Last_Bias_Direction = 1; Car_TurnRight(SPEED_TURN);
    } else if (SensorState[0] && SensorState[1] && SensorState[2] && !SensorState[3]) {
        Last_Bias_Direction = 1; Car_TurnRight(SPEED_SPIN);
    } else if (SensorState[0] && !SensorState[1] && SensorState[2] && SensorState[3]) {
        Last_Bias_Direction = 2; Car_TurnLeft(SPEED_TURN);
    } else if (!SensorState[0] && SensorState[1] && SensorState[2] && SensorState[3]) {
        Last_Bias_Direction = 2; Car_TurnLeft(SPEED_SPIN);
    } else if (SensorState[0] && SensorState[1] && SensorState[2] && SensorState[3]) {
        if (Last_Bias_Direction == 1) Car_TurnRight(SPEED_SPIN);
        else if (Last_Bias_Direction == 2) Car_TurnLeft(SPEED_SPIN);
        else Car_Forward(300);
    } else if (!SensorState[0] && !SensorState[1] && !SensorState[2] && !SensorState[3]) {
        Car_Forward(SPEED_RUN);
    }
    delay_ms(10);
}

/*
 * 模式3：超声波跟随
 */
void Mode3_US_Follow(void)
{
    CurrentDistance = HC_SR04_MeasureDistance();
    Delay_us(50000);

    if (CurrentDistance == 0) Car_Stop();
    else if (CurrentDistance < FOLLOW_MIN_DIST) Car_Backward(FOLLOW_SPEED_LOW);
    else if (CurrentDistance > FOLLOW_TARGET_DIST + 100) Car_Forward(FOLLOW_SPEED_HIGH);
    else if (CurrentDistance > FOLLOW_TARGET_DIST) Car_Forward(FOLLOW_SPEED_MID);
    else if (CurrentDistance < FOLLOW_TARGET_DIST) Car_Forward(FOLLOW_SPEED_LOW);
    else Car_Stop();
}

void Car_ParseUSART_CMD(void)
{
    // 只有在WiFi连接成功且串口命令已启用且电机测试未运行时才处理串口命令
    if (!WIFI_Connected || !serial_cmd_enabled || motor_test_running) return;
    
    int rc = SerialReadChar();
    if (rc < 0) return;
    uint8_t cmd = (uint8_t)rc;

    // 处理命令前先清空所有可能的标志，防止重复执行
    Rx_Cmd_Flag = 0;
    wifi_remote_active = 0;

    switch(cmd)
    {
        case '0':
            Mode_Switch_Blink();  // 模式切换指示
            g_CurMode = MODE_STOP;
            Car_Stop();
            Reset_IR_Tracking_State();
            WiFi_SendResponse("Switched to: Stop Mode\r\n");
            break;
        case '1':
            Mode_Switch_Blink();  // 模式切换指示
            g_CurMode = MODE_MOTOR_TEST;
            Reset_Motor_Test_State();  // 重置电机测试状态
            WiFi_SendResponse("Switched to: Motor Test Mode\r\n");
            break;
        case '2':
            Mode_Switch_Blink();  // 模式切换指示
            g_CurMode = MODE_IR_TRACKING;
            Reset_IR_Tracking_State();
            WiFi_SendResponse("Switched to: IR Tracking Mode\r\n");
            WiFi_SendResponse("Waiting for black line...\r\n");
            break;
        case '3':
            Mode_Switch_Blink();  // 模式切换指示
            g_CurMode = MODE_US_FOLLOW;
            WiFi_SendResponse("Switched to: Ultrasonic Following Mode\r\n");
            WiFi_SendResponse("Start following target...\r\n");
            break;
        default:
            break;
    }
    
    // 处理完命令后再次清空缓冲区，防止积累
    SerialClearBuffer();
}

/*
 * 重置红外巡线状态
 */
void Reset_IR_Tracking_State(void)
{
    Last_Bias_Direction = 0;
    hasFoundBlack = 0;
}

/*
 * 重置电机测试状态
 */
void Reset_Motor_Test_State(void)
{
    motor_test_executed = 0;
}