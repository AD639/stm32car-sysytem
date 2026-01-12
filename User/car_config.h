#ifndef __CAR_CONFIG_H
#define __CAR_CONFIG_H

#include "stm32f10x.h"

/* 电机引脚定义 */
#define MOTOR1_BACKWARD_PIN  GPIO_Pin_0  // PA0 -> TIM2_CH1
#define MOTOR1_FORWARD_PIN   GPIO_Pin_1  // PA1 -> TIM2_CH2
#define MOTOR1_PORT          GPIOA

#define MOTOR2_BACKWARD_PIN  GPIO_Pin_2  // PA2 -> TIM2_CH3
#define MOTOR2_FORWARD_PIN   GPIO_Pin_3  // PA3 -> TIM2_CH4
#define MOTOR2_PORT          GPIOA

#define MOTOR3_BACKWARD_PIN  GPIO_Pin_6  // PB6 -> TIM4_CH1
#define MOTOR3_FORWARD_PIN   GPIO_Pin_7  // PB7 -> TIM4_CH2
#define MOTOR3_PORT          GPIOB

#define MOTOR4_BACKWARD_PIN  GPIO_Pin_8  // PB8 -> TIM4_CH3
#define MOTOR4_FORWARD_PIN   GPIO_Pin_9  // PB9 -> TIM4_CH4
#define MOTOR4_PORT          GPIOB

/* 红外传感器引脚定义 */
#define SENSOR1_PIN  GPIO_Pin_1  // PB1 -> 左前
#define SENSOR2_PIN  GPIO_Pin_0  // PB0 -> 左后
#define SENSOR3_PIN  GPIO_Pin_4  // PB4 -> 右后
#define SENSOR4_PIN  GPIO_Pin_5  // PB5 -> 右前
#define SENSOR_PORT  GPIOB

/* 超声波传感器引脚定义 */
#define ULTRASONIC_TRIG_PIN  GPIO_Pin_15  // PB15 -> Trig
#define ULTRASONIC_ECHO_PIN  GPIO_Pin_15  // PC15 -> Echo
#define ULTRASONIC_TRIG_PORT GPIOB
#define ULTRASONIC_ECHO_PORT GPIOC

/* 速度参数定义 */
#define SPEED_RUN          300
#define SPEED_TURN         300
#define SPEED_SPIN         250

/* 超声波跟踪参数 */
#define FOLLOW_TARGET_DIST   300
#define FOLLOW_MIN_DIST      150
#define FOLLOW_SPEED_HIGH    400
#define FOLLOW_SPEED_MID     250
#define FOLLOW_SPEED_LOW     100

/* PWM参数 */
#define PWM_MAX 1000
#define PWM_VAL 300

/* 缓冲区大小定义 */
#define ESP_RX_BUF_LEN   128
#define SERIAL_RX_BUF_LEN 64

/* 工作模式枚举 */
typedef enum
{
    MODE_STOP = 0,
    MODE_MOTOR_TEST,
    MODE_IR_TRACKING,
    MODE_US_FOLLOW,
    MODE_MAX
} Car_WorkMode;

/* 小车状态枚举 */
typedef enum{
    CAR_STATE_STOP = 0,
    CAR_STATE_FORWARD,
    CAR_STATE_BACKWARD,
    CAR_STATE_LEFT,
    CAR_STATE_RIGHT,
    CAR_STATE_TURN_LEFT,
    CAR_STATE_TURN_RIGHT
} CarState_t;

#endif