/*******************************************************************************
* @file     --> FB_Attitude.ino
* @author   --> Lichangchun
* @version  --> v2.1
* @date     --> 2019-07-18
* @update   --> 2019-07-22
* @brief    --> 姿控模块
*           1. 读取 IMU 数据
*           2. 控制电机正/反转，利用 PWM 控制电机转速
*           3. 控制两个磁力矩器（todo.）
*           4. 测量电机转速、磁力矩等
*******************************************************************************/
  
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <Wire.h>
#include "IMUGY85.h"
#include <string.h>     // 使用内存拷贝函数
#include <MsTimer2.h>   // 使用定时器

/* Private macro -------------------------------------------------------------*/
// 串口调试宏开关，正式编译时注释掉
// #define DEBUG

/* Private define ------------------------------------------------------------*/
#define I2C_WIRE_BUS 0x20

/* Private variables ---------------------------------------------------------*/
const int ledpin = 13;  // LED 引脚
const int WHEEL_IN1 = 22;     // 电机控制引脚 1
const int WHEEL_IN2 = 33;     // 电机控制引脚 2
const int WHEEL_ENA = 5;      // 电机使能引脚

// IMU 数据记录
float eulers[3] = {0};  // [yaw, pitch, roll]
float angularVelocities[3] = {0};
float accelerations[3] = {0};

// 飞轮控制开关
volatile bool wheelOn = false;
// 飞轮转速设置值
volatile uint8_t dWheelSpeed = 0; // 无符号(0 ~ 255)
volatile bool wheelIsForward = true; // 正转
// 飞轮转速测量值
volatile int16_t cWheelSpeed = 0; // 16 位整型，以使测量值范围精确一些

// 磁力矩器控制开关
volatile bool magnetbarsOn = false;
// 磁力矩大小设置
volatile int8_t dMagnetbarX = 0; // 磁矩 X 设置值
volatile int8_t dMagnetbarY = 0; // 磁矩 Y 设置值
volatile int8_t dMagnetbarZ = 0; // 磁矩 Z 设置值
// 磁力矩测量值
volatile int16_t cMagnetbarX = 0; // 磁矩 X 电流
volatile int16_t cMagnetbarY = 0; // 磁矩 Y 电流
volatile int16_t cMagnetbarZ = 0; // 磁矩 Z 电流

uint8_t ledFlag = LOW; // LED 状态标志
uint64_t previousMillis = 0; // 毫秒时间记录
uint16_t wheelCnt = 0;  // 飞轮编码器计数

IMUGY85 imu;    // IMU(九轴)传感器

uint8_t recvBuffer[20] = {0};   // 数据接收缓冲区

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* @brief    --> I2C 数据请求回调函数
*   欧拉角(12) + 三轴角速度(12) + 三轴加速度(12) + 飞轮转速(2) + 飞轮状态(1) + \
*   三轴磁棒电流(6) + 磁棒状态(1)
*******************************************************************************/
void requestEvent()
{
    // 欧拉角
    Wire.write((uint8_t *)eulers, 12);
    // 三轴角速度
    Wire.write((uint8_t *)angularVelocities, 12);
    // 三轴加速度
    Wire.write((uint8_t *)accelerations, 12);
    // 飞轮转速
    Wire.write((uint8_t *)&cWheelSpeed, 2);
    // 飞轮状态
    Wire.write((uint8_t)wheelOn);
    // 三轴磁棒电流
    Wire.write((uint8_t *)&cMagnetbarX, 2);
    Wire.write((uint8_t *)&cMagnetbarY, 2);
    Wire.write((uint8_t *)&cMagnetbarZ, 2);
    // 磁棒状态
    Wire.write((uint8_t)magnetbarsOn);
}

/*******************************************************************************
* @brief    --> I2C 数据接收回调函数
*******************************************************************************/
void receiveEvent(int count)
{
    uint8_t i = 0;
    // 读取 I2C 数据到缓冲区
    while (Wire.available())
    {
        recvBuffer[i++]  = Wire.read();
    }

    if (recvBuffer[0] =- 0x00)      // 磁矩设置
    {
        dMagnetbarX = recvBuffer[1];
        dMagnetbarY = recvBuffer[2];
    }

    else if (recvBuffer[0] == 0x02) // 磁控开关
    {
        if (recvBuffer[1] == 0x00)
            magnetbarsOn = false; // 关闭
        else
            magnetbarsOn = true;
    }

    else if (recvBuffer[0] == 0x04) // 飞轮转速设置
    {
        if (recvBuffer[1] >= 0)
        {
            wheelIsForward = true;
            dWheelSpeed = recvBuffer[1] / 100 * 255;
            digitalWrite(WHEEL_IN1, HIGH);
            digitalWrite(WHEEL_IN2, LOW);
        }
        else
        { 
            wheelIsForward = false;
            dWheelSpeed = -recvBuffer[1] / 100 * 255;
            digitalWrite(WHEEL_IN1, LOW);
            digitalWrite(WHEEL_IN2, HIGH);
        }
    }

    else if (recvBuffer[0] == 0x06) // 飞轮控制开关
    {
        if (recvBuffer[1] = 0x00)
            wheelOn = false;
        else
            wheelOn = true;
    }
}

/*******************************************************************************
* @brief    --> 定时器中断回调函数
*           每 500ms 计算一次速度
*******************************************************************************/
void measurement()
{
    cWheelSpeed = wheelCnt * 12;
    wheelCnt = 0;
    if (!wheelIsForward)
        cWheelSpeed = -cWheelSpeed;
#ifdef SERIAL_DEBUG
    Serial.print("当前转速: ");
    Serial.print(cWheelSpeed);
    Serial.println("r/min");
#endif
}

/*******************************************************************************
* @brief    --> 外部中断回调函数
*           每个上升沿脉冲增加计数
*******************************************************************************/
void counter()
{
    wheelCnt++;
}

void setup() 
{
    pinMode(ledpin, OUTPUT);
    pinMode(WHEEL_IN1, OUTPUT);
    pinMode(WHEEL_IN2, OUTPUT);
    pinMode(WHEEL_ENA, OUTPUT);
    // 绑定到外部中断 0(数字引脚 2)，上升沿触发
    attachInterrupt(0, counter, RISING); 
    MsTimer2::set(500, measurement); // 500ms period
    imu.init();
#ifdef DEBUG
    Serial.begin(115200);
#endif
    Wire.begin(I2C_WIRE_BUS);
    Wire.onReceive(receiveEvent); 
    Wire.onRequest(requestEvent);
}

void loop()
{
    uint64_t currentMillis = millis();
    // 每 500ms 翻转 LED
    if (currentMillis - previousMillis >= 500)
    {
        previousMillis = currentMillis; // 更新时间记录
        ledFlag = !ledFlag;
        digitalWrite(ledpin, ledFlag);
    }
    // 电机调速
    if (wheelOn)
        analogWrite(WHEEL_ENA, dWheelSpeed);
    else 
        analogWrite(WHEEL_ENA, 0);
    // 更新 IMU 数据
    imu.update();
    double temp1, temp2, temp3;
    imu.getEuler(&temp1, &temp2, &temp3);
    eulers[0] = temp1;
    eulers[1] = temp2;
    eulers[2] = temp3;
    imu.getGyro(&temp1, &temp2, &temp3);
    angularVelocities[0] = temp1;
    angularVelocities[1] = temp2;
    angularVelocities[2] = temp3;
    imu.getAcceleration(&temp1, &temp2, &temp3);
    accelerations[0] = temp1;
    accelerations[1] = temp2;
    accelerations[2] = temp3;
    delay(20);
}
