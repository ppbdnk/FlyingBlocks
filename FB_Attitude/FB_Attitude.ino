/*******************************************************************************
* @file     --> FB_Attitude.ino
* @author   --> Lichangchun
* @version  --> v2.1
* @date     --> 2019-07-18
* @update   --> 2019-07-20
* @brief    --> 姿控模块
*           1. 读取 IMU 数据
*           2. 控制电机正/反转，利用 PWM 控制电机转速
*           3. 控制两个磁力矩器（todo.）
*           4. 测量电机转速、磁力矩等
*******************************************************************************/
  
/* Includes ------------------------------------------------------------------*/
#include <Wire.h>
#include <ADXL345.h>    // 三轴加速度计
#include <HMC5883L.h>   // 三轴电子罗盘
#include <ITG3200.h>    // 三轴陀螺仪
#include <OneWire.h>
#include <DallasTemperature.h>
#include <string.h>     // 使用内存拷贝函数

/* Private macro -------------------------------------------------------------*/
// #define DEBUG

/* Private define ------------------------------------------------------------*/
#define ONE_WIRE_BUS 42
#define I2C_WIRE_BUS 0x20

/* Private variables ---------------------------------------------------------*/
const int ledpin = 49;  // LED 引脚
const int IN1 = 22;     // 电机输入引脚 1
const int IN2 = 33;     // 电机输入引脚 2
const int ENA = 5;      // 电机使能引脚

// IMU 数据记录
double roll, pitch, yaw; 

float magCount[3] = {0};    // 电子罗盘
float quaternion[4] = {0};  // 四元数
float angularVelocities[3] = {0};
float accelerations[3] = {0};

// 飞轮控制开关
volatile bool wheelOn = false;
// 飞轮转速设置值
volatile int8_t dWheelSpeed = 0; // 有符号(-100 ~ 100)，可设置正负 100 档
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
volatile int16_t cMagnetbarZ = 0; // 磁矩Z 电流

// 实例化温度传感器
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

volatile float temperature = 0;  // 温度数据记录

uint8_t ledFlag = 0; // LED 状态标志
uint64_t previousMillis = 0; // 毫秒时间记录

uint8_t recvBuffer[20] = {0};   // 数据接收缓冲区

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* @brief    --> I2C 数据请求回调函数
* 温度(2) + 三轴角度(6) + 三轴角速度(6) + 三轴加速度(6) + 飞轮转速(2)
*******************************************************************************/
void requestEvent()
{
    // 温度
    Wire.write((uint8_t *)&temperature, 4);
    // 四元数
    Wire.write((uint8_t *)quaternion, 16);
    // 三轴角速度
    Wire.write((uint8_t *)angularVelocities, 12);
    // 三轴加速度
    Wire.write((uint8_t *)accelerations, 12);
    // 飞轮转速
    Wire.write((uint8_t *)&cWheelSpeed, 2);
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
        dWheelSpeed = recvBuffer[1];
    }

    else if (recvBuffer[0] == 0x06) // 飞轮控制开关
    {
        if (recvBuffer[1] = 0x00)
            wheelOn = false;
        else
            wheelOn = true;
    }
}

void setup() 
{
    pinMode(ledpin, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);
#ifdef DEBUG
    Serial.begin(115200);
#endif
    Wire.begin(I2C_WIRE_BUS);
    Wire.onReceive(receiveEvent); 
    Wire.onRequest(requestEvent);

    sensors.begin();
    sensors.setWaitForConversion(false); // 设置为非阻塞模式
    sensors.requestTemperatures();
}

void loop()
{
    uint64_t currentMillis = millis();
    // 每 500ms 翻转 LED，并获取一次温度
    if (currentMillis - previousMillis >= 500)
    {
        previousMillis = currentMillis; // 更新时间记录
        temperature = sensors.getTempCByIndex(0);
    #ifdef DEBUG
        Serial.print("当前温度：");
        Serial.print(temperature);
        Serial.println("℃");
    #endif
        if (ledFlag == 0)
        {
            ledFlag = 1;
            digitalWrite(ledpin, LOW);
        }
        else 
        {
            ledFlag = 0;
            digitalWrite(ledpin, HIGH);
        }
        sensors.requestTemperatures(); // 发起新的温度转换
    }
    delay(50);
}
