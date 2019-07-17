/*******************************************************************************
* @file     --> FB_Thermak.ino
* @author   --> Lichangchun
* @version  --> v2.1
* @date     --> 2019-07-14
* @update   --> 2019-07-14
* @brief    --> 热控模块
*           1. 读取热控板温度数据并发送给主控板
*           2. 从主控板接收所有板的温度数据和温度控制指令
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

/* Private define ------------------------------------------------------------*/
#define ONE_WIRE_BUS 42

/* Private variables ---------------------------------------------------------*/
// 引脚定义
const int ledpin = 22; // LED
const int EN1 = 10;  //主控加热
const int EN2 = 11;  //电源加热
const int EN3 = 12;  //姿控加热
const int EN4 = 13;  //热控加热

// 实例化温度传感器
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// 温控开关
volatile bool thermalOn = true;

// 温度设定值
volatile float dTemperMain = 38.0;    // 主控设定温度
volatile float dTemperAttitude = 38.0;// 姿态设定温度
volatile float dTemperPower = 38.0;   // 电源设定温度
volatile float dTemperThermal = 38.0; // 热控设定温度

// 温度测量值
volatile float cTemperMain = 0;    // 主控测量温度
volatile float cTemperAttitude = 0;// 姿态测量温度
volatile float cTemperPower = 0;   // 电源测量温度
volatile float cTemperThermal = 0; // 热控测量温度

volatile float tem = 0; // 热控板测量温度临时存储

uint8_t recvBuffer[20] = {0};   // 数据接收缓冲区

uint8_t ledFlag = 0; // LED 状态标志

/* Private functions ---------------------------------------------------------*/

void setup()
{
    pinMode(ledpin, OUTPUT);
    pinMode(EN1, OUTPUT);
    pinMode(EN2, OUTPUT);
    pinMode(EN3, OUTPUT);
    pinMode(EN4, OUTPUT);

    Wire.begin(0x12);
    Wire.onRequest(requestEvent);
    Wire.onReceive(receiveEvent);
    sensors.begin();
}

void loop()
{
    if (ledFlag == 0)
    {
        ledpin = 1;
        digitalWrite(ledpin, LOW);
    }
    else
    {
        ledpin = 0;
        digitalWrite(ledpin, HIGH);
    }

    sensors.requestTemperatures();
    tem = sensors.getTempCByIndex(0);

    // Note：此处为简单的温度控制算法，用户可自行更换更高级、更合理的算法
    if (thermalOn) // 如果温控打开
    {
        if (cTemperMain < dTemperMain-2) {digitalWrite(EN1, HIGH);}
        else if (cTemperMain > dTemperMain+2) {digitalWrite(EN1, LOW);}

        if (cTemperAttitude < dTemperAttitude-2) {digitalWrite(EN3, HIGH);}
        else if (cTemperAttitude > dTemperMainAttitude+2) {digitalWrite(EN3, LOW);}

        if (cTemperPower < dTemperPower-2) {digitalWrite(EN2, HIGH);}
        else if (cTemperPower > dTemperPower+2) {digitalWrite(EN2, LOW);}

        if (cTemperThermal < dTemperThermal-2) {digitalWrite(EN4, HIGH);}
        else if (cTemperThermal > dTemperThermal+2) {digitalWrite(EN4, LOW);}
    }
    else        // 如果温控关闭
    {
        digitalWrite(EN1, LOW);
        digitalWrite(EN2, LOW);
        digitalWrite(EN3, LOW);
        digitalWrite(EN4, LOW);
    }

    delay(50);
}

// I2C 数据请求回调函数
void requestEvent(int count)
{
    int16_t temp = tem * 10; // ×10 保留 1 位小数
    Wire.write(temp >> 8);
    Wire.write(temp & 0xFF);
}

// I2C 数据接收回调函数
/**
 * recvBuffer[0]:
 *      0x00 关闭热控
 *      0x01 打开热控
 *      0x02 测量温度
 *      0x03 设置温度
 **/
void receiveEvent(int count)
{
    uint8_t i = 0;
    // 读取 I2C 数据到缓冲区
    while (Wire.available())
    {
        recvBuffer[i++] = (uint8_t)Wire.read();
    }

    if (recvBuffer[0] == 0x00)      // 关闭温控
    {
        thermalOn = false;
    }
    else if (recvBuffer[0] == 0x01) // 打开温控
    {
        thermalOn = true;
    }
    else if (recvBuffer[0] == 0x02) // 测量温度
    {
        // 拷贝数据到当前温度值变量
        cTemperMain = ((int16_t)((recvBuffer[1] << 8) + recvBuffer[2])) * 0.1;
        cTemperAttitude = ((int16_t)((recvBuffer[3] << 8) + recvBuffer[4])) * 0.1;
        cTemperPower = ((int16_t)((recvBuffer[5] << 8) + recvBuffer[6])) * 0.1;
        cTemperThermal = ((int16_t)((recvBuffer[7] << 8) + recvBuffer[8])) * 0.1;
    }
    else if (recvBuffer[0] == 0x03) // 设置温度
    {
        // 拷贝数据到设置温度值变量
        dTemperMain = ((int16_t)((recvBuffer[1] << 8) + recvBuffer[2])) * 0.1;
        dTemperAttitude = ((int16_t)((recvBuffer[3] << 8) + recvBuffer[4])) * 0.1;
        dTemperPower = ((int16_t)((recvBuffer[5] << 8) + recvBuffer[6])) * 0.1;
        dTemperThermal = ((int16_t)((recvBuffer[7] << 8) + recvBuffer[8])) * 0.1;
    }
}
