/*******************************************************************************
* @file     --> FB_Thermal.ino
* @author   --> Lichangchun
* @version  --> v2.1
* @date     --> 2019-07-14
* @update   --> 2019-07-21
* @brief    --> 热控模块
*           1. 读取热控板温度数据并发送给主控板
*           2. 从主控板接收所有板的温度数据和温度控制指令
*           Update: 
*           1. 所有板子的温度都改由热控板测量
*           2. 添加串口调试编译开关
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <string.h>

/* Private macro -------------------------------------------------------------*/
// 定义串口调试宏开关，用于开发阶段的数据查看，正式编译的时候应当注释掉此行
#define SERIAL_DEBUG

/* Private define ------------------------------------------------------------*/
#define I2C_WIRE_BUS 0x12

/* Private variables ---------------------------------------------------------*/
// 引脚定义
const uint8_t ledpin = 13; // LED
const uint8_t EN1 = 32;  //主控加热
const uint8_t EN2 = 33;  //姿控加热
const uint8_t EN3 = 34;  //电源加热
const uint8_t EN4 = 35;  //热控加热
const uint8_t DS18B20_1 = 42; // 主控
const uint8_t DS18B20_2 = 43; // 姿控
const uint8_t DS18B20_3 = 44; // 电源
const uint8_t DS18B20_4 = 45; // 热控

// 实例化温度传感器
OneWire oneWire1(DS18B20_1);
DallasTemperature sensors1(&oneWire1);
OneWire oneWire2(DS18B20_2);
DallasTemperature sensors2(&oneWire2);
OneWire oneWire3(DS18B20_3);
DallasTemperature sensors3(&oneWire3);
OneWire oneWire4(DS18B20_4);
DallasTemperature sensors4(&oneWire4);

// 温控开关
bool thermalOn = false;

// 温度设定值
float dTemperMain = 38.0;    // 主控设定温度
float dTemperAttitude = 38.0;// 姿态设定温度
float dTemperPower = 38.0;   // 电源设定温度
float dTemperThermal = 38.0; // 热控设定温度

// 温度测量值
float cTemperMain = 0;    // 主控测量温度
float cTemperAttitude = 0;// 姿态测量温度
float cTemperPower = 0;   // 电源测量温度
float cTemperThermal = 0; // 热控测量温度

double kp, ki, kd; // PID Parameter 
double kLp;        // LowPass Parameter

uint8_t recvBuffer[64] = {0};   // 数据接收缓冲区

uint8_t ledFlag = LOW; // LED 状态标志
uint8_t ledCnt = 0; // LED 计数
uint64_t previousMillis = 0; // 毫秒时间记录

/* Private functions ---------------------------------------------------------*/

void setup()
{
    pinMode(ledpin, OUTPUT);
    pinMode(EN1, OUTPUT);
    pinMode(EN2, OUTPUT);
    pinMode(EN3, OUTPUT);
    pinMode(EN4, OUTPUT);
#ifdef SERIAL_DEBUG
    Serial.begin(115200);
#endif
    // I2C 总线初始化
    Wire.begin(I2C_WIRE_BUS);
    Wire.onRequest(requestEvent);
    Wire.onReceive(receiveEvent);
    // 温度传感器初始化
    sensors1.begin();
    sensors1.setWaitForConversion(false); // 设置为非阻塞模式
    sensors1.requestTemperatures();
    sensors2.begin();
    sensors2.setWaitForConversion(false); // 设置为非阻塞模式
    sensors2.requestTemperatures();
    sensors3.begin();
    sensors3.setWaitForConversion(false); // 设置为非阻塞模式
    sensors3.requestTemperatures();
    sensors4.begin();
    sensors4.setWaitForConversion(false); // 设置为非阻塞模式
    sensors4.requestTemperatures();

    // PID Parameter
    // kp = 2.8;
    // ki = 0;
    // kd = 8;

    // LowPass Parameter
    // kLp = 0.9;
}

void loop()
{
    uint64_t currentMillis = millis();
    // 每 100ms 获取一次温度，每 500ms 翻转一次 LED
    if (currentMillis - previousMillis >= 100)
    {
        // 更新时间记录
        previousMillis = currentMillis; 
        // 更新温度读数
        cTemperMain = sensors1.getTempCByIndex(0);
        cTemperAttitude = sensors2.getTempCByIndex(0);
        cTemperPower = sensors3.getTempCByIndex(0);
        cTemperThermal = sensors4.getTempCByIndex(0);
        // 闪灯
        if (ledCnt == 5)
        {
            ledCnt = 0;
            ledFlag = !ledFlag;
            digitalWrite(ledpin, ledFlag);
        #ifdef SERIAL_DEBUG
            // 打印以供查看
            Serial.print("当前温度: ");
            Serial.print(cTemperMain);
            Serial.print(", ");
            Serial.print(cTemperAttitude);
            Serial.print(", ");
            Serial.print(cTemperPower);
            Serial.print(", ");
            Serial.println(cTemperThermal);
        #endif
        }
        ledCnt++;

        // 发起新一轮的温度转换
        sensors1.requestTemperatures(); 
        sensors2.requestTemperatures(); 
        sensors3.requestTemperatures(); 
        sensors4.requestTemperatures(); 
    }

    // Note：此处为简单的温度控制算法，用户可自行更换更高级、更合理的算法
    if (thermalOn) // 如果温控打开
    {
        if (cTemperMain < dTemperMain-2) {digitalWrite(EN1, HIGH);}
        else if (cTemperMain > dTemperMain+2) {digitalWrite(EN1, LOW);}

        if (cTemperAttitude < dTemperAttitude-2) {digitalWrite(EN3, HIGH);}
        else if (cTemperAttitude > dTemperAttitude+2) {digitalWrite(EN3, LOW);}

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

    delay(20);
}

// I2C 数据请求回调函数
void requestEvent()
{
    Wire.write((uint8_t *)&cTemperMain, 4);
    Wire.write((uint8_t *)&cTemperAttitude, 4);
    Wire.write((uint8_t *)&cTemperPower, 4);
    Wire.write((uint8_t *)&cTemperThermal, 4);
}

// I2C 数据接收回调函数
/**
 * recvBuffer[0]:
 *      0x00 热控开关
 *      0x01 设置温度
 **/
void receiveEvent(int count)
{
    uint8_t i = 0;
    // 读取 I2C 数据到缓冲区
    while (Wire.available())
    {
        recvBuffer[i++] = (uint8_t)Wire.read();
    }

    if (recvBuffer[0] == 0x00) // 温控开关
    {
        if (recvBuffer[1] == 0x00)   // 温控开关
        {
            thermalOn = false;
        #ifdef SERIAL_DEBUG
            Serial.println("关闭热控");
        #endif
        }
        else // (recvBuffer[1] == 0x01) 打开温控
        {
            thermalOn = true;
        #ifdef SERIAL_DEBUG
            Serial.println("打开热控");
        #endif
        }
    }
    else if (recvBuffer[0] == 0x01) // 设置温度
    {
        // 拷贝数据到设置温度值变量
        memcpy(&dTemperMain, &recvBuffer[1], 4);
        memcpy(&dTemperAttitude, &recvBuffer[5], 4);
        memcpy(&dTemperPower, &recvBuffer[9], 4);
        memcpy(&dTemperThermal, &recvBuffer[13], 4);
    #ifdef SERIAL_DEBUG
        Serial.print("设置温度: ");
        Serial.print(dTemperMain);
        Serial.print(", ");
        Serial.print(dTemperAttitude);
        Serial.print(", ");
        Serial.print(dTemperPower);
        Serial.print(", ");
        Serial.println(dTemperThermal);
    #endif
    }
}

//PID_Controller
double Compute(double Ip)
{
   /*How long since we last calculated*/
   double now = double(millis())/1000;
   //Serial.println(now);
   double timeChange = (double)(now - lastTime);
  
   /*Compute all the working error variables*/
   double error = Setpoint - Ip;
   errSum += (error * timeChange);
   double dErr = (error - lastErr) / timeChange;
  
   /*Compute PID Output*/
   double Op = kp * error + ki * errSum + kd * dErr;
  
   /*Remember some variables for next time*/
   lastErr = error;
   lastTime = now;
   
   return Op;
}

//LowPass_Filter
double Lp_Filter(double old_value, double value)
{ 
   double out_value = kLp*old_value + (1-kLp)*value;
   return out_value;
  }