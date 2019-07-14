/*******************************************************************************
* @file     --> FB_Thermak.ino
* @author   --> 
* @version  --> v2.1
* @date     --> 2019-07-14
* @update   --> 2019-07-14
* @brief    --> 热控模块
*           
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

// 温度数据
float dTemperMain = 40.0;    // 主控设定温度
float dTemperAttitude = 40.0;// 姿态设定温度
float dTemperPower = 40.0;   // 电源设定温度
float dTemperThermal = 40.0; // 热控设定温度

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
    Wire.onReceive(receiveEvent); // register event
    sensors.begin();
}

void loop()
{

}