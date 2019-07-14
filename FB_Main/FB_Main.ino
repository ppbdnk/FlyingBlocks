/*******************************************************************************
* @file     --> FB_Main.ino
* @author   --> Lichangchun
* @version  --> v2.1
* @date     --> 2019-07-11
* @update   --> 2019-07-11
* @brief    --> 主控模块
*           1. 通过串口接收上位机发送的指令，并通过 I2C 总线转发给各个子系统模块
*           2. 从各个子系统模块收集数据，分别通过串口数据报发送给上位机
*           3. 此外，通过 433M 无线串口和其它桌面卫星模拟系统进行广播通信，并通过各个
*              主控板上相应的一组 LED 展示通信结果。（Todo）
*******************************************************************************/
  
/* Includes ------------------------------------------------------------------*/
#include <Wire.h>
#include <OneWire.h>    // 用于 DS18B20 的驱动
#include <DallasTemperature.h>
#include <SCoop.h>      // 使用 SCoop 线程库

/* Private define ------------------------------------------------------------*/
#define ONE_WIRE_BUS 42 // 定义 DS18B20 所在引脚

// 定义子模块的 IIC 通信地址
#define I2C_ADDR_ATTITUDE   0x20 // 姿控
#define I2C_ADDR_THERMAL    0X12 // 热控
#define I2C_ADDR_POWER           // 电源

/* Private variables ---------------------------------------------------------*/
uint8_t sendBuffer[14];
uint8_t recvBuffer[8];
uint32_t t = 0;

// 状态控制标志量
volatile bool powerOn = false;
volatile bool thermalOn = false;
volatile bool wheelOn = false;
volatile bool magnetbarOn = false;

int8_t wheelSpeed = 0;      // 飞轮转速记录
float temperMain = 40.0;    // 主控设定温度
float temperAttitude = 40.0;// 姿态设定温度
float temperPower = 40.0;   // 电源设定温度
float temperThermal = 40.0; // 热控设定温度

OneWire oneWire(ONE_WIRE_BUS);
// 实例化温度传感器
DallasTemperature sensors(&oneWire);

/*******************************************************************************
 *                              上位机数据接收线程                               *
*******************************************************************************/
defineTask(RecvThread); // 定义一个接收线程

void RecvThread::setup()
{
}

void RecvThread::loop()
{
    t = 0;
    if (Serial1.available() > 0)
        sleep(20);  // 等待串口数据完全进入缓冲区
    while (Serial1.available() > 0)
    {
        digitalWrite(ledpin, HIGH);
        recvBuffer[t] = (uint8_t)Serial1.read();
        t++;
    }
    digitalWrite(ledpin, LOW);
    // 接收到有效数据帧
    if (t>0 && recvBuffer[0]==0xAA && recvBuffer[1]==0x55 &&\
        recvBuffer[4]==0x01 && recvBuffer[5]==0x01)
    {
        ////// 电源控制 ////// 
        if (recvBuffer[2] == 0x00)
        {
            if (recvBuffer[3] == 0x00)
            {
                powerOn = false; // 关闭
                // todo.
            }
            else
            {
                powerOn = true;
                // todo.
            }
        }

        ////// 飞轮控制 ////// 
        if (recvBuffer[2] == 0x01)
        {
            if (recvBuffer[3] == 0x00)
            {
                wheelOn = false;
                // 与姿控板通信，关闭飞轮
                Wire.beginTransmisson(I2C_ADDR_ATTITUDE);
                Wire.write(0x06);
                Wire.write(0x00);
                Wire.endTransmission();
                // sleep(20);
            }
            else
            {
                wheelOn = true;
                // 与姿控板通信，打开飞轮
                Wire.beginTransmisson(I2C_ADDR_ATTITUDE);
                Wire.write(0x06);
                Wire.write(0x01);
                Wire.endTransmission();
                // sleep(20);
            }
        }

        ////// 温度控制 ////// 
        if (recvBuffer[2] == 0x02)
        {
            if (recvBuffer[3] == 0x00)
            {
                thermalOn = false;
                //与温控板通信，发送温度数据
                // Wire.beginTransmission(0x12);
                // Wire.write(recvBuffer[3]);
                // Wire.write(te0);
                // Wire.write(te1);
                // Wire.write(te2);
                // Wire.write(te3);
                // Wire.write(te4);
                // Wire.write(te5);
                // Wire.write(te6);
                // Wire.write(te7);
                // Wire.endTransmission();
                // sleep(20);
            }
            else
            {
                thermalOn = true;
            } 
        }

        ////// 磁力矩器控制 ////// 
        if (recvBuffer[2] == 0x03)
        {
            if (recvBuffer[3] == 0x00)
            {
                magnetbarOn = false;
                // todo.
            }
            else
            {
                magnetbarOn = true;
                // todo.
            }
        } 

        ////// 飞轮转速设置 ////// 
        if (recvBuffer[2] == 0x04)
        {
            wheelSpeed = (int8_t)(recvBuffer[3]);
            //与姿控板通信，发送飞轮转速
            Wire.beginTransmission(I2C_ADDR_ATTITUDE);
            Wire.write(0x04);
            Wire.write(wheelSpeed);
            Wire.endTransmission();
            // sleep(20);
        }

        ////// 关闭所有模块 ////// 
        if (recvBuffer[2] == 0x06)
        {
            // 关闭电源板
            // todo.

            // 关闭温控
            // todo.

            // 关闭飞轮
            Wire.beginTransmission(I2C_ADDR_ATTITUDE);
            Wire.beginTransmission(0x20);
            Wire.write(0x06);
            Wire.write(0x00);
            Wire.endTransmission();
            // sleep(20);

            // 关闭磁力矩器
            // todo.
        }
    }
}

/*******************************************************************************
 *                                   主线程                                     *
*******************************************************************************/
void setup()
{
    pinMode(ledpin, OUTPUT);
    Wire.begin();
    Serial1.begin(38400);   // 使用串口 1 与上位机通信
    mySCoop.start();        // 开启线程调度
}

void loop()
{
    float tem;
    sensors.requestTemperatures();
    tem = sensors.getTempCByIndex(0);
    //
    yield();    // 让给另一个线程
}