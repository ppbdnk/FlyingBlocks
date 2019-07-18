/*******************************************************************************
* @file     --> FB_Main.ino
* @author   --> Lichangchun
* @version  --> v2.1
* @date     --> 2019-07-11
* @update   --> 2019-07-17
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
#define I2C_ADDR_POWER      0x40 // 电源

/* Private variables ---------------------------------------------------------*/
// 引脚定义
const int ledpin = 29; // LED

// 串口数据缓冲区
uint8_t sendBuffer[30];
uint8_t recvBuffer[8];

// I2C 数据缓冲区
uint8_t iicBuffer[30];

// 状态控制标志量
volatile bool powerControlOn = false;
volatile bool thermalControlOn = false;
volatile bool wheelControlOn = false;
volatile bool magnetbarControlOn = false;

// 飞轮转速记录
volatile int16_t cWheelSpeed = 0; // 转速测量值，使用 2 个字节
volatile int8_t dWheelSpeed = 0; // 转速设置值，正负分别 100 档

// 磁矩值记录
// 测量值
volatile int8_t cMagneticMomentX = 0; // 符号表正负
volatile int8_t cMagneticMomentY = 0;
//设置值，todo.
volatile int8_t dMagneticMomentX = 0;
volatile int8_t dMagneticMomentY = 0;

// 电池电量记录
volatile int8_t powerLevel = 9;     

// 温度设定值，×10 以保留 1 位小数
volatile int16_t dTemperMain = 380;    // 主控设定温度
volatile int16_t dTemperAttitude = 380;// 姿态设定温度
volatile int16_t dTemperPower = 380;   // 电源设定温度
volatile int16_t dTemperThermal = 380; // 热控设定温度

// 温度测量值，×10 以保留 1 位小数
volatile int16_t cTemperMain = 0;    // 主控测量温度
volatile int16_t cTemperAttitude = 0;// 姿态测量温度
volatile int16_t cTemperPower = 0;   // 电源测量温度
volatile int16_t cTemperThermal = 0; // 热控测量温度

// 九轴数据
uint8_t angles[6] = {0};
uint8_t angularVelocities[6] = {0};
uint8_t accelerations[6] = {0};

// 实例化温度传感器
OneWire oneWire(ONE_WIRE_BUS);
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
    uint16_t t = 0;
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
        ////// 电源控制开关 ////// 
        if (recvBuffer[2] == 0x00)
        {
            if (recvBuffer[3] == 0x00)
            {
                powerControlOn = false; // 关闭
                // todo.
            }
            else
            {
                powerControlOn = true;
                // todo.
            }
        }

        ////// 飞轮控制开关 ////// 
        if (recvBuffer[2] == 0x01)
        {
            if (recvBuffer[3] == 0x00)
            {
                wheelControlOn = false;
                // 与姿控板通信，关闭飞轮
                Wire.beginTransmission(I2C_ADDR_ATTITUDE);
                Wire.write(0x06);
                Wire.write(0x00);
                Wire.endTransmission();
                // sleep(20);
            }
            else
            {
                wheelControlOn = true;
                // 与姿控板通信，打开飞轮
                Wire.beginTransmission(I2C_ADDR_ATTITUDE);
                Wire.write(0x06);
                Wire.write(0x01);
                Wire.endTransmission();
                // sleep(20);
            }
        }

        ////// 温度控制开关 ////// 
        if (recvBuffer[2] == 0x02)
        {
            if (recvBuffer[3] == 0x00) // 热控关闭
            {
                thermalControlOn = false;
                uint8_t *pt;
                // 与热控板通信，关闭热控
                Wire.beginTransmission(I2C_ADDR_THERMAL);
                Wire.write(0x00);
                Wire.write(0x00);
                Wire.endTransmission();
                // sleep(20);
            }
            else // recvBuffer[3] == 0x01 热控打开
            {
                thermalControlOn = true;
                uint8_t *pt;
                // 与热控板通信，关闭热控
                Wire.beginTransmission(I2C_ADDR_THERMAL);
                Wire.write(0x00);
                Wire.write(0x01);
                Wire.endTransmission();
                // sleep(20);
            }
        }

        ////// 磁力矩器控制开关 ////// 
        if (recvBuffer[2] == 0x03)
        {
            if (recvBuffer[3] == 0x00)
            {
                magnetbarControlOn = false;
                // todo.
            }
            else
            {
                magnetbarControlOn = true;
                // todo.
            }
        } 

        ////// 飞轮转速设置 ////// 
        if (recvBuffer[2] == 0x04)
        {
            dWheelSpeed = (int8_t)(recvBuffer[3]);
            //与姿控板通信，发送飞轮转速
            Wire.beginTransmission(I2C_ADDR_ATTITUDE);
            Wire.write(0x04);
            Wire.write(dWheelSpeed);
            Wire.endTransmission();
            // sleep(20);
        }

        ////// 模块温度设置 //////
        if (recvBuffer[2] == 0x05)
        {
            // todo.
        }

        ////// 关闭所有模块控制 ////// 
        if (recvBuffer[2] == 0x06)
        {
            // 关闭电源板
            // todo.

            // 关闭热控
            thermalControlOn = false;
            uint8_t *pt;
            Wire.beginTransmission(I2C_ADDR_THERMAL);
            Wire.write(0x00);
            Wire.write(0x00);
            Wire.endTransmission();

            // 关闭飞轮
            Wire.beginTransmission(I2C_ADDR_ATTITUDE);
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
    uint8_t *pt;
    uint8_t i = 0;

    ////// 获取各个板子的数据 //////

    // 主控板
    sensors.requestTemperatures();
    cTemperMain = sensors.getTempCByIndex(0) * 10; // ×10 为保留 1 位小数

    // 姿控板：温度(2) + 三轴角度(6) + 三轴角速度(6) + 三轴加速度(6) + 飞轮转速(2)
    i = 0;
    // 将 I2C 数据读到接收缓冲区，共 22 字节
    Wire.requestFrom(I2C_ADDR_ATTITUDE, 22);
    while (Wire.available())
    {
        iicBuffer[i++] = (uint8_t)Wire.read();
    }
    // 更新姿态板温度数据
    cTemperAttitude = (int16_t)((recvBuffer[0] << 8) + recvBuffer[1]);
    // 更新三轴角度数据
    for (i = 0; i < 6; i++) 
    {
        angles[i] = iicBuffer[i+2];
    }
    // 更新三轴角速度数据
    for (i = 0; i < 6; i++) 
    {
        angularVelocities[i] = iicBuffer[i+8];
    }
    // 更新三轴加速度数据
    for (i = 0; i < 6; i++) 
    {
        accelerations[i] = iicBuffer[i+14];
    }
    // 更新飞轮转速测量值
    pt = (uint8_t *)&cWheelSpeed;
    pt[0] = iicBuffer[20];
    pt[1] = iicBuffer[21];

    // 电源板：温度(2) + 电量(1)
    i = 0;
    // 将 I2C 数据读到接收缓冲区，共 3 字节
    Wire.requestFrom(I2C_ADDR_POWER, 3);
    while (Wire.available())
    {
        pt[i++] = (uint8_t)Wire.read();
    }
    // 更新电源板温度数据
    cTemperPower = (int16_t)((recvBuffer[0] << 8) + recvBuffer[1]); 
    // 更新电池电量数据
    powerLevel = recvBuffer[2];

    // 热控板：温度(2)
    i = 0;
    // 将 I2C 数据读到接收缓冲区，共 2 字节
    Wire.requestFrom(I2C_ADDR_THERMAL, 2);
    while (Wire.available())
    {
        pt[i++] = (uint8_t)Wire.read();
    }
    cTemperThermal = (int16_t)((recvBuffer[0] << 8) + recvBuffer[1]);

    ////// 发送电量数据到上位机 //////

    sendBuffer[0] = 0xAA;
    sendBuffer[1] = 0x55;
    sendBuffer[2] = 0x00; // 电量数据
    sendBuffer[3] = powerLevel;
    if (powerControlOn)
        sendBuffer[4] = 0x01;
    else
        sendBuffer[4] = 0x00;
    sendBuffer[5] = 0x0A;
    sendBuffer[6] = 0x0D;
    for (i = 0; i < 7; i++)
    {
        Serial1.write(sendBuffer[i]);
    }
    sleep(20);

    ////// 发送温度数据到上位机 //////

    sendBuffer[0] = 0xAA;
    sendBuffer[1] = 0x55;
    sendBuffer[2] = 0x02; // 温度数据
    pt = (uint8_t *)&cTemperMain;
    sendBuffer[3] = *pt++;
    sendBuffer[4] = *pt++;
    pt = (uint8_t *)&cTemperAttitude;
    sendBuffer[5] = *pt++;
    sendBuffer[6] = *pt++;
    pt = (uint8_t *)&cTemperPower;
    sendBuffer[7] = *pt++;
    sendBuffer[8] = *pt++;
    pt = (uint8_t *)&cTemperThermal;
    sendBuffer[9] = *pt++;
    sendBuffer[10] = *pt++;
    if (thermalControlOn)
        sendBuffer[11] = 0x01;
    else
        sendBuffer[11] = 0x00;
    sendBuffer[12] = 0x0A;
    sendBuffer[13] = 0x0D;
    for (i = 0; i < 14; i++)
    {
        Serial1.write(sendBuffer[i]);
    }
    // 如果热控打开则发送温度数据给热控板
    if (thermalControlOn)
    {
        Wire.write(I2C_ADDR_THERMAL);
        Wire.write(0x01);
        Wire.write(cTemperMain >> 8);
        Wire.write(cTemperMain & 0xFF);
        Wire.write(cTemperAttitude >> 8);
        Wire.write(cTemperAttitude & 0xFF);
        Wire.write(cTemperPower >> 8);
        Wire.write(cTemperPower & 0xFF);
        Wire.write(cTemperThermal >> 8);
        Wire.write(cTemperThermal & 0xFF);
        Wire.endTransmission();
    }
    sleep(20);

    ////// 发送姿态（九轴）数据到上位机 //////

    sendBuffer[0] = 0xAA;
    sendBuffer[1] = 0x55;
    sendBuffer[2] = 0x04; // 九轴数据
    for (i = 0; i < 6; i++)
        sendBuffer[i+3] = angles[i];
    for (i = 0; i < 6; i++)
        sendBuffer[i+9] = angularVelocities[i];
    for (i = 0; i < 6; i++)
        sendBuffer[i+15] = accelerations[i];
    sendBuffer[21] = 0x0A;
    sendBuffer[22] = 0x0D;
    for (i = 0; i < 23; i++)
    {
        Serial1.write(sendBuffer[i]);
    }
    sleep(20);

    ////// 发送飞轮数据到上位机 //////

    sendBuffer[0] = 0xAA;
    sendBuffer[1] = 0x55;
    sendBuffer[2] = 0x06; // 飞轮数据
    sendBuffer[3] = cWheelSpeed >> 8;
    sendBuffer[4] = cWheelSpeed & 0xFF;
    if (wheelControlOn)
        sendBuffer[5] = 0x01;
    else
        sendBuffer[5] = 0x00;
    sendBuffer[6] = 0x0A;
    sendBuffer[7] = 0x0D;
    for (i = 0; i < 8; i++)
    {
        Serial1.write(sendBuffer[i]);
    }
    sleep(20);

    ////// 发送磁控数据到上位机 //////

    sendBuffer[0] = 0xAA;
    sendBuffer[1] = 0x55;
    sendBuffer[2] = 0x08; // 磁控数据
    sendBuffer[3] = cMagneticMomentX;
    sendBuffer[4] = cMagneticMomentY;
    if (magnetbarControlOn)
        sendBuffer[5] = 0x01;
    else
        sendBuffer[5] = 0x00;
    sendBuffer[6] = 0x0A;
    sendBuffer[7] = 0x0D;
    for (i = 0; i < 8; i++)
    {
        Serial1.write(sendBuffer[i]);
    }
    sleep(20);

    yield();    // 让给另一个线程
}
