/*******************************************************************************
* @file     --> FB_Main.ino
* @author   --> Lichangchun
* @version  --> v2.1
* @date     --> 2019-07-11
* @update   --> 2019-07-22
* @brief    --> 主控模块
*           1. 通过串口接收上位机发送的指令，并通过 I2C 总线转发给各个子系统模块
*           2. 从各个子系统模块收集数据，分别通过串口数据报发送给上位机
*           3. 此外，通过 433M 无线串口和其它桌面卫星模拟系统进行广播通信，并通过各个
*              主控板上相应的一组 LED 展示通信结果。（Todo）
*           update:
*           1. 主控板不设置控制状态的记录，直接从各个板子获取
*           2. 所有温度数据从热控板获取
*           3. 四元数改为欧拉角（yaw-pitch-roll）
*******************************************************************************/
  
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <Wire.h>
#include <SCoop.h>      // 使用 SCoop 线程库
#include <string.h>     // 使用内存拷贝

/* Private macro -------------------------------------------------------------*/
// 宏开关，可启用 Serial 用于串口调试（与上位机通信使用的是 Serial1）
// 正式编译的时候请注释掉
//#define SERIAL_DEBUG

/* Private define ------------------------------------------------------------*/
// 定义子模块的 IIC 通信地址
#define I2C_ADDR_ATTITUDE   0x20 // 姿控
#define I2C_ADDR_THERMAL    0X12 // 热控
#define I2C_ADDR_POWER      0x40 // 电源

/* Private variables ---------------------------------------------------------*/
// 引脚定义
const int ledpin = 13; // LED

// 串口数据缓冲区
// 单次收发数据帧限制在 64 字节以内
uint8_t sendBuffer[64];
uint8_t recvBuffer[64];

// I2C 数据缓冲区
uint8_t iicBuffer[64];

// 状态控制标志量
volatile bool powerControlOn = false;
volatile bool thermalControlOn = false;
volatile bool wheelControlOn = false;
volatile bool magnetbarsControlOn = false;

// 飞轮转速记录
volatile int16_t cWheelSpeed = 0; // 转速测量值，-32768 ~ 32767
volatile int8_t dWheelSpeed = 0; // 转速设置值，正负分别 100 档

// 磁矩值记录
// 测量值
volatile int16_t cMagneticMomentX = 0; // 符号表正负
volatile int16_t cMagneticMomentY = 0;
volatile int16_t cMagneticMomentZ = 0;
//设置值
volatile int8_t dMagneticMomentX = 0;
volatile int8_t dMagneticMomentY = 0;
volatile int8_t dMagneticMomentZ = 0;

// 电池电量记录
volatile uint8_t powerLevel = 9;     

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

// IMU 数据
float eulers[3] = {0};
float angularVelocities[3] = {0};
float accelerations[3] = {0};

// LED 状态记录
uint8_t ledFlag = LOW;
uint64_t previousMillis = 0; // 毫秒时间记录

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
    uint8_t len = 0;
    if (Serial1.available() > 0)
        sleep(20);  // 等待串口数据完全进入缓冲区
    while (Serial1.available()>0 && t<64/*防止溢出*/)
    {
        digitalWrite(ledpin, HIGH);
        recvBuffer[t] = (uint8_t)Serial1.read();
        t++;
        digitalWrite(ledpin, LOW);
    }
    len = recvBuffer[3]; // 有效数据长度
    // 检验数据帧有效性
    if (len<57 && recvBuffer[0]==0xAA && recvBuffer[1]==0x55 &&\
        recvBuffer[5+len]==0x0A && recvBuffer[6+len]==0x0D)
    {
        ////// 关闭所有模块控制 ////// 
        if (recvBuffer[2] == 0x00)
        {
        #ifdef SERIAL_DEBUG
            Serial.println("关闭所有");
        #endif
            // 关闭电源控制
            Wire.beginTransmission(I2C_ADDR_POWER);
            Wire.write(0x00);
            Wire.write(0x00);
            Wire.endTransmission();

            // 关闭热控
            Wire.beginTransmission(I2C_ADDR_THERMAL);
            Wire.write(0x00);
            Wire.write(0x00);
            Wire.endTransmission();

            // 关闭飞轮控制
            Wire.beginTransmission(I2C_ADDR_ATTITUDE);
            Wire.write(0x06);
            Wire.write(0x00);
            Wire.endTransmission();

            // 关闭磁力矩器控制
            Wire.beginTransmission(I2C_ADDR_ATTITUDE);
            Wire.write(0x02);
            Wire.write(0x00);
            Wire.endTransmission();
        }

        ////// 电源控制开关 ////// 
        if (recvBuffer[2] == 0x01)
        {
            if (recvBuffer[4] == 0x00)
            {
                Wire.beginTransmission(I2C_ADDR_POWER);
                Wire.write(0x00);
                Wire.write(0x00);
                Wire.endTransmission();
            #ifdef SERIAL_DEBUG
                Serial.println("关闭电控");
            #endif
            }
            else
            {
                Wire.beginTransmission(I2C_ADDR_POWER);
                Wire.write(0x00);
                Wire.write(0x01);
                Wire.endTransmission();
            #ifdef SERIAL_DEBUG
                Serial.println("打开电控");
            #endif
            }
        }

        ////// 飞轮控制开关 ////// 
        if (recvBuffer[2] == 0x02)
        {
            if (recvBuffer[4] == 0x00)
            {
                // 与姿控板通信，关闭飞轮
                Wire.beginTransmission(I2C_ADDR_ATTITUDE);
                Wire.write(0x06);
                Wire.write(0x00);
                Wire.endTransmission();
            #ifdef SERIAL_DEBUG
                Serial.println("关闭飞轮");
            #endif
            }
            else
            {
                // 与姿控板通信，打开飞轮
                Wire.beginTransmission(I2C_ADDR_ATTITUDE);
                Wire.write(0x06);
                Wire.write(0x01);
                Wire.endTransmission();
            #ifdef SERIAL_DEBUG
                Serial.println("打开飞轮");
            #endif
            }
        }

        ////// 温度控制开关 ////// 
        if (recvBuffer[2] == 0x03)
        {
            if (recvBuffer[4] == 0x00) // 热控关闭
            {
                // 与热控板通信，关闭热控
                Wire.beginTransmission(I2C_ADDR_THERMAL);
                Wire.write(0x00);
                Wire.write(0x00);
                Wire.endTransmission();
            #ifdef SERIAL_DEBUG
                Serial.println("关闭热控");
            #endif
            }
            else // recvBuffer[4] == 0x01 热控打开
            {
                // 与热控板通信，关闭热控
                Wire.beginTransmission(I2C_ADDR_THERMAL);
                Wire.write(0x00);
                Wire.write(0x01);
                Wire.endTransmission();
            #ifdef SERIAL_DEBUG
                Serial.println("打开热控");
            #endif
            }
        }

        ////// 磁力矩器控制开关 ////// 
        if (recvBuffer[2] == 0x04)
        {
            if (recvBuffer[4] == 0x00)
            {
                Wire.beginTransmission(I2C_ADDR_ATTITUDE);
                Wire.write(0x02);
                Wire.write(0x00);
                Wire.endTransmission();
            #ifdef SERIAL_DEBUG
                Serial.println("关闭磁棒");
            #endif
            }
            else
            {
                Wire.beginTransmission(I2C_ADDR_ATTITUDE);
                Wire.write(0x02);
                Wire.write(0x01);
                Wire.endTransmission();
            #ifdef SERIAL_DEBUG
                Serial.println("打开磁棒");
            #endif
            }
        } 

        ////// 飞轮转速设定 ////// 
        if (recvBuffer[2] == 0x05)
        {
            dWheelSpeed = (recvBuffer[4]);
            //与姿控板通信，发送飞轮转速
            Wire.beginTransmission(I2C_ADDR_ATTITUDE);
            Wire.write(0x04);
            Wire.write(dWheelSpeed);
            Wire.endTransmission();
        #ifdef SERIAL_DEBUG
            Serial.print("设定飞轮转速: ");
            Serial.println(dWheelSpeed);
        #endif
        }

        ////// 模块温度设定 //////
        if (recvBuffer[2] == 0x06)
        {
            memcpy(&dTemperMain, &recvBuffer[4], sizeof(float));
            memcpy(&dTemperAttitude, &recvBuffer[8], sizeof(float));
            memcpy(&dTemperPower, &recvBuffer[12], sizeof(float));
            memcpy(&dTemperThermal, &recvBuffer[16], sizeof(float));
            Wire.beginTransmission(I2C_ADDR_ATTITUDE);
            Wire.write(0x02); // 设置温度
            Wire.write((uint8_t *)&dTemperMain, sizeof(float));
            Wire.write((uint8_t *)&dTemperAttitude, sizeof(float));
            Wire.write((uint8_t *)&dTemperPower, sizeof(float));
            Wire.write((uint8_t *)&dTemperThermal, sizeof(float));
            Wire.endTransmission();
        #ifdef SERIAL_DEBUG
            Serial.print("设定模块温度: ");
            Serial.print(dTemperMain);
            Serial.print(", ");
            Serial.print(dTemperAttitude);
            Serial.print(", ");
            Serial.print(dTemperPower);
            Serial.print(", ");
            Serial.println(dTemperThermal);
        #endif
        }

        ////// 磁矩大小设定 //////
        if (recvBuffer[2] == 0x07)
        {
            dMagneticMomentX = recvBuffer[4];
            dMagneticMomentY = recvBuffer[5];
            dMagneticMomentZ = recvBuffer[6];
            Wire.beginTransmission(I2C_ADDR_ATTITUDE);
            Wire.write(0x00);
            Wire.write(dMagneticMomentX);
            Wire.write(dMagneticMomentY);
            Wire.write(dMagneticMomentZ);
            Wire.endTransmission();
        #ifdef SERIAL_DEBUG
            Serial.print("设定磁矩: ");
            Serial.print(dMagneticMomentX);
            Serial.print(", ");
            Serial.print(dMagneticMomentY);
            Serial.print(", ");
            Serial.println(dMagneticMomentZ);
        #endif
        }
    }
    sleep(20);
}

/*******************************************************************************
 *                                   主线程                                     *
*******************************************************************************/
void setup()
{
    pinMode(ledpin, OUTPUT);
    Wire.begin();
    Serial1.begin(38400);   // 使用串口 1 与上位机通信
#ifdef SERIAL_DEBUG
    Serial.begin(115200);   // 使用串口调试
#endif
    delay(20);
    mySCoop.start();        // 开启线程调度
}

void loop()
{
    uint8_t i = 0;

    uint64_t currentMillis = millis();
    if (currentMillis - previousMillis >= 500)
    {
        previousMillis = currentMillis; // 更新时间记录

        ledFlag = !ledFlag;
        digitalWrite(ledpin, ledFlag);
    }

    ////// 获取各个板子的数据 //////

    // 主控板
    // todo.

    // 姿控板： 欧拉角(12) + 三轴角速度(12) + 三轴加速度(12) + 
       //      飞轮转速(2) + 飞轮控制状态(1) 三轴磁矩(6) + 磁棒控制状态(1)
    i = 0;
    // 将 I2C 数据读到接收缓冲区，共 46 字节
    Wire.requestFrom(I2C_ADDR_ATTITUDE, 46);
    while (Wire.available())
    {
        iicBuffer[i++] = Wire.read();
    }
    // 更新欧拉角数据
    memcpy(eulers, &iicBuffer[0], sizeof eulers);
    // 更新三轴角速度数据
    memcpy(angularVelocities, &iicBuffer[12], sizeof angularVelocities);
    // 更新三轴加速度数据
    memcpy(accelerations, &iicBuffer[24], sizeof accelerations);
    // 更新飞轮转速测量值
    memcpy(&cWheelSpeed, &iicBuffer[36], sizeof(int16_t));
    // 飞轮控制状态
    wheelControlOn = (bool)iicBuffer[38];
    // 更新三轴磁力矩器电流
    memcpy(&cMagneticMomentX, &iicBuffer[39], sizeof(int16_t));
    memcpy(&cMagneticMomentY, &iicBuffer[41], sizeof(int16_t));
    memcpy(&cMagneticMomentZ, &iicBuffer[43], sizeof(int16_t));
    // 磁力矩器开关状态
    magnetbarsControlOn = (bool)iicBuffer[45];

    // 电源板：电量(1) + 电源控制状态(1)
    i = 0;
    // 将 I2C 数据读到接收缓冲区，共 2 字节
    Wire.requestFrom(I2C_ADDR_POWER, 2);
    while (Wire.available())
    {
        iicBuffer[i++] = Wire.read();
    }
    // 更新电池电量数据
    powerLevel = iicBuffer[0];
    powerControlOn = (bool)iicBuffer[1];

    // 热控板：温度(4) ×4
    i = 0;
    // 将 I2C 数据读到接收缓冲区，共 16 字节
    Wire.requestFrom(I2C_ADDR_THERMAL, 16);
    while (Wire.available())
    {
        iicBuffer[i++] = Wire.read();
    }
    // 更新各个板子的温度读数
    memcpy(&cTemperMain, &iicBuffer[0], sizeof(float));
    memcpy(&cTemperAttitude, &iicBuffer[4], sizeof(float));
    memcpy(&cTemperPower, &iicBuffer[8], sizeof(float));
    memcpy(&cTemperThermal, &iicBuffer[12], sizeof(float));

    ////// 发送电源状态数据到上位机 //////

    sendBuffer[0] = 0xAA;
    sendBuffer[1] = 0x55;
    sendBuffer[2] = 0x00; // 电量数据
    sendBuffer[3] = 2;
    sendBuffer[4] = powerLevel;
    if (powerControlOn)
        sendBuffer[5] = 0x01;
    else
        sendBuffer[5] = 0x00;
    sendBuffer[6] = 0; // 校验和，todo.
    sendBuffer[7] = 0x0A;
    sendBuffer[8] = 0x0D;
    Serial1.write(sendBuffer, 9);
    sleep(25);

    ////// 发送温度数据到上位机 //////

    sendBuffer[0] = 0xAA;
    sendBuffer[1] = 0x55;
    sendBuffer[2] = 0x02; // 温度数据
    sendBuffer[3] = 17;
    memcpy(&sendBuffer[4], &cTemperMain, sizeof(float));
    memcpy(&sendBuffer[8], &cTemperAttitude, sizeof(float));
    memcpy(&sendBuffer[12], &cTemperPower, sizeof(float));
    memcpy(&sendBuffer[16], &cTemperThermal, sizeof(float));
    if (thermalControlOn)
        sendBuffer[20] = 0x01;
    else
        sendBuffer[20] = 0x00;
    sendBuffer[21] = 0; // 校验和，todo.
    sendBuffer[22] = 0x0A;
    sendBuffer[23] = 0x0D;
    Serial1.write(sendBuffer, 24);
    // 如果热控打开则发送温度数据给热控板
    if (thermalControlOn)
    {
        Wire.write(I2C_ADDR_THERMAL);
        Wire.write(0x01);
        Wire.write(&sendBuffer[4], 16);
        Wire.endTransmission();
    }
    sleep(25);

    ////// 发送 IMU（九轴）数据到上位机 //////

    sendBuffer[0] = 0xAA;
    sendBuffer[1] = 0x55;
    sendBuffer[2] = 0x04; // IMU 数据
    sendBuffer[3] = 40;
    memcpy(&sendBuffer[4], eulers, sizeof eulers);
    memcpy(&sendBuffer[16], angularVelocities, sizeof angularVelocities);
    memcpy(&sendBuffer[28], accelerations, sizeof accelerations);
    sendBuffer[40] = 0; // 校验和，todo.
    sendBuffer[41] = 0x0A;
    sendBuffer[42] = 0x0D;
    Serial1.write(sendBuffer, 43);
    sleep(25);

    ////// 发送飞轮数据到上位机 //////

    sendBuffer[0] = 0xAA;
    sendBuffer[1] = 0x55;
    sendBuffer[2] = 0x06; // 飞轮数据
    sendBuffer[3] = 3;
    memcpy(&sendBuffer[4], &cWheelSpeed, sizeof cWheelSpeed);
    if (wheelControlOn)
        sendBuffer[6] = 0x01;
    else
        sendBuffer[6] = 0x00;
    sendBuffer[7] = 0; // 校验和，todo.
    sendBuffer[8] = 0x0A;
    sendBuffer[9] = 0x0D;
    Serial1.write(sendBuffer, 10);
    sleep(25);

    ////// 发送磁力矩器数据到上位机 //////

    sendBuffer[0] = 0xAA;
    sendBuffer[1] = 0x55;
    sendBuffer[2] = 0x08; // 磁控数据
    sendBuffer[3] = 7;
    memcpy(&sendBuffer[4], &cMagneticMomentX, sizeof cMagneticMomentX);
    memcpy(&sendBuffer[6], &cMagneticMomentY, sizeof cMagneticMomentY);
    memcpy(&sendBuffer[8], &cMagneticMomentZ, sizeof cMagneticMomentZ);
    if (magnetbarsControlOn)
        sendBuffer[10] = 0x01;
    else
        sendBuffer[10] = 0x00;
    sendBuffer[11] = 0; // 校验和，todo.
    sendBuffer[12] = 0x0A;
    sendBuffer[13] = 0x0D;
    Serial1.write(sendBuffer, 14);
    sleep(25);

    yield();    // 让给另一个线程
}
