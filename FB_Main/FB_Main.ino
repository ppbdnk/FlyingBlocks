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
#include <string.h>     // 使用内存拷贝

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
// 单次收发数据帧限制在 64 字节以内
uint8_t sendBuffer[64];
uint8_t recvBuffer[64];

// I2C 数据缓冲区
uint8_t iicBuffer[64];

// 状态控制标志量
volatile bool powerControlOn = false;
volatile bool thermalControlOn = false;
volatile bool wheelControlOn = false;
volatile bool magnetbarControlOn = false;

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
float quaternion[4] = {0};
float angularVelocities[3] = {0};
float accelerations[3] = {0};

// 实例化温度传感器
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);

// LED 状态记录
bool ledOn = false;
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
            // 关闭电源控制
            powerControlOn = false;
            Wire.beginTransmission(I2C_ADDR_POWER);
            Wire.write(0x00);
            Wire.write(0x00);
            Wire.endTransmission();

            // 关闭热控
            thermalControlOn = false;
            Wire.beginTransmission(I2C_ADDR_THERMAL);
            Wire.write(0x00);
            Wire.write(0x00);
            Wire.endTransmission();

            // 关闭飞轮控制
            wheelControlOn = false;
            Wire.beginTransmission(I2C_ADDR_ATTITUDE);
            Wire.write(0x06);
            Wire.write(0x00);
            Wire.endTransmission();

            // 关闭磁力矩器控制
            magnetbarControlOn = false;
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
                powerControlOn = false; // 关闭
                Wire.beginTransmission(I2C_ADDR_POWER);
                Wire.write(0x00);
                Wire.write(0x00);
                Wire.endTransmission();
            }
            else
            {
                powerControlOn = true;
                Wire.beginTransmission(I2C_ADDR_POWER);
                Wire.write(0x00);
                Wire.write(0x01);
                Wire.endTransmission();
            }
        }

        ////// 飞轮控制开关 ////// 
        if (recvBuffer[2] == 0x02)
        {
            if (recvBuffer[4] == 0x00)
            {
                wheelControlOn = false;
                // 与姿控板通信，关闭飞轮
                Wire.beginTransmission(I2C_ADDR_ATTITUDE);
                Wire.write(0x06);
                Wire.write(0x00);
                Wire.endTransmission();
            }
            else
            {
                wheelControlOn = true;
                // 与姿控板通信，打开飞轮
                Wire.beginTransmission(I2C_ADDR_ATTITUDE);
                Wire.write(0x06);
                Wire.write(0x01);
                Wire.endTransmission();
            }
        }

        ////// 温度控制开关 ////// 
        if (recvBuffer[2] == 0x03)
        {
            if (recvBuffer[4] == 0x00) // 热控关闭
            {
                thermalControlOn = false;
                // 与热控板通信，关闭热控
                Wire.beginTransmission(I2C_ADDR_THERMAL);
                Wire.write(0x00);
                Wire.write(0x00);
                Wire.endTransmission();
            }
            else // recvBuffer[4] == 0x01 热控打开
            {
                thermalControlOn = true;
                // 与热控板通信，关闭热控
                Wire.beginTransmission(I2C_ADDR_THERMAL);
                Wire.write(0x00);
                Wire.write(0x01);
                Wire.endTransmission();
            }
        }

        ////// 磁力矩器控制开关 ////// 
        if (recvBuffer[2] == 0x04)
        {
            if (recvBuffer[4] == 0x00)
            {
                magnetbarControlOn = false;
                Wire.beginTransmission(I2C_ADDR_ATTITUDE);
                Wire.write(0x02);
                Wire.write(0x00);
                Wire.endTransmission();
            }
            else
            {
                magnetbarControlOn = true;
                Wire.beginTransmission(I2C_ADDR_ATTITUDE);
                Wire.write(0x02);
                Wire.write(0x01);
                Wire.endTransmission();
            }
        } 

        ////// 飞轮转速设置 ////// 
        if (recvBuffer[2] == 0x05)
        {
            dWheelSpeed = (recvBuffer[4]);
            //与姿控板通信，发送飞轮转速
            Wire.beginTransmission(I2C_ADDR_ATTITUDE);
            Wire.write(0x04);
            Wire.write(dWheelSpeed);
            Wire.endTransmission();
        }

        ////// 模块温度设置 //////
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
        }

        ////// 磁矩大小设置 //////
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
    ds18b20.begin();        // 温度传感器初始化
    ds18b20.setWaitForConversion(false); // 设置为非阻塞模式
    ds18b20.requestTemperatures();
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

        if (ledOn)
        {
            ledOn = false;
            digitalWrite(ledpin, LOW);
        }
        else 
        {
            ledOn = true;
            digitalWrite(ledpin, HIGH);
        }
    }

    ////// 获取各个板子的数据 //////

    // 主控板
    cTemperMain = ds18b20.getTempCByIndex(0);
    ds18b20.requestTemperatures();

    // 姿控板：温度(4) + 四元数(16) + 三轴角速度(12) + 三轴加速度(12) + 飞轮转速(2)
    i = 0;
    // 将 I2C 数据读到接收缓冲区，共 46 字节
    Wire.requestFrom(I2C_ADDR_ATTITUDE, 46);
    while (Wire.available())
    {
        iicBuffer[i++] = Wire.read();
    }
    // 更新姿态板温度数据
    // cTemperAttitude = ((int16_t)recvBuffer[0] << 8) | recvBuffer[1]);
    memcpy(&cTemperAttitude, &iicBuffer[0], sizeof(float));
    // 更新四元数数据
    memcpy(quaternion, &iicBuffer[4], sizeof quaternion);
    // 更新三轴角速度数据
    memcpy(angularVelocities, &iicBuffer[20], sizeof angularVelocities);
    // 更新三轴加速度数据
    memcpy(accelerations, &iicBuffer[32], sizeof accelerations);
    // 更新飞轮转速测量值
    memcpy(&cWheelSpeed, &iicBuffer[44], sizeof(int16_t));

    // 电源板：温度(4) + 电量(1)
    i = 0;
    // 将 I2C 数据读到接收缓冲区，共 5 字节
    Wire.requestFrom(I2C_ADDR_POWER, 5);
    while (Wire.available())
    {
        iicBuffer[i++] = Wire.read();
    }
    // 更新电源板温度数据
    memcpy(&cTemperPower, &iicBuffer[0], sizeof(float));
    // 更新电池电量数据
    powerLevel = recvBuffer[4];

    // 热控板：温度(4)
    i = 0;
    // 将 I2C 数据读到接收缓冲区，共 4 字节
    Wire.requestFrom(I2C_ADDR_THERMAL, 4);
    while (Wire.available())
    {
        iicBuffer[i++] = Wire.read();
    }
    memcpy(&cTemperThermal, &iicBuffer[0], sizeof(float));

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
    sleep(50);

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
    sleep(50);

    ////// 发送 IMU（九轴）数据到上位机 //////

    sendBuffer[0] = 0xAA;
    sendBuffer[1] = 0x55;
    sendBuffer[2] = 0x04; // IMU 数据
    sendBuffer[3] = 40;
    memcpy(&sendBuffer[4], quaternion, sizeof quaternion);
    memcpy(&sendBuffer[20], angularVelocities, sizeof angularVelocities);
    memcpy(&sendBuffer[32], accelerations, sizeof accelerations);
    sendBuffer[44] = 0; // 校验和，todo.
    sendBuffer[45] = 0x0A;
    sendBuffer[46] = 0x0D;
    Serial1.write(sendBuffer, 47);
    sleep(50);

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
    sleep(50);

    ////// 发送磁力矩器数据到上位机 //////

    sendBuffer[0] = 0xAA;
    sendBuffer[1] = 0x55;
    sendBuffer[2] = 0x08; // 磁控数据
    sendBuffer[3] = 7;
    memcpy(&sendBuffer[4], &cMagneticMomentX, sizeof cMagneticMomentX);
    memcpy(&sendBuffer[6], &cMagneticMomentY, sizeof cMagneticMomentY);
    memcpy(&sendBuffer[8], &cMagneticMomentZ, sizeof cMagneticMomentZ);
    if (magnetbarControlOn)
        sendBuffer[10] = 0x01;
    else
        sendBuffer[10] = 0x00;
    sendBuffer[11] = 0; // 校验和，todo.
    sendBuffer[12] = 0x0A;
    sendBuffer[13] = 0x0D;
    Serial1.write(sendBuffer, 14);
    sleep(50);

    yield();    // 让给另一个线程
}
