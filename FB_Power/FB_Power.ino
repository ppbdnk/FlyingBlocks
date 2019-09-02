/*************************************************************************
* @file     --> FB_Power.ino
* @author   --> Zhenghao.Zhang
* @version  --> v2.1
* @date     --> 2019-09-01
* @brief    --> 电源模块
*           1. 读取电池电量数据并发送给主控板
*           2. 从主控板接收上位机发送的电源开闭指令，控制相应每一路电源开闭
**************************************************************************/
#include <Wire.h>
#include <OneWire.h>
// #include <DallasTemperature.h>   ***
// #define ONE_WIRE_BUS 42          ***

/* Private macro -------------------------------------------------------------*/
// 定义串口调试宏开关，用于开发阶段的数据查看，正式编译的时候应当注释掉此行
// #define SERIAL_DEBUG

/* Private define ------------------------------------------------------------*/
#define I2C_WIRE_BUS 0x40

/* Private variables ---------------------------------------------------------*/
// 引脚定义
int SW0 = A7;  // SWITCH 0
int SW1 = A6;  // SWITCH 1
int SW2 = A5;  // SWITCH 2
int SW3 = A4;  // SWITCH 3
int SW4 = A3;  // SWITCH 4
int SW5 = A2;  // SWITCH 5
int SW6 = A1;  // SWITCH 6
int SW7 = A0;  // SWITCH 7

// 电压相关
int Volt; 
unsigned char v0;
int t;

int x = 0; // 判断是否收到主控板指令

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void setup() 
{
    // IO 初始化
    pinMode(SW0,OUTPUT);
    pinMode(SW1,OUTPUT); 
    pinMode(SW2,OUTPUT); 
    pinMode(SW3,OUTPUT);
    pinMode(SW4,OUTPUT);
    pinMode(SW5,OUTPUT);
    pinMode(SW6,OUTPUT);
    pinMode(SW7,OUTPUT);

// 调试串口初始化
#ifdef SERIAL_DEBUG
    Serial.begin(115200);
#endif

    // I2C 总线初始化
    Wire.begin(I2C_WIRE_BUS);
    Wire.onRequest(requestEvent);
    Wire.onReceive(receiveEvent); 
}

void loop() 
{ 
    //电池电量
    volt = analogRead(A8);

    // 判断是否收到主控板指令
    if (x == 0) // 若未收到主控板指令，则打开所有电源开关
    {
        digitalWrite(SW0,HIGH);
        digitalWrite(SW1,HIGH);
        digitalWrite(SW2,HIGH);
        digitalWrite(SW3,HIGH);
        digitalWrite(SW4,HIGH);
        digitalWrite(SW5,HIGH);
        digitalWrite(SW6,HIGH);
        digitalWrite(SW7,HIGH);
    }
    else       // 收到主控板指令之后，控制不同开关开闭
    {
        // Switch0 开关
        if (recvBuffer[4] == 0x00)
        {
            digitalWrite(SW0,HIGH);
        #ifdef SERIAL_DEBUG
            Serial.println("打开SW0");
        #endif    
        }
        else
        {
            digitalWrite(SW0,LOW);
        #ifdef SERIAL_DEBUG
            Serial.println("关闭SW0");
        #endif  
        }

        // Switch1 开关
        if (recvBuffer[5] == 0x00) 
        {
            digitalWrite(SW1,HIGH);
        #ifdef SERIAL_DEBUG
            Serial.println("打开SW1");
        #endif  
        }
        else
        {
            digitalWrite(SW1,LOW);
        #ifdef SERIAL_DEBUG
            Serial.println("关闭SW1");
        #endif  
        }

        // Switch2 开关
        if (recvBuffer[6] == 0x00) 
        {
            digitalWrite(SW2,HIGH);
        #ifdef SERIAL_DEBUG
            Serial.println("打开SW2");
        #endif  
        }
        else
        {
            digitalWrite(SW2,LOW);
        #ifdef SERIAL_DEBUG
            Serial.println("关闭SW2");
        #endif  
        }

        // Switch3 开关
        if (recvBuffer[7] == 0x00) 
        {
            digitalWrite(SW3,HIGH);
        #ifdef SERIAL_DEBUG
            Serial.println("打开SW3");
        #endif 
        }
        else
        {
            digitalWrite(SW3,LOW);
        #ifdef SERIAL_DEBUG
            Serial.println("关闭SW3");
        #endif 
        }

        // Switch4 开关
        if (recvBuffer[8] == 0x00) 
        {
            digitalWrite(SW4,HIGH);
        #ifdef SERIAL_DEBUG
            Serial.println("打开SW4");
        #endif 
        }
        else
        {
            digitalWrite(SW4,LOW);
        #ifdef SERIAL_DEBUG
            Serial.println("关闭SW4");
        #endif 
        }

        // Switch5 开关
        if (recvBuffer[9] == 0x00) 
        {
            digitalWrite(SW5,HIGH);
        #ifdef SERIAL_DEBUG
            Serial.println("打开SW5");
        #endif 
        }
        else
        {
            digitalWrite(SW5,LOW);
        #ifdef SERIAL_DEBUG
            Serial.println("关闭SW5");
        #endif 
        }

        // Switch6 开关
        if (recvBuffer[10] == 0x00) 
        {
            digitalWrite(SW6,HIGH);
        #ifdef SERIAL_DEBUG
            Serial.println("打开SW6");
        #endif 
        }
        else
        {
            digitalWrite(SW6,LOW);
        #ifdef SERIAL_DEBUG
            Serial.println("关闭SW6");
        #endif 
        }

        // Switch7 开关
        if (recvBuffer[11] == 0x00) 
        {
            digitalWrite(SW7,HIGH);
        #ifdef SERIAL_DEBUG
            Serial.println("打开SW7");
        #endif 
        }
        else
        {
            digitalWrite(SW7,LOW);
        #ifdef SERIAL_DEBUG
            Serial.println("关闭SW7");
        #endif 
        }
    }
}

void requestEvent(int count) 
{
    //发送电量数据
    t = (double)volt/1023*10; // 转换为 0-10 的 int
    v0 = t;                   // 转换为 unsigned char
    Wire.write(v0);           // 发送数据给主控板
 }

 void receiveEvent()
 {
    uint8_t i = 0;
    // 读取 I2C 数据到缓冲区
    while (Wire.available())
    {
        recvBuffer[i++] = (uint8_t)Wire.read();
    }
    x = 1;
 }        
