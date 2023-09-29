/*
 * 舵机数据读取实验
 * --------------------------
 * 作者: 深圳市华馨京科技有限公司
 * 网站：https://fashionrobo.com/
 * 更新时间: 2023/03/13
 **/
#include "FashionStar_UartServoProtocol.h" // 串口总线舵机通信协议
#include "FashionStar_UartServo.h" // Fashion Star串口总线舵机的依赖

// 配置
#define SERVO_ID 4 //舵机ID号
#define BAUDRATE 115200 // 波特率

// 调试串口的配置
#if defined(ARDUINO_AVR_UNO)
    #include <SoftwareSerial.h>
    #define SOFT_SERIAL_RX 6 
    #define SOFT_SERIAL_TX 7
    SoftwareSerial softSerial(SOFT_SERIAL_RX, SOFT_SERIAL_TX); // 创建软串口
    #define DEBUG_SERIAL softSerial
    #define DEBUG_SERIAL_BAUDRATE 4800
    
#elif defined(ARDUINO_AVR_MEGA2560)
    #define DEBUG_SERIAL Serial
    #define DEBUG_SERIAL_BAUDRATE 115200
    
#elif defined(ARDUINO_ARCH_ESP32)
    #define DEBUG_SERIAL Serial
    #define DEBUG_SERIAL_BAUDRATE 115200
    
#elif defined(ARDUINO_ARCH_STM32)
    #include <HardwareSerial.h>
    //                      RX    TX
    HardwareSerial Serial1(PA10, PA9);
    //HardwareSerial Serial2(PA3, PA2); //这里串口2不需要定义
    HardwareSerial Serial3(PB11, PB10);
    #define DEBUG_SERIAL Serial1
    #define DEBUG_SERIAL_BAUDRATE (uint32_t)115200
    
#endif 

FSUS_Protocol protocol(BAUDRATE); //协议
FSUS_Servo uservo(SERVO_ID, &protocol); // 创建舵机

// 读取数据
uint16_t voltage;       // 电压 mV
uint16_t current;       // 电流 mA
uint16_t power;         // 功率 mW
uint16_t temperature;   // 温度 ℃

void setup(){
    
    protocol.init(); // 舵机通信协议初始化
    uservo.init(); // 串口总线舵机初始化
    // 打印例程信息
    DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUDRATE);
    DEBUG_SERIAL.println("Start To Test Servo Data Read \n"); // 打印日志
    
    uservo.setAngle(-25.0,  1000, 200); // 设置舵机角度(限制功率)
}

void loop(){
    voltage = uservo.queryVoltage();
    current = uservo.queryCurrent();
    power = uservo.queryPower();
    temperature = uservo.queryTemperature();
    DEBUG_SERIAL.println("voltage: "+String((float)voltage, 1)+" mV\n");
    delay(100);
    DEBUG_SERIAL.println("current: "+String((float)current, 1)+" mA\n");
    delay(100);
    DEBUG_SERIAL.println("power: "+String((float)power, 1)+" mW\n");
    delay(100);
    DEBUG_SERIAL.println("temperature: "+String((float)temperature, 1)+" Celsius\n");
    delay(1000);
}
