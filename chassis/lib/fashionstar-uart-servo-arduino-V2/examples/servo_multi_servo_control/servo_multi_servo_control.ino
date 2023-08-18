/* 
 * 多个舵机控制示例
 * --------------------------
 * 作者: 深圳市华馨京科技有限公司
 * 网站：https://fashionrobo.com/
 * 更新时间: 2023/03/13
 */
#include "FashionStar_UartServoProtocol.h"
#include "FashionStar_UartServo.h" // Fashion Star串口总线舵机的依赖

// 串口总线舵机配置参数
#define SERVO0 0 // 舵机0的ID号
#define SERVO1 1 // 舵机1的ID号
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
FSUS_Servo servo0(SERVO0, &protocol); // 创建舵机
FSUS_Servo servo1(SERVO1, &protocol); // 创建舵机


/* 等待所有的舵机完成动作 */
void wait_all_servo_done(){
    servo0.wait();
    servo1.wait();
}

void setup(){
    protocol.init();    // 通信协议初始化
    servo0.init();      // 舵机0初始化
    servo1.init();      // 舵机1初始化
    // 打印例程信息
    DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUDRATE);
    DEBUG_SERIAL.println("Test Multi Servo Control");
}

void loop(){
    DEBUG_SERIAL.println("Servo0 = 45.0, Servo1 = 30.0");
    servo0.setRawAngle(45.0, 1000);         // 设置舵机0的角度
    servo1.setRawAngle(30.0, 1000);         // 设置舵机1的角度
    wait_all_servo_done();                  // 等待动作完成
    delay(2000);                            // 延时2s

    
    DEBUG_SERIAL.println("Servo0 = -45.0, Servo1 = -30.0");
    servo0.setRawAngle(-45, 1000);          // 设置舵机0的角度
    servo1.setRawAngle(-30.0, 1000);        // 设置舵机1的角度
    wait_all_servo_done();                  // 等待动作完成
    delay(2000);                            // 延时2s
}
