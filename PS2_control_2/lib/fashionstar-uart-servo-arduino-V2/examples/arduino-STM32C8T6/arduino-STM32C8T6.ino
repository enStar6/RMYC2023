/* 
 * 设置舵机的角度(单圈模式)
 * 提示: PCB板上电之后, 记得按下的RESET按键
 * --------------------------
 * 作者: 深圳市华馨京科技有限公司
 * 网站：https://fashionrobo.com/
 * 更新时间: 2023/03/13
 */

#include <HardwareSerial.h>

//                      RX    TX
HardwareSerial Serial1(PA10, PA9);
//HardwareSerial Serial2(PA3, PA2); //这里串口2不需要定义
HardwareSerial Serial3(PB11, PB10);

#include "FashionStar_UartServoProtocol.h" // 串口总线舵机通信协议
#include "FashionStar_UartServo.h" // Fashion Star串口总线舵机的依赖

// 串口总线舵机配置
#define USERVO_BAUDRATE (uint32_t)115200 // 波特率

// 调试串口的配置
#define DEBUG_SERIAL Serial
#define DEBUG_SERIAL_BAUDRATE (uint32_t)115200

//多串口版本主要区别在于：串口舵机管理器&舵机挂载在串口上，2个部分//
// 串口1舵机管理器
FSUS_Protocol protocol_ch1(&Serial1, USERVO_BAUDRATE);
// 串口2舵机管理器
FSUS_Protocol protocol_ch2(&Serial2, USERVO_BAUDRATE);
// 串口3舵机管理器
FSUS_Protocol protocol_ch3(&Serial3, USERVO_BAUDRATE);

// 舵机 #0 挂载在串口1上
FSUS_Servo uservo_0(0, &protocol_ch1); // 创建舵机
// 舵机 #1 挂载在串口2上
FSUS_Servo uservo_1(1, &protocol_ch2); // 创建舵机
// 舵机 #2 挂载在串口3上
FSUS_Servo uservo_2(2, &protocol_ch3); // 创建舵机
///////////////////*请以上面串口1,2范例为标准*////////////////

void setup() {
  
    // 串口总线舵机 #0 初始化
    uservo_0.init(); 
    // 串口总线舵机 #1 初始化
    uservo_1.init(); 
    // 串口总线舵机 #2 初始化
    uservo_2.init();
    
    // 打印例程信息
    DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUDRATE);
    DEBUG_SERIAL.println("Start To Ping Servo\n");
}

void loop() {


    uservo_0.setRawAngle(90.0);  // 设置舵机的角度
    uservo_1.setRawAngle(90.0);  // 设置舵机的角度
    uservo_2.setRawAngle(90.0);  // 设置舵机的角度
    delay(2000);

    uservo_0.setRawAngle(-90);
    uservo_1.setRawAngle(-90);
    uservo_2.setRawAngle(-90);
    delay(2000);
}
