/*
 * 舵机通讯检测 Mega2560多串口版本
 * --------------------------
 * 作者: 深圳市华馨京科技有限公司
 * 网站：https://fashionrobo.com/
 * 更新时间: 2023/03/13
 **/
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

// 舵机 #0 挂载在串口1上
FSUS_Servo uservo_0(0, &protocol_ch2); // 创建舵机
// 舵机 #1 挂载在串口2上
FSUS_Servo uservo_1(1, &protocol_ch2); // 创建舵机
///////////////////*请以上面串口1,2范例为标准*////////////////

void setup(){
    // 串口总线舵机 #0 初始化
    uservo_0.init(); 
    // 串口总线舵机 #1 初始化
    uservo_1.init(); 
    
    // 打印例程信息
    DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUDRATE);
    DEBUG_SERIAL.println("Start To Ping Servo\n");
}

void loop(){
    // 舵机通讯检测
    bool u0_valid = uservo_0.ping(); 
    String message1 = "servo #"+String(uservo_0.servoId,DEC) + " is ";  // 日志输出
    if(u0_valid){
        message1 += "online";
    }else{
        message1 += "offline";
    }    
    // 调试串口初始化
    DEBUG_SERIAL.println(message1);


    // 舵机通讯检测
    bool u1_valid = uservo_1.ping(); 
    String message2 = "servo #"+String(uservo_1.servoId,DEC) + " is ";  // 日志输出
    if(u1_valid){
        message2 += "online";
    }else{
        message2 += "offline";
    }    
    // 调试串口初始化
    DEBUG_SERIAL.println(message2);
    
    // 等待1s
    delay(1000);
}
