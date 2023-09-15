/*
 * 测试软串口
 * 
 * <接线方式>
 * - Arduino Pin 6(软串口RX)  -> USB转TTL TX
 * - Arduino Pin 7(软串口TX)  -> USB转TTL RX
 * - Arduino GND              -> USB转TTL GND
 */
#include <SoftwareSerial.h>

// 软串口的配置
#define SOFT_SERIAL_RX 6
#define SOFT_SERIAL_TX 7
#define SOFT_SERIAL_BAUDRATE 4800
// 创建软串口
SoftwareSerial softSerial(SOFT_SERIAL_RX, SOFT_SERIAL_TX);

void setup(){    
    softSerial.begin(SOFT_SERIAL_BAUDRATE); // 初始化软串口的波特率
}

void loop(){
    //发送一个数据
    softSerial.print("Hello World\n");
    delay(1000);
}