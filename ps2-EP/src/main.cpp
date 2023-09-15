#include <Arduino.h>
// #include <PS2X_lib.h>
// #include <string.h>
// #include <math.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>
// #include "FashionStar_UartServoProtocol.h" // 串口总线舵机通信协议
// #include "FashionStar_UartServo.h" // Fashion Star串口总线舵机的依赖

// #define PS2_DAT        13  //14    
// #define PS2_CMD        11  //15
// #define PS2_SEL        10  //16
// #define PS2_CLK        12  //17
// #define pressures   false    //按键模式
// #define rumble      false    //振动模式
// PS2X ps2x;
// int error = 0;
// byte type = 0;
// byte vibrate = 0;

float tran_speed = 1;
// float rot_speed = 120;

// uint8_t spd_swc = 0;

// FSUS_Protocol protocol(115200);       //协议
// FSUS_Servo uservo0(0, &protocol); // 创建大臂舵机
// FSUS_Servo uservo1(1, &protocol); // 创建小臂舵机
// FSUS_Servo uservo2(2, &protocol); // 创建手腕舵机
// FSUS_Servo uservo3(3, &protocol); // 创建夹爪舵机

TaskHandle_t NonBlock_Handle;
void TaskNonBlock(void *pvParameters);
// void TaskArm(void *pvParameters);
// void TaskBackPart(void *pvParameters);

// void PS2_control(void);



void(* resetFunc) (void) = 0;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  delay(1000);
  Serial.println("start...");
  delay(500);
  // protocol.init(); // 舵机通信协议初始化
  // uservo0.init(); // 串口总线舵机初始化
  // uservo1.init(); // 串口总线舵机初始化
  // uservo2.init(); // 串口总线舵机初始化

  // error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);

  // #pragma region

  // if(error == 0){
  //   Serial.print("Found Controller, configured successful ");
  //   Serial.print("pressures = ");
	// if (pressures)
	//   Serial.println("true ");
	// else
	//   Serial.println("false");
	// Serial.print("rumble = ");
	// if (rumble)
	//   Serial.println("true)");
	// else
	//   Serial.println("false");
  //   Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
  //   Serial.println("holding L1 or R1 will print out the analog stick values.");
  //   Serial.println("Note: Go to www.billporter.info for updates and to report bugs.");
  // }  
  // else if(error == 1)
  //   Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
   
  // else if(error == 2)
  //   Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");

  // else if(error == 3)
  //   Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
  
  // // type = ps2x.readType(); 
  // switch(type) {
  //   case 0:
  //     Serial.println("Unknown Controller type found ");
  //     break;
  //   case 1:
  //     Serial.println("DualShock Controller found ");
  //     break;
  //   case 2:
  //     Serial.println("GuitarHero Controller found ");
  //     break;
	//   case 3:
  //     Serial.println("Wireless Sony DualShock Controller found ");
  //     break;
  // }

  // #pragma endregion
      

  Serial.println("start to create the tasks...");
  xTaskCreate(TaskNonBlock, "s", 800, NULL, 3, &NonBlock_Handle);
  // xTaskCreate(TaskArm, "a", 800, NULL, 3, NULL);
  // xTaskCreate(TaskBackPart, "d", 800, NULL, 3, NULL);
  // Serial.println("already create the tasks...");
  vTaskStartScheduler();
}

void loop() {
  // Serial.println("String(tran_speed)");
}

void TaskNonBlock(void *pvParameters){
  (void) pvParameters;
  for(;;){
    Serial.println(String(tran_speed));
    // if(error == 1) resetFunc();   //没连上就重启
    // if(type==2){
    //   PS2_control();
    // }
    
    vTaskDelay(1);
  }
}

// void TaskArm(void *pvParameters){
//   (void) pvParameters;
//   for(;;){

//     vTaskDelay(1);
//   }
// }

// void TaskBackPart(void *pvParameters){
//   (void) pvParameters;
//   for(;;){

//     vTaskDelay(1);
//   }
// }

// void PS2_control(void){
//   float Xc, Yc, Zc;    //手动遥控底盘速度
//   float Xc1, Yc1, Zc1;    //手动遥控底盘速度备份
//   float X1, X2, Y1, Y2;    //手柄摇杆数据

//   //摇杆数据
//   // X1 = ps2x.Analog(PSS_LX);
//   // Y1 = ps2x.Analog(PSS_LX);
//   // X2 = ps2x.Analog(PSS_RX);
//   // Y2 = ps2x.Analog(PSS_RY);

//   //错误处理
//   if (error == 1)           //skip loop if no controller found
//     return;
//   if (type != 1)            //skip loop if no controller found
//     return;
//   // ps2x.read_gamepad();      //读取手柄数据

//   #pragma region    //----------------------------------------------------------------------------------------------------

//   // if(ps2x.Button(PSB_L2)){                //按住潜行
//   //   tran_speed = 0.5;
//   // }else if(ps2x.Button(PSB_L1)){          //按住疾跑
//   //   tran_speed = 1.5;
//   // }else{                                  //正常行走
//   //   tran_speed = 1;
//   // }

//   Serial.println(tran_speed);
//   // if(Y1 <= 5){        //左摇杆向前
//   //   Xc = tran_speed;
//   // }else if(Y1 >= 230){        //左摇杆向后
//   //   Xc = -1*tran_speed;
//   // }else{
//   //   Xc = 0;
//   // }

//   // if(X1 <= 5){        //左摇杆向左
//   //   Yc = -1*tran_speed;
//   // }else if(X1 >= 230){        //左摇杆向右
//   //   Yc = tran_speed;
//   // }else{
//   //   Yc = 0;
//   // }

//   // if(X2 <= 5){        //右摇杆向左
//   //   Zc = -1*rot_speed;
//   // }else if(X2 >= 230){        //右摇杆向右
//   //   Zc = rot_speed;
//   // }else{
//   //   Zc = 0;
//   // }

  
//   // if(Xc!=Xc1 || Yc!=Yc1 || Zc!=Zc1)Serial1.print("chassis speed x " + String(Xc) + " y " + String(Yc) + " z " + String(Zc) + ";");
//   // Xc1=Xc;Yc1=Yc;Zc1=Zc;

//   #pragma endregion     //----------------------------------------------------------------------------------------------------
// }

