#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <PS2X_lib.h>
#include <FashionStar_UartServo.h>
#include <FashionStar_UartServoProtocol.h>
#include <NewPing.h>

//PS2相关定义-----------------------------------------------------------------------------------------------------------------------------------------
#define PS2_DAT        13   
#define PS2_CMD        11
#define PS2_SEL        10
#define PS2_CLK        12
#define pressures   true
#define rumble      true

PS2X ps2x;

int error = 0;
byte type = 0;

bool is_SELECT    = 0;
bool is_START     = 0;
bool is_PAD_DOWN  = 0;
bool is_PAD_UP    = 0;
bool is_PAD_LEFT  = 0;
bool is_PAD_RIGHT = 0;
bool is_PAD_DOWN_prs  = 0;
bool is_PAD_UP_prs    = 0;
bool is_PAD_LEFT_prs  = 0;
bool is_PAD_RIGHT_prs = 0;
bool is_X         = 0;
bool is_A         = 0;
bool is_B         = 0;
bool is_Y         = 0;
bool is_R1        = 0;
bool is_R2        = 0;
bool is_R3        = 0;
bool is_L3        = 0;

//----------------------------------------------------------------------------------------------------------------------------------------------------


//底盘相关定义-----------------------------------------------------------------------------------------------------------------------------------------
float tran_speed = 1;       //0.8:正常速度  1.2:疾跑速度  0.2:潜行速度
float rot_speed = 120;        //绕Z轴旋转速度（一般保持不变）
int8_t facing = 0;
bool update_chassis = 0;

//----------------------------------------------------------------------------------------------------------------------------------------------------


//DJI舵机定义-----------------------------------------------------------------------------------------------------------------------------------------
#define head_servo_id 1
#define ban_servo_id 2
#define box_servo_id 3

#define BOX_UP -150
#define BOX_DOWN 25
#define BAN_UP 115
#define BAN_DOWN 10

bool is_facing_change = 0;

//---------------------------------------------------------------------------------------------------------------------------------------------------


//机械臂相关定义---------------------------------------------------------------------------------------------------------------------------------------
FSUS_Protocol protocol(&Serial3, 115200);   // 协议
FSUS_Servo uservo0(0, &protocol); // 创建大臂舵机
FSUS_Servo uservo1(1, &protocol); // 创建小臂舵机
FSUS_Servo uservo2(2, &protocol); // 创建手腕舵机
FSUS_Servo uservo3(3, &protocol); // 创建夹爪舵机

int16_t OPEN_DYP_AN   = -70;
int16_t CLOSE_DYP_AN  = -130;
int16_t CLOSE_DDW_AN  = -110;
int16_t SMALL_DDW     =  60;
int16_t SMALL_DYP     = -91;
int16_t BIG_UP        = -90;
int16_t SMALL_UP      = -80;

uint8_t arm_sts  = 0;    //arm_sts是手动阶段夹取时，机械臂的局部执行状态，体现的是机械臂放下、夹取抬升等  
                        //0:下方弹药瓶准备  1:大弹丸准备  2:夹取  3:微调后

uint8_t box_sts = 0;    //box_sts是储弹盒局部执行状态，体现储弹盒倾倒和平放
                        //0:储弹盒平放  1:储弹盒倾倒
                      
uint8_t cID=0;          //微调的自由度编号
                        //1:夹爪  2:小臂

bool is_trimming = 0;   //机械臂微调模式状态变量

double t_an[2]      = {0,90};
double t_pos[2]      = {209.35,0};

#define TRIG 2
#define ECHO 3
#define MAX_DISTANCE 200
#define firstDYP 11
#define secondDYP 29
#define thirdDYP 46

uint8_t pingSpeed = 100;  
unsigned long pingTimer;
double dis1 = 0;

NewPing sonar1(TRIG, ECHO, MAX_DISTANCE);
//----------------------------------------------------------------------------------------------------------------------------------------------------


//普通函数声明-------------------------------------------------------------------------------------------------------------------------------------------
void (* resetFunc) (void) = 0;    //重启单片机
void man_get_dyp(void);
void direct_servo(void);
void unload(void);
String get_djiservo_string(int id, int degrees);                        //获取控制DJI舵机的SDK命令
String getMoveString(double x, double y, double z, int facing = 0);     //获取控制底盘移动的SDK命令
void echoCheck(void);
void move_with_wheels(float vx, float vy, float vw);
void match_3_dyp(float error);
void gou_3_dyp();
void get_pos(double an1_in, double an2_in, double big_arm = 89.55, double small_arm = 119.8);
void get_angles(double target_x, double target_y, double big_arm = 89.55, double small_arm = 119.8);
void go_line(double begin_x, double begin_y, double end_x, double end_y, double step=1);

//----------------------------------------------------------------------------------------------------------------------------------------------------


//任务函数及任务句柄声明---------------------------------------------------------------------------------------------------------------------------------------------
void TaskReset(void *Pvp);            //进入SDK模式，整车初始化
void TaskNonBlocking(void *Pvp);      //非阻塞任务
void TaskBan(void *Pvp);              //控制挡板升降
void TaskCamera(void *Pvp);           //控制第一图传转向
void TaskArm(void *Pvp);              //控制机械臂及储弹盒子
void TaskAuto(void *Pvp);             //阻塞程序
void TaskAutoBreak(void *Pvp);        //阻塞程序中断
void TaskSR04(void *Pvp);
TaskHandle_t TaskReset_Handle;
TaskHandle_t TaskAuto_Handle;
TaskHandle_t TaskAutoBreak_Handle;

//----------------------------------------------------------------------------------------------------------------------------------------------------


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);

  Serial.println("start...");

  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  type = ps2x.readType();

  xTaskCreate(TaskReset, "start the sdk mode", 300, NULL, 3, &TaskReset_Handle);
  xTaskCreate(TaskNonBlocking, "a non blocking task", 1200, NULL, 3, NULL);
  xTaskCreate(TaskBan, "control the ban", 300, NULL, 3, NULL);
  xTaskCreate(TaskCamera, "control the camera", 300, NULL, 3, NULL);
  xTaskCreate(TaskArm, "control the arm and the box", 500, NULL, 3, NULL);
  xTaskCreate(TaskAuto, "control the arm and the box", 1000, NULL, 3, &TaskAuto_Handle);
  xTaskCreate(TaskAutoBreak, "control the arm and the box", 500, NULL, 3, &TaskAutoBreak_Handle);
  xTaskCreate(TaskSR04, "control the arm and the box", 500, NULL, 3, NULL);

  vTaskStartScheduler();
}

void loop() {

}

void TaskNonBlocking(void *Pvp){
  int Xl, Yl;           //左摇杆数据
  int Xr, Yr;           //右摇杆数据
  float Xc, Yc, Zc;     //底盘数据
  float Xc1, Yc1, Zc1;  //底盘数据备份
  byte vibrate = 0;
  for(;;){
    if(type!=2){
      ps2x.read_gamepad(false, vibrate);      //获取手柄数据

      //摇杆数据------------------------------------------------
      Xl = ps2x.Analog(PSS_LX);
      Yl = ps2x.Analog(PSS_LY);
      Xr = ps2x.Analog(PSS_RX);
      Yr = ps2x.Analog(PSS_RY);
      //------------------------------------------------

      
      //底盘速度控制-------------------------------------------------------------
      vibrate = ps2x.Analog(PSAB_L1);
      if(ps2x.Button(PSB_L1)){       //按住L1疾跑
        tran_speed=1.2;
        // update_chassis=1;
      }else if(ps2x.Button(PSB_L2)){ //按住L2潜行
        tran_speed=0.2;
        rot_speed=20;
        // update_chassis=1;
      }else{                          //正常速度
        tran_speed=0.8;
        rot_speed=120;
      }
      //------------------------------------------------------------------------

      //底盘X方向（左摇杆Y方向）控制------------------------------------------------
      if(Yl<=0){
        Xc=tran_speed;
      }else if(Yl>=255){
        Xc=-1*tran_speed;
      }else{
        Xc=0;
      }
      //------------------------------------------------------------------------

      //底盘Y方向（左摇杆X方向）控制------------------------------------------------
      if(Xl<=0){
        Yc=-1*tran_speed;
      }else if(Xl>=255){
        Yc=tran_speed;
      }else{
        Yc=0;
      }
      //------------------------------------------------------------------------

      //底盘Z方向（右摇杆X方向）控制------------------------------------------------
      if(Xr<=0){
        Zc=-1*rot_speed;
      }else if(Xr>=255){
        Zc=rot_speed;
      }else{
        Zc=0;
      }
      //------------------------------------------------------------------------

      //发送消息到EP-------------------------------------------------------------
      if(Xc!=Xc1 || Yc!=Yc1 || Zc!=Zc1 || update_chassis==1){
        Serial1.print(getMoveString(Xc, Yc, Zc, facing));
        update_chassis=0;
      }
      //------------------------------------------------------------------------

      //更新数据备份-------------------------------------------------------------
      Xc1 = Xc;
      Yc1 = Yc;
      Zc1 = Zc;
      //------------------------------------------------------------------------

      //检测挡板控制按键、第一图传控制按键-----------------------------------------
      is_PAD_DOWN  = (ps2x.ButtonPressed(PSB_PAD_DOWN)) ? 1 : 0;
      is_PAD_UP    = (ps2x.ButtonPressed(PSB_PAD_UP)) ? 1 : 0;
      is_PAD_LEFT  = (ps2x.ButtonPressed(PSB_PAD_LEFT)) ? 1 : 0;
      is_PAD_RIGHT = (ps2x.ButtonPressed(PSB_PAD_RIGHT)) ? 1 : 0;
      is_PAD_DOWN_prs  = (ps2x.Button(PSB_PAD_DOWN)) ? 1 : 0;
      is_PAD_UP_prs    = (ps2x.Button(PSB_PAD_UP)) ? 1 : 0;
      is_PAD_LEFT_prs  = (ps2x.Button(PSB_PAD_LEFT)) ? 1 : 0;
      is_PAD_RIGHT_prs = (ps2x.Button(PSB_PAD_RIGHT)) ? 1 : 0;
      is_X = (ps2x.ButtonPressed(PSB_SQUARE)) ? 1 : 0;
      is_A = (ps2x.ButtonPressed(PSB_CROSS)) ? 1 : 0;
      is_B = (ps2x.ButtonPressed(PSB_CIRCLE)) ? 1 : 0;
      is_Y = (ps2x.ButtonPressed(PSB_TRIANGLE)) ? 1 : 0;
      is_R1 = (ps2x.ButtonPressed(PSB_R1)) ? 1 : 0;
      is_R2 = (ps2x.ButtonPressed(PSB_R2)) ? 1 : 0;
      is_R3 = (ps2x.ButtonPressed(PSB_R3)) ? 1 : 0;
      is_L3 = (ps2x.ButtonPressed(PSB_L3)) ? 1 : 0;
      is_SELECT = (ps2x.ButtonPressed(PSB_SELECT)) ? 1 : 0;
      is_START = (ps2x.ButtonPressed(PSB_START)) ? 1 : 0;
      //------------------------------------------------------------------------

      //串口返回信息-----------------------------------------------------
      // Serial2.println("Xc: "+String(Xc)+" "+
      //                "Yc: "+String(Yc)+" "+
      //                "Zc: "+String(Zc));

      // Serial2.println("Xl: "+String(Xl)+" "+
      //                "Yl: "+String(Yl)+" "+
      //                "Xr: "+String(Xr)+" "+
      //                "Yr: "+String(Yr));

      // Serial2.println("vibrate: "+String(vibrate));

      // Serial2.println("facing: "+String(facing));
      //------------------------------------------------------------------------

    }
    
    vTaskDelay(1);
  }
}

void TaskBan(void *Pvp){
  bool ban_sts = 0;
  for(;;){
    if(abs(facing)==2 && ban_sts==0){
      Serial1.print(get_djiservo_string(ban_servo_id, BAN_UP));
      ban_sts=1;
    }
    if(!is_trimming){
      //挡板控制-----------------------------------------------------------------
      if(is_PAD_DOWN==1){
        is_PAD_DOWN = 0;
        Serial2.println("PSAB_PAD_DOWN is pressed...");
        if(ban_sts==0){
          Serial1.print(get_djiservo_string(ban_servo_id, BAN_UP)); 
          ban_sts=1;
        }else if(ban_sts==1 && abs(facing)!=2){
          Serial1.print(get_djiservo_string(ban_servo_id, BAN_DOWN));
          ban_sts=0;
        }
        vTaskDelay(200/portTICK_PERIOD_MS);
      }
      //------------------------------------------------------------------------
    }
    vTaskDelay(1);
  }
}

void TaskCamera(void *Pvp){
  for(;;){
    //计算facing值---------------------
    if(is_A){    //180°扭头
      is_A=0;
      facing = (facing==0) ? 2 : 0;
      is_facing_change = 1;
    }else if(is_X){  //左扭头
      is_X=0;
      facing -= 1;
      is_facing_change = 1;
    }else if(is_B){  //右扭头
      is_B=0;
      facing += 1;
      is_facing_change = 1;
    }
    facing = constrain(facing, -1, 2);
    //--------------------------------

    //将facing数据转化为舵机控制--------
    if(is_facing_change){
      is_facing_change = 0;
      update_chassis = 1;
      Serial1.print(get_djiservo_string(head_servo_id, facing*89));
      vTaskDelay(200/portTICK_PERIOD_MS);
    }
    //--------------------------------
    vTaskDelay(1);
  }
}

void TaskArm(void *Pvp){
  for(;;){
    uservo0.setSpeed(360);
    uservo1.setSpeed(360);
    uservo2.setSpeed(360);
    uservo3.setSpeed(360);
    man_get_dyp();
    direct_servo();
    unload();
    vTaskDelay(1);
  }
}

void TaskSR04(void *pvParameters){
  for(;;){
    if (millis() >= pingTimer) {   // pingSpeed milliseconds since last ping, do another ping.
      pingTimer += pingSpeed;      // Set the next ping time.
      sonar1.ping_timer(echoCheck); // Send out the ping, calls "echoCheck" function every 24uS where you can check the ping status.
      Serial2.println(dis1);
    }
    vTaskDelay(1);
  }
}

void TaskAuto(void *Pvp){
  for(;;){
    if(!is_trimming){
      if(is_PAD_LEFT){
        //到达资源岛前
        Serial1.print(getMoveString(1, 0, 0));
        vTaskDelay(800/portTICK_PERIOD_MS);
        Serial1.print(getMoveString(0, 0, 90));
        vTaskDelay(450/portTICK_PERIOD_MS);
        Serial1.print(getMoveString(1, 0, 0));
        vTaskDelay(100/portTICK_PERIOD_MS);
        Serial1.print(getMoveString(1.9, -2, 0));
        vTaskDelay(900/portTICK_PERIOD_MS);
        Serial1.print(getMoveString(0, -0.5, 0));
      }else if(is_PAD_UP){
        Serial1.print(getMoveString(-0.5, 0, 0));
        vTaskDelay(300/portTICK_PERIOD_MS);
        move_with_wheels(0,0,0);
        //准备钩上方弹药瓶
        uservo0.setAngle(-20);
        uservo1.setAngle(0);
        uservo2.setAngle(-90);
        uservo3.setAngle(CLOSE_DYP_AN);
        vTaskDelay(50/portTICK_PERIOD_MS);
        //调整位置
        Serial1.print("chassis wheel w1 100 w2 100 w3 100 w4 100;");
        vTaskDelay(2000/portTICK_PERIOD_MS);
        Serial1.print(getMoveString(0, 0, 0));
      }else if(is_PAD_RIGHT){
        uservo0.setSpeed(1080);    //设置0舵机速度
        uservo1.setSpeed(1080);    //设置1舵机速度
        uservo2.setSpeed(1080);    //设置2舵机速度
        uservo3.setSpeed(1080);    //设置2舵机速度
        //摆好机械臂位置   
        uservo0.setAngle(8);
        vTaskDelay(100/portTICK_PERIOD_MS);
        uservo3.setAngle(-2);
        vTaskDelay(100/portTICK_PERIOD_MS);
        uservo1.setAngle(-100);
        vTaskDelay(300/portTICK_PERIOD_MS);
        uservo0.setAngle(-20);
        uservo1.setAngle(-90);
        uservo2.setAngle(-90);
        vTaskDelay(200/portTICK_PERIOD_MS);
        float error = firstDYP-dis1;
        float ok_err = 1;
        ///---------接近初始位--------////////////////////////////////////////////////////////
        Serial1.print(getMoveString(0.1, -0.4, 0));
        while(!(dis1<=20))vTaskDelay(1);
        Serial1.print(getMoveString(0, 0, 0));
        vTaskDelay(40/portTICK_PERIOD_MS);
        Serial1.print("chassis wheel w1 100 w2 100 w3 100 w4 100;");
        vTaskDelay(500/portTICK_PERIOD_MS);
        move_with_wheels(0,0,0);
        vTaskDelay(50/portTICK_PERIOD_MS);
        ///---------第一个3号---------////////////////////////////////////////////////////////
        //底盘超声波P控制
        error = firstDYP-dis1;
        while(!(abs(error)<=ok_err)){
          error = firstDYP-dis1;
          match_3_dyp(error);
          vTaskDelay(40/portTICK_PERIOD_MS);
        }
        Serial1.print("chassis wheel w2 50 w1 50 w3 50 w4 50;");
        vTaskDelay(200/portTICK_PERIOD_MS);
        //钩
        gou_3_dyp();
        move_with_wheels(0,0,0);
        vTaskDelay(50/portTICK_PERIOD_MS);
        ///---------第二个3号---------////////////////////////////////////////////////////////
        //底盘超声波P控制
        error = secondDYP-dis1;
        while(!(abs(error)<=ok_err)){
          error = secondDYP-dis1;
          match_3_dyp(error);
          vTaskDelay(40/portTICK_PERIOD_MS);
        }
        Serial1.print("chassis wheel w2 50 w1 50 w3 50 w4 50;");
        vTaskDelay(200/portTICK_PERIOD_MS);
        //钩
        gou_3_dyp();
        move_with_wheels(0,0,0);
        vTaskDelay(50/portTICK_PERIOD_MS);
        ///---------第三个3号---------////////////////////////////////////////////////////////
        //底盘超声波P控制
        error = thirdDYP-dis1;
        while(!(abs(error)<=ok_err)){
          error = thirdDYP-dis1;
          match_3_dyp(error);
          vTaskDelay(40/portTICK_PERIOD_MS);
        }
        Serial1.print("chassis wheel w2 50 w1 50 w3 50 w4 50;");
        vTaskDelay(200/portTICK_PERIOD_MS);
        //钩
        gou_3_dyp();
        move_with_wheels(0,0,0);
        vTaskDelay(50/portTICK_PERIOD_MS);
        //摆好机械臂
        uservo0.setDamping(0);
        uservo1.setDamping(0);
        uservo2.setDamping(0);
        uservo3.setDamping(0);
        //后退
        Serial1.print(getMoveString(-1, 0, 0));
        vTaskDelay(500/portTICK_PERIOD_MS);
        Serial1.print(getMoveString(0, 0, 0));
        uservo0.setSpeed(1080);    //设置0舵机速度
        uservo1.setSpeed(1080);    //设置1舵机速度
        uservo2.setSpeed(1080);    //设置2舵机速度
        uservo3.setSpeed(1080);    //设置2舵机速度
        uservo0.setAngle(BIG_UP);
        uservo1.setAngle(SMALL_UP);
        uservo2.setAngle(0);
        uservo3.setAngle(OPEN_DYP_AN);
      }
    }
    vTaskDelay(1);
  }
}

void TaskAutoBreak(void *Pvp){
  for(;;){
    if(is_R3){
      vTaskDelete(TaskAuto_Handle);
      Serial1.print(getMoveString(0,0,0));
      uservo0.setDamping(0);
      uservo1.setDamping(0);
      uservo2.setDamping(0);
      uservo3.setDamping(0);
      vTaskDelay(1);
      xTaskCreate(TaskAuto, "control the arm and the box", 1000, NULL, 3, &TaskAuto_Handle);
    }
    vTaskDelay(1);
  }
}

void TaskReset(void *Pvp){                
  // vTaskDelay(12000/portTICK_PERIOD_MS);
  while(digitalRead(27)==HIGH);
  while(digitalRead(27)==LOW);
  //进入SDK模式
  Serial2.println("reset reset reset reset reset");
  Serial1.print("command;");
  vTaskDelay(5);
  Serial1.print("game_msg on;");
  vTaskDelay(5);
  #pragma region
  uservo0.setSpeed(360);
  uservo1.setSpeed(360);
  uservo2.setSpeed(360);
  uservo3.setSpeed(360);
  uservo0.setAngle(BIG_UP);
  vTaskDelay(500/portTICK_PERIOD_MS);
  uservo1.setAngle(10);
  vTaskDelay(500/portTICK_PERIOD_MS);
  Serial1.print(get_djiservo_string(head_servo_id, 0));
  vTaskDelay(50/portTICK_PERIOD_MS);
  Serial1.print(get_djiservo_string(box_servo_id, BOX_DOWN));
  vTaskDelay(50/portTICK_PERIOD_MS);
  Serial1.print(get_djiservo_string(ban_servo_id, BAN_UP));
  vTaskDelay(500/portTICK_PERIOD_MS);
  Serial1.print(get_djiservo_string(ban_servo_id, BAN_DOWN));
  vTaskDelay(500/portTICK_PERIOD_MS);
  uservo0.setAngle(BIG_UP);
  uservo1.setAngle(SMALL_UP);
  uservo2.setAngle(0);
  uservo3.setAngle(OPEN_DYP_AN);
  vTaskDelay(500/portTICK_PERIOD_MS);
  uservo1.setDamping(0);
  #pragma endregion
  arm_sts = 2;
  box_sts = 0;
  facing = 0;
  vTaskDelete(TaskReset_Handle);
}

//普通函数定义----------------------------------------------------------------------------------------------------------------------------------------------------------

void man_get_dyp(){
  if(box_sts == 0){   //机械臂处于空闲时或手动夹取时（就是当前状态），执行以下操作
    if(is_R1 && arm_sts != 0){
      if(arm_sts==1){
        uservo2.setAngle(0);
        uservo3.setAngle(OPEN_DYP_AN);

        uservo1.setAngle(-120);
        vTaskDelay(80/portTICK_PERIOD_MS);

        uservo0.setAngle(85);
        uservo1.setAngle(SMALL_DYP);
      }else{
        uservo2.setAngle(0);
        uservo3.setAngle(OPEN_DYP_AN);

        uservo0.setAngle(85);
        vTaskDelay(50/portTICK_PERIOD_MS);

        uservo1.setAngle(-120);
        vTaskDelay(50/portTICK_PERIOD_MS);
        
        uservo1.setAngle(SMALL_DYP);
      }
      arm_sts=0;
    }else if(is_Y && arm_sts!=1){
      arm_sts=1;

      uservo0.setAngle(-70);
      vTaskDelay(100/portTICK_PERIOD_MS);

      uservo1.setAngle(SMALL_DDW);
      uservo2.setAngle(0);
      uservo3.setAngle(OPEN_DYP_AN);
    }else if(is_R2 && arm_sts!=2){
      if(arm_sts==0){
        uservo2.setAngle(0);
        uservo3.setAngle(CLOSE_DYP_AN);
        vTaskDelay(150/portTICK_PERIOD_MS);
        
        uservo1.setAngle(-120);
        vTaskDelay(100/portTICK_PERIOD_MS);

        uservo0.setAngle(BIG_UP);
        vTaskDelay(300/portTICK_PERIOD_MS);

        uservo1.setAngle(SMALL_UP);
        vTaskDelay(200/portTICK_PERIOD_MS);
        
        uservo3.setAngle(OPEN_DYP_AN);
      }else if(arm_sts==1){
        uservo2.setAngle(0);
        uservo3.setAngle(CLOSE_DDW_AN);
        vTaskDelay(150/portTICK_PERIOD_MS);

        uservo1.setAngle(-100);
        vTaskDelay(300/portTICK_PERIOD_MS);

        uservo0.setAngle(-64);
        vTaskDelay(100/portTICK_PERIOD_MS);
        
        uservo1.setAngle(0);
        vTaskDelay(150/portTICK_PERIOD_MS);

        uservo3.setAngle(OPEN_DYP_AN);
        vTaskDelay(100/portTICK_PERIOD_MS);

        uservo0.setAngle(BIG_UP);
        vTaskDelay(100/portTICK_PERIOD_MS);

        uservo1.setAngle(SMALL_UP);
      }else{
        uservo2.setAngle(0);
        uservo3.setAngle(CLOSE_DYP_AN);
        vTaskDelay(150/portTICK_PERIOD_MS);
        
        uservo1.setAngle(-120);
        vTaskDelay(100/portTICK_PERIOD_MS);

        uservo0.setAngle(BIG_UP);
        vTaskDelay(200/portTICK_PERIOD_MS);

        uservo1.setAngle(SMALL_UP);
        vTaskDelay(200/portTICK_PERIOD_MS);
        
        uservo3.setAngle(OPEN_DYP_AN);
      }          
      arm_sts=2;
      vTaskDelay(6);
      uservo1.setDamping(0);
    }
  }
}

void unload(){
  if(is_START){
    if(box_sts==0){    //按下按键，倾倒
      box_sts = 1;
      uservo0.setAngle(-90);
      uservo1.setAngle(10);
      vTaskDelay(500/portTICK_PERIOD_MS);
      Serial1.print(get_djiservo_string(box_servo_id, BOX_UP));         
    }else if(box_sts==1){    //按下按键，平放
      box_sts = 0;
      Serial1.print(get_djiservo_string(box_servo_id, BOX_DOWN));  
      vTaskDelay(600/portTICK_PERIOD_MS);
      uservo0.setAngle(BIG_UP);
      uservo1.setAngle(SMALL_UP);
    }
  }
}

void direct_servo(){    //直接控制舵机旋转，优先级最低
  float jzangle = uservo3.queryAngle(), sangle = uservo1.queryAngle(), bangle = uservo0.queryAngle();
  uservo0.setSpeed(1080);
  uservo1.setSpeed(1080);
  uservo2.setSpeed(1080);
  uservo3.setSpeed(1080);
  if(is_L3)is_trimming = (is_trimming==0) ? 1 : 0;
  if(is_trimming && arm_sts!=2){
    //夹爪
    if(is_PAD_LEFT_prs){            //夹爪张开
      jzangle += 10;
      cID = 1;
      uservo3.setAngle(jzangle);
    }else if(is_PAD_RIGHT_prs){     //夹爪闭合
      jzangle -= 10;
      cID = 1;
      uservo3.setAngle(jzangle);
    }
    //小臂
    if(is_PAD_DOWN_prs){            //小臂下降
      sangle += 3;
      cID = 2;
      uservo1.setAngle(sangle);
    }else if(is_PAD_UP_prs){        //小臂抬升
      sangle -= 5;
      cID = 2;
      uservo1.setAngle(sangle);
    }
    if(cID==2){
      if(arm_sts==0){
        SMALL_DYP = sangle;
      }else if(arm_sts==1){
        SMALL_DDW = sangle;
      }
      arm_sts=(arm_sts==1 || arm_sts==0) ? arm_sts : 3;
      cID = 0;
    }
    if(is_SELECT && cID==1){
      if(arm_sts==0){
        CLOSE_DYP_AN = jzangle;
      }else if(arm_sts==1){
        CLOSE_DDW_AN = jzangle;
      }
      arm_sts=(arm_sts==1 || arm_sts==0) ? arm_sts : 3;
      cID = 0;
    }
  }
}  

String get_djiservo_string(int id, int degrees){
  String res = "servo angle id "+String(id)+" angle "+String(degrees)+";";
  return res;
}

String getMoveString(double x, double y, double z, int facing = 0){
  String result;
  facing = (facing == -2) ? 2 : facing;
  facing = (facing == 1) ? 0 : facing;
  switch(facing){
    case 0:
      result = "chassis speed x " + String(x) + " y " + String(y) + " z " + String(z) + ";";
      break;
    case 1:
      result = "chassis speed x " + String(0-y) + " y " + String(x) + " z " + String(z) + ";";
      break;
    case 2:
      result = "chassis speed x " + String(0-x) + " y " + String(0-y) + " z " + String(z) + ";";
      break;
    case -1:
      result = "chassis speed x " + String(y) + " y " + String(0-x) + " z " + String(z) + ";";
      break;
  }
  return result;
}

void echoCheck(){
  if (sonar1.check_timer()){
    dis1 = sonar1.ping_result / US_ROUNDTRIP_CM;
  }
}

void move_with_wheels(float vx, float vy, float vw){
  //明文sdk轮子速度控制
  //////////
  //2    1//
  //      //
  //3    4//
  ////////////向EP发送底盘数据////////////
  Serial1.print("chassis wheel w1 "+String(vy-vx-vw)+" w2 "+String(vy+vx+vw)+" w3 "+String(vy-vx+vw)+" w4 "+String(vy+vx-vw)+";");
}

void match_3_dyp(float error){
  //底盘超声波P控制
  double P = constrain(error*5, -100, 100);
  int lim = 20;
  if(abs(P)<lim)P=(P>=0) ? lim : lim*(-1);
  move_with_wheels(P, 0, 0);
}

void gou_3_dyp(){
  uservo0.setSpeed(1080);
  uservo1.setSpeed(1080);
  get_pos(uservo0.queryAngle()*(-1), uservo1.queryAngle()*(-1));
  go_line(t_pos[0], t_pos[1], t_pos[0]-90, t_pos[1]+60, 5);
  vTaskDelay(200/portTICK_PERIOD_MS);
  //摆好机械臂位置
  uservo0.setAngle(-20);
  uservo1.setAngle(-90);
  uservo3.setAngle(-2);
  vTaskDelay(200/portTICK_PERIOD_MS);
}

void get_pos(double an1_in, double an2_in, double big_arm = 89.55, double small_arm = 119.8){    //传入末端
  double an1_in_rad, an2_in_rad;
  an1_in_rad = an1_in / 180 * M_PI;
  an2_in_rad = an2_in / 180 * M_PI;
  t_pos[0] = (cos(an1_in_rad)*big_arm+cos(an1_in_rad+an2_in_rad)*small_arm);
  t_pos[1] = (sin(an1_in_rad)*big_arm+sin(an1_in_rad+an2_in_rad)*small_arm);
  Serial.println("an1_in:"+String(an1_in)+"an2_in:"+String(an2_in)+" x:"+String(t_pos[0])+" y:"+String(t_pos[1]));
}

void get_angles(double target_x, double target_y, double big_arm = 89.55, double small_arm = 119.8){    //传入末端
  double c, an1, an2;
  c = sqrt((target_x*target_x)+(target_y*target_y));
  an1 = atan2(target_y, target_x);
  an2 = acos((big_arm*big_arm+c*c-small_arm*small_arm)/(2*big_arm*c));
  t_an[1] = (180-(acos((big_arm*big_arm+small_arm*small_arm-c*c)/(2*big_arm*small_arm))*(180/M_PI)))*(-1);
  t_an[0] = (an1 - an2)*(180/M_PI)*(-1);
}

void go_line(double begin_x, double begin_y, double end_x, double end_y, double step=1){
  double c, point_x, point_y;
  int times;
  c = sqrt((begin_x-end_x)*(begin_x-end_x)+(begin_y-end_y)*(begin_y-end_y));    //线段长度
  times = c/step;   //点的数量，不计第一个
  get_angles(begin_x, begin_y);
  uservo0.setAngle(t_an[0]);
  uservo1.setAngle(t_an[1]);
  vTaskDelay(15/portTICK_PERIOD_MS);
  for(double i = 1; i <= times; i++){
    point_x = (double)(i/times)*(double)(end_x-begin_x)+begin_x;
    point_y = (double)(i/times)*(double)(end_y-begin_y)+begin_y;
    get_angles(point_x, point_y);
    uservo0.setAngle(t_an[0]);
    uservo1.setAngle(t_an[1]);
    vTaskDelay(15/portTICK_PERIOD_MS);
  }
}