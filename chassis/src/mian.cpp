/* str_in是ep发来的字符串（完整字符串是有";"的,但是由于读取方式所以str_in没有";"）
例: str_in可为"game msg push [0, 6, 0, 0, 0, 243, 3, 87, 88, 89]"
*/

/*检查任务堆栈水位线
if(SerOc == 0){
  SerOc = 1;
  int memfree = uxTaskGetStackHighWaterMark(NULL);
  Serial.println("free: "+String(memfree)+" used: "+String(800-memfree));
  SerOc = 0;
}
*/
#include <Arduino.h>
#include <string.h>
#include <math.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include "FashionStar_UartServoProtocol.h" // 串口总线舵机通信协议
#include "FashionStar_UartServo.h" // Fashion Star串口总线舵机的依赖
#include <SPI.h>
#include <SD.h>
#include <NewPing.h>
#include <avr/wdt.h>

#define SERVO_ID0 0 //舵机ID号
#define SERVO_ID1 1 
#define head_servo_id 1 //图传舵机
#define ban_servo_id 2  //挡板舵机
#define box_servo_id 3  //储弹盒舵机

#define BOX_UP -150     //储弹盒子抬起角度
#define BOX_DOWN 25     //储弹盒子放下角度
#define BAN_UP 115      //挡板抬起角度
#define BAN_DOWN 10     //挡板放下角度
#define BAUDRATE 115200 // 波特率
FSUS_Protocol protocol(BAUDRATE);       //协议
FSUS_Servo uservo0(0, &protocol); // 创建大臂舵机
FSUS_Servo uservo1(1, &protocol); // 创建小臂舵机
FSUS_Servo uservo2(2, &protocol); // 创建手腕舵机
FSUS_Servo uservo3(3, &protocol); // 创建夹爪舵机

//定义底盘平移和旋转速度
#define mid_speed 0.8;
#define high_speed 1.2;
#define low_speed 0.2;
#define rotate_speed_1 20;
#define rotate_speed_2 40;
#define rotate_speed_3 60;
#define rotate_speed_4 90;
#define rotate_speed_5 100;

//定义底盘麦轮控制速度（屁用没有）
#define low_wheel_speed 50;
#define mid_wheel_speed 150;
#define high_wheel_speed 150;
#define rotate_add_1 20;
#define rotate_add_2 70;

#define TRIG 2
#define ECHO 3
#define MAX_DISTANCE 200
#define firstDYP 15     //第一个三号瓶位置超声波的距离
#define secondDYP 29    //第二个三号瓶位置超声波的距离
#define thirdDYP 46     //第三个三号瓶位置超声波的距离
unsigned int pingSpeed = 100;
unsigned long pingTimer;
double dis1          = 0; //一号超声波传感器检测值
double dis2          = 0; //一号超声波传感器检测值（屁用没有）
double dis3          = 0; //一号超声波传感器检测值（屁用没有）
NewPing sonar1(TRIG, ECHO, MAX_DISTANCE);
void echoCheck(){
  if (sonar1.check_timer()){
    dis1 = sonar1.ping_result / US_ROUNDTRIP_CM;
  }
}

int32_t OPEN_DYP_AN   = -70;  //字面意思
int32_t CLOSE_DYP_AN  = -130; //字面意思
int32_t CLOSE_DDW_AN  = -110; //字面意思
int32_t SMALL_DDW     =  60;  //夹大弹丸时小臂舵机角度
int32_t SMALL_DYP     = -91;  //夹弹药瓶时小臂舵机角度
int32_t BIG_UP        = -90;  //收起夹爪时大臂舵机角度
int32_t SMALL_UP      = -80;  //收起夹爪时小臂舵机角度
bool ban_sts = 0;             //挡板状态，我记得“0”代表放下，“1”代表抬起

String str_in0 = "";  //上一帧消息
String str_in1 = "";  //这一帧消息
//下面这两行超长♂的是所有键在上一时刻和这一时刻按下与否的状态变量
bool SPACE=0, TAB=0, LSHIFT=0, LCTRL=0, LATL=0, CAP=0, ZERO=0, ONE=0, TWO=0, THREE=0, FOUR=0, FIVE=0, SIX=0, SEVEN=0, EIGHT=0, NINE=0, A=0, B=0, C=0, D=0, E=0, F=0, G=0, H=0, I=0, J=0, K=0, L=0, M=0, N=0, O=0, P=0, Q=0, R=0, S=0, T=0, U=0, V=0, W=0, X=0, Y=0, Z=0,
     rSPACE=0, rTAB=0, rLSHIFT=0, rLCTRL=0, rLATL=0, rCAP=0, rZERO=0, rONE=0, rTWO=0, rTHREE=0, rFOUR=0, rFIVE=0, rSIX=0, rSEVEN=0, rEIGHT=0, rNINE=0, rA=0, rB=0, rC=0, rD=0, rE=0, rF=0, rG=0, rH=0, rI=0, rJ=0, rK=0, rL=0, rM=0, rN=0, rO=0, rP=0, rQ=0, rR=0, rS=0, rT=0, rU=0, rV=0, rW=0, rX=0, rY=0, rZ=0;
//下面这两行超短♂的是鼠标按键在上一时刻和这一时刻按下与否的状态变量，但是屁用没有
uint32_t MOUSE_PRESS=0, MOUSE_X=0, MOUSE_Y=0,
         rMOUSE_PRESS=0, rMOUSE_X=0, rMOUSE_Y=0;
bool isJUE          = 0;  //是否开启jue模式，但是因为DJI屁用没有了
bool isChange       = 0;  //图传舵机是否要旋转
bool update         = 0;  //底盘是否更新（比如说图传转向后底盘也要跟着换向，此时update会变成1，底盘更新后update变回0）
bool SerOc = 0, SerOc1 = 0; //emmm，大概就是代表是否有任务要用串口的状态变量，以前为了解决发疯乱搞的，现在屁用没有，但是可能有残留在一些不常用函数中
int32_t disChassis  = 0;    //是否关闭手动底盘控制（1表示关闭底盘手动控制，0就是解锁底盘手动控制）
int32_t facing      = 0;    //表示图传面向的方向，0：正面 1：右面 2：后面 -2：后面 -1:左面，为什么后面有2和-2呢？因为我也不知道
double t_an[2]      = {0,90};       //逆解后得到的舵机角度值，第一个是大臂，第二个是小臂
double t_pos[2]      = {209.35,0};  //正解后得到的机械臂末端坐标

TaskHandle_t TaskSerial_Handle;
TaskHandle_t TaskArm_Handle;
TaskHandle_t TaskTurnCamera_Handle;
TaskHandle_t TaskTurnBody_Handle;
TaskHandle_t TaskTurnBan_Handle;
TaskHandle_t TaskAuto_Handle;
TaskHandle_t TaskAutoBreak_Handle;
TaskHandle_t TaskSR04_Handle;
TaskHandle_t TaskReset_Handle;
TaskHandle_t TaskQuit_Handle;
void TaskSerial(void *pvParameters);
void TaskArm(void *pvParameters);
void TaskTurnCamera(void *pvParameters);
void TaskTurnBody(void *pvParameters);
void TaskTurnBan(void *pvParameters);
void TaskAuto(void *pvParameters);
void TaskAutoBreak(void *pvParameters);
void TaskSR04(void *pvParameters);
void TaskReset(void *pvParameters);
void TaskQuit(void *pvParameters);


String get_djiservo_string(int id, int degrees)   //获取控制DJI舵机的角度信息
{
  String res = "servo angle id "+String(id)+" angle "+String(degrees)+";";
  return res;
}
String getMoveString(double x, double y, double z, int facing = 0)    //获取手动控制底盘的信息（结合了facing数值）
{
  String result;
  facing = (facing == -2) ? 2 : facing;
  facing = (facing == 1) ? 0 : facing;
  y = (y!=0) ? y*1.3 : y;
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
bool checkArray(int arr[], int x)   //这。。。可能是用来检测数据中是否有键值，并根据结果肤质给键值状态变量
{
  for(int i = 0; i < 10; i++) {
    if(arr[i] == x)return true;
  }
  return false;
}
//运动函数，传入键值数组，处理其中关于移动的键值，转化为运动参数，采用chassis speed x 0.1 y 0.1 z 1;格式
void move_with_xyz()    //手动控制底盘
{
  float moveArr[20]  = {0, 0, 0, 0};
  float temp_speed=0, temp_rotate_speed=0;
  memset(moveArr, 0, sizeof(moveArr));
  //判断疾跑/潜行
  if(LCTRL == 1){
    temp_speed = high_speed;
    temp_rotate_speed = rotate_speed_4;
  }else if(LSHIFT == 1){
    temp_speed = low_speed;
    temp_rotate_speed = rotate_speed_1;
  }else{
    temp_speed = mid_speed;
    temp_rotate_speed = rotate_speed_4;
  }
  //自动手操，将键值转化为运动数组的内容
  if(isJUE==1){
    if(TWO == 1 && R == 0)  moveArr[0] = temp_speed;
    if(R == 1 && TWO == 0)  moveArr[0] = 0-temp_speed;
    if(Q == 1 && E == 0)    moveArr[1] = 0-temp_speed;
    if(E == 1 && Q == 0)    moveArr[1] = temp_speed;
    if(ONE == 1 && THREE == 0)  moveArr[2] = 0-temp_rotate_speed;
    if(THREE == 1 && ONE == 0)  moveArr[2] = temp_rotate_speed;
    //判断键值是否和上一帧键值不同，如果是则发送新的数据
    if(disChassis==0){
      if((TWO != rTWO) || (Q != rQ) || (E != rE) || (ONE != rONE) || (THREE != rTHREE) || (R != rR) || (update == 1)){
        Serial1.print(getMoveString(moveArr[0], moveArr[1], moveArr[2], facing));
        update = 0;
      }
    }
    
  }else if(isJUE==0){
    //手动手操，将键值转化为运动数组的内容
    if(W == 1 && S == 0)  moveArr[0] = temp_speed;
    if(S == 1 && W == 0)  moveArr[0] = 0-temp_speed;
    if(A == 1 && D == 0)  moveArr[1] = 0-temp_speed;
    if(D == 1 && A == 0)  moveArr[1] = temp_speed;
    if(J == 1 && L == 0)  moveArr[2] = 0-temp_rotate_speed;
    if(L == 1 && J == 0)  moveArr[2] = temp_rotate_speed;   
    //判断键值是否和上一帧键值不同，如果是则发送新的数据
    if(disChassis==0){
      if((W != rW) || (A != rA) || (S != rS) || (D != rD) || (J != rJ) || (L != rL) || (LCTRL != rLCTRL) || (LSHIFT != rLSHIFT) || (update == 1)){
        Serial1.print(getMoveString(moveArr[0], moveArr[1], moveArr[2], facing));
        update = 0;
      }
    }
  } 
}
void move_with_wheels(float vx, float vy, float vw)   //屁用没有
{
  //明文sdk轮子速度控制
  //////////
  //2    1//
  //      //
  //3    4//
  ////////////向EP发送底盘数据////////////
  Serial1.print("chassis wheel w1 "+String(vy-vx-vw)+" w2 "+String(vy+vx+vw)+" w3 "+String(vy-vx+vw)+" w4 "+String(vy+vx-vw)+";");
}
void get_pos(double an1_in, double an2_in, double big_arm = 89.55, double small_arm = 119.8)    //机械臂正解
{
  double an1_in_rad, an2_in_rad;
  an1_in_rad = an1_in / 180 * M_PI;
  an2_in_rad = an2_in / 180 * M_PI;
  t_pos[0] = (cos(an1_in_rad)*big_arm+cos(an1_in_rad+an2_in_rad)*small_arm);
  t_pos[1] = (sin(an1_in_rad)*big_arm+sin(an1_in_rad+an2_in_rad)*small_arm);
  Serial.println("an1_in:"+String(an1_in)+"an2_in:"+String(an2_in)+" x:"+String(t_pos[0])+" y:"+String(t_pos[1]));
}
void get_angles(double target_x, double target_y, double big_arm = 89.55, double small_arm = 119.8)   //机械臂逆解
{
  double c, an1, an2;
  c = sqrt((target_x*target_x)+(target_y*target_y));
  an1 = atan2(target_y, target_x);
  an2 = acos((big_arm*big_arm+c*c-small_arm*small_arm)/(2*big_arm*c));
  t_an[1] = (180-(acos((big_arm*big_arm+small_arm*small_arm-c*c)/(2*big_arm*small_arm))*(180/M_PI)))*(-1);
  t_an[0] = (an1 - an2)*(180/M_PI)*(-1);
}
void go_line(double begin_x, double begin_y, double end_x, double end_y, double step=1)     //机械臂末端走直线（有bug）
{   
  double c, point_x, point_y;
  int times;
  c = sqrt((begin_x-end_x)*(begin_x-end_x)+(begin_y-end_y)*(begin_y-end_y));    //线段长度
  times = c/step;   //点的数量，不计第一个
  get_angles(begin_x, begin_y);
  uservo0.setSpeed(360);
  uservo1.setSpeed(360);
  uservo0.setAngle(t_an[0]);
  uservo1.setAngle(t_an[1]);
  vTaskDelay(1);
  for(double i = 1; i <= times; i++){
    point_x = (double)(i/times)*(double)(end_x-begin_x)+begin_x;
    point_y = (double)(i/times)*(double)(end_y-begin_y)+begin_y;
    get_angles(point_x, point_y);
    uservo0.setAngle(t_an[0]);
    uservo1.setAngle(t_an[1]);
    vTaskDelay(1);
  }
}
void match_3_dyp(float error)   //对准3号弹药瓶（改崩了，屁用没有）
{
  //底盘超声波P控制
  double P = (error>=0) ? 15 : -15;
  uint8_t lim = 10;
  if(abs(P)<lim) P=(P>=0) ? lim : lim*(-1);
  move_with_wheels(P, 0, 0);
  // Serial.println(error);
}
void gou_3_dyp()    //钩取3号弹药瓶（有bug）
{
  get_pos(uservo0.queryAngle()*(-1), uservo1.queryAngle()*(-1));
  go_line(t_pos[0], t_pos[1], t_pos[0]-90, t_pos[1]+60, 3);
  vTaskDelay(200/portTICK_PERIOD_MS);
  //摆好机械臂位置
  uservo0.setAngle(-20);
  uservo1.setAngle(-90);
  uservo3.setAngle(-2);
  while(!(uservo0.isStop()&&uservo1.isStop()&&uservo3.isStop()));
}
void init_all()   //开机按按钮后的初始化程序
{
  uservo0.setSpeed(360);
  uservo1.setSpeed(360);
  uservo2.setSpeed(360);
  uservo3.setSpeed(360);
  vTaskDelay(500/portTICK_PERIOD_MS);
  Serial1.print(get_djiservo_string(head_servo_id, 0));
  vTaskDelay(50/portTICK_PERIOD_MS);
  Serial1.print(get_djiservo_string(ban_servo_id, BAN_UP));
  vTaskDelay(500/portTICK_PERIOD_MS);
  Serial1.print(get_djiservo_string(ban_servo_id, BAN_DOWN));
  vTaskDelay(50/portTICK_PERIOD_MS);
  Serial1.print(get_djiservo_string(box_servo_id, BOX_DOWN));
  vTaskDelay(1000/portTICK_PERIOD_MS);
  uservo0.setAngle(BIG_UP);
  uservo1.setAngle(SMALL_UP);
  uservo2.setAngle(0);
  uservo3.setAngle(OPEN_DYP_AN);
  // uservo1.setDamping(0);
  vTaskDelay(50/portTICK_PERIOD_MS);
}
class p_data{   //定义键值类，数据成员有鼠标键值、鼠标移动、包序号、按键数、按键值数组（闲哥出品，必属精品）
  public:
  String  str_raw;
  int cmd_id, len, mouse_press, mouse_x, mouse_y, seq, key_num, key_vals[20], values[20];
  p_data(String str_init){//初始化函数获取SDK字符串并解析成各个数据
    str_raw = String(str_init);
    // Serial.println("ok?"+String(str_init));
    if(str_init.startsWith("game") && str_init.indexOf('game',5)==-1){//长度够长的字符串才被认为是键值数据
      // Serial.println(str_init);
      
      int count = 0;
      int start = str_init.indexOf("[") + 1;
      int end = str_init.indexOf("]");
      String substr = str_init.substring(start, end);
      int pos = substr.indexOf(",");
      while (pos >= 0) {
        String num = substr.substring(0, pos);
        values[count] = num.toInt();
        count++;
        substr = substr.substring(pos + 2);
        pos = substr.indexOf(",");
      }
      values[count] = substr.toInt();

      cmd_id      = values[0];
      if(cmd_id == 0){
        len         = values[1];
        mouse_press = values[2];
        mouse_x     = values[3];
        mouse_y     = values[4];
        seq         = values[5];
        key_num     = values[6];
        if(key_num == 0){
          key_vals[0] = 0;
          key_vals[1] = 0;
          key_vals[2] = 0;
          key_vals[3] = 0;
        }else if(key_num == 1){
          key_vals[0] = values[7];
          key_vals[1] = 0;
          key_vals[2] = 0;
          key_vals[3] = 0;
        }else if(key_num == 2){
          key_vals[0] = values[7];
          key_vals[1] = values[8];
          key_vals[2] = 0;
          key_vals[3] = 0;
        }else if(key_num == 3){
          key_vals[0] = values[7];
          key_vals[1] = values[8];
          key_vals[2] = values[9];
          key_vals[3] = 0;
        }else if(key_num == 4){
          key_vals[0] = values[7];
          key_vals[1] = values[8];
          key_vals[2] = values[9];
          key_vals[3] = values[10];
        }
      }
    }else{
      
    }
  }
  void key_init(int key_vals[], int mouse_press, int mouse_x, int mouse_y, bool r=1){
    if(r==1){
      SPACE = checkArray(key_vals, 8) ? 1 : 0;
      TAB = checkArray(key_vals, 9) ? 1 : 0;
      LSHIFT = checkArray(key_vals, 16) ? 1 : 0;
      LCTRL = checkArray(key_vals, 17) ? 1 : 0;
      LATL = checkArray(key_vals, 18) ? 1 : 0;
      CAP = checkArray(key_vals, 20) ? 1 : 0;
      ZERO = checkArray(key_vals, 48) ? 1 : 0;
      ONE = checkArray(key_vals, 49) ? 1 : 0;
      TWO = checkArray(key_vals, 50) ? 1 : 0;
      THREE = checkArray(key_vals, 51) ? 1 : 0;
      FOUR = checkArray(key_vals, 52) ? 1 : 0;
      FIVE = checkArray(key_vals, 53) ? 1 : 0;
      SIX = checkArray(key_vals, 54) ? 1 : 0;
      SEVEN = checkArray(key_vals, 55) ? 1 : 0;
      EIGHT = checkArray(key_vals, 56) ? 1 : 0;
      NINE = checkArray(key_vals, 57) ? 1 : 0;
      A = checkArray(key_vals, 65) ? 1 : 0;
      B = checkArray(key_vals, 66) ? 1 : 0;
      C = checkArray(key_vals, 67) ? 1 : 0;
      D = checkArray(key_vals, 68) ? 1 : 0;
      E = checkArray(key_vals, 101) ? 1 : 0;
      F = checkArray(key_vals, 70) ? 1 : 0;
      G = checkArray(key_vals, 71) ? 1 : 0;
      H = checkArray(key_vals, 72) ? 1 : 0;
      I = checkArray(key_vals, 73) ? 1 : 0;
      J = checkArray(key_vals, 74) ? 1 : 0;
      K = checkArray(key_vals, 75) ? 1 : 0;
      L = checkArray(key_vals, 76) ? 1 : 0;
      M = checkArray(key_vals, 77) ? 1 : 0;
      N = checkArray(key_vals, 78) ? 1 : 0;
      O = checkArray(key_vals, 79) ? 1 : 0;
      P = checkArray(key_vals, 80) ? 1 : 0;
      Q = checkArray(key_vals, 113) ? 1 : 0;
      R = checkArray(key_vals, 114) ? 1 : 0;
      S = checkArray(key_vals, 83) ? 1 : 0;
      T = checkArray(key_vals, 116) ? 1 : 0;
      U = checkArray(key_vals, 85) ? 1 : 0;
      V = checkArray(key_vals, 86) ? 1 : 0;
      W = checkArray(key_vals, 87) ? 1 : 0;
      X = checkArray(key_vals, 88) ? 1 : 0;
      Y = checkArray(key_vals, 89) ? 1 : 0;
      Z = checkArray(key_vals, 90) ? 1 : 0;
      MOUSE_PRESS = mouse_press;
      MOUSE_X = mouse_x;
      MOUSE_Y = mouse_y;
    }else if(r==0){
      rSPACE = checkArray(key_vals, 8) ? 1 : 0;
      rTAB = checkArray(key_vals, 9) ? 1 : 0;
      rLSHIFT = checkArray(key_vals, 16) ? 1 : 0;
      rLCTRL = checkArray(key_vals, 17) ? 1 : 0;
      rLATL = checkArray(key_vals, 18) ? 1 : 0;
      rCAP = checkArray(key_vals, 20) ? 1 : 0;
      rZERO = checkArray(key_vals, 48) ? 1 : 0;
      rONE = checkArray(key_vals, 49) ? 1 : 0;
      rTWO = checkArray(key_vals, 50) ? 1 : 0;
      rTHREE = checkArray(key_vals, 51) ? 1 : 0;
      rFOUR = checkArray(key_vals, 52) ? 1 : 0;
      rFIVE = checkArray(key_vals, 53) ? 1 : 0;
      rSIX = checkArray(key_vals, 54) ? 1 : 0;
      rSEVEN = checkArray(key_vals, 55) ? 1 : 0;
      rEIGHT = checkArray(key_vals, 56) ? 1 : 0;
      rNINE = checkArray(key_vals, 57) ? 1 : 0;
      rA = checkArray(key_vals, 65) ? 1 : 0;
      rB = checkArray(key_vals, 66) ? 1 : 0;
      rC = checkArray(key_vals, 67) ? 1 : 0;
      rD = checkArray(key_vals, 68) ? 1 : 0;
      rE = checkArray(key_vals, 101) ? 1 : 0;
      rF = checkArray(key_vals, 70) ? 1 : 0;
      rG = checkArray(key_vals, 71) ? 1 : 0;
      rH = checkArray(key_vals, 72) ? 1 : 0;
      rI = checkArray(key_vals, 73) ? 1 : 0;
      rJ = checkArray(key_vals, 74) ? 1 : 0;
      rK = checkArray(key_vals, 75) ? 1 : 0;
      rL = checkArray(key_vals, 76) ? 1 : 0;
      rM = checkArray(key_vals, 77) ? 1 : 0;
      rN = checkArray(key_vals, 78) ? 1 : 0;
      rO = checkArray(key_vals, 79) ? 1 : 0;
      rP = checkArray(key_vals, 80) ? 1 : 0;
      rQ = checkArray(key_vals, 113) ? 1 : 0;
      rR = checkArray(key_vals, 114) ? 1 : 0;
      rS = checkArray(key_vals, 83) ? 1 : 0;
      rT = checkArray(key_vals, 116) ? 1 : 0;
      rU = checkArray(key_vals, 85) ? 1 : 0;
      rV = checkArray(key_vals, 86) ? 1 : 0;
      rW = checkArray(key_vals, 87) ? 1 : 0;
      rX = checkArray(key_vals, 88) ? 1 : 0;
      rY = checkArray(key_vals, 89) ? 1 : 0;
      rZ = checkArray(key_vals, 90) ? 1 : 0;
      rMOUSE_PRESS = mouse_press;
      rMOUSE_X = mouse_x;
      rMOUSE_Y = mouse_y;
    }
    if(FOUR==1 && rFOUR==0) isJUE = (isJUE==0) ? 1 : 0;
  }
  //打印键值解析数据
  String get_data(){
    if(SerOc==0){
      String data_string = " cmd_id:"+String(cmd_id)+
                  " len:"+String(len)+
                  " mouse_press:"+String(mouse_press)+
                  " mouse_x:"+String(mouse_x)+
                  " mouse_y:"+String(mouse_y)+
                  " key_num:"+String(key_num)+
                  " key_vals[0]:"+String(key_vals[0])+
                  " key_vals[1]:"+String(key_vals[1])+
                  " key_vals[2]:"+String(key_vals[2])+
                  " key_vals[3]:"+String(key_vals[3])+
                  " ";
      return data_string;
    }
  }
  //打印初始数据
  void get_init_data(){
    Serial.println(str_raw);
  }
  void get_mouse_press(){//1为鼠标右键, 2为鼠标左键, 4为鼠标中键
    Serial.println("mouse_press:"+String(mouse_press));
  }
  void get_mouse_xy(){
    Serial.println("mouse_x:"+String(mouse_x)+" mouse_y:"+String(mouse_y));
  }
  void get_key(){
    if(key_num == 0){
      Serial.println("no key is being push");
    }else if(key_num == 1){
      Serial.println("1key is being push:"+String(key_vals[0]));
    }else if(key_num == 2){
      Serial.println("2keys are being push:"+String(key_vals[0])+" "+String(key_vals[1]));
    }else if(key_num == 3){
      Serial.println("3keys are being push:"+String(key_vals[0])+" "+String(key_vals[1])+" "+String(key_vals[2]));
    }
  }
  ~p_data(){
     
  }
};
class p_arm{
  public:
    double status     = 0,      //status是机械臂整体的执行状态，表现的是机械臂正在干什么  
                                //0:空闲  1:手动夹取
           big_arm    = 89.5,
           small_arm  = 180,
           uservo0_an = 0,
           uservo1_an = 0,
           uservo2_an = 0,
           uservo3_an = 0;
    int    arm_sts    = 0;    //arm_sts是手动阶段夹取时，机械臂的局部执行状态，体现的是机械臂放下、夹取抬升等  
                              //0:下方弹药瓶准备  1:大弹丸准备  2:夹取  3:微调后  4:三号弹  5:正在夹取三号弹
    int    box_sts    = 0;    //box_sts是储弹盒局部执行状态，体现储弹盒倾倒和平放
                              //0:储弹盒平放  1:储弹盒倾倒
    int cID=0;

    p_arm(){
      arm_sts=2;
      box_sts=0;
    }
    void direct_servo(){    //直接控制舵机旋转，优先级最低
      float jzangle = uservo3.queryAngle(), sangle = uservo1.queryAngle(), bangle = uservo0.queryAngle();
      uservo0.setSpeed(180);
      uservo1.setSpeed(180);
      uservo2.setSpeed(180);
      uservo3.setSpeed(180);
      if(status==0){
        //夹爪
        if(V==1){
          jzangle += 3;
          cID = 1;
          uservo3.setAngle(jzangle);
        }else if(B==1){
          jzangle -= 3;
          cID = 2;
          uservo3.setAngle(jzangle);
        }
        //小臂
        if(N==1){
          sangle += 3;
          cID = 3;
          uservo1.setAngle(sangle);
        }else if(M==1){
          sangle -= 3;
          cID = 4;
          uservo1.setAngle(sangle);
        }
        //大臂
        if(X==1){
          bangle += 3;
          cID = 5;
          uservo0.setAngle(bangle);
        }else if(C==1){
          bangle -= 10;
          cID = 6;
          uservo0.setAngle(bangle);
        }
        if(cID==3 || cID==4){
          if(arm_sts==0){
            SMALL_DYP = sangle;
          }else if(arm_sts==1){
            SMALL_DDW = sangle;
          }
          arm_sts=(arm_sts==1 || arm_sts==0) ? arm_sts : 3;
          cID = 0;
        }
        if(Z==1&&rZ==0&&(cID==1 || cID==2)){
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
    void hand_get_dyp(){    //在手动阶段按F/R键夹取下方弹药瓶和大弹丸
      uservo0.setSpeed(360);    //设置0舵机速度
      uservo1.setSpeed(360);    //设置1舵机速度
      uservo2.setSpeed(360);    //设置2舵机速度
      uservo3.setSpeed(360);    //设置2舵机速度
      if((status==0 || status==1) && box_sts == 0){   //机械臂处于空闲时或手动夹取时（就是当前状态），执行以下操作
        if(F==1 && rF==0 && arm_sts != 0){
          status = 1;
          if(arm_sts==1){
            uservo2.setAngle(0);
            uservo3.setAngle(OPEN_DYP_AN);

            uservo1.setAngle(-120);
            vTaskDelay(80/portTICK_PERIOD_MS);

            uservo0.setAngle(85);
            uservo1.setAngle(SMALL_DYP);

            arm_sts=0;
            status=0;
          }else{
            uservo2.setAngle(0);
            uservo3.setAngle(OPEN_DYP_AN);

            uservo0.setAngle(85);
            vTaskDelay(50/portTICK_PERIOD_MS);

            uservo1.setAngle(-120);
            vTaskDelay(50/portTICK_PERIOD_MS);
            
            uservo1.setAngle(SMALL_DYP);
            status=0;
          }
          arm_sts=0;
        }else if(G==1 && rG==0 && arm_sts!=1){
          status = 1;
          arm_sts=1;

          uservo0.setAngle(-70);
          vTaskDelay(100/portTICK_PERIOD_MS);

          uservo1.setAngle(SMALL_DDW);
          uservo2.setAngle(0);
          uservo3.setAngle(OPEN_DYP_AN);

          arm_sts=1;
          status=0;
        }else if(NINE==1 && rNINE==0 && arm_sts!=4 && arm_sts!=2){
          status = 1;
          uservo3.setRawAngle(-10);while(!uservo3.isStop());
          uservo0.setRawAngle(-10);
          uservo1.setRawAngle(-90);
          uservo2.setRawAngle(-90);
          arm_sts=4;
          status=0;
        }else if(ZERO==1 && rZERO==0 && arm_sts!=5 && arm_sts!=2){
          status=1;
          arm_sts=5;
          uservo0.setRawAngle(-80);
          uservo1.setRawAngle(-60);vTaskDelay(15);
          uservo0.setRawAngle(-10);
          uservo1.setRawAngle(-90);vTaskDelay(15);
          arm_sts=4;
        }else if(H==1 && rH==0 && arm_sts!=2 && arm_sts!=5){
          status = 1;
          //上锁舵机
          // uservo0.setDamping(1000);
          // uservo1.setDamping(1000);
          // uservo2.setDamping(1000);
          // uservo3.setDamping(1);
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
            vTaskDelay(200/portTICK_PERIOD_MS);
            
            uservo3.setAngle(OPEN_DYP_AN);
          }else if(arm_sts==4){
            uservo1.setRawAngle(-10);vTaskDelay(20);
            uservo0.setRawAngle(BIG_UP);
            uservo1.setRawAngle(SMALL_UP);
            uservo2.setRawAngle(0);
            uservo3.setAngle(OPEN_DYP_AN);vTaskDelay(20);
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
          while(uservo0.isStop()&&uservo1.isStop()&&uservo2.isStop()&&uservo3.isStop());
          uservo1.setDamping(0);    
          arm_sts=2;
          status=0;
        }
      }
    }
    void unload(){
      if(isJUE == 0){
        if(I==1 && rI==0 && EIGHT==1){
          uservo0.setDamping(1000);
          uservo1.setDamping(1000);
          uservo2.setDamping(1000);
          uservo3.setDamping(1000);
          vTaskDelay(50/portTICK_PERIOD_MS);
          uservo0.setSpeed(360);
          uservo1.setSpeed(360);
          uservo2.setSpeed(360);
          uservo3.setSpeed(360);
          if(box_sts==0){    //按下按键，倾倒
            uservo0_an = -90;
            uservo1_an = 10;
            uservo0.setAngle(uservo0_an);
            uservo1.setAngle(uservo1_an);
            vTaskDelay(500/portTICK_PERIOD_MS);
            box_sts = 1;
            Serial1.print(get_djiservo_string(box_servo_id, BOX_UP));         
          }else if(box_sts==1){    //按下按键，平放
            Serial1.print(get_djiservo_string(box_servo_id, BOX_DOWN));  
            vTaskDelay(600/portTICK_PERIOD_MS);
            uservo0_an = BIG_UP;
            uservo1_an = SMALL_UP;
            uservo0.setAngle(uservo0_an);
            uservo1.setAngle(uservo1_an);
            box_sts = 0;
          }
          vTaskDelay(200/portTICK_PERIOD_MS);
        }
      }
    }
};


void setup() {
  Serial1.begin(115200);
  Serial.begin(115200);
  Serial.println("start...");
  protocol.init(); // 舵机通信协议初始化
  uservo0.init(); // 串口总线舵机初始化
  uservo1.init(); // 串口总线舵机初始化
  uservo2.init(); // 串口总线舵机初始化

  pinMode(13, INPUT);   //定义按钮引脚模式
  pinMode(53, OUTPUT);  //定义SD模块CS引脚模式
  pinMode(TRIG, OUTPUT);   //定义超声波TRIG引脚模式
  pinMode(ECHO, INPUT);  //定义超声波ECHO引脚模式

  pingTimer = millis();

  //创建队列、任务
  
  xTaskCreate(TaskSerial, "PrintSerial", 1164, NULL, 3, &TaskSerial_Handle);
  xTaskCreate(TaskArm, "control the arm", 250, NULL, 3, &TaskArm_Handle);
  xTaskCreate(TaskTurnCamera, "turn camera", 270, NULL, 3, &TaskTurnCamera_Handle);
  xTaskCreate(TaskTurnBan, "turn ban", 270, NULL, 3, &TaskTurnBan_Handle);
  xTaskCreate(TaskAuto, "PrintSerial", 1000, NULL, 3, &TaskAuto_Handle);
  xTaskCreate(TaskAutoBreak, "PrintSerial", 500, NULL, 3, &TaskAutoBreak_Handle);
  xTaskCreate(TaskSR04, "PrintSerial", 500, NULL, 3, &TaskSR04_Handle);
  xTaskCreate(TaskReset, "reset game_msg", 208, NULL, 3, &TaskReset_Handle);
  //开启任务调度
  vTaskStartScheduler();
}
void loop(){}

void TaskSerial(void *pvParameters){
  (void) pvParameters;
  str_in0 = "game msg push [0, 6, 1, 0, 0, 255, 3, 19, 43, 23];";
  for(;;){
    // if(SerOc == 0){
    //   SerOc = 1;
    //   int memfree = uxTaskGetStackHighWaterMark(NULL);
    //   Serial.println("free: "+String(memfree)+" used: "+String(800-memfree));
    //   SerOc = 0;
    // }
    if(Serial1.available()){
      str_in1 = Serial1.readStringUntil('\n');
      p_data data0(str_in0);//上一帧键值
      p_data data1(str_in1);//当前键值
      data0.key_init(data0.key_vals, data0.mouse_press, data0.mouse_x, data0.mouse_y, 0);
      data1.key_init(data1.key_vals, data1.mouse_press, data1.mouse_x, data1.mouse_y, 1);
      str_in0 = str_in1;
      move_with_xyz();
      String strToSD = data1.get_data();
      String strTime = " time: "+String(millis())+"ms";
      String strArm  = " CLOSE_DYP_AN: "+String(CLOSE_DYP_AN)+
                       " CLOSE_DDW_AN: "+String(CLOSE_DDW_AN)+
                       " SMALL_DDW: "+String(SMALL_DDW)+
                       " SMALL_DYP: "+String(SMALL_DYP);
      String strDis  = " dis1: "+String(dis1);
      String strFinal= 
                      strTime+
                      strToSD+
                      strArm+
                      strDis;
      
      //串口打印
      // Serial.println(strFinal);h

      //SD卡日志记录
      // File dataFile = SD.open("datas.txt", FILE_WRITE);
      // if (dataFile) {
      //   dataFile.println(strFinal);
      //   dataFile.close();
      // }
    }
    vTaskDelay(15/portTICK_PERIOD_MS);
  }
}
void TaskArm(void *pvParameters){
  p_arm myArm;
  for(;;){
    myArm.hand_get_dyp();
    myArm.direct_servo();
    myArm.unload();
    vTaskDelay(15/portTICK_PERIOD_MS);
  }
}
void TaskTurnCamera(void *pvParameters){
  for(;;){
    //获取朝向数据facing
    if(isJUE==1){   //有JUE模式，T键触发
      if(T == 1 && rT == 0){
        facing = (facing==0) ? 2 : 0;
        isChange = 1;
      }
    }else if(isJUE==0){   //无JUE模式，TAB键触发
      if(TAB == 1 && rTAB == 0){    //180°扭头
        facing = (facing==0) ? 2 : 0;
        isChange = 1;
      }else if(U == 1 && rU == 0){  //左扭头
        facing -= 1;
        isChange = 1;
      }else if(O == 1 && rO == 0){  //右扭头
        facing += 1;
        isChange = 1;
      }
      constrain(facing, -2, 2);
    }
    //将朝向数据转化为舵机控制
    if(isChange){
      isChange = 0;
      update = 1;
      int16_t out;
      switch(facing){
        case 0: out=3;break;
        case 1: out=93;break;
        case 2: out=180;break;
        case -2: out=-178;break;
        case -1: out=-87;break;
        default: out = 0;
      }
      Serial1.print(get_djiservo_string(head_servo_id, out));
      vTaskDelay(200/portTICK_PERIOD_MS);
    }
    vTaskDelay(15/portTICK_PERIOD_MS);
  }
}
void TaskTurnBody(void *pvParameters){
  for(;;){
    if(K==1 && rK==0){
      disChassis = 1;
      Serial1.print("chassis wheel w2 0 w1 0 w3 0 w4 0;");
      vTaskDelay(40/portTICK_PERIOD_MS);
      Serial1.print("chassis wheel w2 400 w1 -400 w3 400 w4 -400;");
      vTaskDelay(310/portTICK_PERIOD_MS);
      vTaskSuspend(TaskTurnBan_Handle);
      vTaskSuspend(TaskTurnCamera_Handle);
      vTaskSuspend(TaskAuto_Handle);
      vTaskSuspend(TaskAutoBreak_Handle);
      Serial1.print("chassis wheel w2 0 w1 0 w3 0 w4 0;");
      disChassis = 0;
      vTaskResume(TaskTurnBan_Handle);
      vTaskResume(TaskTurnCamera_Handle);
      vTaskResume(TaskAuto_Handle);
      vTaskResume(TaskAutoBreak_Handle);
    }
    vTaskDelay(15/portTICK_PERIOD_MS);
  }
}
void TaskTurnBan(void *pvParameters){
  
  for(;;){
    if(abs(facing)==2 && ban_sts==0){
      Serial1.print(get_djiservo_string(ban_servo_id, BAN_UP));
      ban_sts=1;
    }
    if(Y==1 && rY==0){
      if(ban_sts==0){
        Serial1.print(get_djiservo_string(ban_servo_id, BAN_UP)); 
        ban_sts=1;
      }else if(ban_sts==1 && abs(facing)!=2){
        Serial1.print(get_djiservo_string(ban_servo_id, BAN_DOWN));
        ban_sts=0;
      }
      vTaskDelay(200/portTICK_PERIOD_MS);
    }
    vTaskDelay(15/portTICK_PERIOD_MS);
  }
}
  void TaskAutoBreak(void *pvParameters){
    for(;;){
      if(T==1 && rT==0 && isJUE==0){
        vTaskDelete(TaskAuto_Handle);
        disChassis = 0;
        vTaskDelay(3);
        Serial1.print(getMoveString(0, 0, 0));
        xTaskCreate(TaskAuto, "auto skills", 1000, NULL, 4, &TaskAuto_Handle);
        uservo0.setDamping(0);
        uservo1.setDamping(0);
        uservo2.setDamping(0);
        uservo3.setDamping(0);
        
      }
      if(EIGHT==1&&rEIGHT==0){
        vTaskDelete(TaskAuto_Handle);
      }else if(K==1&&rK==0){
        xTaskCreate(TaskAuto, "auto skills", 1000, NULL, 4, &TaskAuto_Handle);
      }
      vTaskDelay(1);
    }
  }
  void TaskAuto(void *pvParameters){
    for(;;){
      if(isJUE==0){
        if(ONE==1 && rONE==0){
          //初始化
          disChassis = 1;
          //设置舵机速度
          uservo0.setSpeed(360);    //设置0舵机速度
          uservo1.setSpeed(360);    //设置1舵机速度 
          uservo2.setSpeed(360);    //设置2舵机速度
          uservo3.setSpeed(360);    //设置2舵机速度
          
          //右路
          // Serial1.print(getMoveString(1.5,0,0));vTaskDelay(30);   //前进
          // Serial1.print(getMoveString(0,0,180));vTaskDelay(17);   //右转
          // Serial1.print(getMoveString(0,-1,0));vTaskDelay(30);    //左移
          // Serial1.print(getMoveString(1.5,-0.2,0));vTaskDelay(75);   //前左移
          // Serial1.print(getMoveString(0,1.5,0));vTaskDelay(40);   //右移
          // Serial1.print(getMoveString(1.5,0,0));vTaskDelay(30);   //前移
          // Serial1.print(getMoveString(-1,0,0));vTaskDelay(18);   //前移
          // Serial1.print(getMoveString(-0.2,-1.5,0));vTaskDelay(120);   //左移
          // Serial1.print(getMoveString(0,0,0));

          //左路
          Serial1.print(get_djiservo_string(head_servo_id, 180));facing=2;
          Serial1.print(get_djiservo_string(ban_servo_id, BAN_UP));ban_sts=1;
          vTaskDelay(2);
          Serial1.print(getMoveString(1,0,0,facing));vTaskDelay(145);   //前进
          Serial1.print(getMoveString(0,0,-90,facing));vTaskDelay(28);   //左转
          Serial1.print(getMoveString(1,0,0,facing));vTaskDelay(92);   //前进
          Serial1.print(getMoveString(0,0,90,facing));vTaskDelay(55);   //右转
          Serial1.print(getMoveString(1.5,0.9,0,facing));vTaskDelay(98);   //右前进
          Serial1.print(getMoveString(0,0,0,facing));vTaskDelay(30);
          while(1){
            Serial1.print(getMoveString(-1,0,0,facing));vTaskDelay(55);   //后退
            Serial1.print(getMoveString(0,0,0,facing));vTaskDelay(30);
            Serial1.print(getMoveString(1,0,0,facing));vTaskDelay(55);   //前进
            Serial1.print(getMoveString(0,0,0,facing));vTaskDelay(30);
          }



        //   //到达资源岛前
        //   Serial1.print(getMoveString(1, 0, 0));
        //   vTaskDelay(800/portTICK_PERIOD_MS);
        //   Serial1.print(getMoveString(0, 0, 90));
        //   vTaskDelay(450/portTICK_PERIOD_MS);
        //   Serial1.print(getMoveString(1, 0, 0));
        //   vTaskDelay(100/portTICK_PERIOD_MS);
        //   Serial1.print(getMoveString(1.9, -2, 0));
        //   vTaskDelay(900/portTICK_PERIOD_MS);
        //   Serial1.print(getMoveString(0, -0.5, 0));
        //   //后续复位
        //   disChassis = 0;
        // }else if(TWO==1 && rTWO==0){
        //   disChassis = 1;
        //   Serial1.print(getMoveString(-0.5, 0, 0));
        //   vTaskDelay(300/portTICK_PERIOD_MS);
        //   move_with_wheels(0,0,0);
        //   //上锁舵机
        //   uservo0.setSpeed(360);
        //   uservo1.setSpeed(360);
        //   uservo2.setSpeed(360);
        //   uservo3.setSpeed(360);
        //   //准备钩上方弹药瓶
        //   uservo0.setAngle(-20);
        //   uservo1.setAngle(0);
        //   uservo2.setAngle(-90);
        //   uservo3.setAngle(CLOSE_DYP_AN);while(!uservo3.isStop());
        //   //调整位置
        //   Serial1.print("chassis wheel w1 100 w2 100 w3 100 w4 100;");
        //   vTaskDelay(2000/portTICK_PERIOD_MS);
        //   Serial1.print(getMoveString(0, 0, 0));
        //   uservo3.setRawAngle(-10);vTaskDelay(20);
        //   uservo0.setRawAngle(-10);
        //   uservo1.setRawAngle(-90);
          //后续复位
          disChassis = 0;
        }else if(THREE==1 && rTHREE==0){
          //初始化动作
          disChassis = 1;
          //上锁舵机
          uservo0.setSpeed(360);
          uservo1.setSpeed(360);
          uservo2.setSpeed(360);
          uservo3.setSpeed(360);
          uservo0.setDamping(1000);
          uservo1.setDamping(1000);
          uservo2.setDamping(1000);
          uservo3.setDamping(1000);
          //抬起摆好机械臂
          uservo1.setRawAngle(-80);
          uservo0.setRawAngle(-70);
          vTaskDelay(400/portTICK_PERIOD_MS);
          uservo1.setRawAngle(-90);
          uservo0.setRawAngle(-10);
          //后续复位
          disChassis = 0;
        }else if(Q==1 && rQ==0){
          disChassis = 1;
          if(isJUE==0)Serial1.print(getMoveString(0,-0.1,0));
          
          // uservo0.setSpeed(360);    //设置0舵机速度
          // uservo1.setSpeed(360);    //设置1舵机速度
          // uservo2.setSpeed(360);    //设置2舵机速度
          // uservo3.setSpeed(360);    //设置2舵机速度
          // //摆好机械臂位置
          // uservo3.setRawAngle(-10);vTaskDelay(20);
          // uservo0.setRawAngle(-10);
          // uservo1.setRawAngle(-90);
          // float error;
          // float ok_err = 1;
          // float z_out, z_out0;
          // // ///---------第一个3号---------////////////////////////////////////////////////////////
          // // //底盘超声波P控制
          // error = firstDYP-dis1;
          // while(!(abs(error)<=ok_err)){
          //   // Serial.print("error:"+String(error));
          //   // Serial.print("dis1:"+String(dis1));
          //   // Serial.print("z_out:"+String(z_out));
          //   // Serial.print("z_out0:"+String(z_out0));
          //   error = firstDYP-dis1;
          //   z_out0=z_out;
          //   z_out = (error>=0) ? 0.1 : -0.1;3
          //   if(z_out!=z_out0)Serial1.println(getMoveString(0, z_out, 0));
          //   vTaskDelay(1);
          // }
          // Serial1.print("chassis wheel w2 50 w1 50 w3 50 w4 50;");
          // vTaskDelay(200/portTICK_PERIOD_MS);
          // //钩
          // // gou_3_dyp();
          // uservo0.setRawAngle(-90);
          // uservo1.setRawAngle(-50);
          // vTaskDelay(15);
          // move_with_wheels(0,0,0);
          // // ///---------第二个3号---------////////////////////////////////////////////////////////
          // error = secondDYP-dis1;
          // while(!(abs(error)<=ok_err)){
          //   error = secondDYP-dis1;
          //   z_out0=z_out;
          //   z_out = (error>=0) ? 40 : -40;
          //   if(z_out!=z_out0)Serial1.print(getMoveString(0, z_out, 0));
          // }
          // Serial1.print("chassis wheel w2 50 w1 50 w3 50 w4 50;");
          // vTaskDelay(200/portTICK_PERIOD_MS);
          // //钩
          // gou_3_dyp();
          // move_with_wheels(0,0,0);
          // // ///---------第三个3号---------////////////////////////////////////////////////////////
          // error = thirdDYP-dis1;
          // while(!(abs(error)<=ok_err)){
          //   error = thirdDYP-dis1;
          //   z_out0=z_out;
          //   z_out = (error>=0) ? 0.5 : -0.5;
          //   if(z_out!=z_out0)Serial1.print(getMoveString(0, out, 0));
          // }
          // Serial1.print("chassis wheel w2 50 w1 50 w3 50 w4 50;");
          // vTaskDelay(200/portTICK_PERIOD_MS);
          // //钩
          // gou_3_dyp(); 
          // move_with_wheels(0,0,0);
          // //后退
          // uservo0.setDamping(0);
          // uservo1.setDamping(0);
          // uservo2.setDamping(0);
          // uservo3.setDamping(0);
          // Serial1.print(getMoveString(-1, 0, 0));
          // vTaskDelay(500/portTICK_PERIOD_MS);
          // Serial1.print(getMoveString(0, 0, 0));
          // //摆好机械臂
          // uservo0.setSpeed(360);
          // uservo1.setSpeed(360);
          // uservo2.setSpeed(360);
          // uservo3.setSpeed(360);
          // uservo0.setAngle(BIG_UP);
          // uservo1.setAngle(SMALL_UP);
          // uservo2.setAngle(0);
          // uservo3.setAngle(OPEN_DYP_AN);
          // vTaskDelay(50/portTICK_PERIOD_MS);
          //后续复位
          disChassis = 0;
        }else if(E==1 && rE==0){
          //初始化动作
          disChassis = 1;
          //解锁舵机
          if(isJUE==0)Serial1.print(getMoveString(0,0.1,0));
          //后续复位
          disChassis = 0;
        }else if(R==1 && rR==0){
          
        }
      }
      vTaskDelay(1);
    }
  }
  void TaskSR04(void *pvParameters){
    for(;;){
      if (millis() >= pingTimer) {   // pingSpeed milliseconds since last ping, do another ping.
        pingTimer += pingSpeed;      // Set the next ping time.
        sonar1.ping_timer(echoCheck); // Send out the ping, calls "echoCheck" function every 24uS where you can check the ping status.
      }
      vTaskDelay(15/portTICK_PERIOD_MS);
    }
  }
void TaskReset(void *pvParameters){
  // while(!digitalRead(27)==LOW);
  vTaskDelay(12000/portTICK_PERIOD_MS);
  for(;;){
    // if(digitalRead(27)==HIGH){
      // //初始化SD模块
      // while(!SD.begin(53));
      //挂起一些任务
      vTaskSuspend(TaskArm_Handle);
      //进入SDK模式
      Serial.println("reset reset reset reset reset");
      Serial1.print("game_msg off;");
      vTaskDelay(5);
      Serial1.print("quit;");
      vTaskDelay(5);
      Serial1.print("command;");
      vTaskDelay(5);
      Serial1.print("game_msg on;");
      vTaskDelay(5);
      init_all();
      //恢复一些任务
      vTaskResume(TaskArm_Handle);
      //删除初始化任务
      vTaskDelete(TaskReset_Handle);
    }
    vTaskDelay(15/portTICK_PERIOD_MS);
  }
}
void TaskQuit(void *pvParameters){
  for(;;){
    if(FIVE==1 && SIX==1 && SEVEN==1){
      Serial.println("quit!!!");
      Serial1.print("quit;");
      vTaskSuspendAll();
      while (1);      
    }
    vTaskDelay(1);
  }
}