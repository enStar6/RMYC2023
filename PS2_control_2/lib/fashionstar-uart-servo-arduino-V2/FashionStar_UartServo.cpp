/* 
 * FashionStar串口总线舵机ArduinoSDK
 * --------------------------
 * 作者: 深圳市华馨京科技有限公司
 * 网站：https://fashionrobo.com/
 * 更新时间: 2023/03/13
 */
#include "FashionStar_UartServo.h"

FSUS_Servo::FSUS_Servo(){

}

FSUS_Servo::FSUS_Servo(uint8_t servoId, FSUS_Protocol *protocol){
    this->servoId = servoId; // 设定舵机的ID号
    this->protocol = protocol; // 舵机的通信协议
    this->angleMin = FSUS_SERVO_ANGLE_MIN; // 设置默认角度的最小值
    this->angleMax = FSUS_SERVO_ANGLE_MAX; // 设置默认角度的最大值
    this->speed = FSUS_SERVO_SPEED; // 设置默认转速
    this->kAngleReal2Raw = FSUS_K_ANGLE_REAL2RAW;
    this->bAngleReal2Raw = FSUS_B_ANGLE_REAL2RAW;
    
}

/* 舵机初始化 */
void FSUS_Servo::init(){
    // ping一下舵机
    ping();
    if(this->isOnline){
        // 舵机如果在线就开始同步角度
        queryAngle();
        // 目标角度跟初始角度一致
        this->targetAngle = this->curAngle;
    }
}

void FSUS_Servo::init(uint8_t servoId, FSUS_Protocol *protocol){
    this->servoId = servoId; // 设定舵机的ID号
    this->protocol = protocol; // 舵机的通信协议
    this->angleMin = FSUS_SERVO_ANGLE_MIN; // 设置默认角度的最小值
    this->angleMax = FSUS_SERVO_ANGLE_MAX; // 设置默认角度的最大值
    this->speed = FSUS_SERVO_SPEED; // 设置默认转速
    this->kAngleReal2Raw = FSUS_K_ANGLE_REAL2RAW;
    this->bAngleReal2Raw = FSUS_B_ANGLE_REAL2RAW;
    this->isMTurn = false;
    init(); // 初始化舵机
}    

/*舵机通讯检测, 判断舵机是否在线*/
bool FSUS_Servo::ping(){
    this->protocol->emptyCache(); //清空串口缓冲区
    this->protocol->sendPing(servoId); //发送PING指令
    FSUS_SERVO_ID_T servoIdTmp;
    this->protocol->recvPing(&servoIdTmp, &(this->isOnline));
    return this->isOnline;
}

// 舵机标定
void FSUS_Servo::calibration(FSUS_SERVO_ANGLE_T rawA, FSUS_SERVO_ANGLE_T realA, FSUS_SERVO_ANGLE_T rawB, FSUS_SERVO_ANGLE_T realB){
    // rawAngle = kAngleReal2Raw*realAngle + bAngleReal2Raw
    this->kAngleReal2Raw = (rawA - rawB)/(realA-realB);
    this->bAngleReal2Raw = rawA - this->kAngleReal2Raw*realA;
}

// 舵机标定
void FSUS_Servo::calibration(float kAngleReal2Raw, float bAngleReal2Raw){
    this->kAngleReal2Raw = kAngleReal2Raw;
    this->bAngleReal2Raw = bAngleReal2Raw;
}

// 真实角度转化为原始角度
FSUS_SERVO_ANGLE_T FSUS_Servo::angleReal2Raw(FSUS_SERVO_ANGLE_T realAngle){
    return this->kAngleReal2Raw*realAngle + this->bAngleReal2Raw;
}

// 原始角度转换为真实角度
FSUS_SERVO_ANGLE_T FSUS_Servo::angleRaw2Real(FSUS_SERVO_ANGLE_T rawAngle){
    return (rawAngle -  this->bAngleReal2Raw) / this->kAngleReal2Raw;
}

// 设置舵机的角度
void FSUS_Servo::setAngleRange(FSUS_SERVO_ANGLE_T angleMin, FSUS_SERVO_ANGLE_T angleMax){
    this->angleMin = angleMin;
    this->angleMax = angleMax;
}

bool FSUS_Servo::isAngleLegal(FSUS_SERVO_ANGLE_T candiAngle){
    return candiAngle >= this->angleMin && candiAngle <= this->angleMax;
}


// 设置舵机的平均转速
void FSUS_Servo::setSpeed(FSUS_SERVO_SPEED_T speed){
    this->speed = speed;
}

/*设置舵机角度*/
void FSUS_Servo::setAngle(FSUS_SERVO_ANGLE_T angle){
    FSUS_SERVO_ANGLE_T dAngle; // 当前角度与目标角度之间的差值
    FSUS_INTERVAL_T interval; // 周期
    // 检舵机查角度是否合法
    angle = (angle < this->angleMin) ? this->angleMin: angle;
    angle = (angle > this->angleMax) ? this->angleMax: angle;
    // 检查舵机角度查询(更新当前的角度)
    this->queryAngle();
    dAngle = abs(angle - this->curAngle);
    // 计算角度差, 估计周期
    interval = (FSUS_INTERVAL_T)((dAngle/speed)*1000);
    setAngle(angle, interval);
}

/* 设置舵机角度,同时指定角度跟时间*/
void FSUS_Servo::setAngle(FSUS_SERVO_ANGLE_T angle, FSUS_INTERVAL_T interval){
    // 约束角度的范围
    angle = (angle < this->angleMin) ? this->angleMin: angle;
    angle = (angle > this->angleMax) ? this->angleMax: angle;
    // 发送舵机角度控制指令
    this->targetAngle = angle; // 设置目标角度
    setRawAngle(angleReal2Raw(this->targetAngle), interval);
}

/* 设置舵机角度 */
void FSUS_Servo::setAngle(FSUS_SERVO_ANGLE_T angle, FSUS_INTERVAL_T interval, FSUS_POWER_T power){
    // 约束角度的范围
    angle = (angle < this->angleMin) ? this->angleMin: angle;
    angle = (angle > this->angleMax) ? this->angleMax: angle;
    // 发送舵机角度控制指令
    this->targetAngle = angle; // 设置目标角度
    setRawAngle(angleReal2Raw(this->targetAngle), interval, power);
}


/* 设置舵机的原始角度 */
void FSUS_Servo::setRawAngle(FSUS_SERVO_ANGLE_T rawAngle, FSUS_INTERVAL_T interval, FSUS_POWER_T power){
    this->isMTurn = false;
    this->targetRawAngle = rawAngle;
    this->protocol->sendSetAngle(this->servoId, rawAngle, interval, power);
}

/* 设置舵机的原始角度 */
void FSUS_Servo::setRawAngle(FSUS_SERVO_ANGLE_T rawAngle, FSUS_INTERVAL_T interval){
    this->isMTurn = false;
    this->targetRawAngle = rawAngle;
    this->protocol->sendSetAngle(this->servoId, rawAngle, interval, 0);
}
/* 设置舵机的原始角度 */
void FSUS_Servo::setRawAngle(FSUS_SERVO_ANGLE_T rawAngle){
    this->isMTurn = false;
    this->targetRawAngle = rawAngle;
    this->protocol->sendSetAngle(this->servoId, rawAngle, 0, 0);
}

// 设置舵机的原始角度(指定周期)
void FSUS_Servo::setRawAngleByInterval(FSUS_SERVO_ANGLE_T rawAngle, FSUS_INTERVAL_T interval, FSUS_INTERVAL_T t_acc, FSUS_INTERVAL_T t_dec, FSUS_POWER_T power){
    this->isMTurn = false;
    this->targetRawAngle = rawAngle;
    this->protocol->sendSetAngleByInterval(this->servoId, rawAngle, interval, t_acc, t_dec, power);
}

// 设定舵机的原始角度(指定转速)
void FSUS_Servo::setRawAngleByVelocity(FSUS_SERVO_ANGLE_T rawAngle, FSUS_SERVO_SPEED_T velocity, FSUS_INTERVAL_T t_acc, FSUS_INTERVAL_T t_dec, FSUS_POWER_T power){
    this->isMTurn = false;
    this->targetRawAngle = rawAngle;
    this->protocol->sendSetAngleByVelocity(this->servoId, rawAngle, velocity, t_acc, t_dec, power);
}

/* 查询舵机当前的真实角度*/
FSUS_SERVO_ANGLE_T FSUS_Servo::queryAngle(){
    queryRawAngle();
    this->curAngle = angleRaw2Real(this->curRawAngle);
    return this->curAngle; 
}

/* 查询舵机当前的原始角度 */
FSUS_SERVO_ANGLE_T FSUS_Servo::queryRawAngle(){
    this->protocol->emptyCache(); //清空串口缓冲区
    this->protocol->sendQueryAngle(this->servoId);
    FSUS_SERVO_ID_T servoIdTmp;
    this->protocol->recvQueryAngle(&servoIdTmp, &this->curRawAngle);
    return this->curRawAngle;
}

// 设定舵机的原始角度(多圈)
void FSUS_Servo::setRawAngleMTurn(FSUS_SERVO_ANGLE_T rawAngle, FSUS_INTERVAL_T_MTURN interval, FSUS_POWER_T power){
    this->isMTurn = true;
    this->targetRawAngle = rawAngle;
    this->protocol->sendSetAngleMTurn(this->servoId, rawAngle, interval, power);
}

// 设定舵机的原始角度(多圈)
void FSUS_Servo::setRawAngleMTurn(FSUS_SERVO_ANGLE_T rawAngle, FSUS_INTERVAL_T_MTURN interval){
    this->isMTurn = true;
    this->targetRawAngle = rawAngle;
    this->protocol->sendSetAngleMTurn(this->servoId, rawAngle, interval, 0);
}

// 设定舵机的原始角度(多圈)
void FSUS_Servo::setRawAngleMTurn(FSUS_SERVO_ANGLE_T rawAngle){
    this->isMTurn = true;
    this->targetRawAngle = rawAngle;
    this->protocol->sendSetAngleMTurn(this->servoId, rawAngle, 0, 0);
}


// 设定舵机的原始角度(多圈+指定周期)
void FSUS_Servo::setRawAngleMTurnByInterval(FSUS_SERVO_ANGLE_T rawAngle, FSUS_INTERVAL_T_MTURN interval, FSUS_INTERVAL_T t_acc, FSUS_INTERVAL_T t_dec, FSUS_POWER_T power){
    this->isMTurn = true;
    this->targetRawAngle = rawAngle;
    this->protocol->sendSetAngleMTurnByInterval(this->servoId, rawAngle, interval, t_acc, t_dec, power);
}

// 设定舵机的原始角度(多圈+指定转速)
void FSUS_Servo::setRawAngleMTurnByVelocity(FSUS_SERVO_ANGLE_T rawAngle, FSUS_SERVO_SPEED_T velocity, FSUS_INTERVAL_T t_acc, FSUS_INTERVAL_T t_dec, FSUS_POWER_T power){
    this->isMTurn = true;
    this->targetRawAngle = rawAngle;
    this->protocol->sendSetAngleMTurnByVelocity(this->servoId, rawAngle, velocity, t_acc, t_dec, power);
}

/* 查询舵机当前的原始角度 */
FSUS_SERVO_ANGLE_T FSUS_Servo::queryRawAngleMTurn(){
    this->protocol->emptyCache(); // 清空串口缓冲区
    this->protocol->sendQueryAngleMTurn(this->servoId);
    FSUS_SERVO_ID_T servoIdTmp;
    this->protocol->recvQueryAngleMTurn(&servoIdTmp, &this->curRawAngle);
    return this->curRawAngle;
}

// 查询舵机的电压(单位mV)
uint16_t FSUS_Servo::queryVoltage(){
    // 发送数据
    this->protocol->sendReadData(this->servoId, FSUS_PARAM_VOLTAGE);
    // 接收数据
    FSUS_SERVO_ID_T servoId;
    uint8_t address;
    uint8_t contentLen;
    uint8_t content[2];
    FSUS_STATUS status = this->protocol->recvReadData(&servoId, &address, &contentLen, content);
    if (status == FSUS_STATUS_SUCCESS){
        // 提取数据
        int16_t voltage;
        byte* valuePtr = (byte*)&voltage;
        valuePtr[0] = content[0];
        valuePtr[1] = content[1];
        return voltage;
    }
    return 0;
}

/* 查询舵机当前的工作电流 (单位 mA) */
uint16_t FSUS_Servo::queryCurrent(){
    // 发送数据
    this->protocol->sendReadData(this->servoId, FSUS_PARAM_CURRENT);
    // 接收数据
    FSUS_SERVO_ID_T servoId;
    uint8_t address;
    uint8_t contentLen;
    uint8_t content[2];
    FSUS_STATUS status = this->protocol->recvReadData(&servoId, &address, &contentLen, content);
    if (status == FSUS_STATUS_SUCCESS){
        // 提取数据
        int16_t current;
        byte* valuePtr = (byte*)&current;
        valuePtr[0] = content[0];
        valuePtr[1] = content[1];
        return current;
    }
    return 0;
}

/* 查询舵机当前的功率 (单位 mW)*/
uint16_t FSUS_Servo::queryPower(){
     // 发送数据
    this->protocol->sendReadData(this->servoId, FSUS_PARAM_POWER);
    // 接收数据
    FSUS_SERVO_ID_T servoId;
    uint8_t address;
    uint8_t contentLen;
    uint8_t content[2];
    FSUS_STATUS status = this->protocol->recvReadData(&servoId, &address, &contentLen, content);
    if (status == FSUS_STATUS_SUCCESS){
        // 提取数据
        int16_t power;
        byte* valuePtr = (byte*)&power;
        valuePtr[0] = content[0];
        valuePtr[1] = content[1];
        return power;
    }
    return 0;
}

/* 查询舵机的温度(单位 ℃) */
uint16_t FSUS_Servo::queryTemperature(){
     // 发送数据
    this->protocol->sendReadData(this->servoId, FSUS_PARAM_TEMPRATURE);
    // 接收数据
    FSUS_SERVO_ID_T servoId;
    uint8_t address;
    uint8_t contentLen;
    uint8_t content[2];
    FSUS_STATUS status = this->protocol->recvReadData(&servoId, &address, &contentLen, content);
    if (status == FSUS_STATUS_SUCCESS){
        // 提取数据
        int16_t temprature;
        byte* valuePtr = (byte*)&temprature;
        valuePtr[0] = content[0];
        valuePtr[1] = content[1];
        return temprature;
    }
    return 0;
}

/* 设置舵机为阻尼模式 */
void FSUS_Servo::setDamping(FSUS_POWER_T power){
    this->protocol->sendDammping(this->servoId, power);
}

/* 设置舵机为阻尼模式, 默认功率为500mW*/
void FSUS_Servo::setDamping(){
    setDamping(500);
}

/* 舵机扭力开关 */
void FSUS_Servo::setTorque(bool enable){
    if(enable){
        queryAngle(); // 查询舵机角度
        setAngle(this->curAngle); //设置角度为当前的角度
    }else{
        setDamping(0); // 设置为阻尼模式, 功率为0
    }
}

/* 判断舵机是否停止 */
bool FSUS_Servo::isStop(){
    if(this->isMTurn){
        queryRawAngleMTurn(); // 查询原始角度(多圈)
    }else{
        queryRawAngle(); // 查询舵机角度
    }
    

    if (this->protocol->responsePack.recv_status != FSUS_STATUS_SUCCESS){
        // 舵机角度查询失败
        return false;
    }

    return abs(this->curRawAngle - this->targetRawAngle) <= FSUS_ANGLE_CTL_DEADBLOCK;
}

/* 舵机等待 */
void FSUS_Servo::wait(){
    int32_t t_start = millis();
    // 角度误差
    float dAngle = abs(this->targetRawAngle - this->curRawAngle);
    
    while (!isStop()){
        // 角度误差保持不变(判断条件1°)
        if(abs(dAngle - abs(this->targetRawAngle - this->curRawAngle)) <= 1.0){
            // 判断是否发生卡死的情况
            if(millis() - t_start >= FSUS_WAIT_TIMEOUT_MS){
                // 等待超过1s
                break;
            }
        }else{
            // 更新t_start
            t_start = millis();
            // 更新角度误差
            dAngle = abs(this->targetRawAngle - this->curRawAngle);
        }
    }
}

// 轮子停止
void FSUS_Servo::wheelStop(){
    this->protocol->sendWheelStop(this->servoId);
}

// 轮子旋转
void FSUS_Servo::wheelRun(uint8_t is_cw){
    this->protocol->sendWheelKeepMove(this->servoId, is_cw, this->speed);
}

// 轮子旋转特定的时间
void FSUS_Servo::wheelRunNTime(uint8_t is_cw, uint16_t time_ms){
    this->protocol->sendWheelMoveTime(this->servoId, is_cw, this->speed, time_ms);
}

// 旋转圈数
void FSUS_Servo::wheelRunNCircle(uint8_t is_cw, uint16_t circle_num){
    this->protocol->sendWheelMoveNCircle(this->servoId, is_cw, this->speed, circle_num);
}

// 检查轮子是否停止
bool FSUS_Servo::wheelIsStop(){
    // TODO
    return false;
}
