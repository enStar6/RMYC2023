/*
 * 用来处理舵机的底层通信协议
 *--------------------------
 * 作者: 深圳市华馨京科技有限公司
 * 网站：https://fashionrobo.com/
 * 更新时间: 2023/03/13
 */
#include "FashionStar_UartServoProtocol.h"

FSUS_Protocol::FSUS_Protocol(HardwareSerial * serial, uint32_t baudrate){
    this->serial = serial;
    this->baudrate = baudrate;
    // 初始化波特率
    serial->begin(baudrate);
}

FSUS_Protocol::FSUS_Protocol(uint32_t baudrate){
    this->baudrate = baudrate;
}

FSUS_Protocol::FSUS_Protocol(){
    this->baudrate = 115200; // 设置默认的波特率
}

void FSUS_Protocol::init(){
#if defined(ARDUINO_AVR_UNO)
    Serial.begin(baudrate); // 设置波特率
    this->serial = &Serial; // 硬件串口指针
#elif defined(ARDUINO_AVR_MEGA2560)
    Serial3.begin(baudrate); // 设置波特率
    this->serial = &Serial3; // 硬件串口指针
#elif defined(ARDUINO_ARCH_ESP32)
    // Serial2初始化
    // API Serial2.begin(baud-rate, protocol, RX pin, TX pin);
    Serial2.begin(baudrate);
    this->serial = &Serial2;

#elif defined(ARDUINO_ARCH_STM32)
 Serial2.begin(baudrate); // 设置波特率
 this->serial = &Serial2; // 硬件串口指针

#else
#error "This library only supports boards with an AVR, ESP32."
#endif
}

void FSUS_Protocol::init(uint32_t baudrate){
    this->baudrate = baudrate;
#if defined(ARDUINO_AVR_UNO)
    Serial.begin(this->baudrate);
    this->serial = &Serial;
#elif defined(ARDUINO_AVR_MEGA2560)
    Serial3.begin(baudrate); // 设置波特率
    this->serial = &Serial3; // 硬件串口指针
#elif defined(ARDUINO_ARCH_ESP32)
    Serial2.begin(baudrate);
    this->serial = &Serial2;

#elif defined(ARDUINO_ARCH_STM32)
 Serial2.begin(baudrate); // 设置波特率
 this->serial = &Serial2; // 硬件串口指针

#endif

}

void FSUS_Protocol::init(HardwareSerial * serial, uint32_t baudrate){
    this->baudrate = baudrate;
    serial->begin(baudrate);
    this->serial = serial;
}


//加工并发送请求数据包
void FSUS_Protocol::sendPack(){
    // 数据加工
    requestPack.header = FSUS_PACK_REQUEST_HEADER; // 数据头
    // 计算校验码
    requestPack.checksum = calcPackChecksum(&requestPack);
    // 通过串口发送出去
    serial->write(requestPack.header & 0xFF);
    serial->write(requestPack.header >> 8);
    serial->write(requestPack.cmdId);
    serial->write(requestPack.content_size);
    for(uint16_t i=0; i<requestPack.content_size; i++){
        serial->write(requestPack.content[i]);
    }
    serial->write(requestPack.checksum);
}

// 清空缓冲区
void FSUS_Protocol::emptyCache(){
    // 清空UART接收缓冲区
    while(serial->available()){
        serial->read();
    }
}

// 初始化响应数据包的数据结构
void FSUS_Protocol::initResponsePack(){
    // 初始化响应包的数据
    responsePack.header = 0;
    responsePack.cmdId = 0;
    responsePack.content_size = 0;
    responsePack.checksum = 0;
    responsePack.recv_cnt = 0;
}

//接收响应包 
FSUS_STATUS FSUS_Protocol::recvPack(){
    uint32_t start_time = millis();
    
    // responsePack.recv_status = 0; //重置标志位
    responsePack.recv_cnt = 0; // 数据帧接收标志位
    
    while(true){
        // 超时判断
        if((millis() - start_time) > FSUS_TIMEOUT_MS){
            return FSUS_STATUS_TIMEOUT;
        }
        // 等待有字节读入
        if (serial->available()==0){
            continue;
        }
        uint8_t curByte = serial->read();
        uint8_t curIdx = responsePack.recv_cnt;
        responsePack.recv_buffer[curIdx] = curByte; // 接收一个字节
        responsePack.recv_cnt+=1; // 计数自增
        
        // 数据接收是否完成
        if(curIdx==1){
            // 校验帧头
            responsePack.header = responsePack.recv_buffer[curIdx-1] | curByte << 8;
            if(responsePack.header!=FSUS_PACK_RESPONSE_HEADER){
                return FSUS_STATUS_WRONG_RESPONSE_HEADER;
            }
        }else if (curIdx==2){
            // 载入cmdId
            responsePack.cmdId = curByte;
            // 检查cmdId是否满足指令范围
            if(responsePack.cmdId > FSUS_CMD_NUM){
                return FSUS_STATUS_UNKOWN_CMD_ID;
            }
        }else if (curIdx==3){
            // 载入Size
            responsePack.content_size = curByte;
            // 判断size是否合法
            if(responsePack.content_size > FSUS_PACK_RESPONSE_MAX_SIZE){
                return FSUS_STATUS_SIZE_TOO_BIG;
            }
        }else if (curIdx < 4+responsePack.content_size){
            // 填充内容
            responsePack.content[curIdx-4] = curByte;
        }else{
            // 接收校验合
            responsePack.checksum = curByte;
            // 检查校验和是否匹配
            FSUS_CHECKSUM_T checksum = calcPackChecksum(&responsePack);
            // if (responsePack.cmdId == FSUS_CMD_QUERY_ANGLE){
            //     checksum -= 0x03;// TODO Delete 不知道为什么要这样
            // }
            
            if (checksum != responsePack.checksum){
                return FSUS_STATUS_CHECKSUM_ERROR;
            }else{
                return FSUS_STATUS_SUCCESS;
            }
        }
    }
}

//计算CRC校验码
FSUS_CHECKSUM_T FSUS_Protocol::calcPackChecksum(const FSUS_PACKAGE_T *package){
    // uint16_t checksum = 0;
    uint16_t checksum = 0;
    checksum += (package->header & 0xFF);
    checksum += (package->header >> 8);
    checksum += package->cmdId;
    checksum += package->content_size;
    for(uint16_t i=0; i<package->content_size; i++){
        checksum += package->content[i];
    }
    
    return (FSUS_CHECKSUM_T)(checksum%256);
}
// 获取请求内容的尺寸
FSUS_PACKAGE_SIZE_T FSUS_Protocol::getPackSize(const FSUS_PACKAGE_T *package){
    // 包头(2 byte) + 指令ID(1byte) + 长度(1byte) + 内容 + 校验码(1byte)
    return package->content_size + 5;
}

// 发送PING的请求包
void FSUS_Protocol::sendPing(FSUS_SERVO_ID_T servoId){
    requestPack.cmdId = FSUS_CMD_PING;
    requestPack.content_size = 1;
    requestPack.content[0] = servoId;
    sendPack(); // 发送包
}

// 接收PING的响应包
FSUS_STATUS FSUS_Protocol::recvPing(FSUS_SERVO_ID_T* servoId, bool *isOnline){
    // 接收数据帧
    FSUS_STATUS status = recvPack();
    
    *servoId = responsePack.content[0]; // 提取舵机ID
    *isOnline = (status == FSUS_STATUS_SUCCESS);
    responsePack.recv_status = status;
    return status;
}

// 发送旋转的请求包
void FSUS_Protocol::sendSetAngle(FSUS_SERVO_ID_T servoId, FSUS_SERVO_ANGLE_T angle,FSUS_INTERVAL_T interval,FSUS_POWER_T power){
    requestPack.cmdId = FSUS_CMD_SET_ANGLE; // 指令ID
    requestPack.content_size = 7; // 内容长度
    requestPack.content[0]=servoId; //舵机ID
    int16_t angle_int = angle * 10; //舵机的角度
    requestPack.content[1] = angle_int & 0xFF;
    requestPack.content[2] = angle_int >> 8;
    requestPack.content[3] = interval & 0xFF; //周期
    requestPack.content[4] = interval >> 8;
    requestPack.content[5] = power & 0xFF; //功率
    requestPack.content[6] = power >> 8;
    sendPack();
}

// 发送旋转的请求包(指定周期)
void FSUS_Protocol::sendSetAngleByInterval(FSUS_SERVO_ID_T servoId, FSUS_SERVO_ANGLE_T angle,FSUS_INTERVAL_T interval, FSUS_INTERVAL_T t_acc, FSUS_INTERVAL_T t_dec, FSUS_POWER_T power){
    requestPack.cmdId = FSUS_CMD_SET_ANGLE_BY_INTERVAL; // 指令ID
    requestPack.content_size = 11; // 内容长度
    requestPack.content[0]=servoId; //舵机ID
    int16_t angle_int = angle * 10; //舵机的角度
    requestPack.content[1] = angle_int & 0xFF;
    requestPack.content[2] = angle_int >> 8;
    requestPack.content[3] = interval & 0xFF; //周期
    requestPack.content[4] = interval >> 8;
    requestPack.content[5] = t_acc & 0xFF;
    requestPack.content[6] = t_acc >> 8;
    requestPack.content[7] = t_dec & 0xFF;
    requestPack.content[8] = t_dec >> 8;
    requestPack.content[9] = power & 0xFF; //功率
    requestPack.content[10] = power >> 8;
    sendPack();
}

// 发送旋转的请求包(指定转速)
void FSUS_Protocol::sendSetAngleByVelocity(FSUS_SERVO_ID_T servoId, FSUS_SERVO_ANGLE_T angle,FSUS_SERVO_SPEED_T velocity, FSUS_INTERVAL_T t_acc, FSUS_INTERVAL_T t_dec, FSUS_POWER_T power){
    requestPack.cmdId = FSUS_CMD_SET_ANGLE_BY_VELOCITY; // 指令ID
    requestPack.content_size = 11;           // 内容长度
    requestPack.content[0]=servoId;         //舵机ID
    int16_t angle_int = angle * 10;             //舵机的角度
    uint16_t velocity_int = velocity * 10; // 转速 
    requestPack.content[1] = angle_int & 0xFF;
    requestPack.content[2] = angle_int >> 8;
    requestPack.content[3] = velocity_int & 0xFF; //周期
    requestPack.content[4] = velocity_int >> 8;
    requestPack.content[5] = t_acc & 0xFF;
    requestPack.content[6] = t_acc >> 8;
    requestPack.content[7] = t_dec & 0xFF;
    requestPack.content[8] = t_dec >> 8;
    requestPack.content[9] = power & 0xFF; //功率
    requestPack.content[10] = power >> 8;
    sendPack();
}

// 发送舵机角度查询指令
void FSUS_Protocol::sendQueryAngle(FSUS_SERVO_ID_T servoId){
    requestPack.cmdId = FSUS_CMD_QUERY_ANGLE;
    requestPack.content_size = 1;
    requestPack.content[0] = servoId;
    sendPack();
}

// 接收角度查询指令
FSUS_STATUS FSUS_Protocol::recvQueryAngle(FSUS_SERVO_ID_T *servoId, FSUS_SERVO_ANGLE_T *angle){
    FSUS_STATUS status = recvPack();
    int16_t angleVal;
    byte* angleValPtr = (byte*)&angleVal;
    
    // angleVal = responsePack.content[1] + responsePack.content[2] << 8;
    // if(status == FSUS_STATUS_SUCCESS){
    // 偶尔会出现校验和错误的情况, 临时允许
    if(status == FSUS_STATUS_SUCCESS || status==FSUS_STATUS_CHECKSUM_ERROR){
        (*servoId) = responsePack.content[0];
        angleValPtr[0] = responsePack.content[1];
        angleValPtr[1] = responsePack.content[2];
        (*angle) = 0.1*(float)angleVal;
        // (*angle) = 0.1 * (uint16_t)(responsePack.content[1] | responsePack.content[2]<< 8);
    }
    responsePack.recv_status = status;
    return status;
}

// 发送旋转的请求包(多圈)
void FSUS_Protocol::sendSetAngleMTurn(FSUS_SERVO_ID_T servoId, FSUS_SERVO_ANGLE_T angle,FSUS_INTERVAL_T_MTURN interval,FSUS_POWER_T power){
    requestPack.cmdId = FSUS_CMD_SET_ANGLE_MTURN; // 指令ID
    requestPack.content_size = 11; // 内容长度
    requestPack.content[0]=servoId; //舵机ID
    int32_t angle_long = angle * 10; //舵机的角度
    // 角度
    requestPack.content[1] = angle_long & 0xFF;
    requestPack.content[2] = (angle_long >> 8) & 0xFF;
    requestPack.content[3] = (angle_long >> 16) & 0xFF;
    requestPack.content[4] = (angle_long >> 24) & 0xFF;
    // 周期
    requestPack.content[5] = interval & 0xFF;
    requestPack.content[6] = (interval >> 8) & 0xFF;
    requestPack.content[7] = (interval >> 16) & 0xFF;
    requestPack.content[8] = (interval >> 24) & 0xFF;
    // 功率
    requestPack.content[9] = power & 0xFF;
    requestPack.content[10] = (power >> 8) & 0xFF;
    sendPack();
}

// 发送旋转的请求包(多圈+指定周期)
void FSUS_Protocol::sendSetAngleMTurnByInterval(FSUS_SERVO_ID_T servoId, FSUS_SERVO_ANGLE_T angle, FSUS_INTERVAL_T_MTURN interval, \
    FSUS_INTERVAL_T t_acc, FSUS_INTERVAL_T t_dec, FSUS_POWER_T power){
    
    requestPack.cmdId = FSUS_CMD_SET_ANGLE_MTURN_BY_INTERVAL; // 指令ID
    requestPack.content_size = 15; // 内容长度
    requestPack.content[0]=servoId; //舵机ID
    int32_t angle_long = angle * 10; //舵机的角度
    // 角度
    requestPack.content[1] = angle_long & 0xFF;
    requestPack.content[2] = (angle_long >> 8) & 0xFF;
    requestPack.content[3] = (angle_long >> 16) & 0xFF;
    requestPack.content[4] = (angle_long >> 24) & 0xFF;
    // 周期
    requestPack.content[5] = interval & 0xFF;
    requestPack.content[6] = (interval >> 8) & 0xFF;
    requestPack.content[7] = (interval >> 16) & 0xFF;
    requestPack.content[8] = (interval >> 24) & 0xFF;
    // 加速时间
    requestPack.content[9] = t_acc & 0xFF;
    requestPack.content[10] = (t_acc >> 8) & 0xFF;
    // 减速时间
    requestPack.content[11] = t_dec & 0xFF;
    requestPack.content[12] = (t_dec >> 8) & 0xFF;
    // 功率
    requestPack.content[13] = power & 0xFF;
    requestPack.content[14] = (power >> 8) & 0xFF;
    sendPack();
}

// 发送旋转的请求包(多圈+指定转速)
void FSUS_Protocol::sendSetAngleMTurnByVelocity(FSUS_SERVO_ID_T servoId, FSUS_SERVO_ANGLE_T angle, FSUS_SERVO_SPEED_T velocity, \
    FSUS_INTERVAL_T t_acc, FSUS_INTERVAL_T t_dec, FSUS_POWER_T power){
    // 请求头
    requestPack.cmdId = FSUS_CMD_SET_ANGLE_MTURN_BY_VELOCITY; // 指令ID
    requestPack.content_size = 13;             // 内容长度
    requestPack.content[0]=servoId;            //舵机ID
    int32_t angle_long = angle * 10;              //舵机的角度
    uint16_t velocity_int = velocity * 10; // 舵机转速 单位0.1°/s
    // 角度
    requestPack.content[1] = angle_long & 0xFF;
    requestPack.content[2] = (angle_long >> 8) & 0xFF;
    requestPack.content[3] = (angle_long >> 16) & 0xFF;
    requestPack.content[4] = (angle_long >> 24) & 0xFF;
    // 周期
    requestPack.content[5] = velocity_int & 0xFF;
    requestPack.content[6] = (velocity_int >> 8) & 0xFF;
    // 加速时间
    requestPack.content[7] = t_acc & 0xFF;
    requestPack.content[8] = (t_acc >> 8) & 0xFF;
    // 减速时间
    requestPack.content[9] = t_dec & 0xFF;
    requestPack.content[10] = (t_dec >> 8) & 0xFF;
    // 功率
    requestPack.content[11] = power & 0xFF;
    requestPack.content[12] = (power >> 8) & 0xFF;
    sendPack();
}

// 发送舵机角度查询指令
void FSUS_Protocol::sendQueryAngleMTurn(FSUS_SERVO_ID_T servoId){
    requestPack.cmdId = FSUS_CMD_QUERY_ANGLE_MTURN;
    requestPack.content_size = 1;
    requestPack.content[0] = servoId;
    sendPack();
}

// 接收角度查询指令(多圈模式)
FSUS_STATUS FSUS_Protocol::recvQueryAngleMTurn(FSUS_SERVO_ID_T *servoId, FSUS_SERVO_ANGLE_T *angle){
    FSUS_STATUS status = recvPack();
    int32_t angleVal;
    byte* angleValPtr = (byte*)&angleVal;
    
    // 偶尔会出现校验和错误的情况, 临时允许
    if(status == FSUS_STATUS_SUCCESS || status==FSUS_STATUS_CHECKSUM_ERROR){
        (*servoId) = responsePack.content[0];
        
        angleValPtr[0] = responsePack.content[1];
        angleValPtr[1] = responsePack.content[2];
        angleValPtr[2] = responsePack.content[3];
        angleValPtr[3] = responsePack.content[4];

        (*angle) = 0.1*angleVal;
    }
    responsePack.recv_status = status;
    return status;
}

// 发送阻尼模式
void FSUS_Protocol::sendDammping(FSUS_SERVO_ID_T servoId, FSUS_POWER_T power){
    requestPack.cmdId = FSUS_CMD_DAMPING;
    requestPack.content_size = 3;
    requestPack.content[0] = servoId;
    requestPack.content[1] = power & 0xFF;
    requestPack.content[2] = power >> 8;
    sendPack();
}

// 轮式控制模式
void FSUS_Protocol::sendWheelMove(FSUS_SERVO_ID_T servoId, uint8_t method, uint16_t speed, uint16_t value){
    requestPack.cmdId = FSUS_CMD_WHEEL;
    requestPack.content_size = 6;
    requestPack.content[0] = servoId;
    requestPack.content[1] = method;
    requestPack.content[2] = speed & 0xFF;
    requestPack.content[3] = speed >> 8;
    requestPack.content[4] = value & 0xFF;
    requestPack.content[5] = value >> 8;
    sendPack();
}

// 轮子停止转动
void FSUS_Protocol::sendWheelStop(FSUS_SERVO_ID_T servoId){
    uint8_t method = 0;
    uint16_t speed = 0;
	uint16_t value = 0;
    sendWheelMove(servoId, method, speed, value);
}

// 轮子持续旋转
void FSUS_Protocol::sendWheelKeepMove(FSUS_SERVO_ID_T servoId, uint8_t is_cw, uint16_t speed){
    uint8_t method = 0x01; // 持续旋转
	if (is_cw){
		// 顺时针旋转
		method = method | 0x80;
	}
	uint16_t value = 0;
    sendWheelMove(servoId, method, speed, value);
}

// 指定旋转的时间
void FSUS_Protocol::sendWheelMoveTime(FSUS_SERVO_ID_T servoId, uint8_t is_cw, uint16_t speed, uint16_t nTime){
    uint8_t method = 0x03; // 旋转一段时间
	if (is_cw){
		// 顺时针旋转
		method = method | 0x80;
	}
    sendWheelMove(servoId, method, speed, nTime);
}

// 轮式模式 旋转特定的圈数
void FSUS_Protocol::sendWheelMoveNCircle(FSUS_SERVO_ID_T servoId, uint8_t is_cw, uint16_t speed, uint16_t nCircle){
    uint8_t method = 0x02; // 旋转特定的圈数
	if (is_cw){
		// 顺时针旋转
		method = method | 0x80;
	}
    sendWheelMove(servoId, method, speed, nCircle);
}

// 发送重置用户数据
void FSUS_Protocol::sendResetUserData(FSUS_SERVO_ID_T servoId){
    requestPack.cmdId = FSUS_CMD_RESET_USER_DATA;
    requestPack.content_size = 1;
    requestPack.content[0] = servoId;
    sendPack();
}

// 接收重置用户数据
FSUS_STATUS FSUS_Protocol::recvResetUserData(FSUS_SERVO_ID_T *servoId, bool *result){
    FSUS_STATUS status = recvPack();
    if(status == FSUS_STATUS_SUCCESS){
        *servoId = responsePack.content[0];
        *result = responsePack.content[1];
    }

    return status;
}

// 发送数据读取指令
void FSUS_Protocol::sendReadData(FSUS_SERVO_ID_T servoId, uint8_t address){
    requestPack.cmdId = FSUS_CMD_READ_DATA;
    requestPack.content_size = 2;
    requestPack.content[0] = servoId;
    requestPack.content[1] = address;
    sendPack();
}

// 接收数据读取指令
FSUS_STATUS FSUS_Protocol::recvReadData(FSUS_SERVO_ID_T *servoId, uint8_t *address, uint8_t *contentLen, uint8_t *content){
    FSUS_STATUS status = recvPack();
    if(status == FSUS_STATUS_SUCCESS){
        *servoId = responsePack.content[0]; 
        *address = responsePack.content[1]; // 获取地址位
        *contentLen = responsePack.content_size - 2; // 计算得到数据位的长度
        // 数据拷贝
        for(uint16_t i=0; i<*contentLen; i++){
            content[i] = responsePack.content[i+2];
        }
    }
    return status;
}

// 发送数据写入指令
void FSUS_Protocol::sendWriteData(FSUS_SERVO_ID_T servoId, uint8_t address, uint8_t contentLen, uint8_t *content){
    requestPack.cmdId = FSUS_CMD_WRITE_DATA;
    requestPack.content_size = 2+contentLen;
    requestPack.content[0] = servoId;
    requestPack.content[1] = address;
    for(uint16_t i=0; i<contentLen; i++){
        requestPack.content[i+2] = content[i];
    }
    sendPack();
}

// 接收数据写入指令
FSUS_STATUS FSUS_Protocol::recvWriteData(FSUS_SERVO_ID_T *servoId, uint8_t *address, bool *result){
    FSUS_STATUS status = recvPack();
    if(status == FSUS_STATUS_SUCCESS){
        *servoId = responsePack.content[0]; 
        *address = responsePack.content[1]; // 获取地址位
        *result = responsePack.content[2];
    }
    return status;
}
