/*
 * Fashion Star 串口舵机驱动库
 * Version: v0.0.1
 * UpdateTime: 2021/02/19
 */
#include "fashion_star_uart_servo.h"
#include "math.h"

// 数据帧转换为字节数组
void FSUS_Package2RingBuffer(PackageTypeDef *pkg,  RingBufferTypeDef *ringBuf){
    uint8_t checksum; // 校验和
    // 写入帧头
    RingBuffer_WriteUShort(ringBuf, pkg->header);
    // 写入指令ID
    RingBuffer_WriteByte(ringBuf, pkg->cmdId);
    // 写入包的长度
    RingBuffer_WriteByte(ringBuf, pkg->size);
    // 写入内容主题
    RingBuffer_WriteByteArray(ringBuf, pkg->content, pkg->size);
    // 计算校验和
    checksum = RingBuffer_GetChecksum(ringBuf);
    // 写入校验和
    RingBuffer_WriteByte(ringBuf, checksum);

}

// 计算Package的校验和
uint8_t FSUS_CalcChecksum(PackageTypeDef *pkg){
    uint8_t checksum;
	// 初始化环形队列
	RingBufferTypeDef ringBuf;
	uint8_t pkgBuf[FSUS_PACK_RESPONSE_MAX_SIZE+1];
	RingBuffer_Init(&ringBuf, FSUS_PACK_RESPONSE_MAX_SIZE, pkgBuf);
    // 将Package转换为ringbuffer
	// 在转换的时候,会自动的计算checksum
    FSUS_Package2RingBuffer(pkg, &ringBuf);
	// 获取环形队列队尾的元素(即校验和的位置)
	checksum = RingBuffer_GetValueByIndex(&ringBuf, RingBuffer_GetByteUsed(&ringBuf)-1);
    return checksum;
}

// 判断是否为有效的请求头的
FSUS_STATUS FSUS_IsValidResponsePackage(PackageTypeDef *pkg){
    // 帧头数据不对
    if (pkg->header != FSUS_PACK_RESPONSE_HEADER){
        // 帧头不对
        return FSUS_STATUS_WRONG_RESPONSE_HEADER;
    }
    // 判断控制指令是否有效 指令范围超出
    if (pkg->cmdId > FSUS_CMD_NUM){
        return FSUS_STATUS_UNKOWN_CMD_ID;
    }
    // 参数的size大于FSUS_PACK_RESPONSE_MAX_SIZE里面的限制
    if (pkg->size > (FSUS_PACK_RESPONSE_MAX_SIZE - 5)){
        return FSUS_STATUS_SIZE_TOO_BIG;
    }
    // 校验和不匹配
    if (FSUS_CalcChecksum(pkg) != pkg->checksum){
        return FSUS_STATUS_CHECKSUM_ERROR;
    }
    // 数据有效
    return FSUS_STATUS_SUCCESS;
}

// 字节数组转换为数据帧
FSUS_STATUS FSUS_RingBuffer2Package(RingBufferTypeDef *ringBuf, PackageTypeDef *pkg){
    // 申请内存
    pkg = (PackageTypeDef *)malloc(sizeof(PackageTypeDef));
    // 读取帧头
    pkg->header = RingBuffer_ReadUShort(ringBuf);
    // 读取指令ID
    pkg->cmdId = RingBuffer_ReadByte(ringBuf);
    // 读取包的长度
    pkg->size = RingBuffer_ReadByte(ringBuf);
    // 申请参数的内存空间
    // pkg->content = (uint8_t *)malloc(pkg->size);
    // 写入content
    RingBuffer_ReadByteArray(ringBuf, pkg->content, pkg->size);
    // 写入校验和
    pkg->checksum = RingBuffer_ReadByte(ringBuf);
    // 返回当前的数据帧是否为有效反馈数据帧
    return FSUS_IsValidResponsePackage(pkg);
}

// 构造发送数据帧
void FSUS_SendPackage(Usart_DataTypeDef *usart, uint8_t cmdId, uint8_t size, uint8_t *content){
    // 申请内存
	// printf("[Package] malloc for pkg\r\n");
    PackageTypeDef pkg;
	
    // 设置帧头
    pkg.header = FSUS_PACK_REQUEST_HEADER;
    // 设置指令ID
    pkg.cmdId = cmdId;
    // 设置尺寸
    pkg.size = size;
	// 逐一拷贝数组里面的内容
	for(int i=0; i<size; i++){
		pkg.content[i] = content[i];
	}
    // 将pkg发送到发送缓冲区sendBuf里面
    FSUS_Package2RingBuffer(&pkg, usart->sendBuf);
	// 通过串口将数据发送出去
    Usart_SendAll(usart);
}

// 接收数据帧 (在接收的时候动态的申请内存)
FSUS_STATUS FSUS_RecvPackage(Usart_DataTypeDef *usart, PackageTypeDef *pkg){
	
	pkg->status = 0; // Package状态初始化
	
    uint8_t bIdx = 0; // 接收的参数字节索引号
    uint16_t header = 0; // 帧头
	
    // 倒计时开始
    SysTick_CountdownBegin(FSUS_TIMEOUT_MS);

    // 如果没有超时
    while (!SysTick_CountdownIsTimeout()){
        if (RingBuffer_GetByteUsed(usart->recvBuf) == 0){
			// 没有新的字节读进来, 继续等待
            continue;
        }

        // 查看校验码是否已经接收到
        if(pkg->status & FSUS_RECV_FLAG_CONTENT){
            // 参数内容接收完成
            // 接收校验码
            pkg->checksum = RingBuffer_ReadByte(usart->recvBuf);
            // 设置状态码-校验码已经接收
            pkg->status = pkg->status | FSUS_RECV_FLAG_CHECKSUM;
            // 直接进行校验码检查
            if (FSUS_CalcChecksum(pkg) != pkg->checksum){
                // 撤销倒计时
                SysTick_CountdownCancel();
                // 校验码错误
                return FSUS_STATUS_CHECKSUM_ERROR;
            }else{
                // 撤销倒计时
                SysTick_CountdownCancel();
                // 数据帧接收成功
                return FSUS_STATUS_SUCCESS;
            }
        }else if(pkg->status & FSUS_RECV_FLAG_SIZE){
            // Size已经接收完成
            // 接收参数字节
            pkg->content[bIdx] = RingBuffer_ReadByte(usart->recvBuf);
            bIdx ++;
            // 判断是否接收完成
            if (bIdx == pkg->size){
                // 标记为参数接收完成
                pkg->status = pkg->status | FSUS_RECV_FLAG_CONTENT;
            }
        }else if(pkg->status & FSUS_RECV_FLAG_CMD_ID){
            // 指令接收完成
            // 接收尺寸信息
            pkg->size = RingBuffer_ReadByte(usart->recvBuf);
            // 判断长度size是否合法
            // 参数的size大于FSUS_PACK_RESPONSE_MAX_SIZE里面的限制
            if (pkg->size > (FSUS_PACK_RESPONSE_MAX_SIZE - 5)){
                // 撤销倒计时
                SysTick_CountdownCancel();
                return FSUS_STATUS_SIZE_TOO_BIG;
            }
            // 设置尺寸接收完成的标志位
            pkg->status = pkg->status | FSUS_RECV_FLAG_SIZE;
        }else if(pkg->status & FSUS_RECV_FLAG_HEADER){
            // 帧头已接收 
            // 接收指令ID
            pkg->cmdId = RingBuffer_ReadByte(usart->recvBuf);
            // 判断指令是否合法
            // 判断控制指令是否有效 指令范围超出
            if (pkg->cmdId > FSUS_CMD_NUM){
                // 撤销倒计时
                SysTick_CountdownCancel();
                return FSUS_STATUS_UNKOWN_CMD_ID;
            }
            // 设置cmdId已经接收到标志位
            pkg->status = pkg->status | FSUS_RECV_FLAG_CMD_ID;
        }else{
            // 接收帧头
            if (header == 0){
                // 接收第一个字节
                header = RingBuffer_ReadByte(usart->recvBuf);
                // 判断接收的第一个字节对不对
                if (header != (FSUS_PACK_RESPONSE_HEADER&0x0F)){
                    // 第一个字节不对 header重置为0
                    header = 0;
                }
            }else if(header == (FSUS_PACK_RESPONSE_HEADER&0x0F)){
                // 接收帧头第二个字节
                header =  header | (RingBuffer_ReadByte(usart->recvBuf) << 8);
				// 检查第二个字节是否正确
                if(header != FSUS_PACK_RESPONSE_HEADER){
                    header = 0;
                }else{
                    pkg->header = header;
                    // 帧头接收成功
                    pkg->status = pkg->status | FSUS_RECV_FLAG_HEADER; 
                }
            }else{
                header = 0;
            }
        }
    }
    // 等待超时
    return FSUS_STATUS_TIMEOUT;
}

// 舵机通讯检测
// 注: 如果没有舵机响应这个Ping指令的话, 就会超时
FSUS_STATUS FSUS_Ping(Usart_DataTypeDef *usart, uint8_t servo_id){
	uint8_t statusCode; // 状态码
	uint8_t ehcoServoId; // PING得到的舵机ID
	// printf("[PING]Send Ping Package\r\n");
	// 发送请求包
	FSUS_SendPackage(usart, FSUS_CMD_PING, 1, &servo_id);
	// 接收返回的Ping
	PackageTypeDef pkg;
	statusCode = FSUS_RecvPackage(usart, &pkg);
	if(statusCode == FSUS_STATUS_SUCCESS){
		// 进一步检查ID号是否匹配
		ehcoServoId = (uint8_t)pkg.content[0];
		if (ehcoServoId != servo_id){
			// 反馈得到的舵机ID号不匹配
			return FSUS_STATUS_ID_NOT_MATCH;
		}
	}
	return statusCode;
}

// 重置舵机的用户资料
FSUS_STATUS FSUS_ResetUserData(Usart_DataTypeDef *usart, uint8_t servo_id){
	const uint8_t size = 1;
	FSUS_STATUS statusCode;
	// 发送请求包
	FSUS_SendPackage(usart, FSUS_CMD_RESET_USER_DATA, size, &servo_id);
	// 接收重置结果
	PackageTypeDef pkg;
	statusCode = FSUS_RecvPackage(usart, &pkg);
	if (statusCode == FSUS_STATUS_SUCCESS){
		// 成功的接收到反馈数据
		// 读取反馈数据中的result
		uint8_t result = (uint8_t)pkg.content[1];
		if (result == 1){
			return FSUS_STATUS_SUCCESS;
		}else{
			return FSUS_STATUS_FAIL;
		}
	}
	return statusCode;
}

// 读取数据
FSUS_STATUS FSUS_ReadData(Usart_DataTypeDef *usart, uint8_t servo_id,  uint8_t address, uint8_t *value, uint8_t *size){
	FSUS_STATUS statusCode;
	// 构造content
	uint8_t buffer[2] = {servo_id, address};
	// 发送请求数据
	FSUS_SendPackage(usart, FSUS_CMD_READ_DATA, 2, buffer);
	// 接收返回信息
	PackageTypeDef pkg;
	statusCode = FSUS_RecvPackage(usart, &pkg);
	if (statusCode == FSUS_STATUS_SUCCESS){
		// 读取数据
		// 读取数据是多少个位
		*size = pkg.size - 2; // content的长度减去servo_id跟address的长度
		// 数据拷贝
		for (int i=0; i<*size; i++){
			value[i] = pkg.content[i+2];
		}
	}
	return statusCode;
}

// 写入数据
FSUS_STATUS FSUS_WriteData(Usart_DataTypeDef *usart, uint8_t servo_id, uint8_t address, uint8_t *value, uint8_t size){
	FSUS_STATUS statusCode;
	// 构造content
	uint8_t buffer[size+2]; // 舵机ID + 地址位Address + 数据byte数
	buffer[0] = servo_id;
	buffer[1] = address;
	// 拷贝数据
	for (int i=0; i<size; i++){
		buffer[i+2] = value[i];
	}
	// 发送请求数据
	FSUS_SendPackage(usart, FSUS_CMD_WRITE_DATA, size+2, buffer);
	// 接收返回信息
	PackageTypeDef pkg;
	statusCode = FSUS_RecvPackage(usart, &pkg);
	if (statusCode == FSUS_STATUS_SUCCESS){
		uint8_t result = pkg.content[2];
		if(result == 1){
			statusCode = FSUS_STATUS_SUCCESS;
		}else{
			statusCode = FSUS_STATUS_FAIL;
		}
	}
	return statusCode;
}



/* 
 * 轮转控制模式
 * speed单位 °/s
 */
FSUS_STATUS FSUS_WheelMove(Usart_DataTypeDef *usart, uint8_t servo_id, uint8_t method, uint16_t speed, uint16_t value){
	// 创建环形缓冲队列
	const uint8_t size = 6;
	uint8_t buffer[size+1];
	RingBufferTypeDef ringBuf;
	RingBuffer_Init(&ringBuf, size, buffer);
	// 写入content
	RingBuffer_WriteByte(&ringBuf, servo_id);  // 舵机ID
	RingBuffer_WriteByte(&ringBuf, method);   // 写入执行方式与旋转方向
	RingBuffer_WriteUShort(&ringBuf, speed);  // 设置舵机的旋转速度 °/s
	RingBuffer_WriteUShort(&ringBuf, value);
	// 发送请求包
	FSUS_SendPackage(usart, FSUS_CMD_SPIN, size, buffer+1);
	
	return FSUS_STATUS_SUCCESS;
}

// 轮转模式, 舵机停止转动
FSUS_STATUS FSUS_WheelStop(Usart_DataTypeDef *usart, uint8_t servo_id){
	uint8_t method = 0x00;
	uint16_t speed = 0;
	uint16_t value = 0;
	return FSUS_WheelMove(usart, servo_id, method, speed, value);
}

// 轮转模式 不停旋转
FSUS_STATUS FSUS_WheelKeepMove(Usart_DataTypeDef *usart, uint8_t servo_id, uint8_t is_cw, uint16_t speed){
	uint8_t method = 0x01; // 持续旋转
	if (is_cw){
		// 顺时针旋转
		method = method | 0x80;
	}
	uint16_t value = 0;
	return FSUS_WheelMove(usart, servo_id, method, speed, value);
}

// 轮转模式 按照特定的速度旋转特定的时间
FSUS_STATUS FSUS_WheelMoveTime(Usart_DataTypeDef *usart, uint8_t servo_id, uint8_t is_cw, uint16_t speed, uint16_t nTime){
	uint8_t method = 0x03; // 旋转一段时间
	if (is_cw){
		// 顺时针旋转
		method = method | 0x80;
	}
	return FSUS_WheelMove(usart, servo_id, method, speed, nTime);
}

// 轮转模式 旋转特定的圈数
FSUS_STATUS FSUS_WheelMoveNCircle(Usart_DataTypeDef *usart, uint8_t servo_id, uint8_t is_cw, uint16_t speed, uint16_t nCircle){
	uint8_t method = 0x02; // 旋转特定的圈数
	if (is_cw){
		// 顺时针旋转
		method = method | 0x80;
	}
	return FSUS_WheelMove(usart, servo_id, method, speed, nCircle);
}

// 设置舵机的角度
// @angle 单位度
// @interval 单位ms
// @power 舵机执行功率 单位mW
//        若power=0或者大于保护值
FSUS_STATUS FSUS_SetServoAngle(Usart_DataTypeDef *usart, uint8_t servo_id, float angle, uint16_t interval, uint16_t power, uint8_t wait){
	// 创建环形缓冲队列
	const uint8_t size = 7;
	uint8_t buffer[size+1];
	RingBufferTypeDef ringBuf;
	RingBuffer_Init(&ringBuf, size, buffer);	
	// 数值约束
	if(angle > 180.0f){
		angle = 180.0f;
	}else if(angle < -180.0f){
		angle = -180.0f;
	}
	// 构造content
	RingBuffer_WriteByte(&ringBuf, servo_id);
	RingBuffer_WriteShort(&ringBuf, (int16_t)(10*angle));
	RingBuffer_WriteUShort(&ringBuf, interval);
	RingBuffer_WriteUShort(&ringBuf, power);
	// 发送请求包
	// 注: 因为用的是环形队列 head是空出来的,所以指针需要向后推移一个字节
	FSUS_SendPackage(usart, FSUS_CMD_ROTATE, size, buffer+1);
	
	if (wait){
		return FSUS_Wait(usart, servo_id, angle, 0); 
	}else{
		return FSUS_STATUS_SUCCESS;
	}
}

/* 设置舵机的角度(指定周期) */
FSUS_STATUS FSUS_SetServoAngleByInterval(Usart_DataTypeDef *usart, uint8_t servo_id, \
				float angle, uint16_t interval, uint16_t t_acc, \
				uint16_t t_dec, uint16_t  power, uint8_t wait){
	// 创建环形缓冲队列
	const uint8_t size = 11;
	uint8_t buffer[size+1];
	RingBufferTypeDef ringBuf;
	RingBuffer_Init(&ringBuf, size, buffer);	
	// 数值约束
	if(angle > 180.0f){
		angle = 180.0f;
	}else if(angle < -180.0f){
		angle = -180.0f;
	}
	if (t_acc < 20){
		t_acc = 20;
	}
	if (t_dec < 20){
		t_dec = 20;
	}
	
	// 协议打包
	RingBuffer_WriteByte(&ringBuf, servo_id);
	RingBuffer_WriteShort(&ringBuf, (int16_t)(10*angle));
	RingBuffer_WriteUShort(&ringBuf, interval);
	RingBuffer_WriteUShort(&ringBuf, t_acc);
	RingBuffer_WriteUShort(&ringBuf, t_dec);
	RingBuffer_WriteUShort(&ringBuf, power);
	
	// 发送请求包
	// 注: 因为用的是环形队列 head是空出来的,所以指针需要向后推移一个字节
	FSUS_SendPackage(usart, FSUS_CMD_SET_SERVO_ANGLE_BY_INTERVAL, size, buffer+1);
	
	if(wait){
		return FSUS_Wait(usart, servo_id, angle, 0);  
	}else{
		return FSUS_STATUS_SUCCESS;
	}
	
}

/* 设置舵机的角度(指定转速) */
FSUS_STATUS FSUS_SetServoAngleByVelocity(Usart_DataTypeDef *usart, uint8_t servo_id, \
				float angle, float velocity, uint16_t t_acc, \
				uint16_t t_dec, uint16_t  power, uint8_t wait){
	// 创建环形缓冲队列
	const uint8_t size = 11;
	uint8_t buffer[size+1];
	RingBufferTypeDef ringBuf;
	RingBuffer_Init(&ringBuf, size, buffer);	
	
	// 数值约束
	if(angle > 180.0f){
		angle = 180.0f;
	}else if(angle < -180.0f){
		angle = -180.0f;
	}
	if(velocity < 1.0f){
		velocity = 1.0f;
	}else if(velocity > 750.0f){
		velocity = 750.0f;
	}
	if(t_acc < 20){
		t_acc = 20;
	}
	if(t_dec < 20){
		t_dec = 20;
	}
	
	// 协议打包
	RingBuffer_WriteByte(&ringBuf, servo_id);
	RingBuffer_WriteShort(&ringBuf, (int16_t)(10.0f*angle));
	RingBuffer_WriteUShort(&ringBuf, (uint16_t)(10.0f*velocity));
	RingBuffer_WriteUShort(&ringBuf, t_acc);
	RingBuffer_WriteUShort(&ringBuf, t_dec);
	RingBuffer_WriteUShort(&ringBuf, power);
	
	// 发送请求包
	// 注: 因为用的是环形队列 head是空出来的,所以指针需要向后推移一个字节
	FSUS_SendPackage(usart, FSUS_CMD_SET_SERVO_ANGLE_BY_VELOCITY, size, buffer+1);
	
	if(wait){
		return FSUS_Wait(usart, servo_id, angle, 0); 
	}else{
		return FSUS_STATUS_SUCCESS;
	}
	
}

/* 查询单个舵机的角度信息 angle 单位度 */
FSUS_STATUS FSUS_QueryServoAngle(Usart_DataTypeDef *usart, uint8_t servo_id, float *angle){
	const uint8_t size = 1; // 请求包content的长度
	uint8_t ehcoServoId;
	int16_t echoAngle;
	
	// 发送舵机角度请求包
	FSUS_SendPackage(usart, FSUS_CMD_READ_ANGLE, size, &servo_id);
	// 接收返回的Ping
	PackageTypeDef pkg;
	uint8_t statusCode = FSUS_RecvPackage(usart, &pkg);
	if (statusCode == FSUS_STATUS_SUCCESS){
		// 成功的获取到舵机角度回读数据
		ehcoServoId = (uint8_t)pkg.content[0];
		// 检测舵机ID是否匹配
		if (ehcoServoId != servo_id){
			// 反馈得到的舵机ID号不匹配
			return FSUS_STATUS_ID_NOT_MATCH;
		}
		
		// 提取舵机角度
		echoAngle = (int16_t)(pkg.content[1] | (pkg.content[2] << 8));
		*angle = (float)(echoAngle / 10.0);
	}
  return statusCode;
}	

/* 设置舵机的角度(多圈模式) */
FSUS_STATUS FSUS_SetServoAngleMTurn(Usart_DataTypeDef *usart, uint8_t servo_id, float angle, \
	uint32_t interval, uint16_t power, uint8_t wait){
	// 创建环形缓冲队列
	const uint8_t size = 11;
	uint8_t buffer[size+1];
	RingBufferTypeDef ringBuf;
	RingBuffer_Init(&ringBuf, size, buffer);	
	// 数值约束			
	if(angle > 368640.0f){
		angle = 368640.0f;
	}else if(angle < -368640.0f){
		angle = -368640.0f;
	}
	if(interval > 4096000){
		angle = 4096000;
	}
	// 协议打包
	RingBuffer_WriteByte(&ringBuf, servo_id);
	RingBuffer_WriteLong(&ringBuf, (int32_t)(10*angle));
	RingBuffer_WriteULong(&ringBuf, interval);
	RingBuffer_WriteShort(&ringBuf, power);
	
	// 发送请求包
	// 注: 因为用的是环形队列 head是空出来的,所以指针需要向后推移一个字节
	FSUS_SendPackage(usart, FSUS_CMD_SET_SERVO_ANGLE_MTURN, size, buffer+1);
	
	if(wait){
		return FSUS_Wait(usart, servo_id, angle, 1);
	}else{
		return FSUS_STATUS_SUCCESS;
	}
}

/* 设置舵机的角度(多圈模式, 指定周期) */
FSUS_STATUS FSUS_SetServoAngleMTurnByInterval(Usart_DataTypeDef *usart, uint8_t servo_id, float angle, \
			uint32_t interval,  uint16_t t_acc,  uint16_t t_dec, uint16_t power, uint8_t wait){
	// 创建环形缓冲队列
	const uint8_t size = 15;
	uint8_t buffer[size+1];
	RingBufferTypeDef ringBuf;
	RingBuffer_Init(&ringBuf, size, buffer);	
	
	// 数值约束			
	if(angle > 368640.0f){
		angle = 368640.0f;
	}else if(angle < -368640.0f){
		angle = -368640.0f;
	}
	if(interval > 4096000){
		interval = 4096000;
	}
	if(t_acc < 20){
		t_acc = 20;
	}
	if(t_dec < 20){
		t_dec = 20;
	}
	// 协议打包
	RingBuffer_WriteByte(&ringBuf, servo_id);
	RingBuffer_WriteLong(&ringBuf, (int32_t)(10*angle));
	RingBuffer_WriteULong(&ringBuf, interval);
	RingBuffer_WriteUShort(&ringBuf, t_acc);
	RingBuffer_WriteUShort(&ringBuf, t_dec);
	RingBuffer_WriteShort(&ringBuf, power);
	
	// 发送请求包
	// 注: 因为用的是环形队列 head是空出来的,所以指针需要向后推移一个字节
	FSUS_SendPackage(usart, FSUS_CMD_SET_SERVO_ANGLE_MTURN_BY_INTERVAL, size, buffer+1);
	
	if(wait){
		return FSUS_Wait(usart, servo_id, angle, 1);
	}else{
		return FSUS_STATUS_SUCCESS;
	}
}

/* 设置舵机的角度(多圈模式, 指定转速) */
FSUS_STATUS FSUS_SetServoAngleMTurnByVelocity(Usart_DataTypeDef *usart, uint8_t servo_id, float angle, \
			float velocity, uint16_t t_acc,  uint16_t t_dec, uint16_t power, uint8_t wait){
	// 创建环形缓冲队列
	const uint8_t size = 13;
	uint8_t buffer[size+1];
	RingBufferTypeDef ringBuf;
	RingBuffer_Init(&ringBuf, size, buffer);	
	// 数值约束
	if(angle > 368640.0f){
		angle = 368640.0f;
	}else if(angle < -368640.0f){
		angle = -368640.0f;
	}
	if(velocity < 1.0f){
		velocity = 1.0f;
	}else if(velocity > 750.0f){
		velocity = 750.0f;
	}
	if(t_acc < 20){
		t_acc = 20;
	}
	if(t_dec < 20){
		t_dec = 20;
	}
	// 协议打包
	RingBuffer_WriteByte(&ringBuf, servo_id);
	RingBuffer_WriteLong(&ringBuf, (int32_t)(10.0f*angle));
	RingBuffer_WriteUShort(&ringBuf, (uint16_t)(10.0f*velocity));
	RingBuffer_WriteUShort(&ringBuf, t_acc);
	RingBuffer_WriteUShort(&ringBuf, t_dec);
	RingBuffer_WriteShort(&ringBuf, power);
	
	// 发送请求包
	// 注: 因为用的是环形队列 head是空出来的,所以指针需要向后推移一个字节
	FSUS_SendPackage(usart, FSUS_CMD_SET_SERVO_ANGLE_MTURN_BY_VELOCITY, size, buffer+1);
	
	if(wait){
		return FSUS_Wait(usart, servo_id, angle, 1);
	}else{
		return FSUS_STATUS_SUCCESS;
	}
	
}

/* 查询舵机的角度(多圈模式) */
FSUS_STATUS FSUS_QueryServoAngleMTurn(Usart_DataTypeDef *usart, uint8_t servo_id, float *angle){
	// 创建环形缓冲队列
	const uint8_t size = 1; // 请求包content的长度
	uint8_t ehcoServoId;
	int32_t echoAngle;
	
	// 发送舵机角度请求包
	FSUS_SendPackage(usart, FSUS_CMD_QUERY_SERVO_ANGLE_MTURN, size, &servo_id);
	// 接收返回的Ping
	PackageTypeDef pkg;
	uint8_t statusCode = FSUS_RecvPackage(usart, &pkg);
	if (statusCode == FSUS_STATUS_SUCCESS){
		// 成功的获取到舵机角度回读数据
		ehcoServoId = (uint8_t)pkg.content[0];
		// 检测舵机ID是否匹配
		if (ehcoServoId != servo_id){
			// 反馈得到的舵机ID号不匹配
			return FSUS_STATUS_ID_NOT_MATCH;
		}
		
		// 提取舵机角度
		echoAngle = (int32_t)(pkg.content[1] | (pkg.content[2] << 8) |  (pkg.content[3] << 16) | (pkg.content[4] << 24));
		*angle = (float)(echoAngle / 10.0);
	}
  return statusCode;
}


/* 舵机阻尼模式 */
FSUS_STATUS FSUS_DampingMode(Usart_DataTypeDef *usart, uint8_t servo_id, uint16_t power){
	const uint8_t size = 3; // 请求包content的长度
	uint8_t buffer[size+1]; // content缓冲区
	RingBufferTypeDef ringBuf; // 创建环形缓冲队列
	RingBuffer_Init(&ringBuf, size, buffer); // 缓冲队列初始化
	// 构造content
	RingBuffer_WriteByte(&ringBuf, servo_id);
	RingBuffer_WriteUShort(&ringBuf, power);
	// 发送请求包
	// 注: 因为用的是环形队列 head是空出来的,所以指针需要向后推移一个字节
	FSUS_SendPackage(usart, FSUS_CMD_DAMPING, size, buffer+1);
	return FSUS_STATUS_SUCCESS;
}


/* 等待电机旋转到特定的位置 */
FSUS_STATUS FSUS_Wait(Usart_DataTypeDef *usart, uint8_t servo_id, float target_angle, uint8_t is_mturn){
	float angle_read; // 读取出来的电机角度
	uint16_t count = 0; // 计数
	float angle_error;
	// 循环等待
	while(1){
		if(is_mturn){
			 FSUS_QueryServoAngleMTurn(usart, servo_id, &angle_read);
		}else{
			 FSUS_QueryServoAngle(usart, servo_id, &angle_read);
		}
		
		angle_error = fabsf(target_angle - angle_read);
		// printf("status:%d, angle:%.1f ,angle_error: %.1f\r\n", status, angle_read, angle_error);
		if(angle_error <= FSUS_ANGLE_DEADAREA){
			return FSUS_STATUS_SUCCESS;
		}
		
		// 超时判断机制
		count += 1;
		if(count>=FSUS_WAIT_COUNT_MAX){
			return FSUS_STATUS_FAIL;
		}
		// 延时100ms
		SysTick_DelayMs(100);
	}
	
}
