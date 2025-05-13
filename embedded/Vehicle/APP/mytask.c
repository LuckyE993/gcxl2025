/**
 * @file mytask.c
 * @brief 机器人控制任务实现
 * @details 实现底盘控制、云台控制、机械臂控制等功能
 */

#include "mytask.h"
#include "cmsis_os.h"
#include "main.h"
#include "CAN_receive.h"
#include "usart.h"
#include "arm_task.h"

/* 常量定义 */
#define TASK_DELAY_SHORT 10     // 短延时(ms)
#define TASK_DELAY_MEDIUM 50    // 中等延时(ms)
#define TASK_DELAY_LONG 2000    // 长延时(ms)
#define TASK_DELAY_STARTUP 3000 // 启动延时(ms)

#define UART_TIMEOUT 50   // 串口超时时间(ms)
#define UART_HEAD 0xFF    // 串口数据包头
#define UART_TAIL 0xFE    // 串口数据包尾
#define UART_CMD_ARM 0x05 // 控制机械臂命令

#define CAMERA_TARGET_X 220 // 相机目标X坐标
#define CAMERA_TARGET_Y 324 // 相机目标Y坐标

#define SPEED_MAX 3000 // 最大速度限制

/* PID控制器 */
PID pid_speed[4];   // 速度环PID控制器
PID pid_pos_0[4];   // 位置环PID控制器(主)
PID pid_pos_1[4];   // 位置环PID控制器(备用)
PID pid_yaw;        // 偏航角PID控制器
PID pid_left_speed; // 云台电机速度PID控制器
PID pid_left_pos;   // 云台电机位置PID控制器

/* 控制模式与目标 */
// 控制模式: 0=位置环+速度环控制, 1=仅速度环控制
uint8_t control_mode = 0;

speed_target_t speed_target = {0, 0, 0};

/* 底盘控制相关变量 */
float motor_current_output[4] = {0}; // 电机电流输出值
float motor_Speed_output[4] = {0};   // 电机速度输出值
float real_x = 0;                    // X方向实际位置
float real_y = 0;                    // Y方向实际位置
float out_x = 0;                     // X方向PID输出
float out_y = 0;                     // Y方向PID输出
chassis_move_t chassis_move;         // 底盘运动参数结构体
struct move_i move_tar;              // 运动目标结构体
float yaw = 0;                       // 偏航角(航向角)
uint8_t pid_pid_pos_1_flag = 0;      // PID位置控制标志位

/* 云台控制相关变量 */
float tar_pos = 720;                 // 云台目标位置
float tar_angle = 0;                 // 云台目标角度
float motor_left_speed_output = 0;   // 云台电机速度输出
float motor_left_current_output = 0; // 云台电机电流输出

/* 视觉与任务相关变量 */
float CAM_X = 0, CAM_Y = 0;                   // 相机识别的坐标
uint8_t color_test[3] = {1, 2, 3};            // 颜色测试数组
uint8_t color_shunxu[6] = {2, 1, 3, 2, 1, 3}; // 颜色顺序数组

/* 通信相关变量 */
// 串口通信数据包
uint8_t senddata2cam[10] = {
    UART_HEAD,  // 头帧
    0x01,       // 命令类型，默认0x01
    0x00, 0x00, // 步进电机1数据(低高位)
    0x00, 0x00, // 步进电机2数据(低高位)
    0x00,       // 舵机1状态
    0x00,       // 舵机2状态
    0x00,       // 保留字节
    UART_TAIL   // 尾帧
};
uint8_t positionConfirm[10] = {
    UART_HEAD,  // 头帧
    0x00,       // 命令类型，默认0x01
    0x01, 
    0x00, 
    0x00, 
    0x00, 
    0x00,       
    0x00,       
    0x00,       
    UART_TAIL   // 尾帧
};
/**
 * @brief 设置底盘速度
 * @param vx X方向目标速度
 * @param vy Y方向目标速度
 * @param vyaw 旋转角速度
 */
void set_chassis_speed(float vx, float vy, float vyaw)
{
    // 限制速度在最大值范围内
    vx = (vx > SPEED_MAX) ? SPEED_MAX : ((vx < -SPEED_MAX) ? -SPEED_MAX : vx);
    vy = (vy > SPEED_MAX) ? SPEED_MAX : ((vy < -SPEED_MAX) ? -SPEED_MAX : vy);
    vyaw = (vyaw > SPEED_MAX) ? SPEED_MAX : ((vyaw < -SPEED_MAX) ? -SPEED_MAX : vyaw);
    target_vx = vx;
    target_vy = vy;
    // target_vyaw = vyaw;
}

/**
 * @brief 切换控制模式
 * @param mode 控制模式: 0=位置环+速度环控制, 1=仅速度环控制
 */
void mode_switch(uint8_t mode)
{
    switch (mode)
    {
    case 1:
        set_chassis_speed(0, 0, 0); // 停止底盘运动

        move_tar.x = real_x / (mm2angle); // 更新X方向目标位置为当前实际位置
        move_tar.y = real_y / (mm2angle); // 更新Y方向目标位置为当前实际位置

        control_mode = 0; // 设置控制模式为位置环+速度环控制
        break;
    case 0:
        control_mode = 1; // 设置控制模式为仅速度环控制
        break;
    default:
        break;
    }
}
// Global state variables for move control
typedef struct
{
    float target_x;
    float target_y;
    float target_yaw;
    float current_x;
    float current_y;
    float acc;
    uint8_t move_in_progress;
} move_control_t;

// Declare as global
move_control_t move_control = {0};

// Call this from your interrupt to initialize movement
void move_set_interrupt(float x, float y, float yaw_t, float acc)
{
    // Store starting position
    move_control.current_x = move_tar.x;
    move_control.current_y = move_tar.y;

    // Set target position (relative to current)
    move_control.target_x = move_control.current_x + x;
    move_control.target_y = move_control.current_y + y;
   while (yaw_t>360.0f)
    {
        yaw_t-=360.0f;/* code */
    }
    
    move_control.target_yaw = yaw_t;

    // Set acceleration
    move_control.acc = acc;

    // Start movement process
    move_control.move_in_progress = 1;
}

// Call this from a periodic timer or main loop
void update_movement(void)
{
    // If no movement in progress, return
    if (!move_control.move_in_progress)
    {
        return;
    }

    // Update X position
    if (move_tar.x < move_control.target_x)
    {
        move_tar.x += move_control.acc;
        if (move_tar.x > move_control.target_x)
            move_tar.x = move_control.target_x;
    }
    else if (move_tar.x > move_control.target_x)
    {
        move_tar.x -= move_control.acc;
        if (move_tar.x < move_control.target_x)
            move_tar.x = move_control.target_x;
    }

    // Update Y position
    if (move_tar.y < move_control.target_y)
    {
        move_tar.y += move_control.acc;
        if (move_tar.y > move_control.target_y)
            move_tar.y = move_control.target_y;
    }
    else if (move_tar.y > move_control.target_y)
    {
        move_tar.y -= move_control.acc;
        if (move_tar.y < move_control.target_y)
            move_tar.y = move_control.target_y;
    }

    // Update yaw (immediate)
    move_tar.yaw = move_control.target_yaw;

    // Check if movement is complete
    if (move_tar.x == move_control.target_x && move_tar.y == move_control.target_y)
    {
        move_control.move_in_progress = 0;
        HAL_UART_Transmit(&huart6, positionConfirm, 10, UART_TIMEOUT);
    }
}
/**
 * @brief 主底盘电机控制任务
 * @param pvParameters RTOS任务参数(未使用)
 */
void motor_task(void const *pvParameters)
{
    vTaskDelay(TASK_DELAY_STARTUP); // 启动延时2000ms，等待系统其他部分初始化

    // PID参数初始化，格式为{Kp, Ki, Kd}
    float init_pid_pos_0[3] = {100, 0.001, 1.5};     // 位置环PID参数
    float init_pid_speed[3] = {1.5, 0.0008, 0.0001}; // 速度环PID参数
    float init_pid_yaw[3] = {150, 0, 0.01};          // 偏航角PID参数
    uint8_t id = 0;

    err_ecd_get(); // 获取电机编码器误差

    // 初始化每个电机的PID控制器
    for (id = 0; id < 4; id++)
    {
        PID_init(&pid_pos_0[id], init_pid_pos_0); // 位置环PID初始化
        PID_init(&pid_speed[id], init_pid_speed); // 速度环PID初始化
    }
    PID_init(&pid_yaw, init_pid_yaw); // 偏航角PID初始化

    vTaskDelay(TASK_DELAY_MEDIUM); // 等待50ms，确保PID初始化完成

    uint32_t currentTime; // 当前时间，用于任务精确延时

    while (1)
    {
        currentTime = xTaskGetTickCount(); // 获取当前系统时钟节拍

        if (control_mode == 0) // 位置环+速度环控制模式
        {
            // 将目标位置从毫米转换为角度单位
            chassis_move.x_set = move_tar.x * mm2angle; // X方向目标位置
            chassis_move.y_set = move_tar.y * mm2angle; // Y方向目标位置

            // 计算偏航角PID输出
            chassis_move.yaw_set = PID_Cal_w(&pid_yaw, yaw, move_tar.yaw);
            // 计算实际偏航角（四个轮子角度的平均值）
            chassis_move.yaw_real = (motor_chassis[0].angle + motor_chassis[1].angle +
                                     motor_chassis[2].angle + motor_chassis[3].angle) /
                                    4;

            // 计算X方向实际位置（根据轮子角度差计算）
            real_x = ((motor_chassis[1].angle - chassis_move.yaw_real) -
                      (motor_chassis[0].angle - chassis_move.yaw_real)) /
                     2.0f;
            // 计算Y方向实际位置
            real_y = ((motor_chassis[2].angle - chassis_move.yaw_real) -
                      (motor_chassis[1].angle - chassis_move.yaw_real)) /
                     2.0f;

            // 位置环PID计算
            out_x = PID_Cal_Pos1(&pid_pos_0[0], real_x, chassis_move.x_set); // X方向位置PID输出
            out_y = PID_Cal_Pos1(&pid_pos_0[1], real_y, chassis_move.y_set); // Y方向位置PID输出

            // 速度限制，防止过大输出
            out_x = Speed_Limit2(out_x, real_x, chassis_move.x_set);
            out_y = Speed_Limit2(out_y, real_y, chassis_move.y_set);

            // 计算四个电机的速度输出（麦克纳姆轮运动学解算）
            motor_Speed_output[0] = -out_x - out_y - chassis_move.yaw_set; 
            motor_Speed_output[1] = out_x - out_y - chassis_move.yaw_set; 
            motor_Speed_output[2] = out_x + out_y - chassis_move.yaw_set;  
            motor_Speed_output[3] = -out_x + out_y - chassis_move.yaw_set; 
        }
        else // 仅速度环控制模式
        {
            chassis_move.yaw_real = (motor_chassis[0].angle + motor_chassis[1].angle +
                                     motor_chassis[2].angle + motor_chassis[3].angle) /
                                    4;

            // 计算X方向实际位置（根据轮子角度差计算）
            real_x = ((motor_chassis[1].angle - chassis_move.yaw_real) -
                      (motor_chassis[0].angle - chassis_move.yaw_real)) /
                     2.0f;
            // 计算Y方向实际位置
            real_y = ((motor_chassis[2].angle - chassis_move.yaw_real) -
                      (motor_chassis[1].angle - chassis_move.yaw_real)) /
                     2.0f;
            // 直接使用目标速度进行麦轮运动学解算
            motor_Speed_output[0] = -target_vx - target_vy - target_vyaw; 
            motor_Speed_output[1] = target_vx - target_vy - target_vyaw;  
            motor_Speed_output[2] = target_vx + target_vy - target_vyaw;  
            motor_Speed_output[3] = -target_vx + target_vy - target_vyaw; 
        }

        // 速度环PID计算，输出电流值
        for (id = 0; id < 4; id++)
        {
            motor_current_output[id] = PID_Cal(&pid_speed[id], motor_chassis[id].speed_rpm, motor_Speed_output[id]);
        }

        // 电流限制
        Current_Limit(motor_current_output);

        // 通过CAN总线发送电机控制命令
        CAN_cmd_chassis(motor_current_output[0], motor_current_output[1],
                        motor_current_output[2], motor_current_output[3]);

        vTaskDelayUntil(&currentTime, 2);
    }
}

/**
 * @brief 云台控制任务
 * @param pvParameters RTOS任务参数(未使用)
 */
void gimbal_task(void const *pvParameters)
{
    // PID参数初始化，格式为{Kp, Ki, Kd}
    float init_pid_left_speed[3] = {1.5, 0.0008, 0.0001}; // 云台速度环PID参数
    float init_pid_left_pos[3] = {100, 0.001, 1.5};       // 云台位置环PID参数

    // 初始化PID控制器
    PID_init(&pid_left_speed, init_pid_left_speed); // 速度环PID初始化
    PID_init(&pid_left_pos, init_pid_left_pos);     // 位置环PID初始化

    vTaskDelay(TASK_DELAY_MEDIUM); // 延时50ms确保PID初始化完成
    uint32_t currentTime;          // 用于任务精确延时的时间变量

    while (1)
    {
        currentTime = xTaskGetTickCount(); // 获取当前系统时钟节拍

        // 位置环PID计算，输出为目标速度
        motor_left_speed_output = PID_Cal_Pos1(&pid_left_pos, motor_chassis[4].angle, tar_angle);

        // 限制云台电机速度，防止过大输出
        Speed_Limit_left(&motor_left_speed_output);

        // 速度环PID计算，输出为电机电流值
        motor_left_current_output = PID_Cal(&pid_left_speed, motor_chassis[4].speed_rpm, motor_left_speed_output);

        // 当前未使用电流限制功能
        // Current_Limit_left(&motor_left_current_output);

        // 通过CAN总线发送云台电机控制命令(只控制第一个电机)
        CAN_cmd_gimbal(motor_left_current_output, 0, 0, 0);

        // 精确延时2个时钟节拍，保证任务周期稳定
        vTaskDelayUntil(&currentTime, 2);
    }
}

/**
 * @brief 主控制任务
 * @param pvParameters RTOS任务参数(未使用)
 */
void main_task(void const *pvParameters)
{
    vTaskDelay(TASK_DELAY_STARTUP); // 启动延时2000ms，等待系统其他部分初始化

    // 初始化运动目标参数
    move_tar.x = 0;   // X方向目标位置初始化为0
    move_tar.y = 0;   // Y方向目标位置初始化为0
    move_tar.yaw = 0; // 目标偏航角初始化为0
    
    // move_set(300, 0, 0, 3); // 设置初始运动目标(300mm前进，0mm侧移，0度偏航，1m/s加速度)
    // osDelay(3000);          // 等待3秒钟，确保运动完成
    // move_set_interrupt(-300, 0, 0, 1);
    // osDelay(100); // 等待3秒钟，确保运动完成
    // arm_move(0,0,0,0); // 设置机械臂初始位置(0度，0度，超前姿态，夹爪打开)
    while (1)
    {
        update_movement(); // 更新运动状态
        osDelay(5);        // 任务循环延时
    }
}

/**
 * @brief 控制机械臂运动
 * @param angle_1 机械臂上下运动的位置值(角度)，上为负值
 * @param angle_2 转盘处转动角度，控制机械臂在水平面上的旋转
 * @param state_1 舵机1状态，0表示超前姿态，1表示朝后姿态
 * @param state_2 舵机2状态，0表示夹爪打开，1表示夹爪抓取
 */
void arm_move(float angle_1, float angle_2, uint8_t state_1, uint8_t state_2)
{
    // 设置云台目标角度，控制机械臂上下移动
    tar_angle = angle_1;

    // 计算并准备转盘角度数据
    int32_t data3;
    int16_t data4;
    data3 = (angle_2 + 14) * 10; // 角度值转换，加14是为了校准偏移
    data4 = data3;

    // 组装通信数据包
    senddata2cam[1] = UART_CMD_ARM; // 0x05表示控制机械臂模式
    senddata2cam[2] = 0;            // 步进电机1低位(当前未使用)
    senddata2cam[3] = 0;            // 步进电机1高位(当前未使用)
    senddata2cam[4] = data4;        // 步进电机2(转盘)低位
    senddata2cam[5] = data4 >> 8;   // 步进电机2(转盘)高位
    senddata2cam[6] = state_1;      // 舵机1状态(机械臂姿态)
    senddata2cam[7] = state_2;      // 舵机2状态(夹爪控制)
    senddata2cam[8] = 0;            // 保留字节

    // 清除接收缓冲区状态位
    data_usart6[1] = 0x00;

    // 发送指令并等待响应
    HAL_UART_Transmit(&huart6, senddata2cam, 10, UART_TIMEOUT); // 通过UART6发送控制数据
    osDelay(2);                                                 // 短暂延时等待处理

    // 等待执行完成的应答信号(0x10)，如果没收到则重发指令
    while (data_usart6[1] != 0x10)
    {
        HAL_UART_Transmit(&huart6, senddata2cam, 10, UART_TIMEOUT);
        osDelay(2);
    }

    // 指令执行完毕，清除状态位
    data_usart6[1] = 0x00;
}

/**
 * @brief 发送视觉处理模式指令
 * @param mode 视觉处理模式:
 *             0: 取消当前模式
 *             1: 二维码扫描模式
 *             2: 颜色识别模式
 *             3: 位置校正模式
 */
void shijue(uint8_t mode)
{
    // 设置视觉处理模式
    senddata2cam[1] = mode;

    // 填充数据包的其余部分为固定值0x08
    // 这些可能是视觉处理的参数或预留位
    senddata2cam[2] = 0x08;
    senddata2cam[3] = 0x08;
    senddata2cam[4] = 0x08;
    senddata2cam[5] = 0x08;
    senddata2cam[6] = 0x08;
    senddata2cam[7] = 0x08;
    senddata2cam[8] = 0x08;

    // 通过UART6发送视觉处理指令
    HAL_UART_Transmit(&huart6, senddata2cam, 10, UART_TIMEOUT);
}

/**
 * @brief 取消当前视觉处理任务
 */
void my_destory(void)
{
    // 清除接收缓冲区状态位
    data_usart6[1] = 0x00;

    // 发送视觉模式0指令(取消当前模式)
    shijue(0);
    osDelay(TASK_DELAY_LONG); // 等待200ms处理时间

    // 等待视觉模块回应0x07(确认接收指令)
    while (data_usart6[1] != 0x07)
    {
        shijue(0);                  // 未收到确认则重发指令
        osDelay(TASK_DELAY_MEDIUM); // 等待50ms再检查
    }
}

/**
 * @brief 选择并启动视觉处理模式
 * @param mode 视觉处理模式:
 *             1: 二维码扫描模式
 *             2: 颜色识别模式
 *             3: 位置校正模式
 */
void mode_choose(uint8_t mode)
{
    if (mode == 1) // 模式1：扫码，返回物料抓取顺序
    {
        // 发送扫码模式指令
        shijue(1);
        osDelay(TASK_DELAY_LONG); // 延时200ms等待视觉模块处理

        // 持续等待视觉模块的确认响应(0x07)，如无响应则重发指令
        while (data_usart6[1] != 0x07)
        {
            shijue(1);                  // 重发扫码模式指令
            osDelay(TASK_DELAY_MEDIUM); // 延时50ms再检查响应
        }
    }
    else if (mode == 2) // 模式2：颜色识别，返回颜色与状态位
    {
        // 发送颜色识别模式指令
        shijue(2);
        osDelay(TASK_DELAY_LONG); // 延时200ms等待视觉模块处理

        // 等待视觉模块的确认响应(0x07)或颜色识别结果信号(0x02)
        while (data_usart6[1] != 0x07 && data_usart6[1] != 0x02)
        {
            shijue(2);                  // 重发颜色识别模式指令
            osDelay(TASK_DELAY_MEDIUM); // 延时50ms再检查响应
        }
    }
    else if (mode == 3) // 模式3：视觉位置校正，返回色环中心位置
    {
        // 发送位置校正模式指令
        shijue(3);
        osDelay(TASK_DELAY_LONG); // 延时200ms等待视觉模块处理

        // 等待视觉模块的确认响应(0x07)或位置校正结果信号(0x03)
        while (data_usart6[1] != 0x07 && data_usart6[1] != 0x03)
        {
            shijue(3);                  // 重发位置校正模式指令
            osDelay(TASK_DELAY_MEDIUM); // 延时50ms再检查响应
        }
    }
}

/**
 * @brief 基于视觉反馈进行位置校正
 * @param state 校正场景:
 *              1: 暂存区、粗加工区、码垛区校正
 *              2: 转盘处校正
 * @param high 高度系数，用于校正比例尺
 * @param times 校正迭代次数，越多越精确
 */
void corret(uint8_t state, float high, uint8_t times)
{
    // 计算校正比例系数，根据高度调整
    float k = 0.015;
    k = k * high; // 转盘处位置校正系数
    float k1 = 0.014;
    k1 = k1 * high; // 其他区域位置校正系数

    if (state == 1) // 暂存区、粗加工区、码垛定位
    {
        // 执行指定次数的位置校正
        for (int i = 1; i < times + 1; i++)
        {
            // 确保相机数据有效
            if (CAM_X != 0 && CAM_Y != 0)
            {
                // 计算校正位移并执行运动
                // 根据目标位置与当前位置的差值计算运动量
                move_set(-(CAM_Y - CAMERA_TARGET_X) * k1, -(CAM_X - CAMERA_TARGET_Y) * k1, move_tar.yaw, 5);
                osDelay(700); // 等待700ms执行完成
            }
            else
            {
                // 如果相机数据无效，则不计入校正次数，等待有效数据
                i--;
                osDelay(1);
            }
        }
    }

    if (state == 2) // 转盘处校正，需要更高精度
    {
        // 执行指定次数的位置校正
        for (int i = 0; i < times; i++)
        {
            // 等待转盘就绪信号(0x02)且相机数据有效
            while (data_usart6[6] != 0x02 || CAM_X == 0)
            {
                osDelay(TASK_DELAY_MEDIUM); // 等待50ms再检查
            }

            // 计算校正位移并执行运动
            move_set(-(CAM_Y - CAMERA_TARGET_X) * k, -(CAM_X - CAMERA_TARGET_Y) * k, move_tar.yaw, 5);
            osDelay(300); // 等待300ms执行完成
        }
    }
}

/**
 * @brief 设置机器人运动目标
 * @param x X方向目标位移(mm)，正值前进，负值后退
 * @param y Y方向目标位移(mm)，正值右移，负值左移
 * @param yaw_t 目标偏航角(度)，0-359
 * @param acc 加速度参数，值越大加速越快
 */
void move_set(float x, float y, float yaw_t, float acc)
{
    uint16_t i;
    float last_x = 0;
    float last_y = 0;

    // 记录当前位置为起始位置
    last_x = move_tar.x;
    last_y = move_tar.y;

    // 最多执行1000次迭代，实现平滑加速
    for (i = 0; i < 1000; i++)
    {
        // X方向位置渐进调整
        if (x != 0)
        {
            if (x > 0)
                move_tar.x += acc; // X正方向移动
            if (x < 0)
                move_tar.x -= acc; // X负方向移动
        }

        // Y方向位置渐进调整
        if (y != 0)
        {
            if (y > 0)
                move_tar.y += acc; // Y正方向移动
            if (y < 0)
                move_tar.y -= acc; // Y负方向移动
        }

        // 检查X方向是否达到或超过目标位置
        if (__fabs(move_tar.x - last_x) > __fabs(x))
            move_tar.x = x + last_x; // 防止超调，设为目标位置

        // 检查Y方向是否达到或超过目标位置
        if (__fabs(move_tar.y - last_y) > __fabs(y))
            move_tar.y = y + last_y; // 防止超调，设为目标位置

        // 如果X和Y方向都已达到目标位置，则结束循环
        if (move_tar.x == (x + last_x) && move_tar.y == (y + last_y))
            break;

        osDelay(TASK_DELAY_SHORT); // 延时10ms，控制调整频率
    }

    // 确保最终位置精确到达目标位置
    move_tar.x = x + last_x;
    move_tar.y = y + last_y;
    move_tar.yaw = yaw_t; // 设置偏航角目标值
}

/**
 * @brief 设置机器人XY方向运动，但不改变偏航角
 * @param x X方向目标位移(mm)，正值前进，负值后退
 * @param y Y方向目标位移(mm)，正值右移，负值左移
 * @param acc 加速度参数，值越大加速越快
 */
void move_set_without_yaw(float x, float y, float acc)
{
    uint16_t i;
    float last_x = 0;
    float last_y = 0;

    // 记录当前位置为起始位置
    last_x = move_tar.x;
    last_y = move_tar.y;

    // 最多执行1000次迭代，实现平滑加速
    for (i = 0; i < 1000; i++)
    {
        // X方向位置渐进调整
        if (x != 0)
        {
            if (x > 0)
                move_tar.x += acc; // X正方向移动
            if (x < 0)
                move_tar.x -= acc; // X负方向移动
        }

        // Y方向位置渐进调整
        if (y != 0)
        {
            if (y > 0)
                move_tar.y += acc; // Y正方向移动
            if (y < 0)
                move_tar.y -= acc; // Y负方向移动
        }

        // 检查X方向是否达到或超过目标位置
        if (__fabs(move_tar.x - last_x) > __fabs(x))
            move_tar.x = x + last_x; // 防止超调，设为目标位置

        // 检查Y方向是否达到或超过目标位置
        if (__fabs(move_tar.y - last_y) > __fabs(y))
            move_tar.y = y + last_y; // 防止超调，设为目标位置

        // 启用位置1控制标志，可能用于特殊控制模式
        pid_pid_pos_1_flag = 1;

        // 如果X和Y方向都已达到目标位置，则结束循环
        if (move_tar.x == (x + last_x) && move_tar.y == (y + last_y))
            break;

        osDelay(TASK_DELAY_SHORT); // 延时10ms，控制调整频率
    }
}

/**
 * @brief 执行完整的机器人任务流程
 */
void goal_task(void)
{
    // move_set(150, 0, 0, 1); // 移动到目标位置，设置加速度为5
}
