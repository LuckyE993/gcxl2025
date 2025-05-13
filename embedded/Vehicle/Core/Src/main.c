/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "CAN_receive.h"
#include "mytask.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 在文件开头添加新的变量和队列定义
typedef struct
{
  int16_t x;
  int16_t y;
  int16_t yaw;
} MoveCommand_t;

// 声明消息队列句柄
QueueHandle_t moveCommandQueue;
// 或使用CMSIS-RTOS队列
// osMessageQId moveCommandQueue;

const motor_measure_t *ID_201_data;
uint8_t rxbuff_usart1;
uint8_t rxbuff_usart6;
uint8_t data_usart1[11];
uint8_t data_usart6[10];
uint8_t sum = 0;
uint16_t YawL, YawH;
uint16_t Cam_X_H, Cam_Y_H, Cam_X_L, Cam_Y_L;
uint8_t set_baudrate_115200[] = {0xFF, 0xAA, 0x04, 0x06, 0x00};
uint8_t reset_z_axis[] = {0xFF, 0xAA, 0x76, 0x00, 0x00};

int16_t yaw_pos;




extern float tar_angle;
/**
 * @brief 串口接收完成回调函数
 *
 * 当UART接收完成时，该函数将被调用。函数处理来自USART1和USART6的数据。
 * USART1用于接收姿态传感器数据（可能是IMU/陀螺仪），数据格式为55 53 ... 和校验和
 * USART6用于接收摄像头数据，包括坐标和颜色识别信息
 *
 * @param huart 指向触发回调的UART句柄
 */
extern void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  // 静态变量用于跟踪每个UART接口的数据接收索引位置
  static uint8_t usart1_i = 0; // USART1数据接收计数器
  static uint8_t usart6_i = 0; // USART6数据接收计数器

  // 处理来自USART1的数据（姿态传感器数据）
  if (huart->Instance == USART1)
  {
    // 将接收到的字节存入数据缓冲区
    data_usart1[usart1_i++] = rxbuff_usart1;

    // 帧头第一字节检查 - 必须是0x55
    if (usart1_i == 1)
      if (data_usart1[0] != 0x55)
      {
        data_usart1[0] = 0; // 重置数据
        usart1_i = 0;       // 重置计数器
      }

    // 帧头第二字节检查 - 必须是0x53
    if (usart1_i == 2)
      if (data_usart1[1] != 0x53)
        usart1_i = 0; // 帧头错误，重置计数器

    // 当接收到完整的11字节数据包时，进行处理
    if (usart1_i == 11)
    {
      // 计算前10个字节的校验和
      for (uint8_t i = 0; i < 10; i++)
        sum += data_usart1[i];

      // 校验和验证
      if (sum == data_usart1[10])
      {
        // 提取偏航角数据（Yaw）
        YawL = data_usart1[6];    // Yaw角低字节
        YawH = data_usart1[7];    // Yaw角高字节
        yaw = (YawH << 8 | YawL); // 合并为16位数值
        yaw = yaw / 32768 * 180;  // 转换为角度值（-180到+180度范围）
      }

      // 重置计数器和校验和，准备接收下一个数据包
      usart1_i = 0;
      sum = 0;
    }

    // 继续启用USART1中断接收，等待下一个字节
    HAL_UART_Receive_IT(&huart1, &rxbuff_usart1, 1);
  }

  // 处理来自USART6的数据（摄像头数据）
  if (huart->Instance == USART6)
  {
    // 将接收到的字节存入数据缓冲区
    data_usart6[usart6_i++] = rxbuff_usart6;

    // 帧头检查 - 第一字节必须是0xFF
    if (usart6_i == 1)
    {
      if (data_usart6[0] != 0xFF)
      {
        data_usart6[0] = 0; // 重置数据
        usart6_i = 0;       // 重置计数器
      }
    }

    // 数据类型检查 - 如果第二字节为0x10，放弃该数据包
    if (usart6_i == 3)
    {
      if (data_usart6[1] == 0x10)
      {
        usart6_i = 0; // 重置计数器，放弃此数据包
      }
    }

    // 处理坐标数据 - 当收到完整的10字节且不是颜色序列或特殊数据包
    if (usart6_i == 10)
    {
      if (data_usart6[1] == 0x01)
      {
        if (data_usart6[2] == 0x01) // 位置环
        {
          mode_switch(1); // 切换到位置环控制模式
          // 提取X坐标数据
          // Extract X position values (bytes 3-6)
          uint16_t x_raw = (data_usart6[3] << 8) | data_usart6[4];
          int16_t x_pos = (int16_t)x_raw; // Convert to signed integer if needed

          // Extract Y position values (bytes 5-8)
          uint16_t y_raw = (data_usart6[5] << 8) | data_usart6[6];
          int16_t y_pos = (int16_t)y_raw; // Convert to signed integer if needed

          // Extract yaw position/angle (bytes 7-8)
          uint16_t yaw_raw = (data_usart6[7] << 8) | data_usart6[8];
          yaw_pos += (int16_t)yaw_raw;           // Convert to signed integer if needed
          move_set_interrupt(x_pos, y_pos, yaw_pos, 1); // 设置运动目标
        }
        else if (data_usart6[2] == 0x00) // 速度环
        {
          mode_switch(0); // 切换到速度环控制模式
          // Extract vx values (bytes 3-4)
          uint16_t vx_raw = (data_usart6[3] << 8) | data_usart6[4];
          int16_t vx = (int16_t)vx_raw; // Convert to signed integer if needed

          // Extract vy values (bytes 5-6)
          uint16_t vy_raw = (data_usart6[5] << 8) | data_usart6[6];
          int16_t vy = (int16_t)vy_raw; // Convert to signed integer if needed

          // Extract vyaw values (bytes 7-8)
          uint16_t vyaw_raw = (data_usart6[7] << 8) | data_usart6[8];
          int16_t vyaw = (int16_t)vyaw_raw; // Convert to signed integer if needed

          // Update global control variables
          target_vx = vx;
          target_vy = vy;
          target_vyaw = vyaw;
        }
      }
      else if (data_usart6[1] == 0x02 && data_usart6[2] == 0x01) // 控制机械臂无刷电机
      {
                // Extract high and low bytes for brushless motor control
        uint16_t brushless_h = data_usart6[3];
        uint16_t brushless_l = data_usart6[4];
        
        // Combine high and low bytes to form target angle
        // First cast to int16_t if you need to handle negative values
        int16_t angle_raw = (int16_t)((brushless_h << 8) | brushless_l);
        tar_angle = (float)angle_raw;  // Explicit conversion to float
        
        // Apply limits
        if (tar_angle > 500.0f)
        {
           tar_angle = 500.0f;
        }
        else if (tar_angle < 0.0f)
        {
           tar_angle = 0.0f;
        }
        
        // tar_angle += 5; //测试机械臂极限
      }
      
      usart6_i = 0; // 重置计数器，准备接收下一个数据包
    }

    // // 处理颜色序列数据 - 当收到完整的10字节且是颜色数据包
    // if(usart6_i == 10 && data_usart6[1] == 0x01 && data_usart6[2] != 0x00)
    // {
    //     // 提取6种颜色的序列数据
    //     color_shunxu[0] = data_usart6[2];
    //     color_shunxu[1] = data_usart6[3];
    //     color_shunxu[2] = data_usart6[4];
    //     color_shunxu[4] = data_usart6[6];  // 注意：索引4和3顺序与数据包中的顺序不同
    //     color_shunxu[3] = data_usart6[5];
    //     color_shunxu[5] = data_usart6[7];

    //     usart6_i = 0;  // 重置计数器，准备接收下一个数据包
    // }

    // 安全检查 - 如果接收的数据超过10字节，重置计数器
    if (usart6_i > 10)
      usart6_i = 0;

    // 继续启用USART6中断接收，等待下一个字节
    HAL_UART_Receive_IT(&huart6, &rxbuff_usart6, 1);
  }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  can_filter_init();

  HAL_UART_Transmit(&huart1, reset_z_axis, 5, 50);
  HAL_UART_Receive_IT(&huart1, &rxbuff_usart1, 1);
  HAL_UART_Receive_IT(&huart6, &rxbuff_usart6, 1);
  ID_201_data = get_chassis_motor_measure_point(0);
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
