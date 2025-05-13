#include "stm32f4xx.h" // Device header
#include "arm_task.h"
#include "mytask.h"
#include "cmsis_os.h"
#include "main.h"
#include "CAN_receive.h"
#include "usart.h"

#define hight_delta 60
#define hight_arm_1 510 - hight_delta // 放置到地面上
#define hight_arm_2 150 - hight_delta // 放置到物料盘上
#define hight_arm_3 155 - hight_delta // 圆盘
#define hight_arm_4 10				  // 最高点，防止打物料
#define hight_arm_5 205 - hight_delta // 码垛

#define hight_corret_1 10 // 圆盘
#define hight_corret_2 35 // 地标
#define hight_corret_3 10 // 码垛

extern uint8_t color_shunxu[6];
float distence = 150;
float high = 20;
uint8_t bef_state_1 = 0;

void zhuaqu_1(uint8_t color) // 圆盘处抓取
{

	arm_move(hight_arm_3, (color - 1) * 120, 0, 2);
	osDelay(100);
	arm_move(hight_arm_3, (color - 1) * 120, 0, 1);
	osDelay(300);
	arm_move(hight_arm_4, (color - 1) * 120, 0, 1);
	osDelay(300);
	arm_move(hight_arm_4, (color - 1) * 120, 1, 1);
	osDelay(800);
	arm_move(hight_arm_2, (color - 1) * 120, 1, 1);
	osDelay(300);
	arm_move(hight_arm_2, (color - 1) * 120, 1, 0);
	osDelay(500);
	arm_move(hight_arm_4, (color - 1) * 120, 1, 0);
	osDelay(400);
	arm_move(hight_arm_4, (color - 1) * 120, 0, 2);
	osDelay(800);
}

void fangzhi_1(uint8_t color) // 正常放置
{
	arm_move(hight_arm_4, (color - 1) * 120, 1, 0);
	osDelay(300);
	arm_move(hight_arm_2, (color - 1) * 120, 1, 0);
	osDelay(300);
	arm_move(hight_arm_2, (color - 1) * 120, 1, 1);
	osDelay(500);
	arm_move(hight_arm_4, (color - 1) * 120, 1, 1);
	osDelay(600);
	arm_move(hight_arm_4, (color - 1) * 120, 0, 1);
	osDelay(600);
	arm_move(hight_arm_1, (color - 1) * 120, 0, 1);
	osDelay(800);
	arm_move(hight_arm_1, (color - 1) * 120, 0, 0);
	osDelay(450);
	arm_move(hight_arm_4, (color - 1) * 120, 0, 0);
	osDelay(1000);
}

void zhuaqu_2(uint8_t color) // 暂存区抓取
{
	arm_move(hight_arm_1, (color - 1) * 120, 0, 0);
	osDelay(500);
	arm_move(hight_arm_1, (color - 1) * 120, 0, 1);
	osDelay(500);
	arm_move(hight_arm_4, (color - 1) * 120, 0, 1);
	osDelay(750);
	arm_move(hight_arm_4, (color - 1) * 120, 1, 1);
	osDelay(800);
	arm_move(hight_arm_2, (color - 1) * 120, 1, 1);
	osDelay(300);
	arm_move(hight_arm_2, (color - 1) * 120, 1, 0);
	osDelay(500);
	arm_move(hight_arm_4, (color - 1) * 120, 1, 0);
	osDelay(500);
	arm_move(hight_arm_4, (color - 1) * 120, 0, 0);
	osDelay(500);
}

void fangzhi_2(uint8_t color) // 码垛放置
{
	arm_move(hight_arm_4, (color - 1) * 120, 1, 0);
	osDelay(300);
	arm_move(hight_arm_2, (color - 1) * 120, 1, 0);
	osDelay(300);
	arm_move(hight_arm_2, (color - 1) * 120, 1, 1);
	osDelay(500);
	arm_move(hight_arm_4, (color - 1) * 120, 1, 1);
	osDelay(600);
	arm_move(hight_arm_4, (color - 1) * 120, 0, 1);
	osDelay(600);
	arm_move(hight_arm_5, (color - 1) * 120, 0, 1);
	osDelay(400);
	arm_move(hight_arm_5, (color - 1) * 120, 0, 0);
	osDelay(200);
	arm_move(hight_arm_4, (color - 1) * 120, 0, 0);
}

void Round1Part1task1(uint8_t i) // 到达转盘处抓取物料
{
	uint8_t a = 0;

	//	uint8_t flag_1 = 0;

	uint8_t m = 0;
	if (i == 1)
		m = 0;
	if (i == 2)
		m = 3;

	arm_move(20, (color_shunxu[m] - 1) * 120, 0, 2);

	corret(2, hight_corret_1, 4); // 位置校正

	while (a < 3)
	{

		if (data_usart6[6] == 0x02)
		{
			if (data_usart6[8] == color_shunxu[a + m])
			{
				zhuaqu_1(color_shunxu[a + m]);
				a++;
				//				flag_1 = 1;
			}
		}

		//		if(color_shunxu[a+m-1] == 1 && flag_1 == 1)
		//		{
		//			flag_1 = 0;
		//			osDelay(1500);
		//		}
	}
	my_destory();
}

void put_half(uint8_t i)
{
	uint8_t times;
	if (i == 1)
		times = 0;
	if (i == 2)
		times = 3;

	corret(1, hight_corret_2, 4);

	for (uint8_t n = 0; n < 3; n++)
	{
		if (n == 0)
		{
			arm_move(hight_arm_4, (color_shunxu[n + times] - 1) * 120, 1, 0);
			if (color_shunxu[n + times] == 2)
				move_set_without_yaw(0, -((color_shunxu[n + times] - 2) * distence), 2);
			else
				move_set_without_yaw(-2, -((color_shunxu[n + times] - 2) * distence), 2);
			osDelay(500);
		}
		if (n == 1)
		{
			arm_move(hight_arm_4, (color_shunxu[n + times] - 1) * 120, 1, 0);
			move_set_without_yaw(-2, -((color_shunxu[n + times] - color_shunxu[n + times - 1]) * distence), 2);
		}
		if (n == 2)
		{
			move_set_without_yaw(-2, -((color_shunxu[n + times] - color_shunxu[n + times - 1]) * distence), 2);
			arm_move(hight_arm_4, (color_shunxu[n + times] - 1) * 120, 0, 0);
			osDelay(500);
			corret(1, hight_corret_2, 2);
			arm_move(hight_arm_4, (color_shunxu[n + times] - 1) * 120, 1, 0);
			osDelay(500);
		}
		fangzhi_1(color_shunxu[n + times]);
	}
	my_destory();
}

void put_final(uint8_t i)
{
	uint8_t times;
	if (i == 1)
	{
		times = 0;
		corret(1, hight_corret_3, 6);
		arm_move(hight_arm_4, (color_shunxu[times] - 1) * 120, 3, 0);
		osDelay(1200);
		for (uint8_t n = 0; n < 3; n++)
		{
			if (n == 0)
			{
				arm_move(hight_arm_4, (color_shunxu[n + times] - 1) * 120, 1, 0);
				move_set_without_yaw(0, 0, 2);
				osDelay(500);
			}
			if (n == 1)
			{
				arm_move(hight_arm_4, (color_shunxu[n + times] - 1) * 120, 1, 0);
				move_set_without_yaw(-2, -((color_shunxu[n + times] - color_shunxu[n + times - 1]) * distence), 2);
				osDelay(500);
			}
			if (n == 2)
			{
				move_set_without_yaw(-2, -((color_shunxu[n + times] - color_shunxu[n + times - 1]) * distence), 2);
				arm_move(hight_arm_4, (color_shunxu[n + times] - 1) * 120, 0, 0);
				osDelay(500);
				corret(1, hight_corret_2, 2);
				arm_move(hight_arm_4, (color_shunxu[n + times] - 1) * 120, 1, 0);
				osDelay(500);
			}
			fangzhi_1(color_shunxu[n + times]);
		}
	}

	if (i == 2)
	{

		times = 3;
		corret(1, hight_corret_3, 4);
		arm_move(hight_arm_4, (color_shunxu[times] - 1) * 120, 3, 0);
		osDelay(1200);
		for (uint8_t n = 0; n < 3; n++)
		{
			if (n == 0)
			{
				arm_move(hight_arm_4, (color_shunxu[n + times] - 1) * 120, 1, 0);
				move_set_without_yaw(0, 0, 3);
				osDelay(500);
			}
			else
			{
				arm_move(hight_arm_4, (color_shunxu[n + times] - 1) * 120, 0, 0);
				osDelay(200);
				arm_move(hight_arm_4, (color_shunxu[n + times] - 1) * 120, 1, 0);
				move_set_without_yaw(-2, -((color_shunxu[n + times] - color_shunxu[n + times - 1]) * distence), 3);
				osDelay(500);
			}
			fangzhi_2(color_shunxu[n + times]);
		}
	}
	my_destory();
}

void get_half(uint8_t i)
{
	uint8_t times;
	if (i == 1)
		times = 0;
	if (i == 2)
		times = 3;

	arm_move(hight_arm_4, (color_shunxu[times] - 1) * 120, 3, 0);
	osDelay(800);

	for (uint8_t n = 0; n < 3; n++)
	{
		if (n == 0)
		{
			move_set_without_yaw(-2, -((color_shunxu[n + times] - color_shunxu[2 + times]) * distence), 3);
		}
		else
		{
			move_set_without_yaw(-2, -((color_shunxu[n + times] - color_shunxu[n + times - 1]) * distence), 3);
		}
		zhuaqu_2(color_shunxu[n + times]);
	}
	//	my_destory();
}

void Round1Part2task1(uint8_t i) // 到达粗加工区放
{
	uint8_t m;
	if (i == 1)
		m = 0;
	if (i == 2)
		m = 3;

	if (color_shunxu[0 + m] == red_area) // ①红
	{
		move_set((1 - 2) * distence, 0, 180, 5);
		osDelay(500);
		corret(1, high, 4);
		osDelay(2000);
		fangzhi_1(color_shunxu[0 + m]);
		osDelay(1000);
		if (color_shunxu[1 + m] == green_area) // 绿
		{
			move_set((2 - 1) * distence, 0, 180, 5);
			osDelay(500);
			//				corret(1,high,4);
			//				osDelay(2000);
			fangzhi_1(color_shunxu[1 + m]);
			if (color_shunxu[2 + m] == blue_area) // 蓝
			{
				move_set((3 - 2) * distence, 0, 180, 5);
				osDelay(500);
				//					corret(1,high,4);
				//					osDelay(2000);
				fangzhi_1(color_shunxu[2 + m]);
				bef_state_1 = color_shunxu[2 + m];
			}
		}
		else if (color_shunxu[1 + m] == blue_area) // 蓝
		{
			move_set((3 - 1) * distence, 0, 180, 5);
			osDelay(500);
			//				corret(1,high,4);
			//				osDelay(2000);
			fangzhi_1(color_shunxu[1 + m]);
			if (color_shunxu[2 + m] == green_area) // 绿
			{
				move_set((2 - 3) * distence, 0, 180, 5);
				osDelay(500);
				corret(1, high, 4);
				osDelay(2000);
				fangzhi_1(color_shunxu[2 + m]);

				bef_state_1 = color_shunxu[2 + m];
			}
		}
	}
	else if (color_shunxu[0 + m] == green_area) // ②绿
	{
		move_set((2 - 2) * distence, 0, 180, 5);
		osDelay(500);
		corret(1, high, 4);
		osDelay(2000);
		fangzhi_1(color_shunxu[0 + m]);
		if (color_shunxu[1 + m] == blue_area) // 蓝
		{
			move_set((3 - 2) * distence, 0, 180, 5);
			osDelay(500);
			//				corret(1,high,4);
			//				osDelay(2000);
			fangzhi_1(color_shunxu[1 + m]);
			if (color_shunxu[2 + m] == red_area) // 红
			{
				move_set((1 - 3) * distence, 0, 180, 5);
				osDelay(500);
				//					corret(1,high,4);
				//					osDelay(2000);
				fangzhi_1(color_shunxu[2 + m]);

				bef_state_1 = color_shunxu[2 + m];
			}
		}
		else if (color_shunxu[1 + m] == red_area) // 红
		{
			move_set((1 - 2) * distence, 0, 180, 5);
			osDelay(500);
			//				corret(1,high,4);
			//				osDelay(2000);
			fangzhi_1(color_shunxu[1 + m]);
			if (color_shunxu[2 + m] == blue_area) // 蓝
			{
				move_set((3 - 1) * distence, 0, 180, 5);
				osDelay(500);
				//					corret(1,high,4);
				//					osDelay(2000);
				fangzhi_1(color_shunxu[2 + m]);

				bef_state_1 = color_shunxu[2 + m];
			}
		}
	}
	else if (color_shunxu[0 + m] == blue_area) // ②蓝
	{
		move_set((3 - 2) * distence, 0, 180, 5);
		osDelay(500);
		corret(1, high, 4);
		osDelay(2000);
		fangzhi_1(color_shunxu[0 + m]);
		if (color_shunxu[1 + m] == green_area) // 绿
		{
			move_set((2 - 3) * distence, 0, 180, 5);
			osDelay(500);
			//				corret(1,high,4);
			//				osDelay(2000);
			fangzhi_1(color_shunxu[1 + m]);
			if (color_shunxu[2 + m] == red_area) // 红
			{
				move_set((1 - 2) * distence, 0, 180, 5);
				osDelay(500);
				//					corret(1,high,4);
				//					osDelay(2000);
				fangzhi_1(color_shunxu[2 + m]);

				bef_state_1 = color_shunxu[2 + m];
			}
		}
		else if (color_shunxu[1 + m] == red_area) // 红
		{
			move_set((1 - 3) * distence, 0, 180, 5);
			osDelay(1000);
			//				corret(1,high,4);
			//				osDelay(2000);
			fangzhi_1(color_shunxu[1 + m]);
			if (color_shunxu[2 + m] == green_area) // 绿
			{
				move_set((2 - 1) * distence, 0, 180, 5);
				osDelay(500);
				//					corret(1,high,4);
				//					osDelay(2000);
				fangzhi_1(color_shunxu[2 + m]);

				bef_state_1 = color_shunxu[2 + m];
			}
		}
	}
}

void Round1Part2task2(uint8_t i) // 粗加工区拿
{
	uint8_t m;
	if (i == 1)
		m = 0;
	if (i == 2)
		m = 3;

	if (bef_state_1 == 1)
	{
		bef_state_1 = 0;
		move_set((2 - 1) * distence, 0, 180, 5);
		osDelay(1000);
	}
	else if (bef_state_1 == 2)
	{
		bef_state_1 = 0;
		move_set((2 - 2) * distence, 0, 180, 5);
		osDelay(1000);
	}
	else if (bef_state_1 == 3)
	{
		bef_state_1 = 0;
		move_set((3 - 2) * distence, 0, 180, 5);
		osDelay(1000);
	}

	if (color_shunxu[0 + m] == red_area) // ①红
	{
		move_set((1 - 2) * distence, 0, 180, 5);
		osDelay(500);
		corret(1, high, 4);
		osDelay(2000);
		zhuaqu_2(color_shunxu[0 + m]);
		osDelay(1000);
		if (color_shunxu[1 + m] == green_area) // 绿
		{
			move_set((2 - 1) * distence, 0, 180, 5);
			osDelay(500);
			corret(1, high, 4);
			osDelay(2000);
			zhuaqu_2(color_shunxu[1 + m]);
			if (color_shunxu[2 + m] == blue_area) // 蓝
			{
				move_set((3 - 2) * distence, 0, 180, 5);
				osDelay(500);
				corret(1, high, 4);
				osDelay(2000);
				zhuaqu_2(color_shunxu[2 + m]);
				bef_state_1 = color_shunxu[2 + m];
			}
		}
		else if (color_shunxu[1 + m] == blue_area) // 蓝
		{
			move_set((3 - 1) * distence, 0, 180, 5);
			osDelay(500);
			corret(1, high, 4);
			osDelay(2000);
			zhuaqu_2(color_shunxu[1 + m]);
			if (color_shunxu[2 + m] == green_area) // 绿
			{
				move_set((2 - 3) * distence, 0, 180, 5);
				osDelay(500);
				corret(1, high, 4);
				osDelay(2000);
				zhuaqu_2(color_shunxu[2 + m]);

				bef_state_1 = color_shunxu[2 + m];
			}
		}
	}
	else if (color_shunxu[0 + m] == green_area) // ②绿
	{
		move_set((2 - 2) * distence, 0, 180, 5);
		osDelay(500);
		corret(1, high, 4);
		osDelay(2000);
		zhuaqu_2(color_shunxu[0 + m]);
		if (color_shunxu[1 + m] == blue_area) // 蓝
		{
			move_set((3 - 2) * distence, 0, 180, 5);
			osDelay(500);
			corret(1, high, 4);
			osDelay(2000);
			zhuaqu_2(color_shunxu[1 + m]);
			if (color_shunxu[2 + m] == red_area) // 红
			{
				move_set((1 - 3) * distence, 0, 180, 5);
				osDelay(500);
				corret(1, high, 4);
				osDelay(2000);
				zhuaqu_2(color_shunxu[2 + m]);

				bef_state_1 = color_shunxu[2 + m];
			}
		}
		else if (color_shunxu[1 + m] == red_area) // 红
		{
			move_set((1 - 2) * distence, 0, 180, 5);
			osDelay(500);
			corret(1, high, 4);
			osDelay(2000);
			zhuaqu_2(color_shunxu[1 + m]);
			if (color_shunxu[2 + m] == blue_area) // 蓝
			{
				move_set((3 - 1) * distence, 0, 180, 5);
				osDelay(500);
				corret(1, high, 4);
				osDelay(2000);
				zhuaqu_2(color_shunxu[2 + m]);

				bef_state_1 = color_shunxu[2 + m];
			}
		}
	}
	else if (color_shunxu[0 + m] == blue_area) // ②蓝
	{
		move_set((3 - 2) * distence, 0, 180, 5);
		osDelay(500);
		corret(1, high, 4);
		osDelay(2000);
		zhuaqu_2(color_shunxu[0 + m]);
		if (color_shunxu[1 + m] == green_area) // 绿
		{
			move_set((2 - 3) * distence, 0, 180, 5);
			osDelay(500);
			corret(1, high, 4);
			osDelay(2000);
			zhuaqu_2(color_shunxu[1 + m]);
			if (color_shunxu[2 + m] == red_area) // 红
			{
				move_set((1 - 2) * distence, 0, 180, 5);
				osDelay(500);
				corret(1, high, 4);
				osDelay(2000);
				zhuaqu_2(color_shunxu[2 + m]);

				bef_state_1 = color_shunxu[2 + m];
			}
		}
		else if (color_shunxu[1 + m] == red_area) // 红
		{
			move_set((1 - 3) * distence, 0, 180, 5);
			osDelay(1000);
			corret(1, high, 4);
			osDelay(2000);
			zhuaqu_2(color_shunxu[1 + m]);
			if (color_shunxu[2 + m] == green_area) // 绿
			{
				move_set((2 - 1) * distence, 0, 180, 5);
				osDelay(500);
				corret(1, high, 4);
				osDelay(2000);
				zhuaqu_2(color_shunxu[2 + m]);

				bef_state_1 = color_shunxu[2 + m];
			}
		}
	}
}

void Round1Part3task(void) // 第一次精加工区放
{

	if (color_shunxu[0] == red_area) // ①红
	{
		move_set((1 - 2) * distence, 0, 90, 5);
		osDelay(500);
		corret(1, high, 4);
		osDelay(2000);
		fangzhi_1(color_shunxu[0]);
		osDelay(1000);
		if (color_shunxu[1] == green_area) // 绿
		{
			move_set((2 - 1) * distence, 0, 90, 5);
			osDelay(500);
			corret(1, high, 4);
			osDelay(2000);
			fangzhi_1(color_shunxu[1]);
			if (color_shunxu[2] == blue_area) // 蓝
			{
				move_set((3 - 2) * distence, 0, 90, 5);
				osDelay(500);
				corret(1, high, 4);
				osDelay(2000);
				fangzhi_1(color_shunxu[2]);
			}
		}
		else if (color_shunxu[1] == blue_area) // 蓝
		{
			move_set((3 - 1) * distence, 0, 90, 5);
			osDelay(500);
			corret(1, high, 4);
			osDelay(2000);
			fangzhi_1(color_shunxu[1]);
			if (color_shunxu[2] == green_area) // 绿
			{
				move_set((2 - 3) * distence, 0, 90, 5);
				osDelay(500);
				corret(1, high, 4);
				osDelay(2000);
				fangzhi_1(color_shunxu[2]);
			}
		}
	}
	else if (color_shunxu[0] == green_area) // ②绿
	{
		move_set((2 - 2) * distence, 0, 90, 5);
		osDelay(500);
		corret(1, high, 4);
		osDelay(2000);
		fangzhi_1(color_shunxu[0]);
		if (color_shunxu[1] == blue_area) // 蓝
		{
			move_set((3 - 2) * distence, 0, 90, 5);
			osDelay(500);
			corret(1, high, 4);
			osDelay(2000);
			fangzhi_1(color_shunxu[1]);
			if (color_shunxu[2] == red_area) // 红
			{
				move_set((1 - 3) * distence, 0, 90, 5);
				osDelay(500);
				corret(1, high, 4);
				osDelay(2000);
				fangzhi_1(color_shunxu[2]);
			}
		}
		else if (color_shunxu[1] == red_area) // 红
		{
			move_set((1 - 2) * distence, 0, 90, 5);
			osDelay(500);
			corret(1, high, 4);
			osDelay(2000);
			fangzhi_1(color_shunxu[1]);
			if (color_shunxu[2] == blue_area) // 蓝
			{
				move_set((3 - 1) * distence, 0, 90, 5);
				osDelay(500);
				corret(1, high, 4);
				osDelay(2000);
				fangzhi_1(color_shunxu[2]);
			}
		}
	}
	else if (color_shunxu[0] == blue_area) // ②蓝
	{
		move_set((3 - 2) * distence, 0, 90, 5);
		osDelay(500);
		corret(1, high, 4);
		osDelay(2000);
		fangzhi_1(color_shunxu[0]);
		if (color_shunxu[1] == green_area) // 绿
		{
			move_set((2 - 3) * distence, 0, 90, 5);
			osDelay(500);
			corret(1, high, 4);
			osDelay(2000);
			fangzhi_1(color_shunxu[1]);
			if (color_shunxu[2] == red_area) // 红
			{
				move_set((1 - 2) * distence, 0, 90, 5);
				osDelay(500);
				corret(1, high, 4);
				osDelay(2000);
				fangzhi_1(color_shunxu[2]);
			}
		}
		else if (color_shunxu[1] == red_area) // 红
		{
			move_set((1 - 3) * distence, 0, 90, 5);
			osDelay(1000);
			corret(1, high, 4);
			osDelay(2000);
			fangzhi_1(color_shunxu[1]);
			if (color_shunxu[2] == green_area) // 绿
			{
				move_set((2 - 1) * distence, 0, 90, 5);
				osDelay(500);
				corret(1, high, 4);
				osDelay(2000);
				fangzhi_1(color_shunxu[2]);
			}
		}
	}
}

void Round2Part3task(void) // 第二次精加工区码垛
{
	if (color_shunxu[3] == red_area) // ①红
	{
		move_set((1 - 2) * distence, 0, 180, 5);
		osDelay(500);
		corret(1, high, 4);
		osDelay(2000);
		fangzhi_2(color_shunxu[3]);
		osDelay(1000);
		if (color_shunxu[4] == green_area) // 绿
		{
			move_set((2 - 1) * distence, 0, 180, 5);
			osDelay(500);
			corret(1, high, 4);
			osDelay(2000);
			fangzhi_2(color_shunxu[4]);
			if (color_shunxu[5] == blue_area) // 蓝
			{
				move_set((3 - 2) * distence, 0, 180, 5);
				osDelay(500);
				corret(1, high, 4);
				osDelay(2000);
				fangzhi_2(color_shunxu[5]);
			}
		}
		else if (color_shunxu[4] == blue_area) // 蓝
		{
			move_set((3 - 1) * distence, 0, 180, 5);
			osDelay(500);
			corret(1, high, 4);
			osDelay(2000);
			fangzhi_2(color_shunxu[4]);
			if (color_shunxu[5] == green_area) // 绿
			{
				move_set((2 - 3) * distence, 0, 180, 5);
				osDelay(500);
				corret(1, high, 4);
				osDelay(2000);
				fangzhi_2(color_shunxu[5]);
			}
		}
	}
	else if (color_shunxu[3] == green_area) // ②绿
	{
		move_set((2 - 2) * distence, 0, 180, 5);
		osDelay(500);
		corret(1, high, 4);
		osDelay(2000);
		fangzhi_2(color_shunxu[3]);
		if (color_shunxu[4] == blue_area) // 蓝
		{
			move_set((3 - 2) * distence, 0, 180, 5);
			osDelay(500);
			corret(1, high, 4);
			osDelay(2000);
			fangzhi_2(color_shunxu[4]);
			if (color_shunxu[5] == red_area) // 红
			{
				move_set((1 - 3) * distence, 0, 180, 5);
				osDelay(500);
				corret(1, high, 4);
				osDelay(2000);
				fangzhi_2(color_shunxu[5]);
			}
		}
		else if (color_shunxu[4] == red_area) // 红
		{
			move_set((1 - 2) * distence, 0, 180, 5);
			osDelay(500);
			corret(1, high, 4);
			osDelay(2000);
			fangzhi_2(color_shunxu[4]);
			if (color_shunxu[5] == blue_area) // 蓝
			{
				move_set((3 - 1) * distence, 0, 180, 5);
				osDelay(500);
				corret(1, high, 4);
				osDelay(2000);
				fangzhi_2(color_shunxu[5]);
			}
		}
	}
	else if (color_shunxu[3] == blue_area) // ②蓝
	{
		move_set((3 - 2) * distence, 0, 180, 5);
		osDelay(500);
		corret(1, high, 4);
		osDelay(2000);
		fangzhi_2(color_shunxu[3]);
		if (color_shunxu[4] == green_area) // 绿
		{
			move_set((2 - 3) * distence, 0, 180, 5);
			osDelay(500);
			corret(1, high, 4);
			osDelay(2000);
			fangzhi_2(color_shunxu[4]);
			if (color_shunxu[5] == red_area) // 红
			{
				move_set((1 - 2) * distence, 0, 180, 5);
				osDelay(500);
				corret(1, high, 4);
				osDelay(2000);
				fangzhi_2(color_shunxu[5]);
			}
		}
		else if (color_shunxu[4] == red_area) // 红
		{
			move_set((1 - 3) * distence, 0, 180, 5);
			osDelay(1000);
			corret(1, high, 4);
			osDelay(2000);
			fangzhi_2(color_shunxu[4]);
			if (color_shunxu[5] == green_area) // 绿
			{
				move_set((2 - 1) * distence, 0, 180, 5);
				osDelay(500);
				corret(1, high, 4);
				osDelay(2000);
				fangzhi_2(color_shunxu[5]);
			}
		}
	}
}
