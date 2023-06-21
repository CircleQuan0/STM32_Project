#ifndef BSP_TIM_H
#define BSP_TIM_H


#include "tim.h"


/**
  * @brief          初始化TIM
  * @retval         无
  */
extern void BSP_TIM_Init(void);


/**
  * @brief          启动循环定时器
  * @retval         无
  */
extern void BSP_TIM_Start_Loop_Timer(void);


/**
  * @brief          停止循环定时器
  * @retval         无
  */
extern void BSP_TIM_Stop_Loop_Timer(void);


/**
  * @brief          更新LED亮度
  * @param[in]      red: 红色亮度 (0-100)
  * @param[in]      green: 绿色亮度 (0-100)
  * @param[in]      blue: 蓝色亮度 (0-100)
  * @retval         无
  */
extern void Update_LED_PWM(uint8_t red, uint8_t green, uint8_t blue);


/**
  * @brief          更新蜂鸣器PWM
  * @param[in]      buzz: 蜂鸣器响度 (0-100)
  * @retval         无
  */
extern void Update_Buzzer_PWM(uint8_t buzz);


#endif //BSP_TIM_H

