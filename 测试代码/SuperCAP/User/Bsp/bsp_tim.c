#include "bsp_tim.h"


/**
  * @brief          初始化TIM
  * @retval         无
  */
void BSP_TIM_Init(void) {
    //蜂鸣器
    MX_TIM1_Init();
    //RGB LED
    MX_TIM2_Init();
    MX_TIM3_Init();
    //20K 功率闭环中断
    MX_TIM6_Init();

    //蜂鸣器
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

    //RGB LED
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}


/**
  * @brief          启动循环定时器
  * @retval         无
  */
void BSP_TIM_Start_Loop_Timer(void) {
    //20K 功率闭环中断
    HAL_TIM_Base_Start_IT(&htim6);
}


/**
  * @brief          停止循环定时器
  * @retval         无
  */
void BSP_TIM_Stop_Loop_Timer(void) {
    //停止20K 功率闭环中断
    HAL_TIM_Base_Stop_IT(&htim6);
}


/**
  * @brief          更新LED亮度
  * @param[in]      red: 红色亮度 (0-100)
  * @param[in]      green: 绿色亮度 (0-100)
  * @param[in]      blue: 蓝色亮度 (0-100)
  * @retval         无
  */
void Update_LED_PWM(uint8_t red, uint8_t green, uint8_t blue) {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, green);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, blue);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, red);
}


/**
  * @brief          更新蜂鸣器PWM
  * @param[in]      buzz: 蜂鸣器响度 (0-100)
  * @retval         无
  */
void Update_Buzzer_PWM(uint8_t buzz) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, buzz);
}

