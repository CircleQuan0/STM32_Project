#ifndef BSP_HRTIM_H
#define BSP_HRTIM_H


#include "hrtim.h"


/**
  * @brief          初始化HRTIM
  * @retval         无
  */
extern void BSP_HRTIM_Init(void);


/**
  * @brief          更新HRTIM比较寄存器
  * @param[in]      cha1: CHA开启时间
  * @param[in]      cha1: CHA关断时间
  * @param[in]      chb1: CHB开启时间
  * @param[in]      chb2: CHB关断时间
  * @retval         无
  */
extern void BSP_HRTIM_Update_PWM(uint32_t cha1, uint32_t cha2, uint32_t chb1, uint32_t chb2);


/**
  * @brief          HRTIM使能输出PWM
  * @retval         无
  */
extern void BSP_HRTIM_Enable_PWM(void);


/**
  * @brief          HRTIM停止输出PWM
  * @retval         无
  */
extern void BSP_HRTIM_Disable_PWM(void);


#endif //BSP_HRTIM_H

