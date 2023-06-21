#ifndef BSP_ADC_H
#define BSP_ADC_H


#include "adc.h"
#include "dma.h"


/**
  * @brief          初始化并校准ADC
  * @retval         无
  */
extern void BSP_ADC_Init(void);


/**
  * @brief          开始使用DMA的ADC采集
  * @param[in,out]  adcx: ADC句柄
  * @param[in,out]  buf: 缓冲区指针
  * @param[in]      len: 数据长度
  * @retval         无
  */
extern void BSP_ADC_Start_DMA(ADC_HandleTypeDef *adcx, uint32_t *buf, uint8_t len);


#endif //BSP_ADC_H

