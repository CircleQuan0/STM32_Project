#include "bsp_adc.h"


/**
  * @brief          初始化并校准ADC
  * @retval         无
  */
void BSP_ADC_Init(void) {
    //初始化DMA
    MX_DMA_Init();

    //初始化两个ADC
    MX_ADC1_Init();
    MX_ADC2_Init();

    //校准ADC
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
}


/**
  * @brief          开始使用DMA的ADC采集
  * @param[in,out]  adcx: ADC句柄
  * @param[in,out]  buf: 缓冲区指针
  * @param[in]      len: 数据长度
  * @retval         无
  */
void BSP_ADC_Start_DMA(ADC_HandleTypeDef *adcx, uint32_t *buf, uint8_t len) {
    HAL_ADC_Start_DMA(adcx, (uint32_t *)buf, len);
}

