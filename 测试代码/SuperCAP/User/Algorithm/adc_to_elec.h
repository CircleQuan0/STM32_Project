#ifndef ADC_TO_ELEC_H
#define ADC_TO_ELEC_H


#include "filter.h"


typedef struct {
    unsigned short raw;             //原始数据
    int raw_bias;                   //原始数据偏移
    float real_valu1;               //真实值1
    float get_volt1;                //测量值1
    float real_valu2;               //真实值2
    float get_volt2;                //测量值2
    float offset;                   //漂移
    float ratio;                    //比例系数
    window_filter_struct filter;    //窗口滤波
    float output;                   //实际值
} elec_info_struct_t;


/**
  * @brief          初始化ADC线性校准补偿
  * @param[in,out]  p: 补偿结构体指针
  * @param[in]      real1: 真实值1
  * @param[in]      get1: 测量值1
  * @param[in]      real2: 真实值2
  * @param[in]      get2: 测量值2
  * @retval         无
  */
extern void ADC_Linear_calibration_init(elec_info_struct_t *p, float real1, float get1, float real2, float get2);


/**
* @brief          ADC数据转换为真实数据并滤波
* @param[in,out]  p: 结构体指针
* @param[in]      input: 原始数据
* @retval         滤波后数据
*/
extern float ADC_Value_To_Elec(elec_info_struct_t *p, int input);


#endif //ADC_TO_ELEC_H

