#include "adc_to_elec.h"


/**
  * @brief          初始化ADC线性校准补偿
  * @param[in,out]  *p: 结构体指针
  * @param[in]      real1: 真实值1
  * @param[in]      get1: 测量值1
  * @param[in]      real2: 真实值2
  * @param[in]      get2: 测量值2
  * @retval         无
  */
void ADC_Linear_calibration_init(elec_info_struct_t *p, float real1, float get1, float real2, float get2) {
    float a, b;
    p->get_volt1 = get1;
    p->real_valu1 = real1;
    p->get_volt2 = get2;
    p->real_valu2 = real2;

    //计算斜率a
    a = (p->real_valu2 - p->real_valu1) / (p->get_volt2 - p->get_volt1);
    //计算纵轴截距b
    b = (p->real_valu1 - p->get_volt1 * a);

    //重新计算ratio
    p->ratio = a * p->ratio;
    p->offset = b;

    //初始化窗口滤波
    Window_Filter_Init(&p->filter);
}


/**
  * @brief          ADC数据转换为真实数据并滤波
  * @param[in,out]  p: 结构体指针
  * @param[in]      input: 原始数据
  * @retval         滤波后数据
  */
float ADC_Value_To_Elec(elec_info_struct_t *p, int input) {
    p->raw = input;

    //窗口滤波
    p->filter.input = p->raw - p->raw_bias;
    Window_Filter_Calc(&p->filter);

    //根据参数计算实际值
    p->output = ((float) p->filter.output) * p->ratio + p->offset;

    return p->output;
}

