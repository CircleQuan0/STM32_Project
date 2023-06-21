#include "power_loop.h"


samp_struct_t samp;                                     //采样结构体
control_struct_t control;                               //控制结构体
status_struct_t status;                                 //状态结构体


/**
  * @brief          电容充放电控制闭环
  * @retval         无
  */
static void control_calc(void);


/**
  * @brief          计算并更新buckboost的pwm
  * @param[in]      volt_ratio: 电压比值 (Vout/Vin)
  * @retval         无
  */
static void buckboost_pwm_update(float volt_ratio);


/**
  * @brief          定时器回调函数
  * @param[in]      htim: 定时器句柄
  * @retval         无
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim6) {
        control_calc();
    }
}


/**
  * @brief          电容充放电控制闭环
  * @retval         无
  */
static void control_calc(void) {
    //ADC原始数据转换为真实数据并滤波
    ADC_Value_To_Elec(&samp.in_v_filter, (uint16_t)samp.adc1_buf[0]);
    ADC_Value_To_Elec(&samp.out_c_filter, (uint16_t)samp.adc1_buf[1]);
    ADC_Value_To_Elec(&samp.in_c_filter, (uint16_t)samp.adc1_buf[2]);
    ADC_Value_To_Elec(&samp.cap_c_filter, (uint16_t)samp.adc2_buf[0]);
    ADC_Value_To_Elec(&samp.cap_v_filter, (uint16_t)samp.adc2_buf[1]);

    //IN
    samp.in_v = samp.in_v_filter.output;                                //输入电压直接测得
    samp.in_c = samp.in_c_filter.output;                                //输入电流直接测得
    samp.in_p = samp.in_v * samp.in_c;                                  //计算输入功率
    //OUT
    samp.out_v = samp.in_v_filter.output;                               //输出电压与输入电压一致
    samp.out_c = samp.out_c_filter.output;                              //输出电流直接测得
    samp.out_p = samp.in_v * samp.in_c;                                 //计算输出功率
    //DCDC
    samp.dcdc_v = samp.in_v_filter.output;                              //dcdc输入电压与输入电压一致
    samp.dcdc_c = samp.in_c - samp.out_c;                               //dcdc电流由输入电流-输出电流得到
    samp.dcdc_p = samp.dcdc_v * samp.dcdc_c;                            //计算dcdc输入功率
    //CAP
    samp.cap_v = samp.cap_v_filter.output;                              //电容电压直接测得
    samp.cap_c = samp.cap_c_filter.output;                              //电容电流直接测得
    samp.cap_p = samp.cap_v * samp.cap_c;                               //计算电容功率

    //外环功率环, 得到dcdc充放电功率
    control.dcdc_power = PID_calc(&control.powerin_loop, samp.in_p, control.power_set);

    //计算并限制最大dcdc充放电电流
    control.dcdc_curr = fmaxf(fminf((control.dcdc_power) / samp.dcdc_v, control.dcdc_max_curr), -control.dcdc_max_curr);

    //判断充放电
    if (control.dcdc_power > 0) {
        //充电
        status.cap_mode = CAP_MODE_CHARGE;

        //电压电流环并行计算
        control.cloop_ratio = PID_calc(&control.currout_loop, samp.dcdc_c, control.dcdc_curr) / samp.dcdc_v;
        control.vloop_ratio = PID_calc(&control.voltout_loop, samp.cap_v, control.cap_v_max) / samp.dcdc_v;

        //判断使用哪个环数据
        if (control.vloop_ratio <= control.cloop_ratio) {
            //恒压
            control.volt_ratio = control.vloop_ratio;
            status.loop_mode = LOOP_MODE_CV;
        } else {
            //恒流
            control.volt_ratio = control.cloop_ratio;
            status.loop_mode = LOOP_MODE_CC;
        }
        //更新PWM
        buckboost_pwm_update(control.volt_ratio);

    } else if (control.dcdc_power < 0) {
        //放电
        status.cap_mode = CAP_MODE_DISCHARGE;
        status.loop_mode = LOOP_MODE_CC;
        //单电流环PID
        control.cloop_ratio = PID_calc(&control.currout_loop, samp.dcdc_c, control.dcdc_curr) / samp.cap_v;
        control.volt_ratio = control.cloop_ratio;
        //更新PWM
        buckboost_pwm_update(control.volt_ratio);
    }
}


/**
  * @brief          计算并更新buckboost的pwm
  * @param[in,out]  volt_ratio: 电压比值 (Vout/Vin)
  * @retval         无
  */
static void buckboost_pwm_update(float volt_ratio)
{
    volatile unsigned int CompareValue;
    volatile unsigned int buck_duty;
    volatile unsigned int boost_duty;
    volt_ratio = fminf(max_volt_ratio, volt_ratio);
    volt_ratio = fmaxf(min_volt_ratio, volt_ratio);

    /*
     * 升降压的PWM方案：
     * 1.由于MOS采用自举电容驱动方式，故上管不能100%占空比导通，这里我们设为95%。
     * 2.本函数的输入参数为总的占空比，需要分别算出两个半桥的占空比。
     * 3.降压工作时，BOOST的半桥的占空比是固定的为95%，通过改变BUCK半桥占空比实现稳压。
     * 3.升压或等压工作时，BUCK半桥的占空比是固定的为95%，通过改变BOOST半桥占空比实现稳压。
     */
    if (volt_ratio >= 1.0f)
        CompareValue = DP_PWM_PER * (shift_duty_2 - shift_duty / volt_ratio);
    else
        CompareValue = shift_duty * volt_ratio * DP_PWM_PER;

    //计算boost半桥占空比
    if ((float) CompareValue > shift_duty * DP_PWM_PER)
        //BOOST半桥占空比，Dboost = D总 - Dbuck
        boost_duty = shift_duty_2 * DP_PWM_PER - (float) CompareValue;
    else
        //BUCK模式下boost_duty给固定占空比
        boost_duty = shift_duty * DP_PWM_PER;

    //计算buck半桥占空比
    if ((float) CompareValue > shift_duty * DP_PWM_PER)
        //BOOST模式下buck_duty固定占空比
        buck_duty = shift_duty * DP_PWM_PER;
    else
        //BUCK半桥占空比
        buck_duty = CompareValue;

    //占空比限制
    if (boost_duty > MAX_PWM_CMP)
        boost_duty = MAX_PWM_CMP;
    if (boost_duty < MIN_PWM_CMP)
        boost_duty = MIN_PWM_CMP;
    if (buck_duty > MAX_PWM_CMP)
        buck_duty = MAX_PWM_CMP;
    if (buck_duty < MIN_PWM_CMP)
        buck_duty = MIN_PWM_CMP;

    //更新比较寄存器
    BSP_HRTIM_Update_PWM((DP_PWM_PER + buck_duty) / 2u, (DP_PWM_PER - buck_duty) / 2u, \
                        (DP_PWM_PER - boost_duty) / 2u, (DP_PWM_PER + boost_duty) / 2u);
}

