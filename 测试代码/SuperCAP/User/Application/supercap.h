#ifndef SUPERCAP_H
#define SUPERCAP_H


#include "bsp_gpio.h"
#include "bsp_tim.h"
#include "bsp_adc.h"
#include "bsp_hrtim.h"
#include "bsp_can.h"

#include "power_loop.h"


//错误码和状态码
#define ERR_OVER_VOLTAGE                            (1u << 0)
#define ERR_LOW_VOLTAGE                             (1u << 1)
#define ERR_CURRENT_SENSOR                          (1u << 2)
#define ERR_OVER_CURRENT                            (1u << 3)
#define ERR_BUCKBOOST                               (1u << 4)
#define ERR_CAN                                     (1u << 5)
#define ERR_MASK                                    (0x0000FFFF)
#define STATUS_CAP_LOW                              (1u << 16)
#define STATUS_CAP_CHARGING                         (1u << 17)
#define STATUS_CAP_DISCHARGING                      (1u << 18)
#define STATUS_CAP_FULL                             (1u << 19)
#define STATUS_MASK                                 (0xFFFF0000)

//ADC相关
#define ADC_MAXVALU                                 4096                                //ADC最大量程
#define ADC_VREF                                    3.300f                              //Vref
#define ADC_FACT                                    ADC_VREF / ADC_MAXVALU

#define V_BIAS                                      0                                   //电压采样偏置
#define V_GAIN_2350                                 10.5f                               //OPA2350电压增益

#define V_RATIO_5050                                ADC_FACT * V_GAIN_2350

#define I_BIAS                                      2048                                //电流采样偏置
#define I_GAIN_A1                                   1.0f / 20.0f                        //INA240A1电压增益20倍
#define I_GAIN_A2                                   1.0f / 50.0f                        //INA240A2电压增益50倍
#define R_SAMP_INV_2M                               500.0f                              //2mOhm采样电阻倒数
#define R_SAMP_INV_4M                               250.0f                              //4mOhm采样电阻倒数
#define R_SAMP_INV_10M                              100.0f                              //10mOhm采样电阻倒数

#define I_RATIO_A1_2M                               ADC_FACT * I_GAIN_A1 * R_SAMP_INV_2M
#define I_RATIO_A1_4M                               ADC_FACT * I_GAIN_A1 * R_SAMP_INV_4M
#define I_RATIO_A1_10M                              ADC_FACT * I_GAIN_A1 * R_SAMP_INV_10M
#define I_RATIO_A2_2M                               ADC_FACT * I_GAIN_A2 * R_SAMP_INV_2M
#define I_RATIO_A2_4M                               ADC_FACT * I_GAIN_A2 * R_SAMP_INV_4M
#define I_RATIO_A2_10M                              ADC_FACT * I_GAIN_A2 * R_SAMP_INV_10M


//硬件限制相关
#define INC_MAX                                     7.5f                                //输入过流阈值
#define OUTC_MAX                                    8.0f                                //输出过流阈值
#define INV_MAX                                     40.0f                               //输入过压阈值
#define INV_MIN                                     19.0f                               //输入欠压阈值
#define CAPV_MAX                                    23.0f                               //电容最大电压
#define ICAP_MAX                                    4.0f                                //电容最大电流


//校准相关
#define INC_CAL_POINT_1                             2.0f
#define INC_CAL_POINT_2                             4.0f
#define INV_CAL_POINT_1                             20.0f
#define INV_CAL_POINT_2                             24.0f
#define OUTC_CAL_POINT_1                            2.0f
#define OUTC_CAL_POINT_2                            4.0f
#define CAPC_CAL_POINT_1                            3.0f
#define CAPC_CAL_POINT_2                            6.0f
#define CAPV_CAL_POINT_1                            17.0f
#define CAPV_CAL_POINT_2                            22.0f

extern void SystemClock_Config(void);


#endif //SUPERCAP_H

