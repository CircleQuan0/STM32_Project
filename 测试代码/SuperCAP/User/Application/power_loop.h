#ifndef POWER_LOOP_H
#define POWER_LOOP_H

#include "adc_to_elec.h"
#include "pid.h"
#include "bsp_hrtim.h"
#include "bsp_tim.h"
#include <math.h>


/*PWM相关*/
#define shift_duty 0.98f                                    //最大占空比
#define shift_duty_2 2 * shift_duty                         //最大占空比 * 2
#define MAX_PWM_CMP (uint32_t)(shift_duty * 1000)           //PWM最大比较值
#define MIN_PWM_CMP (uint32_t)((1 - shift_duty) * 1000)     //PWM最小比较值
#define DP_PWM_PER 1000                                     //定时器HRTIM周期
#define max_volt_ratio 1.2f                                 //最大电压比值
#define min_volt_ratio 0.1f                                 //最小电压比值


//采样结构体
typedef struct {
    //ADC DMA缓冲区
    unsigned int adc1_buf[3];                               //输入电压, 输出电流, 输入电流
    unsigned int adc2_buf[2];                               //电容电流, 电容电压

    //电气数据结构体
    elec_info_struct_t cap_c_filter;                        //电容电流
    elec_info_struct_t cap_v_filter;                        //电容电压
    elec_info_struct_t in_c_filter;                         //输入电流
    elec_info_struct_t in_v_filter;                         //输入电压
    elec_info_struct_t out_c_filter;                        //底盘电流

    //滤波后数据
    float in_v;                                             //输入电压
    float in_c;                                             //输入电流
    float in_p;                                             //输入功率
    float out_v;                                            //输出电压
    float out_c;                                            //输出电流
    float out_p;                                            //输出功率
    float dcdc_v;                                           //DCDC电压
    float dcdc_c;                                           //DCDC电流
    float dcdc_p;                                           //DCDC功率
    float cap_v;                                            //电容电压
    float cap_c;                                            //电容电流
    float cap_p;                                            //电容功率

} samp_struct_t;


//控制结构体
typedef struct {
    pid_type_def powerin_loop, currout_loop, voltout_loop;  //pid结构体
    float power_set;                                        //设定电池输入功率
    float dcdc_power;                                       //dcdc充放电设定功率
    float dcdc_curr;                                        //dcdc充放电设定电流
    float dcdc_max_curr;                                    //dcdc最大电流
    float cap_v_max;                                        //电容最大电压
    float cap_max_curr;                                     //电容最大电流
    float vloop_ratio;                                      //电压环输出
    float cloop_ratio;                                      //电流环输出
    float volt_ratio;                                       //最终输出电压比值 (Vout / Vin)
} control_struct_t;

//电容状态枚举
enum CAP_MODE
{
    CAP_MODE_CHARGE = 1,                                    //电容充电
    CAP_MODE_DISCHARGE,                                     //电容放电
    CAP_MODE_CUTOFF                                         //电容关断
};

//闭环状态枚举
enum LOOP_MODE
{
    LOOP_MODE_CV = 1,                                       //恒压闭环
    LOOP_MODE_CC                                            //恒流闭环
};


//状态结构体
typedef struct {
    enum LOOP_MODE loop_mode;                               //环路模式
    enum CAP_MODE cap_mode;                                 //冲放电模式
    uint8_t cap_percent;                                    //剩余百分比
} status_struct_t;


extern samp_struct_t samp;                                  //采样结构体
extern control_struct_t control;                            //控制结构体
extern status_struct_t status;                              //状态结构体


#endif //POWER_LOOP_H

