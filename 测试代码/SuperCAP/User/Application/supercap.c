#include "supercap.h"


#define BOARD_ID                        0                           //板子编号
#define POWER_SET_DEFAULT               50.0f                       //上电默认功率


#define DISABLE_POST                    0                           //跳过上电检查
#define DISABLE_BUCKBOOST               0                           //关闭H桥
#define DISABLE_PROTECTION              0                           //关闭保护


#define IGNORE_CAN_ERROR                0                           //不显示CAN通信错误


//最后一次CAN接收时间戳
uint32_t last_can_tick;
//状态码
uint32_t s_code = 0;


//0 in_v_bias, 1 in_v_ratio, 2 cap_v_bias, 3 cap_v_ratio, 4 cap_c_bias, 5 cap_c_ratio, 6 in_c_bias, 7 in_c_ratio, 8 out_c_bias, 9 out_c_ratio
const float samp_parameters[][10] = {
{V_BIAS, V_RATIO_5050, V_BIAS, V_RATIO_5050, I_BIAS, I_RATIO_A1_2M, I_BIAS, I_RATIO_A1_10M, I_BIAS, I_RATIO_A1_10M},
{V_BIAS, V_RATIO_5050, V_BIAS, V_RATIO_5050, I_BIAS, I_RATIO_A1_2M, I_BIAS, I_RATIO_A2_4M, I_BIAS, I_RATIO_A2_4M},
{V_BIAS, V_RATIO_5050, V_BIAS, V_RATIO_5050, I_BIAS, I_RATIO_A1_2M, I_BIAS, I_RATIO_A1_10M, I_BIAS, I_RATIO_A1_10M},
{V_BIAS, V_RATIO_5050, V_BIAS, V_RATIO_5050, I_BIAS, I_RATIO_A1_2M, I_BIAS, I_RATIO_A1_10M, I_BIAS, I_RATIO_A1_10M},
{V_BIAS, V_RATIO_5050, V_BIAS, V_RATIO_5050, I_BIAS, I_RATIO_A1_2M, I_BIAS, I_RATIO_A1_10M, I_BIAS, I_RATIO_A1_10M},
};


//0 in_v_1, 1 in_v_2, 2 cap_v_1, 3 cap_v_2, 4 cap_c_1, 5 cap_c_2, 6 in_c_1, 7 in_c_2, 8 out_c_1, 9 out_c_2
const float calibration_parameters[][10] = {
{20.00f, 24.00f, 17.00f, 22.00f, 3.00f, 6.00f, 2.00f, 4.00f, 2.00f, 4.00f},
{19.80f, 23.80f, 16.79f, 21.75f, 3.00f, 6.00f, 1.98f, 4.01f, 2.00f-0.65f, 4.00f-0.65f},
{20.16f, 24.19f, 17.01f, 22.03f, 3.00f, 6.00f, 1.96f, 3.93f, 1.96f, 4.02f},
{19.98f, 23.98f, 17.01f, 22.04f, 3.00f, 6.00f, 1.96f, 3.94f, 1.96f, 4.01f},
{20.00f, 24.00f, 17.00f, 22.00f, 3.00f, 6.00f, 2.00f, 4.00f, 2.00f, 4.00f},
};


/**
  * @brief          闪烁LED显示板子ID
  * @param[in]      id: 板子ID
  * @retval         无
  */
static void Show_Board_ID(uint8_t id);


/**
  * @brief          设置板子参数
  * @param[in]      id: 板子ID
  * @retval         无
  */
static void Set_Board_Config(uint8_t id);


/**
  * @brief          上电自检
  * @retval         错误码
  */
static uint32_t PowerON_Diagnose(void);


/**
  * @brief          计算剩余电容和最大电流
  * @retval         错误码
  */
static uint32_t Calc_CAP_Remain_And_Current(void);


/**
  * @brief          运行状态自检
  * @retval         错误码
  */
static uint32_t Running_Diagnose(void);


/**
  * @brief          LED显示状态
  * @param[in]      id: 板子ID
  * @retval         无
  */
static void Show_Status(uint32_t status_code);


/**
  * @brief          CAN发送
  * @retval         无
  */
static void CAN_Send(void);


__NO_RETURN int main(void) {

    //HAL初始化
    HAL_Init();

    //时钟初始化
    SystemClock_Config();

    //外设初始化
    BSP_GPIO_Init();
    BSP_ADC_Init();
    BSP_TIM_Init();
    BSP_HRTIM_Init();
    BSP_CAN_Init();

    //开始ADC采集
    BSP_ADC_Start_DMA(&hadc1, (uint32_t *) samp.adc1_buf, 3);
    BSP_ADC_Start_DMA(&hadc2, (uint32_t *) samp.adc2_buf, 2);

    //设定蜂鸣器和LED默认状态
    Update_Buzzer_PWM(0);
    Update_LED_PWM(0, 0, 0);

    //显示板子ID
    Show_Board_ID(BOARD_ID);

    //设定校准参数
    Set_Board_Config(BOARD_ID);

    //上电自检
    #if !DISABLE_POST
    s_code |= PowerON_Diagnose();
    #endif

    //自检没有错误
    if ((s_code & ERR_MASK) == 0) {
        //开始PWM和循环定时器
        #if !DISABLE_BUCKBOOST
        BSP_HRTIM_Enable_PWM();
        BSP_TIM_Start_Loop_Timer();
        #endif
    }

    while (1) {
        //清空状态标志位
        s_code &= ~STATUS_MASK;

        //计算剩余容量和最大电流
        s_code |= Calc_CAP_Remain_And_Current();

        //状态检测
        s_code |= Running_Diagnose();

        //发生错误
        if ((s_code & ERR_MASK) != 0) {
            //停止PWM和控制循环
            #if !DISABLE_PROTECTION
            BSP_TIM_Stop_Loop_Timer();
            BSP_HRTIM_Disable_PWM();
            status.cap_mode = CAP_MODE_CUTOFF;
            #endif
        }

        //显示状态
        Show_Status(s_code);

        //CAN发送
        CAN_Send();

        //1K频率延时
        HAL_Delay(1);
    }
}


/**
  * @brief          闪烁LED显示板子ID
  * @param[in]      id: 板子ID
  * @retval         无
  */
static void Show_Board_ID(uint8_t id) {
    for (uint8_t i = 0; i < id; i++) {
        Update_LED_PWM(5, 5, 5);
        HAL_Delay(150);
        Update_LED_PWM(0, 0, 0);
        HAL_Delay(150);
    }
}


/**
  * @brief          设置板子参数
  * @param[in]      id: 板子ID
  * @retval         无
  */
static void Set_Board_Config(uint8_t id) {
    //采样参数
    samp.in_v_filter.raw_bias = (int) samp_parameters[id][0];
    samp.in_v_filter.ratio = samp_parameters[id][1];
    samp.cap_v_filter.raw_bias = (int) samp_parameters[id][2];
    samp.cap_v_filter.ratio = samp_parameters[id][3];
    samp.cap_c_filter.raw_bias = (int)samp_parameters[id][4];
    samp.cap_c_filter.ratio = samp_parameters[id][5];
    samp.in_c_filter.raw_bias = (int)samp_parameters[id][6];
    samp.in_c_filter.ratio = samp_parameters[id][7];
    samp.out_c_filter.raw_bias = (int)samp_parameters[id][8];
    samp.out_c_filter.ratio = samp_parameters[id][9];

    //校准参数
    ADC_Linear_calibration_init(&samp.in_v_filter, INV_CAL_POINT_1, calibration_parameters[id][0], INV_CAL_POINT_2, calibration_parameters[id][1]);
    ADC_Linear_calibration_init(&samp.cap_v_filter, CAPV_CAL_POINT_1, calibration_parameters[id][2], CAPV_CAL_POINT_2, calibration_parameters[id][3]);
    ADC_Linear_calibration_init(&samp.cap_c_filter, CAPC_CAL_POINT_1, calibration_parameters[id][4], CAPC_CAL_POINT_2, calibration_parameters[id][5]);
    ADC_Linear_calibration_init(&samp.in_c_filter, INC_CAL_POINT_1, calibration_parameters[id][6], INC_CAL_POINT_2, calibration_parameters[id][7]);
    ADC_Linear_calibration_init(&samp.out_c_filter, OUTC_CAL_POINT_1, calibration_parameters[id][8], OUTC_CAL_POINT_2, calibration_parameters[id][9]);

    //控制参数
    control.cap_v_max = CAPV_MAX;
    control.cap_max_curr = ICAP_MAX;
    control.power_set = POWER_SET_DEFAULT;

    //PID初始化
    PID_init(&control.currout_loop, PID_DELTA, 0.2f, 0.03f, 0.11f, 150, 25, -200, -25);     //电流环
    PID_init(&control.voltout_loop, PID_DELTA, 1.0f, 0.06f, 3.0f, 100, 25, -100, -25);      //电压环
    PID_init(&control.powerin_loop, PID_DELTA, 0.35f, 0.035f, 3, 400, 50, -400, -50);       //输入功率环
}


/**
  * @brief          上电自检
  * @retval         错误码
  */
static uint32_t PowerON_Diagnose(void) {

    uint32_t err_code = 0;

    //等待ADC数据
    while (!HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC) && \
    !HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc2), HAL_ADC_STATE_REG_EOC)) {
        HAL_Delay(1);
}

    //采样10次
    for (uint8_t i = 0; i < 10; ++i) {
        ADC_Value_To_Elec(&samp.in_v_filter, (uint16_t)samp.adc1_buf[0]);
        ADC_Value_To_Elec(&samp.out_c_filter, (uint16_t)samp.adc1_buf[1]);
        ADC_Value_To_Elec(&samp.in_c_filter, (uint16_t)samp.adc1_buf[2]);
        ADC_Value_To_Elec(&samp.cap_c_filter, (uint16_t)samp.adc2_buf[0]);
        ADC_Value_To_Elec(&samp.cap_v_filter, (uint16_t)samp.adc2_buf[1]);
        HAL_Delay(1);
    }
    samp.in_v = samp.in_v_filter.output;
    samp.out_c = samp.out_c_filter.output;
    samp.in_c = samp.in_c_filter.output;
    samp.cap_c = samp.cap_c_filter.output;
    samp.cap_v = samp.cap_v_filter.output;

    //检测过压
    if (samp.in_v_filter.output > INV_MAX) err_code |= ERR_OVER_VOLTAGE;

    //检测欠压
    if (samp.in_v_filter.output < INV_MIN) err_code |= ERR_LOW_VOLTAGE;

    //检测电流传感器异常
    if (fabsf(samp.in_c_filter.output) > 0.5f || fabsf(samp.out_c_filter.output) > 0.5f || \
        fabsf(samp.in_c_filter.output - samp.out_c_filter.output) > 0.5f) err_code |= ERR_CURRENT_SENSOR;

    return err_code;
}


/**
  * @brief          计算剩余电容和最大电流
  * @retval         错误码
  */
static uint32_t Calc_CAP_Remain_And_Current(void) {

    uint32_t status_code = 0;

    //计算电容最大电流
    //为防止电容电压过低影响最大电流, 设定最大电流的最小值1A
    control.dcdc_max_curr = fmaxf((samp.cap_v / samp.dcdc_v) * (control.cap_max_curr - 1.0f), 1.0f);

    //计算电容剩余能量
    status.cap_percent = (int)(((samp.cap_v * samp.cap_v) / (23.0f * 23.0f)) * 100.0f);
    if (status.cap_percent < 0) status.cap_percent = 0;
    if (status.cap_percent > 100) status.cap_percent = 100;

    //判断电容模式
    if (status.cap_mode == CAP_MODE_DISCHARGE) {
        if (status.cap_percent < 20) {
            //放电到20%以下处于低电量状态
            status_code = STATUS_CAP_LOW;
        } else {
            //正常放电
            status_code = STATUS_CAP_DISCHARGING;
        }
    } else if (status.cap_mode == CAP_MODE_CHARGE) {
        if (status.loop_mode == LOOP_MODE_CC) {
            //恒流可以认为没有充满
            status_code = STATUS_CAP_CHARGING;
        } else if (status.loop_mode == LOOP_MODE_CV) {
            //恒压可以认为已经充满
            status_code = STATUS_CAP_FULL;
        }
    }
    return status_code;
}


/**
  * @brief          运行状态自检
  * @retval         错误码
  */
static uint32_t Running_Diagnose(void) {

    //保护延时
    static uint16_t disable_cnt = 0;
    uint32_t err_code = 0;

    //CAN通信异常
    #if !IGNORE_CAN_ERROR
    if (HAL_GetTick() - last_can_tick > 1000) err_code |= ERR_CAN;
    #endif

    //判断保护条件
    if (((status.loop_mode == LOOP_MODE_CC || samp.cap_v < 22.0f) && fabsf(samp.dcdc_c - control.dcdc_curr) > 1.0f) || samp.in_v > INV_MAX || samp.in_v < INV_MIN || samp.in_c > INC_MAX || samp.in_c < -INC_MAX) {
        disable_cnt++;
        if (disable_cnt > 50) {
            //欠压
            if (samp.in_v < INV_MIN) err_code |= ERR_LOW_VOLTAGE;
            //过压
            if (samp.in_v > INV_MAX) err_code |= ERR_OVER_VOLTAGE;
            //过流
            if (samp.in_c > INC_MAX || samp.in_c < -INC_MAX) err_code |= ERR_OVER_CURRENT;
            //充放电异常
            if ((status.loop_mode == LOOP_MODE_CC || samp.cap_v < 22.0f) && fabsf(samp.dcdc_c - control.dcdc_curr) > 1.0f) err_code |= ERR_BUCKBOOST;
        }
    }
    else{
        disable_cnt = 0;
    }
    return err_code;
}

/**
  * @brief          显示状态
  * @param[in]      id: 板子ID
  * @retval         无
  */
static void Show_Status(uint32_t status_code) {

    static uint32_t status_code_copy = 0;
    static uint32_t state = 0;
    static uint32_t next_time = 0;

    if (status_code_copy == 0) status_code_copy = status_code;

    //正常充放电
    if ((status_code_copy & ERR_MASK) == 0) {
        if ((status_code_copy & STATUS_CAP_DISCHARGING) != 0) {
            //放电
            Update_LED_PWM(5, 0, 0);
            //清空状态码
            status_code_copy = 0;
        } else if ((status_code_copy & STATUS_CAP_CHARGING) != 0) {
            //充电
            Update_LED_PWM(0, 0, 5);
            //清空状态码
            status_code_copy = 0;
        } else if ((status_code_copy & STATUS_CAP_FULL) != 0) {
            //满电
            Update_LED_PWM(0, 5, 0);
            //清空状态码
            status_code_copy = 0;
        } else if ((status_code_copy & STATUS_CAP_LOW) != 0) {
            //低电量
            if (state == 0) {
                //状态0, 红亮250ms
                Update_LED_PWM(5, 0, 0);
                state = 1;
                next_time = HAL_GetTick() + 250;
            } else if (state == 1){
                if (HAL_GetTick() > next_time) {
                    //状态1, 红灭250ms
                    Update_LED_PWM(0, 0, 0);
                    state = 2;
                    next_time = HAL_GetTick() + 250;
                }
            } else if (state == 2){
                if (HAL_GetTick() > next_time) {
                    //状态2, 结束
                    state = 0;
                    //清空状态码
                    status_code_copy = 0;
                }
            }
        }
    //状态异常
    } else if ((status_code_copy & ERR_MASK) != 0) {
        if ((status_code_copy & ERR_OVER_VOLTAGE) != 0) {
            //过压
            if (state == 0) {
                //状态0, 红亮100ms
                Update_LED_PWM(5, 0, 0);
                state = 1;
                next_time = HAL_GetTick() + 100;
            } else if (state == 1 && HAL_GetTick() > next_time){
                //状态1, 蓝亮400ms
                Update_LED_PWM(0, 0, 5);
                state = 2;
                next_time = HAL_GetTick() + 400;
            } else if (state == 2 && HAL_GetTick() > next_time){
                //状态2, 灭500ms
                Update_LED_PWM(0, 0, 0);
                state = 3;
                next_time = HAL_GetTick() + 500;
            } else if (state == 3 && HAL_GetTick() > next_time){
                //状态3, 结束
                state = 0;
                //清空状态码
                status_code_copy &= ~ERR_OVER_VOLTAGE;
            }
        } else if ((status_code_copy & ERR_LOW_VOLTAGE) != 0) {
            //欠压
            if (state == 0) {
                //状态0, 红亮400ms
                Update_LED_PWM(5, 0, 0);
                state = 1;
                next_time = HAL_GetTick() + 400;
            } else if (state == 1 && HAL_GetTick() > next_time){
                //状态1, 蓝亮100ms
                Update_LED_PWM(0, 0, 5);
                state = 2;
                next_time = HAL_GetTick() + 100;
            } else if (state == 2 && HAL_GetTick() > next_time){
                //状态2, 灭500ms
                Update_LED_PWM(0, 0, 0);
                state = 3;
                next_time = HAL_GetTick() + 500;
            } else if (state == 3 && HAL_GetTick() > next_time){
                //状态3, 结束
                state = 0;
                //清空状态码
                status_code_copy &= ~ERR_LOW_VOLTAGE;
            }
        } else if ((status_code_copy & ERR_OVER_CURRENT) != 0 || (status_code_copy & ERR_CURRENT_SENSOR) != 0) {
            //过流或传感器
            if (state == 0) {
                //状态0, 红亮250ms
                Update_LED_PWM(5, 0, 0);
                state = 1;
                next_time = HAL_GetTick() + 250;
            } else if (state == 1 && HAL_GetTick() > next_time){
                //状态1, 绿亮250ms
                Update_LED_PWM(0, 5, 0);
                state = 2;
                next_time = HAL_GetTick() + 250;
            } else if (state == 2 && HAL_GetTick() > next_time){
                //状态2, 灭500ms
                Update_LED_PWM(0, 0, 0);
                state = 3;
                next_time = HAL_GetTick() + 500;
            } else if (state == 3 && HAL_GetTick() > next_time){
                //状态3, 结束
                state = 0;
                //清空状态码
                status_code_copy &= ~ERR_OVER_CURRENT;
                status_code_copy &= ~ERR_CURRENT_SENSOR;
            }
        } else if ((status_code_copy & ERR_BUCKBOOST) != 0) {
            //充放电
            if (state == 0) {
                //状态0, 黄亮500ms
                Update_LED_PWM(5, 5, 0);
                state = 1;
                next_time = HAL_GetTick() + 500;
            } else if (state == 1 && HAL_GetTick() > next_time){
                //状态1, 灭500ms
                Update_LED_PWM(0, 0, 0);
                state = 2;
                next_time = HAL_GetTick() + 500;
            } else if (state == 2 && HAL_GetTick() > next_time){
                //状态2, 结束
                state = 0;
                //清空状态码
                status_code_copy &= ~ERR_BUCKBOOST;
            }
        } else if ((status_code_copy & ERR_CAN) != 0) {
            //CAN
            if (state == 0) {
                //状态0, 青亮500ms
                Update_LED_PWM(0, 5, 5);
                state = 1;
                next_time = HAL_GetTick() + 500;
            } else if (state == 1 && HAL_GetTick() > next_time){
                //状态1, 灭500ms
                Update_LED_PWM(0, 0, 0);
                state = 2;
                next_time = HAL_GetTick() + 500;
            } else if (state == 2 && HAL_GetTick() > next_time){
                //状态2, 结束
                state = 0;
                //清空状态码
                status_code_copy &= ~ERR_CAN;
            }
        }
    }

}


/**
  * @brief          CAN发送
  * @retval         无
  */
static void CAN_Send(void) {
    CAN_TxHeaderTypeDef can_tx_message;
    uint32_t send_mail_box;
    can_tx_message.StdId = 0x211;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    unsigned char can_send_data[8] = {0};
    float in_v_copy = samp.in_v;
    float cap_v_copy = samp.cap_v;
    float in_c_copy = samp.in_c;
    float power_copy = control.power_set;
    if (in_v_copy < 0) in_v_copy = 0;
    if (cap_v_copy < 0) cap_v_copy = 0;
    if (in_c_copy < 0) in_c_copy = 0;
    if (power_copy < 0) power_copy = 0;
    uint16_t in_v_send = (uint16_t) ((int)(in_v_copy * 100.0f));
    uint16_t cap_v_send = (uint16_t) ((int)(cap_v_copy * 100.0f));
    uint16_t in_c_send = (uint16_t) ((int)(in_c_copy * 100.0f));
    uint8_t power_send = (uint8_t) ((int)power_copy);
    can_send_data[0] = in_v_send & 0xff;
    can_send_data[1] = in_v_send >> 8;
    can_send_data[2] = cap_v_send & 0xff;
    can_send_data[3] = cap_v_send >> 8;
    can_send_data[4] = in_c_send & 0xff;
    can_send_data[5] = in_c_send >> 8;
    if (status.cap_mode == CAP_MODE_CUTOFF) {
        can_send_data[6] = 0;
    } else {
        can_send_data[6] = status.cap_percent;
    }
    can_send_data[7] = power_send;
    HAL_CAN_AddTxMessage(&hcan, &can_tx_message, can_send_data, &send_mail_box);
}


/**
  * @brief          CAN接收回调函数
  * @param[in]      can_handle: CAN句柄
  * @retval         无
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *can_handle) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    //获取数据包
    HAL_CAN_GetRxMessage(can_handle, CAN_RX_FIFO0, &rx_header, rx_data);

    //判断包头
    if (rx_header.StdId == 0x210) {
        uint16_t power_tmp = rx_data[0] << 8 | rx_data[1];
        uint16_t buf_tmp = rx_data[2] << 8 | rx_data[3];

        //得到实际设定功率
        float power = ((float) power_tmp) / 100;

        //限幅
        if (power > 130.0f) power = 130.0f;
        if (power < 30.0f) power = 30.0f;

        control.power_set = power;

        //更新CAN接收时间戳
        last_can_tick = HAL_GetTick();
    }
}

