#include "bsp_hrtim.h"


/**
  * @brief          初始化HRTIM
  * @retval         无
  */
void BSP_HRTIM_Init(void) {
    MX_HRTIM1_Init();
    HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | \
                                                        HRTIM_TIMERID_TIMER_B);
}


/**
  * @brief          更新HRTIM比较寄存器
  * @param[in]      cha1: CHA开启时间
  * @param[in]      cha1: CHA关断时间
  * @param[in]      chb1: CHB开启时间
  * @param[in]      chb2: CHB关断时间
  * @retval         无
  */
void BSP_HRTIM_Update_PWM(uint32_t cha1, uint32_t cha2, uint32_t chb1, uint32_t chb2) {
    HRTIM1->sMasterRegs.MCMP1R = cha1;
    HRTIM1->sMasterRegs.MCMP2R = cha2;
    HRTIM1->sMasterRegs.MCMP3R = chb1;
    HRTIM1->sMasterRegs.MCMP4R = chb2;
}


/**
  * @brief          HRTIM使能输出PWM
  * @retval         无
  */
void BSP_HRTIM_Enable_PWM(void) {
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | \
                                                                HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
}


/**
  * @brief          HRTIM停止输出PWM
  * @retval         无
  */
void BSP_HRTIM_Disable_PWM(void) {
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | \
                                                                HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
}

