#include "filter.h"


/**
  * @brief          初始化窗口滤波
  * @param[in,out]  *p: 结构体指针
  * @retval         无
  */
void Window_Filter_Init(window_filter_struct *p) {
    p->ptr = 0;
    p->input = 0;
    p->output = 0;
    for (unsigned char i = 0; i < 10; i++) {
        p->buffer[i] = 0;
    }
}


/**
  * @brief          计算窗口滤波
  * @param[in,out]  *p: 结构体指针
  * @retval         无
  */
void Window_Filter_Calc(window_filter_struct *p) {
    //将新数据放在窗口中
    p->buffer[p->ptr] = p->input;

    //求和
    long sum = 0;
    for (unsigned char i = 0; i < 10; i++) {
        sum += p->buffer[i];
    }

    //求平均
    p->output = sum / 10;

    //计算下一个窗口位置
    p->ptr++;
    if (p->ptr == 10) p->ptr = 0;
}

