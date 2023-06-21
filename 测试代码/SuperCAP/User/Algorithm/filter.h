#ifndef FILTER_H
#define FILTER_H


typedef struct {
    int buffer[10];                 //窗口宽度
    unsigned char ptr;              //下一个窗口位置
    int input;                      //输入数据
    int output;                     //滤波后数据
} window_filter_struct;


/**
  * @brief          初始化窗口滤波
  * @param[in,out]  *p: 结构体指针
  * @retval         无
  */
extern void Window_Filter_Init(window_filter_struct *p);


/**
  * @brief          计算窗口滤波
  * @param[in,out]  *p: 结构体指针
  * @retval         无
  */
extern void Window_Filter_Calc(window_filter_struct *p);


#endif //FILTER_H

