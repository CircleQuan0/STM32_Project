#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "PID.h"
#include "arm_math.h"

osThreadId LEDHandle;
osThreadId Re_crtlHandle;
osThreadId Can_3508_downHandle;

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}

void MX_FREERTOS_Init(void) {

  osThreadDef(LED, StartDefaultTask, osPriorityNormal, 0, 128);
  LEDHandle = osThreadCreate(osThread(LED), NULL);

 
  osThreadDef(Re_crtl, StartTask02, osPriorityIdle, 0, 128);
  Re_crtlHandle = osThreadCreate(osThread(Re_crtl), NULL);

 
  osThreadDef(Can_3508_down, StartTask03, osPriorityIdle, 0, 128);
  Can_3508_downHandle = osThreadCreate(osThread(Can_3508_down), NULL);


}


__weak void StartDefaultTask(void const * argument)
{
	HAL_GPIO_WritePin(GPIOH,GPIO_PIN_11,GPIO_PIN_SET);		//先亮个绿灯
	 for(;;)
  {		
    osDelay(1);
  }
}


__weak void StartTask02(void const * argument)//遥控器初始化和can正常连接判断
{
  remote_control_init();
	motor_info[0].set_voltage=0;
	motor_info[1].set_voltage=0;
	motor_info[2].set_voltage=0;
	motor_info[3].set_voltage=0;//防止bug
  for(;;)
  {		
		if(rc_ctrl.rc.ch[3]>300)
		{
			can_flag=1;
		}
    osDelay(1);
  }

}


__weak void StartTask03(void const * argument)//驱动3508麦轮底盘，can的接收频率明显比uart高(can红灯闪烁频率>uart蓝灯闪烁频率)
{
	for (uint8_t i = 0; i < 4; i++)//先将pid赋初值
	{
		pid_init(&motor_pid[i], 30, 1, 10, 16384, 16384); //init pid parameter, kp=40, ki=3, kd=0, output limit = 16384
	}//P高了就会抖动，返回值溢出，超过80电机有移动就会抖动，超过100直接开抖(除非你设定的速度值>=320)

  for(;;)
  {
		for (uint8_t i = 0; i < 4; i++)
{			//最大值为482*19=9158
      motor_info[i].set_voltage = pid_calc(&motor_pid[i], target_speed[i], motor_info[i].rotor_speed);//解算PID输出
}//经测试，float的值赋给int时，会直接舍去小数点后面的数(不是四舍五入)
if(can_flag==1)//can通信成功,->3508
	{
		
			if((rc_ctrl.rc.ch[2]>=974&&rc_ctrl.rc.ch[2]<=1074)&&((rc_ctrl.rc.ch[3]>=974)&&(rc_ctrl.rc.ch[3]<=1074))&&(rc_ctrl.rc.ch[4]<=1074)&&(rc_ctrl.rc.ch[4]>=974))
		{
			for(int i=0;i<4;i++)//减速
			{
				if(motor_info[i].rotor_speed>720||motor_info[i].rotor_speed<-720)
				{
					target_speed[i]=0;
					motor_info[i].set_voltage = pid_calc(&motor_pid[i], target_speed[i], motor_info[i].rotor_speed);
				}
				else
				{
					motor_info[i].set_voltage=0;
				}
			}
		}
			else
		{
				if(rc_ctrl.rc.ch[4]>1074)
				{target_curl=sqrt((rc_ctrl.rc.ch[4]-1024)*(rc_ctrl.rc.ch[4]-1024))/660;}//不会强制转换，但用这个可以实现
				else if(rc_ctrl.rc.ch[4]<974)
				{target_curl=-(sqrt((rc_ctrl.rc.ch[4]-1024)*(rc_ctrl.rc.ch[4]-1024))/660);}
				else{target_curl=0;}
				target_curl=target_curl*9158;
				int16_t target_curl_int=target_curl;

				r=sqrt((rc_ctrl.rc.ch[3]-1024)*(rc_ctrl.rc.ch[3]-1024)+(rc_ctrl.rc.ch[2]-1024)*(rc_ctrl.rc.ch[2]-1024));
				sin_sita=(rc_ctrl.rc.ch[3]-1024)/r;
				cos_sita=(rc_ctrl.rc.ch[2]-1024)/r;
				target_v=(r/660)*9158;
				 
				if(target_curl==0)
				{
					target_speed[3]=(0.707*target_v*(sin_sita-cos_sita))/2;
					target_speed[1]=-(0.707*target_v*(sin_sita-cos_sita))/2;
					target_speed[0]=(0.707*target_v*(sin_sita+cos_sita))/2;
					target_speed[2]=-(0.707*target_v*(sin_sita+cos_sita))/2;//保护不旋转只移动时的底盘功率上限
				}
				else
				{
					target_int1=(0.707*target_v*(sin_sita-cos_sita))/2;//经测验，double和float类型可以做乘除，做加减会出现NAN,因此转换成int做加减法，注意不要使用uint（无符号型）
					target_int2=(0.707*target_v*(sin_sita+cos_sita))/2;

					target_speed[3]=(target_int1-target_curl_int)/2;
					target_speed[1]=(-target_int1-target_curl_int)/2;
					target_speed[0]=(target_int2-target_curl_int)/2;
					target_speed[2]=(-target_int2-target_curl_int)/2;//不÷2会越界
				}
		}
	

	}
else//连接未成功
	{
				motor_info[0].set_voltage=0;
				motor_info[1].set_voltage=0;
				motor_info[2].set_voltage=0;
				motor_info[3].set_voltage=0;
	}
			if(rc_ctrl.rc.ch[0]>=974&&rc_ctrl.rc.ch[0]<=1074)//回中
		{		
				if(motor_info[4].rotor_speed>10||motor_info[4].rotor_speed<-10)//减震
				{
					target_speed[4]=0;
					motor_info[4].set_voltage = pid_calc(&motor_pid[4], target_speed[4], motor_info[4].rotor_speed);
				}
				else
				{
					motor_info[4].set_voltage=0;
				}
		}
		else
		{
				target_speed[4]=yuntai_step*(rc_ctrl.rc.ch[0]-1024);
				motor_info[4].set_voltage = pid_calc(&motor_pid[4], target_speed[4], motor_info[4].rotor_speed);
	  	}
		set_motor_voltage(0, 
                      motor_info[0].set_voltage, 
                      motor_info[1].set_voltage, 
                      motor_info[2].set_voltage, 
                      motor_info[3].set_voltage);
    
    set_motor_voltage(1, 
                      motor_info[4].set_voltage, 
                      motor_info[5].set_voltage, 
                      motor_info[6].set_voltage, 
                      0);
	
    osDelay(1);
  }
  
}


