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
	HAL_GPIO_WritePin(GPIOH,GPIO_PIN_11,GPIO_PIN_SET);		//�������̵�
	 for(;;)
  {		
    osDelay(1);
  }
}


__weak void StartTask02(void const * argument)//ң������ʼ����can���������ж�
{
  remote_control_init();
	motor_info[0].set_voltage=0;
	motor_info[1].set_voltage=0;
	motor_info[2].set_voltage=0;
	motor_info[3].set_voltage=0;//��ֹbug
  for(;;)
  {		
		if(rc_ctrl.rc.ch[3]>300)
		{
			can_flag=1;
		}
    osDelay(1);
  }

}


__weak void StartTask03(void const * argument)//����3508���ֵ��̣�can�Ľ���Ƶ�����Ա�uart��(can�����˸Ƶ��>uart������˸Ƶ��)
{
	for (uint8_t i = 0; i < 4; i++)//�Ƚ�pid����ֵ
	{
		pid_init(&motor_pid[i], 30, 1, 10, 16384, 16384); //init pid parameter, kp=40, ki=3, kd=0, output limit = 16384
	}//P���˾ͻᶶ��������ֵ���������80������ƶ��ͻᶶ��������100ֱ�ӿ���(�������趨���ٶ�ֵ>=320)

  for(;;)
  {
		for (uint8_t i = 0; i < 4; i++)
{			//���ֵΪ482*19=9158
      motor_info[i].set_voltage = pid_calc(&motor_pid[i], target_speed[i], motor_info[i].rotor_speed);//����PID���
}//�����ԣ�float��ֵ����intʱ����ֱ����ȥС����������(������������)
if(can_flag==1)//canͨ�ųɹ�,->3508
	{
		
			if((rc_ctrl.rc.ch[2]>=974&&rc_ctrl.rc.ch[2]<=1074)&&((rc_ctrl.rc.ch[3]>=974)&&(rc_ctrl.rc.ch[3]<=1074))&&(rc_ctrl.rc.ch[4]<=1074)&&(rc_ctrl.rc.ch[4]>=974))
		{
			for(int i=0;i<4;i++)//����
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
				{target_curl=sqrt((rc_ctrl.rc.ch[4]-1024)*(rc_ctrl.rc.ch[4]-1024))/660;}//����ǿ��ת���������������ʵ��
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
					target_speed[2]=-(0.707*target_v*(sin_sita+cos_sita))/2;//��������תֻ�ƶ�ʱ�ĵ��̹�������
				}
				else
				{
					target_int1=(0.707*target_v*(sin_sita-cos_sita))/2;//�����飬double��float���Ϳ������˳������Ӽ������NAN,���ת����int���Ӽ�����ע�ⲻҪʹ��uint���޷����ͣ�
					target_int2=(0.707*target_v*(sin_sita+cos_sita))/2;

					target_speed[3]=(target_int1-target_curl_int)/2;
					target_speed[1]=(-target_int1-target_curl_int)/2;
					target_speed[0]=(target_int2-target_curl_int)/2;
					target_speed[2]=(-target_int2-target_curl_int)/2;//����2��Խ��
				}
		}
	

	}
else//����δ�ɹ�
	{
				motor_info[0].set_voltage=0;
				motor_info[1].set_voltage=0;
				motor_info[2].set_voltage=0;
				motor_info[3].set_voltage=0;
	}
			if(rc_ctrl.rc.ch[0]>=974&&rc_ctrl.rc.ch[0]<=1074)//����
		{		
				if(motor_info[4].rotor_speed>10||motor_info[4].rotor_speed<-10)//����
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


