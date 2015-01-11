#include "stm32f10x.h"
#include <RTL.h>
#include <RTX_CAN.h>
#include "stm32f10x_conf.h"
#include "user_header.h"

void Device_Init(void);
void SWITs_Enable(u8 enable);
void AC_Detect_Enable(u8 enable);
void DMAReConfig(void);


OS_TID tid_send_CAN;
OS_TID tid_recv_CAN;
OS_TID tid_lamp_ctl;
OS_TID tid_green_led_flash;
OS_TID tid_red_led_flash;
OS_TID tid_conflict_monitor;
OS_TID tid_conflict_analysis;
OS_TID tid_heart_beat;
OS_TID tid_current_report;
OS_TID tid_AC_detector;
OS_TID tid_watchdog;

OS_MUT mutex_ADC_minute;

os_mbx_declare (CAN_send_mailbox, 20);
_declare_box(mpool,sizeof(CAN_msg),32);

__task void init(void);
__task void task_send_CAN(void);
__task void task_recv_CAN(void);
__task void task_lamp_ctl(void);
__task void task_green_led_flash(void);
__task void task_red_led_flash(void);
__task void task_conflict_monitor(void);
__task void task_conflict_analysis(void);
__task void task_heart_beat(void);
__task void task_current_report(void);
__task void task_AC_detector(void);
__task void task_watchdog(void);

int test_n[16] = {0};
u8 Lights_status_1[3] = {0};
u8 Lights_status_2[3] = {0};
u8 Picked_lights_status[12] = {0};
u16 Picked_lights_status_map;
u16 conflict_map;
u16 ADC_status_map;
u8 Channels_enabled = 0;
u8 Walker_channels = 0;
u8 ID_Num = 1;
u8 Work_normal = 1;
__IO uint16_t  ADC_buffer[12];
u16 Last_error = 0;
u8 AC_power_exist = 1;
u32 ADC_minute_sum[12] = {0};
u16 ADC_minute_count[12] = {0};

u8 Loops_CAN_send = 0;
u8 Loops_CAN_recv = 0;
u8 Loops_Lamp_ctl = 0;
u8 Loops_Green_led_flash = 0;
u8 Loops_Red_led_flash = 0;
u8 Loops_Conflict_monitor = 0;
u8 Loops_Conflict_analysis = 0;
u8 Loops_Heart_beat = 0;
u8 Loops_Current_report = 0;
u8 Loops_AC_detector = 0;

u16 timer_count = 0;//��������������

/* �Ƶ��̵ƹ��������� */
#define GY_ERR_MASK			0xF00

unsigned char read_ID_addr()
{
	return ((GPIO_ReadInputData(GPIOD)>>9)&0x3F);
}
/*******************************************************************************
* Function Name  : GPIO_PinReverse
* Description    : GPIOλȡ��
* Input          : GPIO�ڣ�GPIO_TypeDef* GPIOx
* Output         : GPIOλ��uint16_t GPIO_Pin
* Return         : None
*******************************************************************************/
void GPIO_PinReverse(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
    assert_param(IS_GPIO_PIN(GPIO_Pin));
    
    GPIOx->ODR ^= GPIO_Pin;
}
int main (void)
{
	os_sys_init(init);
}

__task void init (void) 
{
	Device_Init();
	
	ID_Num = read_ID_addr();
	
	os_mbx_init (CAN_send_mailbox, sizeof(CAN_send_mailbox));
	_init_box (mpool, sizeof(mpool), sizeof(CAN_msg));
	os_mut_init(mutex_ADC_minute);

	// init lights all red
	GPIO_WriteBit(GPIOB, GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14, Bit_RESET);
	GPIO_WriteBit(GPIOB, GPIO_Pin_14|GPIO_Pin_11|GPIO_Pin_8|GPIO_Pin_5, Bit_SET);
	// turn off 2 led indicators
	GPIO_WriteBit(GPIOD, GPIO_Pin_1|GPIO_Pin_2, Bit_SET);
	
	tid_lamp_ctl = os_tsk_create(task_lamp_ctl, 2);
	tid_green_led_flash = os_tsk_create(task_green_led_flash, 2);
	tid_red_led_flash = os_tsk_create(task_red_led_flash, 2);
	tid_conflict_monitor = os_tsk_create(task_conflict_monitor, 2);
	tid_conflict_analysis = os_tsk_create(task_conflict_analysis, 2);
	tid_heart_beat = os_tsk_create(task_heart_beat, 2);
// 	tid_current_report = os_tsk_create(task_current_report, 2);
	tid_AC_detector = os_tsk_create(task_AC_detector, 5);
	tid_send_CAN = os_tsk_create(task_send_CAN, 10);
	tid_recv_CAN = os_tsk_create(task_recv_CAN, 2);
	tid_watchdog = os_tsk_create(task_watchdog, 2);
//	GPIO_SetBits(GPIOC,GPIO_Pin_7);
	
	os_tsk_delete_self();
}

__task void task_send_CAN(void) 
{
	void *msg;
	OS_RESULT result;

	CAN_init(1, 500000);               /* CAN controller 1 init, 1000 kbit/s   */
	CAN_rx_object (1, 2,  33, DATA_TYPE | STANDARD_TYPE); /* Enable reception */
                                       /* of message on controller 1, channel */
                                       /* is not used for STM32 (can be set to*/
                                       /* whatever value), data frame with    */
                                       /* standard id 33                      */
	CAN_start (1);                     /* Start controller 1                  */

	while (1)
	{
		result = os_mbx_wait (CAN_send_mailbox, &msg, 0x000A);

		if (result != OS_R_TMO)
		{
			CAN_send (1, msg, 0x0F00);  /* Send msg_send on controller 1       */
			_free_box (mpool, msg);
		}

		Loops_CAN_send++;
		os_evt_set(EVT_FEED_DOG_CAN_SEND, tid_watchdog);
	}
}
__task void task_recv_CAN(void)
{	
	CAN_msg RxMessage;
	u8 i;
	u8 f_detect = 1;

	while (1)
	{
		if (CAN_receive (1, &RxMessage, 0x000A) == CAN_OK)
		{
			if(RxMessage.data[0] == IPI && RxMessage.data[1] == 0xB1 && RxMessage.data[2] == 0x00)
			{
				switch (RxMessage.data[3])
				{
					case 0x01:
						for(i = 0; i < 3; i++)
						{
							Lights_status_1[i] = RxMessage.data[i + 4];
						}
						if (f_detect)
						{
							os_evt_clr (EVT_DATA_2_RCVD, tid_lamp_ctl);
							os_evt_set (EVT_DATA_1_RCVD, tid_lamp_ctl);
						}
						break;
						
					case 0x02:
						for(i = 0; i < 3; i++)
						{
							Lights_status_2[i] = RxMessage.data[i + 4];
						}
						if (f_detect)
						{
							os_evt_clr (EVT_DATA_3_RCVD, tid_lamp_ctl);
							os_evt_set (EVT_DATA_2_RCVD, tid_lamp_ctl);
						}
						break;
				}
			}
			else if(RxMessage.data[0] == IPI && RxMessage.data[1] == 0xD1 && RxMessage.data[2] == 0x00)
			{
				// The bitmap of available vehicle channels
				Channels_enabled = ((((RxMessage.data[4]<<8)|RxMessage.data[3])>>(ID_Num-1)*4)&0x0f);
				// The bitmap of available walker channels
				Walker_channels = ((((RxMessage.data[6]<<8)|RxMessage.data[5])>>(ID_Num-1)*4)&0x0f);

				if (f_detect)
				{
					os_evt_set (EVT_DATA_3_RCVD, tid_lamp_ctl);
				}
			}
			else if(RxMessage.data[0] == IPI && RxMessage.data[1] == 0xA6 && RxMessage.data[3] == ID_Num)
			{
				if ((RxMessage.data[6] == 5) || (RxMessage.data[6] == 6) || (RxMessage.data[6] == 7) || (RxMessage.data[6] == 8))
				{
					f_detect = 0;
				}
				else if (RxMessage.data[6] == 10)
				{
					f_detect = 1;
				}
			}
			else if(RxMessage.data[0] == IPI && RxMessage.data[1] == 0xA7 && (RxMessage.data[3] == ID_Num || RxMessage.data[3] == 0xFF) && (RxMessage.data[5] & 0x01))
			{
				// aim to cause reboot
				tsk_lock();
			}
		}

		Loops_CAN_recv++;
		os_evt_set(EVT_FEED_DOG_CAN_RECV, tid_watchdog);
	}
}

void Pick_channels(void)
{
	u8 i = 0;
	u8 k = 0;

	Picked_lights_status_map = 0;
	
	if(ID_Num == 1)
		for(i = 0; i < 3; i++)
		{
			Picked_lights_status[k++] = Lights_status_1[i] & 0x01;
			Picked_lights_status[k++] = (Lights_status_1[i] & 0x02) >> 1;
			Picked_lights_status[k++] = (Lights_status_1[i] & 0x04) >> 2;
			Picked_lights_status[k++] = (Lights_status_1[i] & 0x08) >> 3;
			Picked_lights_status_map = (Picked_lights_status_map << 4) | (Lights_status_1[i] & 0x0f);
		}
	if(ID_Num == 2)
		for(i = 0; i < 3; i++)
		{
			Picked_lights_status[k++] = (Lights_status_1[i] & 0x10) >> 4;
			Picked_lights_status[k++] = (Lights_status_1[i] & 0x20) >> 5;
			Picked_lights_status[k++] = (Lights_status_1[i] & 0x40) >> 6;
			Picked_lights_status[k++] = (Lights_status_1[i] & 0x80) >> 7;
			Picked_lights_status_map = (Picked_lights_status_map << 4) | (Lights_status_1[i] >> 4);
		}
	if(ID_Num == 3)
		for(i = 0; i < 3; i++)
		{
			Picked_lights_status[k++] = Lights_status_2[i] & 0x01;
			Picked_lights_status[k++] = (Lights_status_2[i] & 0x02) >> 1;
			Picked_lights_status[k++] = (Lights_status_2[i] & 0x04) >> 2;
			Picked_lights_status[k++] = (Lights_status_2[i] & 0x08) >> 3;
			Picked_lights_status_map = (Picked_lights_status_map << 4) | (Lights_status_2[i] & 0x0f);
		}
	if(ID_Num == 4)
		for(i = 0; i < 3; i++)
		{
			Picked_lights_status[k++] = (Lights_status_2[i] & 0x10) >> 4;
			Picked_lights_status[k++] = (Lights_status_2[i] & 0x20) >> 5;
			Picked_lights_status[k++] = (Lights_status_2[i] & 0x40) >> 6;
			Picked_lights_status[k++] = (Lights_status_2[i] & 0x80) >> 7;
			Picked_lights_status_map = (Picked_lights_status_map << 4) | (Lights_status_2[i] >> 4);
		}
	
}

void Update_lights(void)
{
	//Channle1_Red
	GPIO_WriteBit(GPIOB, LIGHT_CH_1_R, (BitAction)Picked_lights_status[0]);

	//Channle1_Yellow
	GPIO_WriteBit(GPIOB, LIGHT_CH_1_Y, (BitAction)Picked_lights_status[4]);

	//Channle1_Green
	GPIO_WriteBit(GPIOB, LIGHT_CH_1_G, (BitAction)Picked_lights_status[8]);

	//Channle2_Red
	GPIO_WriteBit(GPIOB, LIGHT_CH_2_R, (BitAction)Picked_lights_status[1]);

	//Channle2_Yellow
	GPIO_WriteBit(GPIOB, LIGHT_CH_2_Y, (BitAction)Picked_lights_status[5]);

	//Channle2_Green
	GPIO_WriteBit(GPIOB, LIGHT_CH_2_G, (BitAction)Picked_lights_status[9]);

	//Channle3_Red
	GPIO_WriteBit(GPIOB, LIGHT_CH_3_R, (BitAction)Picked_lights_status[2]);

	//Channle3_Yellow
	GPIO_WriteBit(GPIOB, LIGHT_CH_3_Y, (BitAction)Picked_lights_status[6]);

	//Channle3_Green
	GPIO_WriteBit(GPIOB, LIGHT_CH_3_G, (BitAction)Picked_lights_status[10]);

	//Channle4_Red
	GPIO_WriteBit(GPIOB, LIGHT_CH_4_R, (BitAction)Picked_lights_status[3]);

	//Channle4_Yellow
	GPIO_WriteBit(GPIOB, LIGHT_CH_4_Y, (BitAction)Picked_lights_status[7]);

	//Channle4_Green
	GPIO_WriteBit(GPIOB, LIGHT_CH_4_G, (BitAction)Picked_lights_status[11]);
}

__task void task_lamp_ctl(void)
{	
	OS_RESULT result;
	CAN_msg *msg_error;
	
	while (1)
	{
		// wait for events to update lamps
		result = os_evt_wait_and (EVT_DATA_1_RCVD | EVT_DATA_2_RCVD | EVT_DATA_3_RCVD, 0x000A);
		if (result == OS_R_TMO) 
		{
			//printf("Event wait timeout.\n");
		}
		else
		{
			Pick_channels();
			Update_lights();
			if (!AC_power_exist)
			{
				// no AC power
				// send a heart beat
				os_evt_set(EVT_SEND_HEART_BEAT, tid_heart_beat);
				
				if(timer_count>=50)//��������10s�Ժ󣬲�����������Ϣ����==��Խ����綪ʧ
				{
// 					if (!(Last_error & 0x8000))
// 					{
// 						msg_error = _calloc_box (mpool);
// 						//						Line_num  ID_Num	  bad_light_num	  error_type
// 						//{ 1, {IPI, 0xD2, 0x00, 0x01,    0xFF,        0xFF,         0xFF,     0xFE}, 8, 2, STANDARD_FORMAT, DATA_FRAME};			
// 						msg_error->id = 1;
// 						msg_error->data[0] = IPI;
// 						msg_error->data[1] = 0xD2;
// 						msg_error->data[3] = 0x01;
// 						msg_error->data[4] = ID_Num;
// 						msg_error->data[7] = MSG_END;
// 						msg_error->len = sizeof(msg_error->data);
// 						msg_error->ch = 2;
// 						msg_error->format = STANDARD_FORMAT;
// 						msg_error->type = DATA_FRAME;
// 						
// 						msg_error->data[6] = ERROR_NO_AC_POWER;

// 						os_mbx_send (CAN_send_mailbox, msg_error, 0xffff);
// 						Last_error |= 0x8000;
// 					}
				}

			}
			else
			{
				os_evt_set(EVT_CONFLICT_MONITOR, tid_conflict_monitor);
			}
		}

		Loops_Lamp_ctl++;
		os_evt_set(EVT_FEED_DOG_LAMP_CTL, tid_watchdog);
	}
}

__task void task_green_led_flash(void)
{
	BitAction state = Bit_SET;
	u8 cnt = 0;
	
	os_itv_set (10);
	
	while (1)
	{
		os_itv_wait ();
		
		cnt++;

		if (Work_normal)
		{
			if (cnt == 15)
			{
				cnt = 0;
				state = (BitAction)(1 - state);
				GPIO_WriteBit(GPIOD, GPIO_Pin_1, state);
			}
		}
		else
		{
			GPIO_WriteBit(GPIOD, GPIO_Pin_1, Bit_SET);
		}

		Loops_Green_led_flash++;
		os_evt_set(EVT_FEED_DOG_GREEN_LED_FLASH, tid_watchdog);
	}
}

__task void task_red_led_flash(void)
{
	BitAction state = Bit_SET;
	u8 cnt = 0;
	
	os_itv_set (10);
	
	while (1)
	{
		os_itv_wait ();

		cnt++;

		//GPIO_PinReverse(GPIOD, GPIO_Pin_4);//hard_watchdog
		if(timer_count<100) timer_count++;//������ʱ10s
		
		if (!Work_normal)
		{
			if (cnt == 2)
			{
				cnt = 0;
				state = (BitAction)(1 - state);
				GPIO_WriteBit(GPIOD, GPIO_Pin_2, state);
			}
		}
		else
		{
			GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_SET);
		}

		Loops_Red_led_flash++;
		os_evt_set(EVT_FEED_DOG_RED_LED_FLASH, tid_watchdog);
	}
}


u16 SWITs_status_map;
u16 xx;

U16 conflict_count[12U] = {0U};
U8  conflict_flag = 0U;
U16  cal_conflict_map = 0U;

__IO int16_t  ADC_threshold[12] = {20,20,20,20,20,20,20,20,20,20,20,20};
__IO int16_t  ADC_low_threshold[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
__task void task_conflict_monitor(void)
{
	OS_RESULT result;
	int i, j;
	uint16_t ADC_samples[36];
	int16_t temp;
	CAN_msg *msg_recovered;
	u8 Conflict_buffer = 0;
	u16 temp_status=0;

	
	/* Initialize message  = { ID, {data[0] .. data[7]}, LEN, CHANNEL, FORMAT, TYPE } */
	//CAN_msg msg_recovered = { 1, {IPI, 0xD2, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE}, 8, 2, STANDARD_FORMAT, DATA_FRAME};

	while (1)
	{
		// wait for event to start a cycle of conflict detection
		result = os_evt_wait_and (EVT_CONFLICT_MONITOR, 0x000A);
		if (result == OS_R_TMO) 
		{
			//printf("Event wait timeout.\n");
		}
		else
		{
			// wait for the current to be stable
			os_dly_wait(10); //100ms
			
			// sample ADC reading 3 times
			for (i=0; i<3; i++)
			{
				DMAReConfig();
				result = os_evt_wait_and (EVT_ADC_DONE, 0xffff);
				if (result == OS_R_TMO)
				{
					i--;
				}
				else
				{
					for (j=0; j<12; j++)
					{
						ADC_samples[12*i+j] = ADC_buffer[j];
					}
					os_dly_wait(2); //20ms
				}
			}
			// samples done

			// update ADC_low_threshold[]
			
			temp_status = Picked_lights_status_map;
			for (i=0; i<12; i++)
			{
				if (!(temp_status & 0x800)) // light is in off state
				{
					ADC_low_threshold[i] = ((ADC_low_threshold[i] + ADC_samples[i] + ADC_samples[12 + i] + ADC_samples[24 + i]) >> 2);
				}
				temp_status <<= 1;
			}

			// get the status map
			ADC_status_map = 0;
			for (i=0; i<12; i++)
			{
				temp = (ADC_samples[i] + ADC_samples[12 + i] + ADC_samples[24 + i]) / 3;
				
				if (3*abs(temp - ADC_low_threshold[i]) < 2*abs(ADC_threshold[i] - temp))
				{
					ADC_status_map <<= 1;
				}
				else
				{
					ADC_threshold[i] = (ADC_threshold[i]+temp)/2;
					
					ADC_status_map = (ADC_status_map << 1) | 0x0001;
					
					os_mut_wait (mutex_ADC_minute, 0xffff);
					// critical region start
						ADC_minute_sum[i] += temp;
						ADC_minute_count[i]++;
					// critical region end
					os_mut_release (mutex_ADC_minute);
				}
			}
			
			// get SWITs status map
			SWITs_Enable(1);
			os_dly_wait(6);
			SWITs_Enable(0);
			
			SWITs_status_map = 0;
			for (i=0; i<12; i++)
			{
				if (test_n[i] < SWIT_INT_THRESHOLD)
				{
					SWITs_status_map = (SWITs_status_map << 1) | 0x0001;
				}
				else
				{
					SWITs_status_map <<= 1;
				}
//				test_n[i] = 0;
			}
			for (i=0; i<12; i++)
			{
				test_n[i] = 0;
			}
			ADC_status_map &= ~((u16)Walker_channels << 4);
			ADC_status_map &= (((u16)Channels_enabled << 8) | ((u16)Channels_enabled << 4) | (u16)Channels_enabled);
			SWITs_status_map &= ~((u16)Walker_channels << 4);
			SWITs_status_map &= (((u16)Channels_enabled << 8) | ((u16)Channels_enabled << 4) | (u16)Channels_enabled);
			
			if(ADC_status_map != SWITs_status_map)
			{
				conflict_map = (ADC_status_map^SWITs_status_map);
			}
			else
			{
				// calculate the conflict map 
				conflict_map = Picked_lights_status_map ^ ADC_status_map;
				// consider the Vehicle_channels and Walker_channels
				conflict_map &= ~((u16)Walker_channels << 4);
				conflict_map &= (((u16)Channels_enabled << 8) | ((u16)Channels_enabled << 4) | (u16)Channels_enabled);
			}
			
			
			// use Error_buffer to hold on initial errors until the maximum count is reached
//			if (conflict_map)
//			{
//				if (Conflict_buffer < CONFLICT_BUFFER_MAX)
//				{
//					Conflict_buffer++;
//				}
//			}
//			else if (Conflict_buffer > 0)
//			{
//				Conflict_buffer=0;
//			}
			
			cal_conflict_map = 0;
			for(i=0;i<12;i++)
			{
				if(((conflict_map>>i)&0x01) == 1)
				{
					conflict_count[i]++;
				}
				else
				{
					conflict_count[i] = 0;
				}
				
				if(conflict_count[i] == CONFLICT_BUFFER_MAX)
				{
					conflict_flag = 1;
					cal_conflict_map |= (U16)((U16)1<<i);
				}
			}
			
			// conflict happens
			if (conflict_flag == 1)
			{		
				conflict_flag = 0;
				
				for(i=0;i<12;i++)
				{
					conflict_count[i] = 0;
				}
// 				os_evt_set(EVT_SEND_HEART_BEAT, tid_heart_beat);
				// handle conflict
				os_evt_set(EVT_CONFLICT_ANALYSIS, tid_conflict_analysis);
			}
			else
			{
				// if it recovers from a conflict
// 				if(!Work_normal)
// 				{
// 					msg_recovered = _calloc_box (mpool);
// 					//{ 1, {IPI, 0xD2, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE}, 8, 2, STANDARD_FORMAT, DATA_FRAME};
// 					msg_recovered->id = 1;
// 					msg_recovered->data[0] = IPI;
// 					msg_recovered->data[1] = 0xD2;
// 					msg_recovered->data[7] = MSG_END;
// 					msg_recovered->len = sizeof(msg_recovered->data);
// 					msg_recovered->ch = 2;
// 					msg_recovered->format = STANDARD_FORMAT;
// 					msg_recovered->type = DATA_FRAME;
// 					
// 					os_mbx_send(CAN_send_mailbox, msg_recovered, 0xffff);
// 					Work_normal = 1;
// 					Last_error = 0;
// 				}
				
				os_evt_set(EVT_SEND_HEART_BEAT, tid_heart_beat);

			}
		}

		Loops_Conflict_monitor++;
		os_evt_set(EVT_FEED_DOG_CONFLICT_MONITOR, tid_watchdog);
	}
}

U8  err_cnt[12] = {0};
__task void task_conflict_analysis(void)
{
	OS_RESULT result;
// 	u16 SWITs_status_map;
	CAN_msg *msg_error, *msg_error_red, *msg_error_green, *msg_error_yellow;
	int i;

	U8 red_err_cnt = 0;
	U8 err_type = 0;
	U16 err_red_data2 = 0;
	U16 err_green_data2 = 0;
	U16 err_yellow_data2 = 0;

	//										   Line_num	 ID_Num	  bad_light_num	  error_type
	//CAN_msg msg_error = { 1, {IPI, 0xD2, 0x00, 0x01,    0xFF,        0xFF,         0xFF,     0xFE}, 8, 2, STANDARD_FORMAT, DATA_FRAME};

	//msg_error.data[4] = ID_Num;

	while (1)
	{
		// wait for event to start a conflict analysis
		result = os_evt_wait_and (EVT_CONFLICT_ANALYSIS, 0x000A);
		if (result == OS_R_TMO) 
		{
			//printf("Event wait timeout.\n");
		}
//////		else if(conflict_map != Last_error)
//////		{
//////			// check each conflict bit
//////			for (i=0; i<12; i++)
//////			{
//////				if ((conflict_map >> i) & 0x0001)
//////				{
//////					msg_error = _calloc_box (mpool);
//////					//						Line_num	 ID_Num	  bad_light_num	  error_type
//////					//{ 1, {IPI, 0xD2, 0x00, 0x01,    0xFF,        0xFF,         0xFF,     0xFE}, 8, 2, STANDARD_FORMAT, DATA_FRAME};			
//////					msg_error->id = ID_Num;
//////					msg_error->data[0] = IPI;
//////					msg_error->data[1] = 0xD2;
//////					msg_error->data[3] = 0x01;
//////					msg_error->data[4] = ID_Num;
//////					msg_error->data[7] = MSG_END;
//////					msg_error->len = sizeof(msg_error->data);
//////					msg_error->ch = 2;
//////					msg_error->format = STANDARD_FORMAT;
//////					msg_error->type = DATA_FRAME;
//////					
//////					// specify bad_light_num and error_type
//////					msg_error->data[5] = ((3-i/4)+3*(i%4));
//////					
//////					if (((Picked_lights_status_map^ADC_status_map) >> i) & 0x0001)
//////					{
//////						// SWIT short cut error
//////						msg_error->data[6] = ERROR_SWIT_CLOSE;
//////					}
//////					else if (((Picked_lights_status_map ^SWITs_status_map) >> i) & 0x0001)
//////					{
//////						msg_error->data[6] = ERROR_LIGHT;
//////					}
//////					else
//////					{
//////						msg_error->data[6] = ERROR_SWIT_OPEN;
//////					}
//////					
//////					/* �̵ƻƵ����ι��ϼ�� */
//////					if(conflict_map&GY_ERR_MASK)
//////						Last_error |= 0x4000;
//////					os_mbx_send (CAN_send_mailbox, msg_error, 0xffff);
//////					
//////				}
//////			}
//////			Last_error = (Last_error & 0x4000) | conflict_map;
//////		}
		else if(cal_conflict_map != (Last_error&0xfff))
		{
			err_red_data2 = 0;
			err_green_data2 = 0;
			err_yellow_data2 = 0;

			// check each conflict bit
			for (i=0; i<12; i++)
			{
				if ((cal_conflict_map >> i) & 0x0001)
				{
					err_cnt[i]++;
					
					if((i/4) == 2)	//���
					{
						if(((Picked_lights_status_map >> i) & 0x0001) == 1)	//can�·����Ϊ��
						{
							if(((ADC_status_map >> i) & 0x0001) == 0)					//ad������Ϊ0
							{
								if(((SWITs_status_map >> i) & 0x0001) == 0)			//�ɿع迪·
								{
									err_type = ERROR_SWIT_OPEN;
								}
								else																						//��ƹ���
								{
									err_type = ERROR_LIGHT;
									err_red_data2 |= 1 << (2*(i%4));
								}
								Last_error |= 0x4000;													//����
							}
						}
						else																								//��
						{
							if(((ADC_status_map >> i) & 0x0001) == 1)					//ad������Ϊ1
							{
								if(((SWITs_status_map >> i) & 0x0001) == 1)			//�ɿع��·
								{
									err_type = ERROR_SWIT_CLOSE;
								}
							}
							else																						//���
							{
								err_type = RED_CONFLICT;
								err_red_data2 |= 1 << (1 + 2*(i%4));
							}
						}
						
					}
					else if((i/4) == 0)	//�̵�
					{
						if(((Picked_lights_status_map >> i) & 0x0001) == 1)	//can�·��̵�Ϊ��
						{
							if(((ADC_status_map >> i) & 0x0001) == 0)					//ad������Ϊ0
							{
								if(((SWITs_status_map >> i) & 0x0001) == 0)			//�ɿع迪·
								{
									err_type = ERROR_SWIT_OPEN;
								}
								else																						//�̵ƹ���
								{
									err_type = ERROR_LIGHT;
									err_green_data2 |= 1 << (2*(i%4));
								}
							}
						}
						else																								//��
						{
							if(((ADC_status_map >> i) & 0x0001) == 1)					//ad������Ϊ1
							{
								if(((SWITs_status_map >> i) & 0x0001) == 1)			//�ɿع��·
								{
									err_type = ERROR_SWIT_CLOSE;
								}
								
							}
							else																						//�̳��ж�
							{
								//�̳�
								Last_error |= 0x4000;												//����
								err_type = GREEN_CONFLICT;
								err_green_data2 |= 1 << (1 + 2*(i%4));
							}
						}
						
					}
					else if((i/4) == 1)	//�Ƶ�
					{
						if(((Picked_lights_status_map >> i) & 0x0001) == 1)	//can�·��Ƶ�Ϊ��
						{
							if(((ADC_status_map >> i) & 0x0001) == 0)					//ad������Ϊ0
							{
								if(((SWITs_status_map >> i) & 0x0001) == 0)			//�ɿع迪·
								{
									err_type = ERROR_SWIT_OPEN;
								}
								else																						//�Ƶƹ���
								{
									err_type = ERROR_LIGHT;
									err_yellow_data2 |= 1 << (2*(i%4));
								}
							}
						}
						else																								//��
						{
							if(((ADC_status_map >> i) & 0x0001) == 1)					//ad������Ϊ1
							{
								if(((SWITs_status_map >> i) & 0x0001) == 1)			//�ɿع��·
								{
									err_type = ERROR_SWIT_CLOSE;
								}
							}
							else																						//�Ƴ��ж�
							{
								err_type = YELLOW_CONFLICT;
								err_yellow_data2 |= 1 << (1 + 2*(i%4));
							}
						}
						
					}
					
					if((err_type!=0)&&(err_cnt[i] >= 2))
					{
						err_cnt[i] = 0;
						
						Work_normal = 0;
						msg_error = _calloc_box (mpool);
						//						Line_num	 ID_Num	  bad_light_num	  error_type
						//{ 1, {IPI, 0xD2, 0x00, 0x01,    0xFF,        0xFF,         0xFF,     0xFE}, 8, 2, STANDARD_FORMAT, DATA_FRAME};			
						msg_error->id = ID_Num;
						msg_error->data[0] = IPI;
						msg_error->data[1] = 0xD2;
						msg_error->data[3] = 0x01;
						msg_error->data[4] = ID_Num;
						msg_error->data[7] = MSG_END;
						msg_error->len = sizeof(msg_error->data);
						msg_error->ch = 2;
						msg_error->format = STANDARD_FORMAT;
						msg_error->type = DATA_FRAME;
						
						// specify bad_light_num and error_type
						msg_error->data[5] = ((3-i/4)+3*(i%4));
					
						msg_error->data[6] = err_type;
						err_type =0;
						
						Last_error = (Last_error & 0x4000) | conflict_map;
						os_mbx_send (CAN_send_mailbox, msg_error, 0xffff);
					}				
				}
			}

			if (err_red_data2)
			{
				msg_error_red = _calloc_box (mpool);

				msg_error_red->id = ID_Num;
				msg_error_red->data[0] = IPI;
				msg_error_red->data[1] = 0xE1;
				msg_error_red->data[3] = ID_Num;
				err_red_data2 |= ((err_red_data2 | 0x55) != 0) << 14;
				err_red_data2 |= (AC_power_exist == 1) << 15;
				msg_error_red->data[4] = (U8)(err_red_data2 >> 8);
				msg_error_red->data[5] = (U8)err_red_data2;
				msg_error_red->data[7] = MSG_END;
				msg_error_red->len = sizeof(msg_error_red->data);
				msg_error_red->ch = 2;
				msg_error_red->format = STANDARD_FORMAT;
				msg_error_red->type = DATA_FRAME;
				os_mbx_send (CAN_send_mailbox, msg_error_red, 0xffff);
			}

			if (err_green_data2)
			{
				msg_error_green = _calloc_box (mpool);

				msg_error_green->id = ID_Num;
				msg_error_green->data[0] = IPI;
				msg_error_green->data[1] = 0xE3;
				msg_error_green->data[3] = ID_Num;
				err_green_data2 |= ((err_green_data2 | 0xAA) != 0) << 14;
				err_green_data2 |= (AC_power_exist == 1) << 15;
				msg_error_green->data[4] = (U8)(err_green_data2 >> 8);
				msg_error_green->data[5] = (U8)err_green_data2;
				msg_error_green->data[7] = MSG_END;
				msg_error_green->len = sizeof(msg_error_green->data);
				msg_error_green->ch = 2;
				msg_error_green->format = STANDARD_FORMAT;
				msg_error_green->type = DATA_FRAME;
				os_mbx_send (CAN_send_mailbox, msg_error_green, 0xffff);
			}

			if (err_yellow_data2)
			{
				msg_error_yellow = _calloc_box (mpool);

				msg_error_yellow->id = ID_Num;
				msg_error_yellow->data[0] = IPI;
				msg_error_yellow->data[1] = 0xE2;
				msg_error_yellow->data[3] = ID_Num;
				err_yellow_data2 |= (AC_power_exist == 1) << 15;
				msg_error_yellow->data[4] = (U8)(err_yellow_data2 >> 8);
				msg_error_yellow->data[5] = (U8)err_yellow_data2;
				msg_error_yellow->data[7] = MSG_END;
				msg_error_yellow->len = sizeof(msg_error_yellow->data);
				msg_error_yellow->ch = 2;
				msg_error_yellow->format = STANDARD_FORMAT;
				msg_error_yellow->type = DATA_FRAME;
				os_mbx_send (CAN_send_mailbox, msg_error_yellow, 0xffff);
			}

		}
		os_evt_set(EVT_SEND_HEART_BEAT, tid_heart_beat);
		Loops_Conflict_analysis++;
		os_evt_set(EVT_FEED_DOG_CONFLICT_ANALYSIS, tid_watchdog);
	}
}

__task void task_heart_beat(void)
{
	OS_RESULT result;
	CAN_msg *msg_heart_beat;
	/* Initialize message  = { ID, {data[0] .. data[7]}, LEN, CHANNEL, FORMAT, TYPE } */
	//CAN_msg msg_heart_beat = { 1, {IPI, 0x83, 0x01, 0xFF, 0x02, 0x00, 0x00, 0x00}, 8, 2, STANDARD_FORMAT, DATA_FRAME};
	
	

	while (1)
	{
		// wait for event to send a heart beat
		result = os_evt_wait_and (EVT_SEND_HEART_BEAT, 0x000A);
////		os_dly_wait (25);
		if (result == OS_R_TMO) 
		{
			//printf("Event wait timeout.\n");
		}
		else
		{
			msg_heart_beat = _calloc_box (mpool);
			//{ 1, {IPI, 0x83, 0x01, 0xFF, 0x02, 0x00, 0x00, 0x00}, 8, 2, STANDARD_FORMAT, DATA_FRAME};
			msg_heart_beat->id = ID_Num;
			msg_heart_beat->data[0] = IPI;
			msg_heart_beat->data[1] = 0x83;
			msg_heart_beat->data[2] = 0x01;
			msg_heart_beat->data[3] = ID_Num;
			msg_heart_beat->data[4] = 0x02;

			if ((Last_error & 0x4000))
			{
				msg_heart_beat->data[5] = ERROR_LIGHT;
			}
			
			msg_heart_beat->len = 6;
			msg_heart_beat->ch = 2;
			msg_heart_beat->format = STANDARD_FORMAT;
			msg_heart_beat->type = DATA_FRAME;
			os_dly_wait(ID_Num);
			os_mbx_send (CAN_send_mailbox, msg_heart_beat, 0xffff);
		}

		Loops_Heart_beat++;
		os_evt_set(EVT_FEED_DOG_HEART_BEAT, tid_watchdog);
	}
}

__task void task_current_report(void)
{
	CAN_msg *msg_current_report_1;
	CAN_msg *msg_current_report_2;
	CAN_msg *msg_current_report_3;
	CAN_msg *msg_current_report_4;
	CAN_msg *msg_current_report_5;
	
	int i;
	u16 ADC_minute_average[12] = {0};
	u16 cnt = 0;
	
	os_itv_set (10); // report once per minute
	
	while (1)
	{
		os_itv_wait();

		cnt++;

		if (cnt == 600)
		{
			cnt = 0;
			for (i=0; i<12; i++)
			{
				if (ADC_minute_count[i])
				{
					os_mut_wait (mutex_ADC_minute, 0xffff);
					// critical region start
						ADC_minute_average[i] = ADC_minute_sum[i]/ADC_minute_count[i];
						ADC_minute_sum[i] = 0;
						ADC_minute_count[i] = 0;
					// critical region end
					os_mut_release (mutex_ADC_minute);
				}
			}

			msg_current_report_1 = _calloc_box (mpool);
			msg_current_report_2 = _calloc_box (mpool);
			msg_current_report_3 = _calloc_box (mpool);
			msg_current_report_4 = _calloc_box (mpool);
			msg_current_report_5 = _calloc_box (mpool);

			//msg_current_report_x { id,{	data[0], 	   data[1] data[2] data[3] data[4] data[5] data[6] 	   data[7]		}, len, ch, format, 		  type};
			//msg_current_report_1 { 1, {	IPI 			0xCC 	0x00 	0x01 	P1Rl 	P1Rh 	P1Yl 	MSG_CONTINUE	}, 8, 	2, STANDARD_FORMAT, DATA_FRAME};
			//msg_current_report_2 { 1, {	MSG_CONTINUE 	P1Yh 	P1Gl 	P1Gh 	0x02 	P2Rl 	P2Rh 	MSG_CONTINUE	}, 8, 	2, STANDARD_FORMAT, DATA_FRAME};
			//msg_current_report_3 { 1, {	MSG_CONTINUE 	P2Yl 	P2Yh 	P2Gl 	P2Gh 	0x03 	P3Rl 	MSG_CONTINUE	}, 8, 	2, STANDARD_FORMAT, DATA_FRAME};
			//msg_current_report_4 { 1, {	MSG_CONTINUE 	P3Rh 	P3Yl 	P3Yh 	P3Gl 	P3Gh 	P4Rl 	MSG_CONTINUE	}, 8, 	2, STANDARD_FORMAT, DATA_FRAME};
			//msg_current_report_5 { 1, {	MSG_CONTINUE 	0x04 	P4Rh 	P4Yl 	P4Yh 	P4Gl 	P4Gh 	MSG_END			}, 8, 	2, STANDARD_FORMAT, DATA_FRAME};
			msg_current_report_1->id = msg_current_report_2->id = msg_current_report_3->id = msg_current_report_4->id = msg_current_report_5->id = 1;
			msg_current_report_1->len = msg_current_report_2->len = msg_current_report_3->len = msg_current_report_4->len = msg_current_report_5->len = 8;
			msg_current_report_1->ch = msg_current_report_2->ch = msg_current_report_3->ch = msg_current_report_4->ch = msg_current_report_5->ch = 2;
			msg_current_report_1->format = msg_current_report_2->format = msg_current_report_3->format = msg_current_report_4->format = msg_current_report_5->format = STANDARD_FORMAT;
			msg_current_report_1->type = msg_current_report_2->type = msg_current_report_3->type = msg_current_report_4->type = msg_current_report_5->type = DATA_FRAME;

			msg_current_report_1->data[0] = IPI;
			msg_current_report_2->data[0] = msg_current_report_3->data[0] = msg_current_report_4->data[0] = msg_current_report_5->data[0] = MSG_CONTINUE;

			msg_current_report_1->data[7] = msg_current_report_2->data[7] = msg_current_report_3->data[7] = msg_current_report_4->data[7] = MSG_CONTINUE;
			msg_current_report_5->data[7] = MSG_END;

			msg_current_report_1->data[1] = 0xCC;
			msg_current_report_1->data[2] = 0x00;
			msg_current_report_1->data[3] = 0x01;
			msg_current_report_2->data[4] = 0x02;
			msg_current_report_3->data[5] = 0x03;
			msg_current_report_5->data[1] = 0x04;

			// ADC_minute_average[0]    1	2	3	4	5	6	7	8	9	10	11
			//			4R				3R	2R	1R	4Y	3Y	2Y	1Y	4G	3G	2G	1G
			msg_current_report_4->data[6] = ADC_minute_average[0] & 0x00FF; //P4Rl
			msg_current_report_5->data[2] = ADC_minute_average[0] >> 8; 	//P4Rh
			msg_current_report_3->data[6] = ADC_minute_average[1] & 0x00FF; //P3Rl
			msg_current_report_4->data[1] = ADC_minute_average[1] >> 8; 	//P3Rh
			msg_current_report_2->data[5] = ADC_minute_average[2] & 0x00FF; //P2Rl
			msg_current_report_2->data[6] = ADC_minute_average[2] >> 8; 	//P2Rh
			msg_current_report_1->data[4] = ADC_minute_average[3] & 0x00FF; //P1Rl
			msg_current_report_1->data[5] = ADC_minute_average[3] >> 8; 	//P1Rh
			msg_current_report_5->data[3] = ADC_minute_average[4] & 0x00FF; //P4Yl
			msg_current_report_5->data[4] = ADC_minute_average[4] >> 8; 	//P4Yh
			msg_current_report_4->data[2] = ADC_minute_average[5] & 0x00FF; //P3Yl
			msg_current_report_4->data[3] = ADC_minute_average[5] >> 8; 	//P3Yh
			msg_current_report_3->data[1] = ADC_minute_average[6] & 0x00FF; //P2Yl
			msg_current_report_3->data[2] = ADC_minute_average[6] >> 8; 	//P2Yh
			msg_current_report_1->data[6] = ADC_minute_average[7] & 0x00FF; //P1Yl
			msg_current_report_2->data[1] = ADC_minute_average[7] >> 8; 	//P1Yh
			msg_current_report_5->data[5] = ADC_minute_average[8] & 0x00FF; //P4Gl
			msg_current_report_5->data[6] = ADC_minute_average[8] >> 8; 	//P4Gh
			msg_current_report_4->data[4] = ADC_minute_average[9] & 0x00FF; //P3Gl
			msg_current_report_4->data[5] = ADC_minute_average[9] >> 8; 	//P3Gh
			msg_current_report_3->data[3] = ADC_minute_average[10] & 0x00FF;//P2Gl
			msg_current_report_3->data[4] = ADC_minute_average[10] >> 8; 	//P2Gh
			msg_current_report_2->data[2] = ADC_minute_average[11] & 0x00FF;//P1Gl
			msg_current_report_2->data[3] = ADC_minute_average[11] >> 8; 	//P1Gh

			os_mbx_send (CAN_send_mailbox, msg_current_report_1, 0xffff);
			os_mbx_send (CAN_send_mailbox, msg_current_report_2, 0xffff);
			os_mbx_send (CAN_send_mailbox, msg_current_report_3, 0xffff);
			os_mbx_send (CAN_send_mailbox, msg_current_report_4, 0xffff);
			os_mbx_send (CAN_send_mailbox, msg_current_report_5, 0xffff);
		}

		Loops_Current_report++;
		os_evt_set(EVT_FEED_DOG_CURRENT_REPORT, tid_watchdog);
	}
}

__task void task_AC_detector(void)
{
	os_itv_set (10); // to detect every 100ms

	while (1)
	{
		os_itv_wait();

		// start detection
		AC_Detect_Enable(1);
		os_dly_wait(6);
		AC_Detect_Enable(0);

		if (test_n[15] < AC_DETECT_INT_THRESHOLD)
		{
			// no AC power detected
			AC_power_exist = 0;
		}
		else
		{
			AC_power_exist = 1;
		}

		test_n[15] = 0;

		Loops_AC_detector++;
		os_evt_set(EVT_FEED_DOG_AC_DETECTOR, tid_watchdog);
	}
}

void feed_dog(void)
{
	GPIO_WriteBit(GPIOD, GPIO_Pin_4, Bit_RESET);
	GPIO_PinReverse(GPIOD, GPIO_Pin_4);
	os_dly_wait(1);
	GPIO_PinReverse(GPIOD, GPIO_Pin_4);
}

__task void task_watchdog(void)
{
	OS_RESULT result;
	u8 errcnt_CAN_send = 0;
	u8 errcnt_CAN_recv = 0;
	u8 errcnt_Lamp_ctl = 0;
	u8 errcnt_Green_led_flash = 0;
	u8 errcnt_Red_led_flash = 0;
	u8 errcnt_Conflict_monitor = 0;
	u8 errcnt_Conflict_analysis = 0;
	u8 errcnt_Heart_beat = 0;
	u8 errcnt_Current_report = 0;
	u8 errcnt_AC_detector = 0;

	while (1)
	{
		// wait for event to feed the dog
		result = os_evt_wait_and (	EVT_FEED_DOG_CAN_SEND |
									EVT_FEED_DOG_CAN_RECV |
									EVT_FEED_DOG_LAMP_CTL |
									EVT_FEED_DOG_GREEN_LED_FLASH |
									EVT_FEED_DOG_RED_LED_FLASH |
									EVT_FEED_DOG_CONFLICT_MONITOR |
									EVT_FEED_DOG_CONFLICT_ANALYSIS |
									EVT_FEED_DOG_HEART_BEAT |
									//EVT_FEED_DOG_CURRENT_REPORT |
									EVT_FEED_DOG_AC_DETECTOR, 50);

		if (result == OS_R_TMO)
		{
			//at lease one task has something wrong, check it out
			if (Loops_CAN_send == 0)
			{
				errcnt_CAN_send++;
				if (errcnt_CAN_send >= 20)
				{
					// task can not be recovered. Reboot the board
					tsk_lock();
				}
				else if (errcnt_CAN_send >= 5)
				{
					// task abnormal, restart it
					os_tsk_delete(tid_send_CAN);
					tid_send_CAN = os_tsk_create(task_send_CAN, 10);
					feed_dog();
				}
			}

			if (Loops_CAN_recv == 0)
			{
				errcnt_CAN_recv++;
				if (errcnt_CAN_recv >= 20)
				{
					// task can not be recovered. Reboot the board
					tsk_lock();
				}
				else if (errcnt_CAN_recv >= 5)
				{
					// task abnormal, restart it
					os_tsk_delete(tid_recv_CAN);
					tid_recv_CAN = os_tsk_create(task_recv_CAN, 2);
					feed_dog();
				}
			}

			if (Loops_Lamp_ctl == 0)
			{
				errcnt_Lamp_ctl++;
				if (errcnt_Lamp_ctl >= 20)
				{
					// task can not be recovered. Reboot the board
					tsk_lock();
				}
				else if (errcnt_Lamp_ctl >= 5)
				{
					// task abnormal, restart it
					os_tsk_delete(tid_lamp_ctl);
					tid_lamp_ctl = os_tsk_create(task_lamp_ctl, 2);
					feed_dog();
				}
			}

			if (Loops_Green_led_flash == 0)
			{
				errcnt_Green_led_flash++;
				if (errcnt_Green_led_flash >= 20)
				{
					// task can not be recovered. Reboot the board
					tsk_lock();
				}
				else if (errcnt_Green_led_flash >= 5)
				{
					// task abnormal, restart it
					os_tsk_delete(tid_green_led_flash);
					tid_green_led_flash = os_tsk_create(task_green_led_flash, 2);
					feed_dog();
				}
			}

			if (Loops_Red_led_flash == 0)
			{
				errcnt_Red_led_flash++;
				if (errcnt_Red_led_flash >= 20)
				{
					// task can not be recovered. Reboot the board
					tsk_lock();
				}
				else if (errcnt_Red_led_flash >= 5)
				{
					// task abnormal, restart it
					os_tsk_delete(tid_red_led_flash);
					tid_red_led_flash = os_tsk_create(task_red_led_flash, 2);
					feed_dog();
				}
			}

			if (Loops_Conflict_monitor == 0)
			{
				errcnt_Conflict_monitor++;
				if (errcnt_Conflict_monitor >= 20)
				{
					// task can not be recovered. Reboot the board
					tsk_lock();
				}
				else if (errcnt_Conflict_monitor >= 5)
				{
					// task abnormal, restart it
					os_tsk_delete(tid_conflict_monitor);
					tid_conflict_monitor = os_tsk_create(task_conflict_monitor, 2);
					feed_dog();
				}
			}

			if (Loops_Conflict_analysis == 0)
			{
				errcnt_Conflict_analysis++;
				if (errcnt_Conflict_analysis >= 20)
				{
					// task can not be recovered. Reboot the board
					tsk_lock();
				}
				else if (errcnt_Conflict_analysis >= 5)
				{
					// task abnormal, restart it
					os_tsk_delete(tid_conflict_analysis);
					tid_conflict_analysis = os_tsk_create(task_conflict_analysis, 2);
					feed_dog();
				}
			}

			if (Loops_Heart_beat == 0)
			{
				errcnt_Heart_beat++;
				if (errcnt_Heart_beat >= 20)
				{
					// task can not be recovered. Reboot the board
					tsk_lock();
				}
				else if (errcnt_Heart_beat >= 5)
				{
					// task abnormal, restart it
					os_tsk_delete(tid_heart_beat);
					tid_heart_beat = os_tsk_create(task_heart_beat, 2);
					feed_dog();
				}
			}
/*
			if (Loops_Current_report == 0)
			{
				errcnt_Current_report++;
				if (errcnt_Current_report >= 20)
				{
					// task can not be recovered. Reboot the board
					tsk_lock();
				}
				else if (errcnt_Current_report >= 5)
				{
					// task abnormal, restart it
					os_tsk_delete(tid_current_report);
					tid_current_report = os_tsk_create(task_current_report, 2);
					feed_dog();
				}
			}
*/
			if (Loops_AC_detector == 0)
			{
				errcnt_AC_detector++;
				if (errcnt_AC_detector >= 20)
				{
					// task can not be recovered. Reboot the board
					tsk_lock();
				}
				else if (errcnt_AC_detector >= 5)
				{
					// task abnormal, restart it
					os_tsk_delete(tid_AC_detector);
					tid_AC_detector = os_tsk_create(task_AC_detector, 5);
					feed_dog();
				}
			}

		}
		else
		{
			// all tasks seem good. feed the dog
			feed_dog();

			Loops_CAN_send = 0;
			Loops_CAN_recv = 0;
			Loops_Lamp_ctl = 0;
			Loops_Green_led_flash = 0;
			Loops_Red_led_flash = 0;
			Loops_Conflict_monitor = 0;
			Loops_Conflict_analysis = 0;
			Loops_Heart_beat = 0;
			Loops_Current_report = 0;
			Loops_AC_detector = 0;

			errcnt_CAN_send = 0;
			errcnt_CAN_recv = 0;
			errcnt_Lamp_ctl = 0;
			errcnt_Green_led_flash = 0;
			errcnt_Red_led_flash = 0;
			errcnt_Conflict_monitor = 0;
			errcnt_Conflict_analysis = 0;
			errcnt_Heart_beat = 0;
			errcnt_Current_report = 0;
			errcnt_AC_detector = 0;
		}

	}
}
