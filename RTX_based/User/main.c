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

int main (void)
{
	os_sys_init(init);
}

__task void init (void) 
{
	Device_Init();
	
	os_mbx_init (CAN_send_mailbox, sizeof(CAN_send_mailbox));
	_init_box (mpool, sizeof(mpool), sizeof(CAN_msg));
	os_mut_init(mutex_ADC_minute);

	// init lights all red
	GPIO_WriteBit(GPIOB, GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14, Bit_RESET);
	GPIO_WriteBit(GPIOB, GPIO_Pin_3|GPIO_Pin_6|GPIO_Pin_9|GPIO_Pin_12, Bit_SET);
	// turn off 2 led indicators
	GPIO_WriteBit(GPIOD, GPIO_Pin_1|GPIO_Pin_2, Bit_SET);
	
	tid_lamp_ctl = os_tsk_create(task_lamp_ctl, 2);
	tid_green_led_flash = os_tsk_create(task_green_led_flash, 2);
	tid_red_led_flash = os_tsk_create(task_red_led_flash, 2);
	tid_conflict_monitor = os_tsk_create(task_conflict_monitor, 2);
	tid_conflict_analysis = os_tsk_create(task_conflict_analysis, 2);
	tid_heart_beat = os_tsk_create(task_heart_beat, 2);
	tid_current_report = os_tsk_create(task_current_report, 2);
	tid_AC_detector = os_tsk_create(task_AC_detector, 2);
	tid_send_CAN = os_tsk_create(task_send_CAN, 2);
	tid_recv_CAN = os_tsk_create(task_recv_CAN, 2);
	
	os_tsk_delete_self();
}

__task void task_send_CAN(void) 
{
	void *msg;

	CAN_init(1, 1000000);               /* CAN controller 1 init, 1000 kbit/s   */
	CAN_rx_object (1, 2,  33, DATA_TYPE | STANDARD_TYPE); /* Enable reception */
                                       /* of message on controller 1, channel */
                                       /* is not used for STM32 (can be set to*/
                                       /* whatever value), data frame with    */
                                       /* standard id 33                      */
	CAN_start (1);                     /* Start controller 1                  */

	while (1)
	{
		os_mbx_wait (CAN_send_mailbox, &msg, 0xffff);
		
		CAN_send (1, msg, 0x0F00);  /* Send msg_send on controller 1       */
		_free_box (mpool, msg);
	}
}
__task void task_recv_CAN(void)
{	
	CAN_msg RxMessage;
	u8 i;

	for (;;)
	{
		if (CAN_receive (1, &RxMessage, 0x00FF) == CAN_OK)
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
						os_evt_clr (EVT_DATA_2_RCVD, tid_lamp_ctl);
						os_evt_set (EVT_DATA_1_RCVD, tid_lamp_ctl);
						break;
						
					case 0x02:
						for(i = 0; i < 3; i++)
						{
							Lights_status_2[i] = RxMessage.data[i + 4];
						}
						os_evt_clr (EVT_DATA_3_RCVD, tid_lamp_ctl);
						os_evt_set (EVT_DATA_2_RCVD, tid_lamp_ctl);
						break;
				}
			}
			else if(RxMessage.data[0] == IPI && RxMessage.data[1] == 0xD1 && RxMessage.data[2] == 0x00)
			{
				// The bitmap of available vehicle channels
				Channels_enabled = ((((RxMessage.data[4]<<8)|RxMessage.data[3])>>(ID_Num-1)*4)&0x0f);
				// The bitmap of available walker channels
				Walker_channels = ((((RxMessage.data[6]<<8)|RxMessage.data[5])>>(ID_Num-1)*4)&0x0f);

				os_evt_set (EVT_DATA_3_RCVD, tid_lamp_ctl);
			}
		}
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
	//Channle0_Red
	GPIO_WriteBit(GPIOB, GPIO_Pin_3, (BitAction)Picked_lights_status[0]);

	//Channle0_Yellow
	GPIO_WriteBit(GPIOB, GPIO_Pin_4, (BitAction)Picked_lights_status[4]);

	//Channle0_Green
	GPIO_WriteBit(GPIOB, GPIO_Pin_5, (BitAction)Picked_lights_status[8]);

	//Channle1_Red
	GPIO_WriteBit(GPIOB, GPIO_Pin_6, (BitAction)Picked_lights_status[1]);

	//Channle1_Yellow
	GPIO_WriteBit(GPIOB, GPIO_Pin_7, (BitAction)Picked_lights_status[5]);

	//Channle1_Green
	GPIO_WriteBit(GPIOB, GPIO_Pin_8, (BitAction)Picked_lights_status[9]);

	//Channle2_Red
	GPIO_WriteBit(GPIOB, GPIO_Pin_9, (BitAction)Picked_lights_status[2]);

	//Channle2_Yellow
	GPIO_WriteBit(GPIOB, GPIO_Pin_10, (BitAction)Picked_lights_status[6]);

	//Channle2_Green
	GPIO_WriteBit(GPIOB, GPIO_Pin_11, (BitAction)Picked_lights_status[10]);

	//Channle3_Red
	GPIO_WriteBit(GPIOB, GPIO_Pin_12, (BitAction)Picked_lights_status[3]);

	//Channle3_Yellow
	GPIO_WriteBit(GPIOB, GPIO_Pin_13, (BitAction)Picked_lights_status[7]);

	//Channle3_Green
	GPIO_WriteBit(GPIOB, GPIO_Pin_14, (BitAction)Picked_lights_status[11]);
}

__task void task_lamp_ctl(void)
{	
	OS_RESULT result;
	CAN_msg *msg_error;
	
	while (1)
	{
		// wait for events to update lamps
		result = os_evt_wait_and (EVT_DATA_1_RCVD | EVT_DATA_2_RCVD | EVT_DATA_3_RCVD, 0xffff);
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
				
				if (!(Last_error & 0x8000))
				{
					msg_error = _calloc_box (mpool);
					//						Line_num  ID_Num	  bad_light_num	  error_type
					//{ 1, {IPI, 0xD2, 0x00, 0x01,    0xFF,        0xFF,         0xFF,     0xFE}, 8, 2, STANDARD_FORMAT, DATA_FRAME};			
					msg_error->id = 1;
					msg_error->data[0] = IPI;
					msg_error->data[1] = 0xD2;
					msg_error->data[3] = 0x01;
					msg_error->data[4] = ID_Num;
					msg_error->data[7] = MSG_END;
					msg_error->len = sizeof(msg_error->data);
					msg_error->ch = 2;
					msg_error->format = STANDARD_FORMAT;
					msg_error->type = DATA_FRAME;
					
					msg_error->data[6] = ERROR_NO_AC_POWER;

					os_mbx_send (CAN_send_mailbox, msg_error, 0xffff);
					Last_error |= 0x8000;
				}

			}
			else
			{
				os_evt_set(EVT_CONFLICT_MONITOR, tid_conflict_monitor);
			}
		}
	}
}

__task void task_green_led_flash(void)
{
	BitAction state = Bit_SET;
	
	os_itv_set (150);
	
	while (1)
	{
		os_itv_wait ();
		
		if (Work_normal)
		{
			state = (BitAction)(1 - state);
			GPIO_WriteBit(GPIOD, GPIO_Pin_1, state);
		}
		else
		{
			GPIO_WriteBit(GPIOD, GPIO_Pin_1, Bit_SET);
		}
	}
}

__task void task_red_led_flash(void)
{
	BitAction state = Bit_SET;
	
	os_itv_set (20);
	
	while (1)
	{
		os_itv_wait ();
		
		if (!Work_normal)
		{
			state = (BitAction)(1 - state);
			GPIO_WriteBit(GPIOD, GPIO_Pin_2, state);
		}
		else
		{
			GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_SET);
		}
	}
}

__task void task_conflict_monitor(void)
{
	OS_RESULT result;
	int i, j;
	uint16_t ADC_samples[36];
	uint16_t temp;
	CAN_msg *msg_recovered;
	u8 Conflict_buffer = 0;

	/* Initialize message  = { ID, {data[0] .. data[7]}, LEN, CHANNEL, FORMAT, TYPE } */
	//CAN_msg msg_recovered = { 1, {IPI, 0xD2, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE}, 8, 2, STANDARD_FORMAT, DATA_FRAME};

	while (1)
	{
		// wait for event to start a cycle of conflict detection
		result = os_evt_wait_and (EVT_CONFLICT_MONITOR, 0xffff);
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
			
			// get the status map
			ADC_status_map = 0;
			for (i=0; i<12; i++)
			{
				temp = (ADC_samples[i] + ADC_samples[12 + i] + ADC_samples[24 + i]) / 3;
				if (temp < ADC_THRESHOLD)
				{
					ADC_status_map <<= 1;
				}
				else
				{
					ADC_status_map = (ADC_status_map << 1) | 0x0001;
					
					os_mut_wait (mutex_ADC_minute, 0xffff);
					// critical region start
						ADC_minute_sum[i] += temp;
						ADC_minute_count[i]++;
					// critical region end
					os_mut_release (mutex_ADC_minute);
				}
			}
			
			// calculate the conflict map 
			conflict_map = Picked_lights_status_map ^ ADC_status_map;
			// consider the Vehicle_channels and Walker_channels
			conflict_map &= ~((u16)Walker_channels << 4);
			conflict_map &= (((u16)Channels_enabled << 8) | ((u16)Channels_enabled << 4) | (u16)Channels_enabled);
			
			// use Error_buffer to hold on initial errors until the maximum count is reached
			if (conflict_map)
			{
				if (Conflict_buffer < CONFLICT_BUFFER_MAX)
				{
					Conflict_buffer++;
				}
			}
			else if (Conflict_buffer > 0)
			{
				Conflict_buffer--;
			}
			
			// conflict happens
			if (Conflict_buffer == CONFLICT_BUFFER_MAX)
			{			
				Work_normal = 0;
				// handle conflict
				os_evt_set(EVT_CONFLICT_ANALYSIS, tid_conflict_analysis);
			}
			else
			{
				// if it recovers from a conflict
				if(!Work_normal)
				{
					msg_recovered = _calloc_box (mpool);
					//{ 1, {IPI, 0xD2, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE}, 8, 2, STANDARD_FORMAT, DATA_FRAME};
					msg_recovered->id = 1;
					msg_recovered->data[0] = IPI;
					msg_recovered->data[1] = 0xD2;
					msg_recovered->data[7] = MSG_END;
					msg_recovered->len = sizeof(msg_recovered->data);
					msg_recovered->ch = 2;
					msg_recovered->format = STANDARD_FORMAT;
					msg_recovered->type = DATA_FRAME;
					
					os_mbx_send(CAN_send_mailbox, msg_recovered, 0xffff);
					Work_normal = 1;
					Last_error = 0;
				}
				
				os_evt_set(EVT_SEND_HEART_BEAT, tid_heart_beat);

			}
		}
	}
}

__task void task_conflict_analysis(void)
{
	OS_RESULT result;
	u16 SWITs_status_map;
	CAN_msg *msg_error;
	int i;

	//										   Line_num	 ID_Num	  bad_light_num	  error_type
	//CAN_msg msg_error = { 1, {IPI, 0xD2, 0x00, 0x01,    0xFF,        0xFF,         0xFF,     0xFE}, 8, 2, STANDARD_FORMAT, DATA_FRAME};

	//msg_error.data[4] = ID_Num;

	while (1)
	{
		// wait for event to start a conflict analysis
		result = os_evt_wait_and (EVT_CONFLICT_ANALYSIS, 0xffff);
		if (result == OS_R_TMO) 
		{
			//printf("Event wait timeout.\n");
		}
		else if(conflict_map != Last_error)
		{
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
				test_n[i] = 0;
			}
			
			// check each conflict bit
			for (i=0; i<12; i++)
			{
				if ((conflict_map >> i) & 0x0001)
				{
					msg_error = _calloc_box (mpool);
					//						Line_num	 ID_Num	  bad_light_num	  error_type
					//{ 1, {IPI, 0xD2, 0x00, 0x01,    0xFF,        0xFF,         0xFF,     0xFE}, 8, 2, STANDARD_FORMAT, DATA_FRAME};			
					msg_error->id = 1;
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
					msg_error->data[5] = ((i/4) << 4) | (i%4);
					
					if ((ADC_status_map >> i) & 0x0001)
					{
						// SWIT short cut error
						msg_error->data[6] = ERROR_SWIT_CLOSE;
					}
					else if ((SWITs_status_map >> i) & 0x0001)
					{
						msg_error->data[6] = ERROR_LIGHT;
						Last_error |= 0x4000;
					}
					else
					{
						msg_error->data[6] = ERROR_SWIT_OPEN;
					}

					os_mbx_send (CAN_send_mailbox, msg_error, 0xffff);
					
				}
			}
			Last_error = (Last_error & 0x4000) | conflict_map;
		}
		os_evt_set(EVT_SEND_HEART_BEAT, tid_heart_beat);

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
		result = os_evt_wait_and (EVT_SEND_HEART_BEAT, 0xffff);
		if (result == OS_R_TMO) 
		{
			//printf("Event wait timeout.\n");
		}
		else
		{
			msg_heart_beat = _calloc_box (mpool);
			//{ 1, {IPI, 0x83, 0x01, 0xFF, 0x02, 0x00, 0x00, 0x00}, 8, 2, STANDARD_FORMAT, DATA_FRAME};
			msg_heart_beat->id = 1;
			msg_heart_beat->data[0] = IPI;
			msg_heart_beat->data[1] = 0x83;
			msg_heart_beat->data[2] = 0x01;
			msg_heart_beat->data[3] = ID_Num;
			msg_heart_beat->data[4] = 0x02;
			if (!AC_power_exist)
			{
				msg_heart_beat->data[5] = ERROR_NO_AC_POWER;
			}
			else if ((Last_error & 0x4000))
			{
				msg_heart_beat->data[5] = ERROR_LIGHT;
			}
			
			msg_heart_beat->len = 6;
			msg_heart_beat->ch = 2;
			msg_heart_beat->format = STANDARD_FORMAT;
			msg_heart_beat->type = DATA_FRAME;
			os_mbx_send (CAN_send_mailbox, msg_heart_beat, 0xffff);
		}
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
	
	os_itv_set (6000); // report once per minute
	
	while (1)
	{
		os_itv_wait();
		
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
}

__task void task_AC_detector(void)
{
	os_itv_set (25); // to detect every 250ms

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

	}
}
