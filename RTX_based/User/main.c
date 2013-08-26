#include "stm32f10x.h"
#include <RTL.h>
#include <RTX_CAN.h>
#include "stm32f10x_conf.h"
#include "user_header.h"
int test_n[16] = {0};
void Device_Init(void);
void SWITs_Enable(u8 enable);

OS_TID tid_send_CAN;
OS_TID tid_recv_CAN;
OS_TID tid_lamp_ctl;
OS_TID tid_green_led_flash;
OS_TID tid_red_led_flash;
OS_TID tid_conflict_monitor;
OS_TID tid_conflict_analysis;
OS_TID tid_heart_beat;

os_mbx_declare (CAN_send_mailbox, 20);

__task void init(void);
__task void task_send_CAN(void);
__task void task_recv_CAN(void);
__task void task_lamp_ctl(void);
__task void task_green_led_flash(void);
__task void task_red_led_flash(void);
__task void task_conflict_monitor(void);
__task void task_conflict_analysis(void);
__task void task_heart_beat(void);

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

extern void DMAReConfig(void);

int main (void)
{
	os_sys_init(init);
}

__task void init (void) 
{
	Device_Init();
	
	os_mbx_init (CAN_send_mailbox, sizeof(CAN_send_mailbox));
	// init lights all red
	GPIO_WriteBit(GPIOB, GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14, Bit_RESET);
	GPIO_WriteBit(GPIOB, GPIO_Pin_3|GPIO_Pin_6|GPIO_Pin_9|GPIO_Pin_12, Bit_SET);
	// turn off 2 led indicators
	GPIO_WriteBit(GPIOD, GPIO_Pin_1|GPIO_Pin_2, Bit_SET);
	
	tid_send_CAN = os_tsk_create(task_send_CAN, 3);
	tid_recv_CAN = os_tsk_create(task_recv_CAN, 4);
	tid_lamp_ctl = os_tsk_create(task_lamp_ctl, 2);
	tid_green_led_flash = os_tsk_create(task_green_led_flash, 2);
	tid_red_led_flash = os_tsk_create(task_red_led_flash, 2);
	tid_conflict_monitor = os_tsk_create(task_conflict_monitor, 2);
	tid_conflict_analysis = os_tsk_create(task_conflict_analysis, 2);
	tid_heart_beat = os_tsk_create(task_heart_beat, 2);

	os_tsk_delete_self();
}

__task void task_send_CAN(void) 
{
	void *msg;

	CAN_init(1, 500000);               /* CAN controller 1 init, 500 kbit/s   */
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
		//free(msg);
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
			os_evt_set(EVT_CONFLICT_MONITOR, tid_conflict_monitor);
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

	/* Initialize message  = { ID, {data[0] .. data[7]}, LEN, CHANNEL, FORMAT, TYPE } */
	CAN_msg msg_recovered = { 1, {IPI, 0xD2, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE}, 8, 2, STANDARD_FORMAT, DATA_FRAME};

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
				}
			}
			
			// calculate the conflict map 
			conflict_map = Picked_lights_status_map ^ ADC_status_map;
			// consider the Vehicle_channels and Walker_channels
			conflict_map &= ~((u16)Walker_channels << 4);
			conflict_map &= (((u16)Channels_enabled << 8) | ((u16)Channels_enabled << 4) | (u16)Channels_enabled);
			
			// conflict happens
			if (conflict_map)
			{			
				Work_normal = 0;
				// handle conflict
				os_evt_set(EVT_CONFLICT_ANALYSIS, tid_conflict_analysis);
			}
			else
			{
				// everything goes well. send a heart beat
				os_evt_set(EVT_SEND_HEART_BEAT, tid_heart_beat);
				// if it recovers from a conflict
				if(!Work_normal)
				{
					os_mbx_send(CAN_send_mailbox, &msg_recovered, 0xffff);
					Work_normal = 1;
				}
			}
		}
	}
}

__task void task_conflict_analysis(void)
{
	OS_RESULT result;
	u8 Error_type;
	u16 SWITs_status_map;
	int i;

	//										   Line_num	 ID_Num	  bad_light_num	  error_type
	CAN_msg msg_error = { 1, {IPI, 0xD2, 0x00, 0x01,    0xFF,        0xFF,         0xFF,     0xFE}, 8, 2, STANDARD_FORMAT, DATA_FRAME};

	msg_error.data[4] = ID_Num;

	while (1)
	{
		// wait for event to start a conflict analysis
		result = os_evt_wait_and (EVT_CONFLICT_ANALYSIS, 0xffff);
		if (result == OS_R_TMO) 
		{
			//printf("Event wait timeout.\n");
		}
		else
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
					// specify bad_light_num and error_type
					msg_error.data[5] = ((i/4) << 4) | (i%4);
					
					if ((ADC_status_map >> i) & 0x0001)
					{
						// SWIT short cut error
						msg_error.data[6] = ERROR_SWIT_CLOSE;
					}
					else if ((SWITs_status_map >> i) & 0x0001)
					{
						msg_error.data[6] = ERROR_LIGHT;
					}
					else
					{
						msg_error.data[6] = ERROR_SWIT_OPEN;
					}
					os_mbx_send (CAN_send_mailbox, &msg_error, 0xffff);
				}
			}

		}
	}
}

__task void task_heart_beat(void)
{
	OS_RESULT result;
	void *msg;
	/* Initialize message  = { ID, {data[0] .. data[7]}, LEN, CHANNEL, FORMAT, TYPE } */
	CAN_msg msg_heart_beat = { 1, {IPI, 0x83, 0x01, 0xFF, 0x02, 0x00, 0x00, 0x00}, 8, 2, STANDARD_FORMAT, DATA_FRAME};
	
	msg_heart_beat.data[3] = ID_Num;

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
			//msg = malloc(1, sizeof(CAN_msg));
			//memcpy(msg, &msg_heart_beat, sizeof(CAN_msg));
			os_mbx_send (CAN_send_mailbox, &msg_heart_beat, 0xffff);
		}
	}
}
