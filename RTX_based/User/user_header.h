
#define LIGHT_CH_4_G ((uint16_t)0x0008)  //GPIO_Pin_3
#define LIGHT_CH_4_Y ((uint16_t)0x0010)  //GPIO_Pin_4
#define LIGHT_CH_4_R ((uint16_t)0x0020)  //GPIO_Pin_5
#define LIGHT_CH_3_G ((uint16_t)0x0040)  //GPIO_Pin_6
#define LIGHT_CH_3_Y ((uint16_t)0x0080)  //GPIO_Pin_7
#define LIGHT_CH_3_R ((uint16_t)0x0100)  //GPIO_Pin_8
#define LIGHT_CH_2_G ((uint16_t)0x0200)  //GPIO_Pin_9
#define LIGHT_CH_2_Y ((uint16_t)0x0400)  //GPIO_Pin_10
#define LIGHT_CH_2_R ((uint16_t)0x0800)  //GPIO_Pin_11
#define LIGHT_CH_1_G ((uint16_t)0x1000)  //GPIO_Pin_12
#define LIGHT_CH_1_Y ((uint16_t)0x2000)  //GPIO_Pin_13
#define LIGHT_CH_1_R ((uint16_t)0x4000)  //GPIO_Pin_14
#define LIGHT_CH_ALL ((uint16_t)0x7FF8)


#define EVT_DATA_1_RCVD			0x0001
#define EVT_DATA_2_RCVD			0x0002
#define EVT_DATA_3_RCVD			0x0004
#define EVT_CONFLICT_MONITOR	0x0008
#define EVT_CONFLICT_ANALYSIS	0x0010
#define EVT_ADC_DONE			0x0020
#define EVT_SEND_HEART_BEAT		0x0040
#define EVT_AC_DETECTION		0x0080

#define ADC_THRESHOLD			20
#define SWIT_INT_THRESHOLD		4
#define AC_DETECT_INT_THRESHOLD	2
#define CONFLICT_BUFFER_MAX		6

#define IPI						0x20
#define MSG_CONTINUE			0xFD
#define MSG_END					0xFE

#define NO_ERROR				0x00
#define ERROR_LIGHT				0x01
#define ERROR_SWIT_CLOSE		0x02
#define ERROR_SWIT_OPEN			0x03
#define ERROR_NO_AC_POWER		0x04
#define RED_CONFLICT				0x05
#define GREEN_CONFLICT			0x06
#define YELLOW_CONFLICT			0x07
#define ERROR_OPTOCOUPLER		0x08
#define ERROR_INDUCTOR_1		0x09
#define ERROR_INDUCTOR_2		0x0A
#define ERROR_SCR				0x0B
