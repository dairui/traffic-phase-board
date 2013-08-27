

#define EVT_DATA_1_RCVD			0x0001
#define EVT_DATA_2_RCVD			0x0002
#define EVT_DATA_3_RCVD			0x0004
#define EVT_CONFLICT_MONITOR	0x0008
#define EVT_CONFLICT_ANALYSIS	0x0010
#define EVT_ADC_DONE			0x0020
#define EVT_SEND_HEART_BEAT		0x0040

#define ADC_THRESHOLD			1000
#define SWIT_INT_THRESHOLD		4

#define IPI						0x20
#define MSG_CONTINUE			0xFD
#define MSG_END					0xFE

#define ERROR_LIGHT				0x01
#define ERROR_SWIT_CLOSE		0x02
#define ERROR_SWIT_OPEN			0x03
