#pragma once

#define HK_FIFO_EMPTY 				0x00
#define HK_FIFO_FULL 				0x01
#define HK_FIFO_OK 					0x02

#define HK_FIFO_BUFFER_SIZE 		255U
#define HK_FIFO_BUFFER_CNT	 		20U

#define HK_FRAME_MAIN				0x01
#define HK_FRAME_APPEND				0x02

#define HK_MAIN_LENGTH			    sizeof(HK_Main_t)
#define HK_APPEND_LENGTH			sizeof(HK_Append_t)

/*一个遥测文件最多存储200条遥测信息*/
#define HK_FILE_MAX_COUNT			200

typedef struct __attribute__((packed)) {
	unsigned char 	frame[HK_FIFO_BUFFER_CNT][HK_FIFO_BUFFER_SIZE];
	unsigned int 	bufferCount;
	unsigned int 	front;
	unsigned int 	rear;
} HK_Fifo_t;


/*星务计算机本地遥测，37 Byte*/
typedef struct __attribute__((packed))
{
											//卫星号
	uint8_t         sat_id;                 //1
											/**软件版本号*/
	uint8_t         soft_id;                //1
											/**重启计数*/
	uint16_t        reboot_count;           //2
											/**上行本地指令计数*/
	uint16_t        rec_cmd_count;          //2
											/**下行遥测帧总计数*/
	uint32_t        hk_down_count;          //4
											/**存储遥测帧总计数*/
	uint32_t        hk_store_count;         //4
											/**i2c驱动错误计数*/
	uint32_t        i2c_error_count;        //4
											/**上次复位时间*/
	uint32_t        last_reset_time;        //4
											/**工作模式*/
	uint8_t         work_mode;              //1
											/**UTC时间*/
	uint32_t        utc_time;               //4
											/**CPU片内温度*/
	uint16_t        tmep_mcu;               //2
											/**obc板上温度*/
	uint16_t        tmep_board;             //2
											/**开关状态*/
	uint32_t        on_off_status;          //4
											/**RAM延时遥测主帧索引*/
	uint8_t         mindex;                 //1
											/**RAM延时遥测辅帧索引*/
	uint8_t         aindex;                 //1
} obc_hk_t;

/*电源分系统遥测，64 Byte*/
typedef struct __attribute__((packed))
{
											//两路电池板温度
	int16_t         temp_batt_board[2];     //4
											/**四路电源控制板温度*/
	int16_t         temp_eps[4];            //8
											/**六路光电流*/
	uint16_t        sun_c[6];               //12
											/**六路光电压*/
	uint16_t        sun_v[6];               //12
											/**输出母线电流*/
	uint16_t        out_BusC;               //2
											/**输出母线电压*/
	uint16_t        out_BusV;               //2
											/**通信板电流*/
	uint16_t        UV_board_C;             //2
											/**六路可控输出的电流遥测*/
	uint16_t        Vol_5_C[6];             //12
											/**五路母线保护输出电流遥测*/
	uint16_t        Bus_c[5];               //10
} eps_hk_t;

/*获取接收单元遥测响应结构体*/
typedef  struct __attribute__((packed)) {
	uint16_t DopplerOffset;
	uint16_t TotalCurrent;
	uint16_t BusVoltage;
	uint16_t OscillatorTemp;
	uint16_t AmplifierTemp;
	uint16_t RSSI;
} rsp_rx_tm;

/*接收机收到上行数据时的遥测*/
typedef  struct __attribute__((packed)) {
	uint16_t DopplerOffset;
	uint16_t RSSI;
} receiving_tm;

/*获取发射单元遥测响应结构体*/
typedef  struct __attribute__((packed)) {
	uint16_t ReflectedPower;
	uint16_t ForwardPower;
	uint16_t BusVoltage;
	uint16_t TotalCurrent;
	uint16_t AmplifierTemp;
	uint16_t OscillatorTemp;
} rsp_tx_tm;

/*获取发射单元遥测响应结构体*/
typedef  struct __attribute__((packed)) {
	uint8_t IdleState : 1;    // bit0
	uint8_t BeaconAct : 1;    // bit1
	uint8_t BitRate : 2;      // bit2~bit3
	uint8_t FM_On : 1;        // bit4
	uint8_t padding : 3;
} rsp_transmitter_state;


/*ISISvu通信机遥测，46 Byte*/
typedef struct __attribute__((packed))
{
											/**接收单元自上次复位以来的运行时间*/
	uint32_t        ru_uptime;              //4
											/**接收单元当前所有遥测*/
	rsp_rx_tm       ru_curt;                //12
											/**接收单元收到上行数据时遥测*/
	receiving_tm    ru_last;                //2
											/**发射单元自上次复位以来的运行时间*/
	uint32_t        tu_uptime;              //4
											/**发射单元当前所有遥测*/
	rsp_tx_tm       tu_curt;                //12
											/**发射单元上次下行数据时所有遥测*/
	rsp_tx_tm       tu_last;                //12
											/**发射机工作状态*/
	rsp_transmitter_state tx_state;

} vu_isis_hk_t;


/* 主遥测帧, 178 byte*/
typedef struct __attribute__((packed))
{
	obc_hk_t        obc;
	eps_hk_t        eps;
	vu_isis_hk_t    ttc;
} HK_Main_t;

typedef struct __attribute__((packed))
{
	uint16_t        rst_cnt;
	uint16_t        rcv_cnt;
	uint16_t        ack_cnt;
	uint32_t        rst_time;
	uint32_t        utc_time;
	uint16_t        cpu_temp;
	uint8_t         adcs_ctrl_mode;
	uint16_t        downAdcsMagDotDmpCnt;
	uint16_t        downAdcsPitFltComCnt;
	uint16_t        downAdcsAttStaCnt;
	uint8_t         error;
} adcs_hk_workmode_t;

typedef struct __attribute__((packed))
{
	uint16_t        sw_status;
	int16_t         downAdcsMagnetometer[3];
	int16_t         downAdcsGyro_Meas[3];
	uint16_t        downAdcsSun1_Meas[4];
	uint16_t        downAdcsSun2_Meas[4];
	uint16_t        downAdcsSun3_Meas[4];
	uint8_t         downAdcsSunSensorFlag;
	float           downAdcsSun_Meas[3];  //上面是两个字节
	uint16_t        downAdcsWheelSpeed_Meas;
	int16_t         downAdcsMTQOut[3];
	int16_t         downAdcsMagInO[3];

} adcs_hk_component_t;

typedef struct __attribute__((packed))
{
	int16_t         downAdcsPitAngle;
	int16_t         downAdcsPitFltState[2];
	float           downAdcsPitFltNormP;
	float           downAdcsTwoVector_euler[3];
	uint16_t        downAdcsTwoVectorCnt;
	uint16_t        downAdcsMagSunFltCnt;
	uint16_t        downAdcsMagGyroFltCnt;
	float           downAdcsMagSunFltQ[4];
	float           downAdcsMagSunFltW[3];
	float           downAdcsMagSunFltNormP;
	float           downAdcsMagGyroFltQ[4];
	float           downAdcsMagGyroFltw[3];
	float           downAdcsMagGyroFltNormP;
} adcs_hk_attitude_t;

typedef struct __attribute__((packed))
{
	float           downAdcsOrbPos[3];
	int16_t         downAdcsOrbVel[3];
	uint8_t         GPS_status;
	uint8_t         GPS_numV;
	uint16_t        GPS_pdop;
} adcs_hk_orbit_t;

typedef struct __attribute__((packed))
{
	int16_t         adc[10];
} adcs_hk_temp_t;

typedef struct __attribute__((packed))
{
	adcs_hk_workmode_t     adcs_hk_workmode;
	adcs_hk_component_t    adcs_hk_component;
	adcs_hk_attitude_t     adcs_hk_attitude;
	adcs_hk_orbit_t        adcs_hk_orbit;
	adcs_hk_temp_t         adcs_hk_temp;
} adcs_hk_t;

typedef struct __attribute__((packed))
{
	adcs_hk_t		adcs_hk;
} HK_Append_t;


typedef struct __attribute__((packed))
{
	HK_Main_t 	main_frame;
	HK_Append_t append_frame;
} HK_Store_t;


extern HK_Fifo_t hk_main_fifo;
extern HK_Fifo_t hk_append_fifo;
extern uint32_t hk_down_cnt;

void HK_fifoInit(HK_Fifo_t *Q);
uint8_t HK_fifoIn(HK_Fifo_t *Q, unsigned char *pdata, uint8_t opt);
uint8_t HK_fifoOut(HK_Fifo_t *Q, unsigned char *pdata, uint8_t opt);
uint16_t hk_fifo_find(const HK_Fifo_t *Q, uint32_t timevalue);
int hk_store_init(void);
void hk_out(void);
int hk_store_add(void);
void vTelemetryFileManage(void * paragram);




typedef struct EpsAdcValue_t
{
	uint16_t    In_SunC[6];
	uint16_t    In_SunV[6];
	uint16_t    Out_ComC;
	uint16_t    Out_BusC;
	uint16_t    Out_BusV;
	uint16_t    Out_BranchC[9];
	int16_t     EpsTemp[4];
	int16_t     BatTemp[4];
} EpsAdcValue_t;