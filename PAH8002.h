#define TOTAL_CHANNELS 4 //Using channel numbers 
#define HEART_RATE_MODE_SAMPLES_PER_CH_READ (20) //Numbers of PPG data per channel 
#define HEART_RATE_MODE_SAMPLES_PER_READ (TOTAL_CHANNELS* HEART_RATE_MODE_SAMPLES_PER_CH_READ)
#define TOTAL_CHANNELS_FOR_ALG 3
#define MEMS_ZERO 0 //Default Accelerometer data are all zero 
#define PPG_MODE_ONLY

enum {
  SUSPEND_MODE  = 0,
  TOUCH_MODE,
  NORMAL_MODE,
  NORMAL_LONG_ET_MODE,
  STRESS_MODE,
  NONE,
};

typedef struct pah8002_data {        
uint8_t      frame_count;        //Frame Count        
uint32_t     time;              //FIFO Data Ready  Interval,  unit ms              
uint8_t      touch_flag;         //Touch Status, 1 for Touch and 0 for De-Touch         
uint32_t     nf_ppg_channel;     //Using channel numbers, ex.3        
uint32_t     nf_ppg_per_channel; //Numbers of PPG data per channel, ex.20         
int32_t      *ppg_data;          //Pointer to FIFO Raw Data        
uint32_t     nf_mems;            //Numbers of Accelerometer data(X,Y,Z), must larger or equal  
								//than numbers of PPG data per channel, ex.25        
int16_t      *mems_data;         //Pointer to Accelerometer data 
} pah8002_data_t;

static uint8_t _mode = NONE ;
static uint8_t pah8002_ppg_data[HEART_RATE_MODE_SAMPLES_PER_READ * 4] ;
static uint8_t _touch_flag = 0 ;
static volatile uint8_t _pah8002_interrupt = 0 ;
static pah8002_data_t _pah8002_data;
static uint32_t _timestamp = 0 ;

#ifdef MEMS_ZERO
static int16_t _mems_data[HEART_RATE_MODE_SAMPLES_PER_READ * 3] ;
#endif

static uint8_t _ir_dac = 0 ;
static uint8_t _ir_expo = 0 ;
static uint8_t _chip_id = 0 ;
static void *_pah8002_alg_buffer = NULL;
static bool pah8002_sw_reset(void);
static bool pah8002_start(void);
static int pah8002_wakeup(void);
static int pah8002_check(void);