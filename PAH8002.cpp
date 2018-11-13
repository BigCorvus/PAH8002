// #include "pah8002.h"
// #include "pah8002_comm.h"
// #include "pah8002_api_c.h"
// #include "board.h"
// #include "dd_vendor_1.h"
// #include "uart.h"
// #include "main.h"
// #include "accelerometer.h"
// #include <stdint.h>
// #include <string.h>
// #include <stdlib.h>

 #include "pah8002.h"


//++++++++++++++++++++++PAH8002 functions++++++++++++++++++++++++++++++++++++++++++++++++++++
static bool pah8002_sw_reset() {
  uint8_t               data               ;
  //debug_printf(">>>pah8002_sw_reset\r\n");
  pah8002_wakeup();
  if (0 != pah8002_write_reg(0x7f, 0x00)) {
    gotoRTN;
  }
  if (0 != pah8002_read_reg(0, &data)) {
    goto  RTN;
  }
  //debug_printf("ID=%d\r\n", data);
  if (data  != 0x02) {
    goto RTN;
  }
  if (0 != pah8002_write_reg(0xe1, 0)) //write 0 to trigger Software Reset
  {
    goto   RTN;    //delay               5ms
  }
  delay(5);
  //debug_printf("<<<pah8002_sw_reset\r\n");
  return true;
RTN: return false;
}

static bool pah8002_start() {
  uint8_t data = 0;
  int samples_per_read =  HEART_RATE_MODE_SAMPLES_PER_READ ;
  //debug_printf(">>>pah8002_start  \r\n");
  pah8002_wakeup();
  if (0 != pah8002_write_reg(0x7f, 0x01))   {
    goto RTN;
  }
  else if (0 != pah8002_write_reg(0xea, (samples_per_read + 1)))   {
    goto  RTN;
  }
  else if (0 != pah8002_write_reg(0xd5, 1)) //TG enable. REQTIMER_ENABLE
  {
    goto  RTN;
  }
  else if (0 != pah8002_read_reg(0xd5, &data)) //TG enable. REQTIMER_ENABLE
  {
    goto RTN;
  }
  pah8002_check();
  //debug_printf("<<<  pah8002_start  %d\r\n", data);
  return true;
RTN:   return false;
}

static bool pah8002_touch_mode_init() {
  int i = 0 ;
  //debug_printf(">>> pah8002_touch_mode_init \r\n");
  pah8002_wakeup();
  for (i = 0; i < INIT_TOUCH_REG_ARRAY_SIZE; i++)            {
    if ( pah8002_write_reg(  init_touch_register_array[i][0], init_touch_register_array[i][1]) != 0 ) {
      goto RTN;
    }
  }
  //debug_printf("<<< pah8002_touch_mode_init \r\n");
  return   true;
RTN: return   false;


} static bool pah8002_normal_mode_init() {
  int i = 0 ;
  //debug_printf(">>> pah8002_normal_mode_init \r\n");
  pah8002_wakeup();
  for (i = 0; i < INIT_PPG_REG_ARRAY_SIZE; i++)  {
    if ( pah8002_write_reg( init_ppg_register_array[i][0], init_ppg_register_array[i][1]) != 0 )     {
      goto RTN;
    }
  }
  //debug_printf("<<< pah8002_normal_mode_init \r\n");
  return true;
RTN:  return false;
}

static bool pah8002_stress_mode_init() {
  int i = 0 ;
  //debug_printf(">>> pah8002_stress_mode_init \r\n");
  pah8002_wakeup();
  for (i = 0; i < INIT_STRESS_REG_ARRAY_SIZE; i++)               {
    if ( pah8002_write_reg(  init_stress_register_array[i][0], init_stress_register_array[i][1]) != 0 )     {
      goto RTN;
    }
  }
  //debug_printf("<<< pah8002_stress_mode_init \r\n");
  return  true;
RTN:  return  false;
}

static uint8_t pah8002_get_touch_flag_ppg_mode() {
  static uint8_t touch_sts_output = 1 ;
  int32_t *s = (int32_t *)pah8002_ppg_data ;
  int32_t ch0 ;
  int32_t ch1 ;
  int64_t ir_rawdata;
  int i;
  static int touch_cnt = 0, no_touch_cnt = 0 ;
  
#define TouchDetection_Upper_TH (600)
#define TouchDetection_Lower_TH (512)
#define TouchDetection_Count_TH  (3)    //(3+1)*50ms  = 200ms                
#define NoTouchDetection_Count_TH (3) //(3+1)*50ms = 200ms   

  for (i = 0; i < HEART_RATE_MODE_SAMPLES_PER_READ; i += TOTAL_CHANNELS)   {
    ch0 = *s; ch1 = *(s + 1);
    ir_rawdata = ch0 - ch1 ;
    ir_rawdata = (ir_rawdata * _ir_dac * _ir_expo) >> 20 ;
    if ( ir_rawdata > TouchDetection_Upper_TH) {
      touch_cnt++; no_touch_cnt = 0;
    } else if ( ir_rawdata < TouchDetection_Lower_TH) {
      no_touch_cnt++; touch_cnt = 0 ;
    } else {
      touch_cnt = 0 ; no_touch_cnt = 0;
    }
    s += TOTAL_CHANNELS;
  }
  if (touch_cnt > TouchDetection_Count_TH)   {
    touch_sts_output = 1;
  }   else if ( no_touch_cnt > NoTouchDetection_Count_TH)   {
    touch_sts_output = 0;
  }
  //debug_printf("<<< pah8002_get_touch_flag_ppg_mode %d, %d\n", touch_cnt, no_touch_cnt);
  //debug_printf("<<< pah8002_get_touch_flag_ppg_mode %d\n", touch_sts_output);
  return touch_sts_output;
}

static bool pah8002_enter_normal_mode() {
  //debug_printf(">>> pah8002_enter_normal_mode\r\n");
  if (_mode == NORMAL_MODE) return true;
  //1. software reset
  if ( !pah8002_sw_reset())  goto  RTN;
  //2. load registers for normal mode
  if (  !pah8002_normal_mode_init())    goto  RTN;
  pah8002_write_reg(0x7f,  0x00);               //Bank0
  pah8002_read_reg(0x0D, &_ir_expo); // IR Exposure Time
  pah8002_write_reg(0x7f, 0x01);               //Bank1
  pah8002_read_reg(0xBA, &_ir_dac); //IR Led DAC
  //3.  enable               sensor
  if (  !pah8002_start()) goto RTN;
  _mode = NORMAL_MODE;
  //debug_printf("<<< pah8002_enter_normal_mode ir_dac %x, ir_expo %x\r\n", _ir_dac, _ir_expo);
  return  true;
RTN:  return false ;
}

static bool pah8002_enter_stress_mode() {
  //debug_printf(">>> pah8002_enter_stress_mode\r\n");
  if (_mode == STRESS_MODE) return true;
  //1.               software               reset
  if ( !pah8002_sw_reset())   goto  RTN;
  //2. load registers for normal mode
  if ( !pah8002_stress_mode_init())  goto RTN;
  pah8002_write_reg(0x7f, 0x00);               //Bank0
  pah8002_read_reg(0x0D, &_ir_expo); // IR Exposure Time
  pah8002_write_reg(0x7f, 0x01);               //Bank1
  pah8002_read_reg(0xBA, &_ir_dac); //IR Led DAC
  //3.               enable               sensor
  if ( !pah8002_start()) goto RTN;
  _mode =  STRESS_MODE;
  //debug_printf("<<< pah8002_enter_stress_mode \r\n");
  return  true;
RTN:  return false ;
}

static bool pah8002_enter_touch_mode() {
  //debug_printf(">>> pah8002_enter_touch_mode\r\n");
  if (_mode == TOUCH_MODE) return true;
  //1.               software               reset
  if ( !pah8002_sw_reset() ) goto RTN;
  //2. load registers for touch mode
  if ( !pah8002_touch_mode_init())   goto  RTN;
  //3.               enable               sensor
  if ( !pah8002_start())  goto  RTN;
  _mode = TOUCH_MODE;
  //debug_printf("<<< pah8002_enter_touch_mode\r\n");
  return  true;
RTN:  return false ;
}

static bool pah8002_get_touch_flag( uint8_t *touch_flag) {
  //debug_printf(">>> pah8002_touch_status \r\n");
  pah8002_wakeup();
  if (0 != pah8002_write_reg(0x7f, 0x02))               {
    goto  RTN;
  } else if (0 != pah8002_read_reg(0x45, touch_flag)) //
  {
    goto RTN;
  }
  //debug_printf("<<< pah8002_touch_status %d\r\n", *touch_flag);
  return   true;
RTN:    return  false;
}

static int pah8002_wakeup() {
  int retry = 0 ;
  int success = 0 ;
  uint8_t data = 0 ;
  pah8002_read_reg(0, &data);
  pah8002_read_reg(0, &data);
  do   {
    pah8002_write_reg(0x7f, 0x00);
    pah8002_read_reg(0, &data);
    if (data == 0x02) success++; else success = 0 ;
    if (success >= 2) break;
    retry ++;
  } while (retry < 20);

  if (_chip_id == 0)  {
    pah8002_read_reg(0x02, &data);
    _chip_id = data & 0xF0 ;
    if (_chip_id != 0xD0) {
      //debug_printf("Not support anymore\r\n");
      while (1) {};
    }
  }
  pah8002_write_reg(0x7f, 0x02);
  pah8002_write_reg(0x70,  0x00);
  //debug_printf("pah8002_wakeup retry %d \r\n", retry);
  return retry;
}

static int pah8002_check() {
  int retry = 0 ;
  int success = 0 ;
  uint8_t data = 0 ;
  uint8_t b1_0xd5 = 0 ;
  uint8_t b1_0xe6 = 0 ;
  pah8002_read_reg(0, &data);
  pah8002_read_reg(0, &data);
  do  {
    pah8002_write_reg(0x7f, 0x00);
    pah8002_read_reg(0, &data);
    if (data == 0x02) success++; else success = 0 ;
    if (success >= 2)   break;
    retry               ++;
  } while (retry < 20);
  pah8002_write_reg(0x7f,  0x01);

  pah8002_read_reg(0xd5,               &b1_0xd5);
  pah8002_read_reg(0xe6,               &b1_0xe6);
  //debug_printf("pah8002_check retry %d \r\n", retry);
  if (b1_0xd5  !=   1)
    //debug_printf("pah8002_check error  Bank1 0xD5 0x%x \r\n", b1_0xd5);
  if (b1_0xe6 !=  0xC8)
    //debug_printf("pah8002_check error  Bank1 0xE6 0x%x \r\n", b1_0xe6);
  return              retry;
}

static bool pah8002_enter_suspend_mode() {
  int i = 0 ;
  //debug_printf("pah8002_enter_suspend_mode");
  pah8002_sw_reset();
  for (i = 0; i < SUSPEND_REG_ARRAY_SIZE; i++)               {
    if ( pah8002_write_reg(suspend_register_array[i][0], suspend_register_array[i][1]) != 0 )     {
      return false;
    }
  }
  _mode = SUSPEND_MODE;
  pah8002_check();
  return true;
}

static bool _pah8002_task() {
  uint8_t cks[4] ;
  uint8_t int_req = 0;
  //debug_printf(">>> pah8002_task\n");
  pah8002_wakeup();
  if (0 != pah8002_write_reg(0x7f, 0x02)) {

  } else if (0 != pah8002_read_reg(0x73, &int_req)) {

  } else {
    if ( (int_req & 0x04) != 0)   {   //overflow
      while (1);
    }
    if ( (int_req & 0x02) != 0)
    { //touch
      //debug_printf("touch               interrupt\n");
    } if ( (int_req & 0x08) != 0)   {
      //overflow
      while (1);
    }
    if ( (int_req & 0x01) != 0)   {
      int samples_per_read = HEART_RATE_MODE_SAMPLES_PER_READ ;
      //debug_printf("FIFO               interrupt\n");
      //pah8002_get_touch_flag(&state->pah8002_touch_flag);
      if (0 !=  pah8002_write_reg(0x7f, 0x03)) {

      } else  if (0 !=   pah8002_burst_read_reg(0, pah8002_ppg_data, samples_per_read * 4))    {

      } else if (0 != pah8002_write_reg(0x7f, 0x02))    {

      }  else if (0  !=   pah8002_burst_read_reg(0x80, cks, 4))    {

      } else if (0 != pah8002_write_reg(0x75, 0x01)) //read fifo first, then clear SRAM FIFO interrupt
      {} else if (0 != pah8002_write_reg(0x75, 0x00))    {

      } else {
        uint32_t  *s  = (uint32_t *)pah8002_ppg_data;
        uint32_t cks_cal = *s;
        uint32_t cks_rx = *((uint32_t *)cks) ;
        uint32_t i ;      //checksum    compare
        for (i = 1; i < samples_per_read; i++)  {
          cks_cal  = cks_cal ^   (*(s + i))  ;
        }
        if (cks_cal != cks_rx)   {
          //debug_printf("checksum error\r\n");
        }   else   {
          //debug_printf("checksum  OK %d\r\n", cks_cal);
        }
        _touch_flag =   pah8002_get_touch_flag_ppg_mode();
      }
    } else {
      //debug_printf("not fifo  interrupt%d\r\n",  int_req);
    }
  }
  //debug_printf("<<< pah8002_task\n");
  return    true;
}

static bool pah8002_normal_long_et_mode_init() {
  int i = 0 ;
  //debug_printf(">>>  pah8002_normal_long_et_mode_init \r\n");
  pah8002_wakeup();
  for (i = 0; i < INIT_PPG_LONG_REG_ARRAY_SIZE; i++)               {
    if ( pah8002_write_reg( init_ppg_long_register_array[i][0], init_ppg_long_register_array[i][1]) != 0 )     {
      goto RTN;
    }
  }
  //debug_printf("<<< pah8002_normal_long_et_mode_init \r\n");
  return     true;
RTN:  return   false;
}

static bool pah8002_enter_normal_long_et_mode() {
  //debug_printf(">>>               pah8002_enter_normal_long_et_mode\r\n");
  if (_mode == NORMAL_LONG_ET_MODE) return true;    //1.   software  reset
  if (   !pah8002_sw_reset())
    goto  RTN;
  //2. load registers for normal mode
  if ( !pah8002_normal_long_et_mode_init())
    goto RTN;
  pah8002_write_reg(0x7f, 0x00);     //Bank0
  pah8002_read_reg(0x0D, &_ir_expo); // IR Exposure Time
  pah8002_write_reg(0x7f,  0x01);     //Bank1
  pah8002_read_reg(0xBA, &_ir_dac); //IR Led DAC
  //3.  enable sensor
  if (  !pah8002_start())  goto RTN;
  _mode    =    NORMAL_LONG_ET_MODE;
  //debug_printf("<<< pah8002_enter_normal_long_et_mode ir_dac %x, ir_expo %x\r\n", _ir_dac, _ir_expo);
  return   true;
RTN:  return false ;
}

static void pah8002_dyn_switch_ppg_mode() {
  uint8_t b2a4, b2a5 ;
  uint16_t value ;
  pah8002_wakeup();
  pah8002_write_reg(0x7F, 0x02);
  pah8002_read_reg(0xa4, &b2a4);
  pah8002_read_reg(0xa5, &b2a5);
  value = b2a5 ;
  value <<= 8 ;
  value += b2a4 ;
  if (value >  4639)               {
    pah8002_enter_normal_long_et_mode();
  }
}

//---------------------------------------PAH8002 functions-----------------------------------------------

bool pah8002_init(void) {

  uint8_t ret = 0;
  uint32_t open_size = 0;
  //Algorithm               initialization
  _pah8002_data.frame_count = 0 ;
  _pah8002_data.nf_ppg_channel  = TOTAL_CHANNELS_FOR_ALG;
  _pah8002_data.nf_ppg_per_channel = HEART_RATE_MODE_SAMPLES_PER_CH_READ;
  _pah8002_data.ppg_data = (int32_t *)pah8002_ppg_data;
#ifdef MEMS_ZERO
  memset(_mems_data, 0, sizeof(_mems_data));
  _pah8002_data.nf_mems = HEART_RATE_MODE_SAMPLES_PER_CH_READ;
  _pah8002_data.mems_data  =               _mems_data;
#endif
  open_size               =               pah8002_query_open_size();
  _pah8002_alg_buffer               =               malloc(open_size);
  ret               =               pah8002_open(_pah8002_alg_buffer);
  if (ret != MSG_SUCCESS)   return false; // Set 0: +/-2G, 1: +/-4G, 2: +/-8G, 3: +/-16G
  if (MSG_SUCCESS != pah8002_set_param(PAH8002_PARAM_IDX_GSENSOR_MODE, 1))      return false;
  log_printf("PPG CH#, %d\n", TOTAL_CHANNELS_FOR_ALG);
  delay(300);
#ifdef PPG_MODE_ONLY
  return               pah8002_enter_normal_mode();
#else
  return              pah8002_enter_touch_mode();
#endif
}

void pah8002_deinit(void) {
  pah8002_enter_suspend_mode();
  pah8002_close();
  if  (_pah8002_alg_buffer)  {
    free(_pah8002_alg_buffer);
    _pah8002_alg_buffer  =    NULL;
  }
}


void pah8002_log(void) {
  int i = 0 ;
  uint32_t *ppg_data = (uint32_t *)_pah8002_data.ppg_data ;
  int16_t *mems_data = _pah8002_data.mems_data ;
  log_printf("Frame Count, %d \n", _pah8002_data.frame_count);
  log_printf("Time, %d \n", _pah8002_data.time);
  log_printf("PPG, %d, %d, ", _pah8002_data.touch_flag, _pah8002_data.nf_ppg_per_channel);
  for (i = 0; i < _pah8002_data.nf_ppg_channel  * _pah8002_data.nf_ppg_per_channel; i++)  {
    log_printf("%d, ", *ppg_data);
    ppg_data ++;
  }
  log_printf("\n");
  log_printf("MEMS, %d, ", _pah8002_data.nf_mems);
  for (i = 0; i < _pah8002_data.nf_mems * 3; i++) {
    log_printf("%d, ",  *mems_data);
    mems_data ++;
  }
  log_printf("\n");
}

static void data_convert_4ch_to_3ch(uint32_t *pdata, uint32_t len) {
  uint32_t i = 0, j = 0;
  for (i = 0, j = 2; j < len; i += 3, j += 4)  {
    *(pdata + i + 1)   =  *(pdata + j);
    *(pdata + i + 2) =  *(pdata + j + 1);
  }
}

void pah8002_task(void) {
  uint8_t   ret;
  float hr = 0 ;
  uint32_t  sys_tick;
  if (_pah8002_interrupt == 1)  {
    _pah8002_interrupt   =   0;
    if (_mode  == TOUCH_MODE)  {
      pah8002_enter_normal_mode();
      _timestamp = get_sys_tick();
      accelerometer_start();
    } else if (_mode == NORMAL_MODE || _mode == NORMAL_LONG_ET_MODE) {
      _pah8002_task();
      pah8002_dyn_switch_ppg_mode();
	  
#ifdef PPG_MODE_ONLY

#else
      if (_touch_flag  ==  0)  {
        pah8002_enter_touch_mode();                   
		accelerometer_stop();
      }
#endif
      //process algorithm
#ifdef MEMS_ZERO
#else
      accelerometer_get_fifo(&_pah8002_data.mems_data, &_pah8002_data.nf_mems);
#endif
      sys_tick = get_sys_tick();
      _pah8002_data.time = sys_tick - _timestamp;
      _timestamp  =  sys_tick;
      _pah8002_data.touch_flag =  _touch_flag;
      data_convert_4ch_to_3ch((uint32_t  *)pah8002_ppg_data,  HEART_RATE_MODE_SAMPLES_PER_READ);
      // log 3ch   ppg_data  before pah8002_entrance()
      pah8002_log();
      ret  =   pah8002_entrance(&_pah8002_data);
      if ((ret & 0x0f) != 0)    {
        switch (ret) //check error status
        { case MSG_ALG_NOT_OPEN:
            //debug_printf("Algorithm is not initialized.\r\n");
            break;
          case MSG_MEMS_LEN_TOO_SHORT:
            //debug_printf("MEMS data   length is  shorter than PPG data  length.\r\n");
            break;
          case MSG_NO_TOUCH:
            //debug_printf("PPG  is    no touch.\r\n");
            break;
          case MSG_PPG_LEN_TOO_SHORT:
            //debug_printf("PPGdata  length   is  too  short.\r\n");
            break;
          case MSG_FRAME_LOSS:
            //debug_printf("Frame  count   is  not  continuous.\r\n");
            break;
        }
      }
      if ((ret & 0xf0) == MSG_HR_READY)    {
        pah8002_get_hr(&hr)               ;
        //debug_printf("HR  =  %d\r\n",  (int)(hr));
      }
      _pah8002_data.frame_count++;
    }
  }
}

void pah8002_intr_isr(void) {
  _pah8002_interrupt = 1 ;
}

//pah8002_comm_i2c.c

#include "pah8002_comm.h"
#include "i2c.h"
#define I2C_ID_PAH8002  0x15  //I2C 7-bit ID

uint8_t pah8002_write_reg(uint8_t addr, uint8_t data) {
  return i2c_write_reg(I2C_ID_PAH8002, addr, data);
}

uint8_t pah8002_read_reg(uint8_t addr, uint8_t *data) {
  return i2c_read_reg(I2C_ID_PAH8002, addr, data);
}

uint8_t pah8002_burst_read_reg(uint8_t addr, uint8_t *data, uint32_t rx_size) {
  return   i2c_burst_read_reg(I2C_ID_PAH8002, addr, data, rx_size);
}








