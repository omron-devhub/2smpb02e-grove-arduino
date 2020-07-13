// Arduion Library for Omron 2SMPB02E
// 2019/1/16: akita11 (akita@ifdl.jp)

#ifndef Omron2SMPB02E_h
#define Omron2SMPB02E_h

#include "BigNumber.h"

#include "Arduino.h"

#define MODE_SLEEP  0
#define MODE_FORCE  1
#define MODE_NORMAL 2

// registers
#define TEMP_TXD0 0xfc
#define TEMP_TXD1 0xfb
#define TEMP_TXD2 0xfa
#define PRESS_TXD0 0xf9
#define PRESS_TXD1 0xf8
#define PRESS_TXD2 0xf7
#define IO_SETUP 0xf5
#define CTRL_MEAS 0xf4
#define DEVICE_STAT 0xd3
#define I2C_SET 0xf2
#define IIR_CNT 0xf1
#define RESET 0xe0
#define CHIP_ID 0xd1
#define COE_b00_a0_ex 0xb8
#define COE_a2_0 0xb7
#define COE_a2_1 0xb6
#define COE_a1_0 0xb5
#define COE_a1_1 0xb4
#define COE_a0_0 0xb3
#define COE_a0_1 0xb2
#define COE_bp3_0 0xb1
#define COE_bp3_1 0xb0
#define COE_b21_0 0xaf
#define COE_b21_1 0xae
#define COE_b12_0 0xad
#define COE_b12_1 0xac
#define COE_bp2_0 0xab
#define COE_bp2_1 0xaa
#define COE_b11_0 0xa9
#define COE_b11_1 0xa8
#define COE_bp1_0 0xa7
#define COE_bp1_1 0xa6
#define COE_bt2_0 0xa5
#define COE_bt2_1 0xa4
#define COE_bt1_0 0xa3
#define COE_bt1_1 0xa2
#define COE_b00_0 0xa1
#define COE_b00_1 0xa0

#define COE_a2 0xb6
#define COE_a1 0xb4
#define COE_a0 0xb2
#define COE_bp3 0xb0
#define COE_b21 0xae
#define COE_b12 0xac
#define COE_bp2 0xaa
#define COE_b11 0xa8
#define COE_bp1 0xa6
#define COE_bt2 0xa4
#define COE_bt1 0xa2
#define COE_b00 0xa0

// arg of set_mode()
#define MODE_SLEEP 0x0
#define MODE_FORCED 0x1
#define MODE_NORMAL 0x3

// arg of set_average()
#define AVG_SKIP 0x0
#define AVG_1    0x1
#define AVG_2    0x2
#define AVG_4    0x3
#define AVG_8    0x4
#define AVG_16   0x5
#define AVG_32   0x6
#define AVG_64   0x7

// arg of set_filter()
#define FILTER_OFF 0x0
#define FILTER_2   0x1
#define FILTER_4   0x2
#define FILTER_8   0x3
#define FILTER_16  0x4
#define FILTER_32  0x5

// conversion coefficients
#define A_a1 "-0.0063"
#define S_a1 "0.00043"
#define A_a2 "-0.000000000019"
#define S_a2 "0.00000000012"
#define A_bt1	"0.100000000000000000"
#define S_bt1	"0.091000000000000000"
#define A_bt2	"0.000000012000000000"
#define S_bt2	"0.000001200000000000"
#define A_bp1	"0.033000000000000000"
#define S_bp1	"0.019000000000000000"
#define A_b11	"0.000000210000000000"
#define S_b11	"0.000000140000000000"
#define A_bp2	"-0.000000000630000000"
#define S_bp2	"0.000000000350000000"
#define A_b12	"0.000000000000290000"
#define S_b12	"0.000000000000760000"
#define A_b21	"0.000000000000002100"
#define S_b21	"0.000000000000012000"
#define A_bp3	"0.000000000000000130"
#define S_bp3	"0.000000000000000079"

class Omron2SMPB02E
{
 private:
  // calibration coefficients
  uint8_t i2c_addr = 0x56; // SDO=1 / 0x70@SDO=0
  uint8_t read_reg(uint8_t addr);
  void write_reg(uint8_t addr, uint8_t data);
  int read_reg16(uint8_t addr); // read {(@addr):(@addr+1)}, 2's complement
  long read_raw_temp();
  BigNumber read_calc_temp();
  long read_raw_pressure();
  BigNumber conv_K0(int x, BigNumber a, BigNumber s);
  BigNumber conv_K1(long x);

  // COE values
  // for read_pressure
  uint8_t uint8_COE_b00_1;
  uint8_t uint8_COE_b00_0;
  uint8_t uint8_COE_b00_a0_ex;
  int int_COE_bt1;
  int int_COE_b11;
  int int_COE_bt2;
  int int_COE_b12;
  int int_COE_bp1;
  int int_COE_bp2;
  int int_COE_b21;
  int int_COE_bp3;
  
  // for read_calc_temp
  uint8_t uint8_COE_a0_1;
  uint8_t uint8_COE_a0_0;
  int int_COE_a1;
  int int_COE_a2;
  
 public:
  Omron2SMPB02E(uint8_t SDO = 1);
  void begin();
  //  BigNumber read_temp(); // [degC]
  //  BigNumber read_pressure(); // [Pa]
  float read_temp(); // [degC]
  float read_pressure(); // [Pa]
  void set_mode(uint8_t mode); // MODE_{SLEEP,FORCE,NORMAL}
  uint8_t read_id();
  void reset();
  void set_average(uint8_t temp_avg, uint8_t pressure_avg);
  uint8_t is_busy();
  void set_filter(uint8_t mode);

  void read_all_coe(); // call once 
};
#endif
