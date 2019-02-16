// Arduion Library for Omron 2SMPB02E
// 2019/1/16: akita11 (akita@ifdl.jp)

#include <Arduino.h>
#include <Omron2SMPB02E.h>
#include <Wire.h>
#include "BigNumber.h"

uint8_t Omron2SMPB02E::read_reg(uint8_t addr)
{
  uint8_t d;
  Wire.beginTransmission(i2c_addr);
  Wire.write(addr);
  Wire.endTransmission();

  Wire.requestFrom((int)i2c_addr, 1);
  d = Wire.read();
  Wire.endTransmission();
  return(d);
}

void Omron2SMPB02E::write_reg(uint8_t addr, uint8_t data)
{
  Wire.beginTransmission(i2c_addr);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission();
}

// read {(@addr):(@addr+1)}, 2's complement
int Omron2SMPB02E::read_reg16(uint8_t addr)
{
  uint16_t d = (read_reg(addr) << 8) | read_reg(addr + 1); // [@(addr):@(addr+1)]
  return(-(d & 0b1000000000000000) | (d & 0b0111111111111111)); // 2's complement
  return(d);

}

Omron2SMPB02E::Omron2SMPB02E(uint8_t SDO)
{
  i2c_addr = 0x56;
  if (SDO == 0) i2c_addr = 0x70;
}

BigNumber Omron2SMPB02E::conv_K0(int x, BigNumber a, BigNumber s)
{
  return(a + ((s * (BigNumber)x) / (BigNumber)32767.0));
}

BigNumber Omron2SMPB02E::conv_K1(long x)
{
  char s[10];
  sprintf(s, "%ld", x);
  return((BigNumber)s / (BigNumber)16);
}

void Omron2SMPB02E::begin()
{
  BigNumber::begin(20);
  Wire.begin();
  write_reg(IO_SETUP, 0x00); // IO_SETUP
  /*
  uint32_t coe_b00_a0_ex = (uint32_t)read_reg(COE_b00_a0_ex);
  a0 = ((uint32_t)read_reg(COE_a0_1) << 12) | ((uint32_t)read_reg(COE_a0_0) << 4) | ((uint32_t)coe_b00_a0_ex & 0x0000000f);
  a0 = -(a0 & (uint32_t)1 << 19) + (a0 & ~((uint32_t)1 << 19)); // 2's complement

  b00 =((uint32_t)read_reg(COE_b00_1) << 12) | ((uint32_t)read_reg(COE_b00_0) << 4) | (coe_b00_a0_ex >> 4);
  */
  set_average(AVG_1, AVG_1);

}

uint8_t Omron2SMPB02E::read_id()
{
  return(read_reg(CHIP_ID)); // CHIP_ID, would be 0x5c
}

void Omron2SMPB02E::reset()
{
  write_reg(RESET, 0xe6); // software reset
}

long Omron2SMPB02E::read_raw_temp()
{
  return((((uint32_t)read_reg(TEMP_TXD2) << 16)
	  | ((uint32_t)read_reg(TEMP_TXD1) <<  8)
	  | ((uint32_t)read_reg(TEMP_TXD0)      )) - ((uint32_t)1 << 23));
}

long Omron2SMPB02E::read_raw_pressure()
{
  return((((uint32_t)read_reg(PRESS_TXD2) << 16)
	  | ((uint32_t)read_reg(PRESS_TXD1) <<  8)
	  | ((uint32_t)read_reg(PRESS_TXD0)      )) - ((uint32_t)1 << 23));
}

// read temperature in [degC]
float Omron2SMPB02E::read_temp()
{
  BigNumber t = read_calc_temp() / (BigNumber)256;
  return((float)(t * (BigNumber)10000) / 10000.0);
}

BigNumber Omron2SMPB02E::read_calc_temp()
{
  // Tr = a0 + a1 * Dt + a2 * Dt^2
  // -> temp = Re / 256 [degC]
  //   Dt : raw temperature value from TEMP_TXDx reg.

  //   a0, b00 : OTP / 16
  //     {19:12}   {11:4}    {3:0}
  // a0  COE_a0_1  COE_a0_0  COE_a0_ex
  // b00 COE_b00_1 COE_b00_0 COE_b00_ex

  //   a1, ... : A + (S * OTP) / 32767
  //        A        S        OTP
  // a1    -6e-03    4.3e-04  [COE_a1_1,COE_a1_0]
  // a2    -1.9e-11  1.2e-10  [COE_a2_1,COE_a2_0]
  // bt1   1.0e-01   9.1e-02  [COE_bt1_1,COE_bt1_0]
  // bt2   1.2e-8    1.2e-06  [COE_bt2_1,COE_bt2_0]
  // bp1   3.3e-02   1.9e-02  [COE_bp1_1,COE_bp1_0]
  // b11   2.1e-07   1.4e-07  [COE_b11_1,COE_b11_0]
  // bp2   -6.3e-10  3.5e-10  [COE_bp2_1,COE_bp2_0]
  // b12   2.9e-13   7.6e-13  [COE_b12_1,COE_b12_0]
  // b21   2.1e-15   1.2e-14  [COE_b21_1,COE_b21_0]
  // bp3   1.3e-16   7.9e-17  [COE_bp3_1,COE_bp3_0]

  char dt[10];
  sprintf(dt, "%ld", read_raw_temp());
  BigNumber Bdt = (BigNumber)dt;
  long a0;
  a0 = ((uint32_t)read_reg(COE_a0_1) << 12) | ((uint32_t)read_reg(COE_a0_0) << 4) | ((uint32_t)read_reg(COE_b00_a0_ex) & 0x0000000f);
  a0 = -(a0 & (uint32_t)1 << 19) + (a0 & ~((uint32_t)1 << 19)); // 2's complement

  BigNumber temp = conv_K1(a0)
    + (conv_K0(read_reg16(COE_a1), (BigNumber)A_a1, (BigNumber)S_a1)
       + conv_K0(read_reg16(COE_a2), (BigNumber)A_a2, (BigNumber)S_a2) * Bdt) * Bdt;
  return(temp);
}

// read pressure in [Pa]
//BigNumber Omron2SMPB02E::read_pressure()
float Omron2SMPB02E::read_pressure()
{
  // Pr = b00 + (bt1 * Tr) + (bp1 * Dp) + (b11 * Dp * Tr) + (bt2 * Tr^2)
  //      + (bp2 * Dp^2) + (b12 * Dp * Tr^2) + (b21 * Dp^2 * Tr) + (bp3 * Dp^3)
  //   Tr : raw temperature from TEMP_TXDx reg.
  //   Dp : raw pressure from PRESS_TXDx reg.
  BigNumber Bprs;
  long b00 =((uint32_t)read_reg(COE_b00_1) << 12) | ((uint32_t)read_reg(COE_b00_0) << 4) | ((uint32_t)read_reg(COE_b00_a0_ex) >> 4);

  char dp[10];
  sprintf(dp, "%ld", read_raw_pressure());
  BigNumber Bdp = (BigNumber)dp;
  BigNumber Btr = (BigNumber)read_calc_temp();
  /*
  Pr = b00
    + (bt1 * Tr)
    + (b11 * Dp * Tr)
    + (bt2 * Tr^2)
    + (b12 * Dp * Tr^2)
    + (bp1 * Dp)
    + (bp2 * Dp^2)
    + (b21 * Dp^2 * Tr)
    + (bp3 * Dp^3)
  pressure = Bb00
    + Btr * (Bbt1 + Bb11 * Bdp + Btr * (Bbt2 + Bb12 * Bdp))
    + Bdp * (Bbp1 + Bdp * (Bbp2 + Bb21 * Btr + Bbp3 * Bdp));
  */
  BigNumber w;
  BigNumber w2;
  Bprs = conv_K1(b00);
  w = conv_K0(read_reg16(COE_bt1), (BigNumber)A_bt1, (BigNumber)S_bt1);
  w += conv_K0(read_reg16(COE_b11), (BigNumber)A_b11, (BigNumber)S_b11) * Bdp;
  w += Btr * (conv_K0(read_reg16(COE_bt2), (BigNumber)A_bt2, (BigNumber)S_bt2)
	      + conv_K0(read_reg16(COE_b12), (BigNumber)A_b12, (BigNumber)S_b12) * Bdp);
  Bprs += Btr * w;
  w = conv_K0(read_reg16(COE_bp1), (BigNumber)A_bp1, (BigNumber)S_bp1);
  w2 = conv_K0(read_reg16(COE_bp2), (BigNumber)A_bp2, (BigNumber)S_bp2);
  w2 += conv_K0(read_reg16(COE_b21), (BigNumber)A_b21, (BigNumber)S_b21) * Btr;
  w2 += conv_K0(read_reg16(COE_bp3), (BigNumber)A_bp3, (BigNumber)S_bp3) * Bdp;
  w += Bdp * w2;
  Bprs += Bdp * w;
  return((float)(Bprs * (BigNumber)10000) / 10000.0);
}

void Omron2SMPB02E::set_average(uint8_t temp_avg, uint8_t pressure_avg)
{
  uint8_t r = read_reg(CTRL_MEAS) & 0x03;
  r = r | (temp_avg << 5);
  r = r | (pressure_avg << 2);
  write_reg(CTRL_MEAS, r);
}

void Omron2SMPB02E::set_mode(uint8_t mode)
{
  uint8_t r = read_reg(CTRL_MEAS) & 0xfc;
  r = r | mode;
  write_reg(CTRL_MEAS, r);
}

uint8_t Omron2SMPB02E::is_busy()
{
  if ((read_reg(DEVICE_STAT) & 0x08) == 0) return(0);
  else return(1); // busy
}

void Omron2SMPB02E::set_filter(uint8_t mode)
{
  write_reg(IIR_CNT, mode);
}
