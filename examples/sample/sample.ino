#include "Omron2SMPB02E.h"

Omron2SMPB02E prs;
// Omron2SMPB02E prs(0); // in case of SDO=0 configuration

void setup()
{
  prs.begin();
  Serial.begin(9600);
  prs.set_mode(MODE_NORMAL);
  delay(300);
  prs.read_all_coe();
  delay(300);
}

void loop()
{
  float tmp = prs.read_temp();
  Serial.println("temperature [degC]");
  Serial.println(tmp);
  float pressure = prs.read_pressure();
  Serial.println("pressure [Pa]");
  Serial.println(pressure);
  delay(500);
}
