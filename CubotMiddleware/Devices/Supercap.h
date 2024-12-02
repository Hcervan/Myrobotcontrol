#ifndef SUPERCAP_H
#define SUPERCAP_H
#include "driver_can.h"
#include "dr16.h"




typedef struct
{
	  struct
  {
	  float    Voltage;
	  float    Current;
	  uint8_t  Supercap_Mode;
		uint8_t  Supercap_Flag;
		uint8_t  Supercap_Charge;
		uint8_t  Supercap_Charge_mode;
  }cap_state;
}Supercap;



#endif