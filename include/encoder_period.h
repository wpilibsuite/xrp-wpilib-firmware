#pragma once
#include <Arduino.h>
#include <limits>
#include "encoder_period.pio.h"

namespace xrp {

class EncoderPeriod {
public:
/****************************************************************
*
*  EncoderPeriod::init()
*     Initialize PIO program that measures encoder period.
*
*  Returns true on sucess.
*  
*****************************************************************/
  bool init(int pin);

/****************************************************************
*
*  EncoderPeriod::update()
*     Get the latest encoder period from the PIO if available
*     and store it.
*  
*  Returns number of samples that were stored (0 or 1);
*  
*****************************************************************/
  int update();

/****************************************************************
*
*  EncoderPeriod::getPeriod()
*     Get last encoder period value returned by PIO.
*  
*****************************************************************/

  uint getPeriod();

/****************************************************************
*
*  EncoderPeriod::getDivisor()
*     Get divisor for encoder period in ticks per second.
*  
*****************************************************************/

 static constexpr uint getDivisor() {
    return F_CPU * encoder_period_CYCLES_PER_COUNT;
 }

private:
  uint value = UINT_MAX;
  int StateMachineIdx = -1;
  unsigned long last_sample_time = 0;
  PIO PioInstance = nullptr;
}; 

}