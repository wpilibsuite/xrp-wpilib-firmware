#pragma once
#include <Arduino.h>
#include <limits>

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
*     Convert the latest period measurement of the encoder from
*     an integer count to the length of the period in seconds
*     as a double.
*  
*****************************************************************/

  double getPeriod();

private:
  unsigned value = INT_MAX;
  bool direction = true;
  int StateMachineIdx = -1;
  unsigned long last_sample_time = 0;
  PIO PioInstance = nullptr;
}; 

}