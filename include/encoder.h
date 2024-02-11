#pragma once
#include <Arduino.h>
#include <limits>
#include "encoder2.pio.h"

namespace xrp {

class Encoder {
public:
/****************************************************************
*
*  Encoder::init()
*     Initialize PIO program that measures encoder period.
*
*  Returns true on sucess.
*  
*****************************************************************/
  bool init(int pin);

/****************************************************************
*
*  Encoder::update()
*     Get the latest encoder period(s) from the PIO if available
*     store the latest, and updated the tick count.
*  
*  Returns number of samples that were retrieved.
*  
*****************************************************************/
  int update();

/****************************************************************
*
*  Encoder::getPeriod()
*     Return the period calculated by the PIO in the following format:
*     31                           1   0
*    |  Period in 12-cycle ticks    | dir |
*  
*    This is the same format return by the PIO.
*  
*****************************************************************/
  uint getPeriod();

/****************************************************************
*
*  Encoder::getCount()
*     Return the number of encoder ticks since the robot started.
*
*****************************************************************/
  int getCount() ;

/****************************************************************
*
*  Encoder::getDivisor()
*     Get divisor for encoder period in ticks per second.
*  
*****************************************************************/

 static constexpr uint getDivisor() {
    return F_CPU * encoder2_CYCLES_PER_COUNT;
 }

private:
  uint period = UINT_MAX;
  int count = 0;
  int StateMachineIdx = -1;
  unsigned long last_sample_time = 0;
  PIO PioInstance = nullptr;
}; 

}