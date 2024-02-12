/* The Encoder class reads encoder periods calculated by the
   encoder2 pio program, stores the calculated encoder periods
   and keeps track of encoder tick count.

 Copyright (C) 2024 Brian LePage

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:
 1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
 3. The name of the author may not be used to endorse or promote products
    derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "encoder.h"
#include "encoder2.pio.h"

namespace xrp {

static PIOProgram encoderProgram(&encoder2_program);

/****************************************************************
*
*  Encoder::init()
*     Initialize PIO program that measures encoder period.
*
*  Returns true on sucess.
*  
*****************************************************************/

bool Encoder::init(int pin) {
  last_sample_time = millis();
  int offset = -1;

  if (!encoderProgram.prepare(&PioInstance, &StateMachineIdx, &offset)) {
    return false;
  }

  encoder2_program_init(PioInstance, StateMachineIdx, offset, pin);

  return true;
}

/****************************************************************
*
*  Encoder::update()
*     Get the latest encoder period(s) from the PIO if available
*     store the latest, and updated the tick count.
*  
*  Returns number of samples that were retrieved.
*  
*****************************************************************/

int Encoder::update() {
  if(!PioInstance)
    return 0;

  int found = 0;
  uint raw_rx_fifo = period;
  bool prev_direction;
  bool direction = period & 1;
  while(!pio_sm_is_rx_fifo_empty(PioInstance, StateMachineIdx)) {
    prev_direction = direction;
    raw_rx_fifo = pio_sm_get_blocking(PioInstance, StateMachineIdx);
    direction = raw_rx_fifo & 1;
    count += direction ? 1 : -1;
    found++;
  }

  if(found) {
    last_sample_time = millis();
    period = (prev_direction == direction) ? raw_rx_fifo : INT_MAX;
  }

  return found;
}


/****************************************************************
*
*  Encoder::getPeriod()
*     Return the period calculated by the PIO in the following format:
*     31                           1   0
*    |  Period in 12-cycle ticks    | dir |
*  
*    This is the same format return by the PIO.
*    (Note: PIO calculated value may be overidden if a relatively
*     large amount of time has passed since the last sample from the PIO).
*
*****************************************************************/

static constexpr uint ONE_MINUTE = 60000;
static constexpr uint TICKS_PER_MS = F_CPU / (1000 * encoder2_CYCLES_PER_COUNT);

uint Encoder::getPeriod() {
  unsigned long time = millis()-last_sample_time;
  //If the time since the last sample is more than the last calculated period,
  //motor has slowed down significantly.  It may be a bit before we get another tick (motor may be stopped);
  //use the actual time passed instead of the period calculated by the PIO to minimize
  //the delay until the robot gets updated speed.

  if(time > ONE_MINUTE)  //If it's been a minute, assume motor is stopped.
    return UINT_MAX;     //Return max period; forward direction (direction shouldn't matter).

  //Convert PIO calculated period to ms
  uint period_ms = (period >> 1) / TICKS_PER_MS;

  //If PIO value is correct, the amount of time that has passed since the last sample
  //should be less than or equal to period calculated by the PIO.
  //Otherwise, motor may have slowed way down; use the time passed for the current period length.
  if(time > period_ms) {
    uint adjusted_period = time * TICKS_PER_MS;
    return (adjusted_period << 1) | (period & 1);
  }

  //We've received a sample within a reasonable period of time.  Return the PIO calculated value.
  return period;
}

/****************************************************************
*
*  Encoder::getCount()
*     Return the number of encoder ticks since the robot started.
*
*****************************************************************/
int Encoder::getCount() {
  return count;
}
} //namespace XRP