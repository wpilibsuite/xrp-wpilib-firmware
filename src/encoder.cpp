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

bool Encoder::init(const int pin) {
  last_sample_time = millis();

  this->pin = pin;

  if (!encoderProgram.prepare(&PioInstance, &StateMachineIdx, &offset, pin, 2)) {
    return false;
  }

  return true;
}

/****************************************************************
*
*  Encoder::enable()
*     Enable the encoder.
*
*
*****************************************************************/

void Encoder::enable() {
  if(PioInstance) {
    encoder2_program_init(PioInstance, StateMachineIdx, offset, pin);
    enabled = true;
  }
}

/****************************************************************
*
*  Encoder::disable()
*     Disable the encoder.
*
*
*****************************************************************/

void Encoder::disable() {
  if(PioInstance)
    pio_sm_set_enabled(PioInstance, StateMachineIdx, false);

  enabled = false;
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

uint Encoder::getFraction(const uint count) const {
  return count & (samples_to_average() - 1);
}

uint Encoder::getWholeNumber(const uint count) const {
  return count >> samples_to_average_shift;
}

int Encoder::update() {
  if(!enabled || !PioInstance)
    return 0;

  int found = 0;
  uint raw_rx_fifo = period;
  uint prev_raw_rx_fifo;
  bool prev_direction;

  //Read any Encoder periods calculated by the PIO from its input FIFO.

  while(!pio_sm_is_rx_fifo_empty(PioInstance, StateMachineIdx)) {
    prev_raw_rx_fifo = raw_rx_fifo;
    prev_direction = direction;

    raw_rx_fifo = pio_sm_get_blocking(PioInstance, StateMachineIdx);
    direction = raw_rx_fifo & 1;
    raw_rx_fifo >>= 1;
    count += direction ? 1 : -1;
    found++;

    //When direction changes, clear the queue
    if(direction != prev_direction) {
      clearPeriodQueue();
    }

    //When queue is full, remove oldest sample from queue
    if(sample_count == samples_to_average()) {
      period_fraction -= getFraction(period_queue[sample_index]);
      if(period_fraction < 0) {
        period -= 1;
        period_fraction += samples_to_average();
      }
      period -= getWholeNumber(period_queue[sample_index]);
      --sample_count;
    }

    //Add new sample to queue
    period += getWholeNumber(raw_rx_fifo);
    period_fraction += getFraction(raw_rx_fifo);
    if(period_fraction >= samples_to_average()) {
      ++period;
      period_fraction = getFraction(period_fraction);
    }
    period_queue[sample_index++] = raw_rx_fifo;
    ++sample_count;
    if(sample_index == samples_to_average())
      sample_index = 0;
  }

  if(found) {
    last_sample_time = millis();
  }

  return found;
}

/****************************************************************
*
*  Encoder::setSamplesToAverage()
*     Set the number of Encoder period samples to average together to
*     calculate period.
*
*****************************************************************/

void Encoder::clearPeriodQueue() {
  sample_count = 0;
  sample_index = 0;
  period = 0;
  period_fraction = 0;
}

void Encoder::setSamplesToAverage(const int n) {
  samples_to_average_shift = 1;
  while(samples_to_average() < n) {
    ++samples_to_average_shift;
  }

  clearPeriodQueue();
  period_queue.resize(samples_to_average());
}

/****************************************************************
*
*  Encoder::getPeriod()
*     Return the period calculated by the PIO in the following format:
*     31                           1   0
*    |  Period in 16-cycle ticks    | dir |
*
*    This is the same format return by the PIO.
*    (Note: PIO calculated value may be overidden if a relatively
*     large amount of time has passed since the last sample from the PIO).
*
*****************************************************************/

static constexpr uint ONE_MINUTE = 60000;
static constexpr uint TICKS_PER_MS = F_CPU / (1000 * encoder2_CYCLES_PER_COUNT);

uint Encoder::getPeriod() {
  if(!enabled)
    return UINT_MAX;

  unsigned long time = millis()-last_sample_time;

  //Only use period calculated by PIO if we've collected enough samples
  //to average together (period_queue is full).
  if(sample_count == samples_to_average()) {
    saved_direction = direction;
    saved_period = period;
  }

  //If the time since the last sample is more than the last calculated period,
  //motor has slowed down significantly.  It may be a bit before we get another tick (motor may be stopped);
  //use the actual time passed instead of the period calculated by the PIO to minimize
  //the delay until the robot gets updated speed.

  if(time > ONE_MINUTE)  //If it's been a minute, assume motor is stopped.
    return UINT_MAX;     //Return max period; forward direction (direction shouldn't matter).

  //Convert PIO calculated period to ms (rounded up)
  uint period_ms = saved_period / TICKS_PER_MS + 1;

  //If PIO value is correct, the amount of time that has passed since the last sample
  //should be less than or equal to period calculated by the PIO.
  //Otherwise, motor may have slowed way down; use the time passed for the current period length.

  //If time since last sample is significantly longer than the last period returned by the PIO,
  //use the time passed instead.

  if(time >= period_ms * 16) {
    uint adjusted_period = time * TICKS_PER_MS;
    return (adjusted_period << 1) | saved_direction;
  }

  //We've received a sample within a reasonable period of time.  Return the PIO calculated value.
  return (saved_period << 1) | saved_direction;
}

/****************************************************************
*
*  Encoder::getCount()
*     Return the number of encoder ticks since the robot started.
*
*****************************************************************/
int Encoder::getCount() const {
  return count;
}
} //namespace XRP

