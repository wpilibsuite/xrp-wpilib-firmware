#include "encoder.h"
#include "encoder2.pio.h"

namespace xrp {

static PIOProgram encoderProgram(&encoder2_program);

/****************************************************************
*
*  EncoderPeriod::init()
*     Initialize PIO program that measures encoder period.
*
*  Returns true on sucess.
*  
*****************************************************************/

bool Encoder::init(int pin) {
  last_sample_time = millis();
  int offset = -1;

  if (!encoderProgram.prepare(&PioInstance, &StateMachineIdx, &offset)) {
    Serial.printf("[ENC-%u] Failed to set up program %p\n", pin);
    return false;
  }

  encoder2_program_init(PioInstance, StateMachineIdx, offset, pin);

  return true;
}

/****************************************************************
*
*  EncoderPeriod::update()
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
*  EncoderPeriod::getPeriod()
*     Return the period calculated by the PIO in the following format:
*     31                           1   0
*    |  Period in 12-cycle ticks    | dir |
*  
*    This is the same format return by the PIO.
*    (Note: PIO calculated value may be overidden if a relatively
*     large amount has passed since the last sample from the PIO).
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
*  EncoderPeriod::getCount()
*     Return the number of encoder ticks since the robot started.
*
*****************************************************************/
int Encoder::getCount() {
  return count;
}
} //namespace XRP