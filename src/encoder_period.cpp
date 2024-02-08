#include "encoder_period.h"
#include "encoder_period.pio.h"
#include "robot.h"

namespace xrp {

static PIOProgram _encoder_periodPgm(&encoder_period_program);

/****************************************************************
*
*  EncoderPeriod::init()
*     Initialize PIO program that measures encoder period.
*
*  Returns true on sucess.
*  
*****************************************************************/

bool EncoderPeriod::init(int pin) {
  last_sample_time = millis();
  return init_pio(PioInstance, StateMachineIdx, _encoder_periodPgm, encoder_period_program_init, pin);
}

/****************************************************************
*
*  EncoderPeriod::update()
*     Get the latest encoder period from the PIO if available
*     and store it.
*  
*  Returns number of samples that were stored (0 or 1);
*  
*****************************************************************/

int EncoderPeriod::update() {
  if(!PioInstance)
    return 0;

  bool found = false;
  uint raw_rx_fifo = 0;
  while(!pio_sm_is_rx_fifo_empty(PioInstance, StateMachineIdx)) {
    raw_rx_fifo = pio_sm_get_blocking(PioInstance, StateMachineIdx);
    found = true;
  }

  if(found) {
    last_sample_time = millis();
    value = raw_rx_fifo;
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
static constexpr uint TICKS_PER_MS = F_CPU / (1000 * encoder_period_CYCLES_PER_COUNT);

uint EncoderPeriod::getPeriod() {
  unsigned long time = millis()-last_sample_time;
  //If the time since the last sample is more than the last calculated period,
  //motor has slowed down significantly.  It may be a bit before we get another tick (motor may be stopped);
  //use the actual time passed instead of the period calculated by the PIO to minimize
  //the delay until the robot gets updated speed.

  if(time > ONE_MINUTE)  //If it's been a minute, assume motor is stopped.
    return UINT_MAX;     //Return max period.

  //Convert PIO calculated period to ms
  uint period_ms = (value >> 1) / TICKS_PER_MS;

  //If PIO value is correct, the amount of time that has passed since the last sample
  //should be less than or equal to period calculated by the PIO.
  //Otherwise, motor may have slowed way down; use the time passed for the current period length.
  if(time > period_ms) {
    uint adjusted_period = time * TICKS_PER_MS;
    return ((adjusted_period-1) << 1) & (value & 1);
  }

  //We've received a sample within a reasonable period of time.  Return the PIO calculated value.
  return value;
}

}