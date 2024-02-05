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
  while(!pio_sm_is_rx_fifo_empty(PioInstance, StateMachineIdx)) {
    unsigned int raw_rx_fifo = pio_sm_get_blocking(PioInstance, StateMachineIdx);
    value = raw_rx_fifo >> 1;
    direction = raw_rx_fifo & 1;
    found = true;
  }

  if(found)
    last_sample_time = millis();

  return found;
}


/****************************************************************
*
*  EncoderPeriod::getPeriod()
*     Convert the latest period measurement of the encoder from
*     an integer count to the length of the period in seconds
*     as a double.
*  
*****************************************************************/

double EncoderPeriod::getPeriod() {
  double period = (double)value * encoder_period_CYCLES_PER_COUNT / F_CPU;
  unsigned long time = millis()-last_sample_time;
  //If the time since the last sample is more than the last calculated period,
  //motor has slowed down significantly.  It may be a bit before we get another tick (motor may be stopped);
  //use the actual time passed instead of the period calculated by the PIO to minimize
  //the delay until the robot gets updated speed.
  if(time > period * 1000)
    period = time / 1000.0;
  return (direction) ? period : -period;
}

}