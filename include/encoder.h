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

#pragma once
#include <Arduino.h>
#include <limits>
#include <vector>
#include "encoder2.pio.h"

namespace xrp {

class Encoder {
public:

  Encoder() : period_queue(samples_to_average()) {}

/****************************************************************
*
*  Encoder::init()
*     Initialize PIO program that measures encoder period.
*
*  Returns true on sucess.
*  
*****************************************************************/
  bool init(const int pin);

/****************************************************************
*
*  Encoder::enable()
*     Enable the encoder.
*
*  
*****************************************************************/
  void enable();

/****************************************************************
*
*  Encoder::disable()
*     Disable the encoder.
*
*  
*****************************************************************/
  void disable();

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
*  Encoder::setSamplesToAverage()
*     Set the number of Encoder period samples to average together to
*     calculate period.
*
*****************************************************************/

void setSamplesToAverage(const int n);

/****************************************************************
*
*  Encoder::getPeriod()
*     Return the period calculated by the PIO in the following format:
*     31                           1   0
*    |  Period in 16-cycle ticks    | dir |
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
  int getCount() const;

/****************************************************************
*
*  Encoder::getDivisor()
*     Get divisor for encoder period in ticks per second.
*  
*****************************************************************/

 static constexpr uint getDivisor() {
    return F_CPU / encoder2_CYCLES_PER_COUNT;
 }

private:
  uint period = 0;
  int period_fraction = 0;
  uint saved_period = UINT_MAX;
  bool direction = true;
  bool saved_direction = true;
  int samples_to_average_shift = 3;
  int sample_count = 0;
  int sample_index = 0;
  std::vector<uint> period_queue;
  uint count = 0;
  int StateMachineIdx = -1;
  unsigned long last_sample_time = 0;
  PIO PioInstance = nullptr;
  int pin = 0;
  int offset = -1;
  bool enabled = false;
  void clearPeriodQueue();
  uint getFraction(const uint count) const;
  uint getWholeNumber(const uint count) const;
  constexpr uint samples_to_average() const {
    return 1 << samples_to_average_shift;
  }
}; 

}
