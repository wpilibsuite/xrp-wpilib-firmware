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