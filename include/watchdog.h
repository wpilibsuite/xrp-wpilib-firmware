#pragma once

#include <string>

#define DEFAULT_WATCHDOG_TIMEOUT 1000

namespace xrp {

class Watchdog {
  public:
    Watchdog(std::string name) :
        _wdTimeout(DEFAULT_WATCHDOG_TIMEOUT),
        _name(name) {}

    Watchdog(std::string name, unsigned long timeout) :
        _wdTimeout(timeout),
        _name(name) {}

    void feed();
    bool satisfied();
    void setTimeout(unsigned long timeout);


  private:
    unsigned long _lastFeedTime;
    unsigned long _wdTimeout;

    bool _lastSatisfiedState;

    std::string _name;
};

} // namespace xrp