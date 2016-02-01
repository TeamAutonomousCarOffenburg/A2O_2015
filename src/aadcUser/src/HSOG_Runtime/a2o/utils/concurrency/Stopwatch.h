#pragma once

#include <chrono>

namespace A2O{
class Stopwatch{
  /** Simple stopwatch to messure time difference
  *
  * \author Peter Walden
  *
  */
  
public:
  Stopwatch(){};
  /** starts stopwatch if not already started */
  void start()
  {
    if(!_running) {
      _running = true;
      _timestamp = std::chrono::system_clock::now();
    }
  };
  
  /** resets stopwatch.  */
  void reset()
  {
    _running = false;
  };
  
  /** retrieve the time elapsed since start is first called in ms */
  long elapsedTimeMs()
  { 
    if(_running)
      return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - _timestamp).count();
    else
      return 0;
  };
  
private:
  std::chrono::system_clock::time_point _timestamp;
  bool _running = false;
};
}