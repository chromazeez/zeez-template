#pragma once
#include <cstddef>

namespace auton_selector {
  void init();          // start background task
  void next();
  void prev();
  void display();
  const char* name();   // current auton name
  void run();           // run selected auton
  bool isLocked();
  void setLocked(bool locked);

}
