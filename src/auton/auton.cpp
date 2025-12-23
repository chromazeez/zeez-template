#include "auton/auton.hpp"
#include "auton/selector.hpp"

namespace auton {
  void initSelector() { auton_selector::init(); }
  void runSelected()  { auton_selector::run(); }
}
