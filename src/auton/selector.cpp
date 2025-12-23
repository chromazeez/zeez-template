#include "auton/selector.hpp"
#include "auton/routines.hpp"
#include "subsystems/devices.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include "pros/misc.hpp"
#include <atomic>


namespace {
  struct AutonEntry {
    const char* name;
    void (*fn)();
  };

  AutonEntry autos[] = {
    {"Do Nothing", auton::doNothing},
    {"Skills",     auton::skills},
    {"Left Rush",  auton::leftRush},
    {"Right Safe", auton::rightSafe},
  };

  constexpr int AUTO_COUNT = sizeof(autos) / sizeof(autos[0]);
  std::atomic<int> index{0};
  std::atomic<bool> locked{false};

  bool risingEdge(bool current, bool& last) {
    bool pressed = current && !last;
    last = current;
    return pressed;
  }

  void selectorTask(void*) {
    bool lastL1=false, lastL2=false, lastX=false;
    while (true) {
      bool L1 = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
      bool L2 = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
      bool X  = master.get_digital(pros::E_CONTROLLER_DIGITAL_X);

      if (risingEdge(X, lastX)) {
        locked.store(!locked.load());
        auton_selector::display();
      }

      if (!locked.load()) {
        if (risingEdge(L1, lastL1)) {
          int i = index.load();
          i = (i + 1) % AUTO_COUNT;
          index.store(i);
          auton_selector::display();
        }
        if (risingEdge(L2, lastL2)) {
          int i = index.load();
          i = (i - 1 + AUTO_COUNT) % AUTO_COUNT;
          index.store(i);
          auton_selector::display();
        }
      } else {
        // still update edge tracking
        risingEdge(L1, lastL1);
        risingEdge(L2, lastL2);
      }

      pros::delay(20);
    }
  }
}

namespace auton_selector {

void init() {
  display();
  pros::Task(selectorTask, nullptr, "Auton Selector");
}

void next() {
  int i = index.load();
  i = (i + 1) % AUTO_COUNT;
  index.store(i);
  display();
}

void prev() {
  int i = index.load();
  i = (i - 1 + AUTO_COUNT) % AUTO_COUNT;
  index.store(i);
  display();
}

const char* name() {
  return autos[index.load()].name;
}

void display() {
  master.clear();
  master.print(0, 0, "Auton:%s", locked.load() ? " LOCK" : "");
  master.print(1, 0, "%s", name());

  pros::lcd::set_text(0, locked.load() ? "Selected (LOCKED):" : "Selected Auton:");
  pros::lcd::set_text(1, name());
}

bool isLocked() { return locked.load(); }
void setLocked(bool v) { locked.store(v); display(); }


void run() {
  autos[index.load()].fn();
}

}
