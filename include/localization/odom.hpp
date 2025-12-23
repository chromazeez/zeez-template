#pragma once
#include "localization/pose.hpp"
#include "drive/drive.hpp"
#include <atomic>

class Odom {
public:
  explicit Odom(Drive& drive);

  void start();           // starts background task
  void reset(Pose p);     // set pose + tare baselines
  Pose get() const;       // current pose snapshot

private:
  void loop();            // task loop

  Drive& drive;
  std::atomic<double> x{0.0};
  std::atomic<double> y{0.0};
  std::atomic<double> theta{0.0}; // radians

  double lastLeftDeg{0.0};
  double lastRightDeg{0.0};
};
