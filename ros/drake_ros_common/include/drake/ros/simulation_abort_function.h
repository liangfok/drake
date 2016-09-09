#pragma once

#include <cmath>

#include "ros/ros.h"
#include "rosgraph_msgs/Clock.h"

#include "drake/systems/simulation_options.h"

namespace drake {
namespace ros {

/**
 * Adds a custom stop function that (1) checks whether the simulation should
 * abort based on a call to ros::ok(), and (2) publishes a clock message so
 * other systems within ROS can be time synchronized with simulation time.
 */
void AddAbortFunction(drake::SimulationOptions* options) {
  options->should_stop = [](double sim_time) {
    return !::ros::ok();
  };
}

}  // namespace ros
}  // namespace drake
