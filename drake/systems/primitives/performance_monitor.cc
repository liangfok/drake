#include "drake/systems/primitives/performance_monitor.h"

#include <iomanip>  // For std::put_time.

using std::chrono::time_point;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::steady_clock;

namespace drake {
namespace systems {

PerformanceMonitor::PerformanceMonitor(double publishing_period)
    : publishing_period_(publishing_period) {
  DRAKE_THROW_UNLESS(publishing_period > 0);
}

void PerformanceMonitor::DoPublish(const Context<double>& context) const {
  if (!start_) {
    start_ = true;
    previous_sim_time_ = context.get_time();
    previous_wall_clock_time_ = steady_clock::now();
    return;
  }

  const time_point<steady_clock> current_wall_clock_time = steady_clock::now();
  auto duration = duration_cast<milliseconds>
      (current_wall_clock_time - previous_wall_clock_time_);

  if (duration.count() >= publishing_period_ * 1000) {
    const double current_sim_time = context.get_time();

    const std::time_t now_c = std::chrono::system_clock::to_time_t(
        std::chrono::system_clock::now());
    std::cout << "Current time: "
        << std::put_time(std::localtime(&now_c), "%F %T")
        << ", Sim time: " << current_sim_time << ", real-time factor: "
        << ((current_sim_time - previous_sim_time_) * 1000) / duration.count()
        << std::endl;

    previous_sim_time_ = current_sim_time;
    previous_wall_clock_time_ = current_wall_clock_time;
  }
}

}  // namespace systems
}  // namespace drake
