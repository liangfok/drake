#pragma once

#include <chrono>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// Monitors system performance and publishes the statistics over LCM. This
/// system has no inputs or outputs.
///
/// @ingroup primitive_systems
class PerformanceMonitor : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PerformanceMonitor)

  /// Constructs a %PerformanceMonitor system.
  ///
  /// @param publish_period The period in wall-clock time at which statistics
  /// messages should be published. This value must be greater than zero.
  explicit PerformanceMonitor(double publish_period = 1.0);

 protected:
  /// This system does not output anything.
  void DoCalcOutput(const Context<double>& context,
      SystemOutput<double>* output) const override {}

  void DoPublish(const Context<double>& context) const override;

 private:
  mutable double publishing_period_{};
  mutable bool start_{true};
  mutable double previous_sim_time_{};
  mutable std::chrono::time_point<std::chrono::steady_clock>
      previous_wall_clock_time_{};
};

}  // namespace systems
}  // namespace drake
