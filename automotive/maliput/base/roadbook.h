#pragma once

#include <memory>

#include "drake/automotive/maliput/api/rules/regions.h"
#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/automotive/maliput/api/rules/speed_limit_rule.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace maliput {

/// An abstract interface for querying the static properties of the road. This
/// includes the rules of the road (e.g., RightOfWayRule), the ways these rules
/// are organized (e.g., RightOfWayPhase and RightOfWayPhaseRing), and their
/// object-level manifestations (e.g., TrafficLight). Note that this class only
/// contains <b>static</b> information that is determined prior to the beginning
/// of a simulation. Associated dynamic state is provided by other classes
/// (e.g., RightOfWayPhaseProvider and RightOfWayStateProvider).
class Roadbook {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Roadbook);

  Roadbook(std::unique_ptr<RoadRulebook> rulebook,
           std::unique_ptr<RightOfWayPhaseBook> ringbook,
           ...);

  virtual ~Roadbook() = default;

  RightOfWayStateProvider::Result
  GetRightOfWayRuleStates(std::vector<LaneSRange> ranges);

  std::unsorted_map<Bulb::Id, BulbState>
  GetBulbStates(std::vector<LaneSRange> ranges);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace maliput
}  // namespace drake
