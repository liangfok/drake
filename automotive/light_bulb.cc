#include "drake/automotive/light_bulb.h"

#include <stdexcept>
#include <string>

#include "drake/common/drake_throw.h"

namespace drake {
namespace automotive {

LightBulb::LightBulb(const LightBulbId& id, const Eigen::Isometry3d& world_pose)
    : id_(id), world_pose_(world_pose) {
}

void LightBulb::AddState(
    const RightOfWayRule::Id& id,
    const RightOfWayRule::DynamicState& initial_state) {
  auto result = dynamic_states_.emplace(id, initial_state);
  if (!result.second) {
     throw std::logic_error(
        "Attempted to add multiple rules with id " + id.string());
  }
}

void LightBulb::SetState(
    const RightOfWayRule::Id& id,
    const RightOfWayRule::DynamicState& state) {
  dynamic_states_.at(id) = state;
}

RightOfWayRule::DynamicState LightBulb::DoGetState(
    const RightOfWayRule::Id& id) const {
  return dynamic_states_.at(id);
}

}  // namespace automotive
}  // namespace drake
