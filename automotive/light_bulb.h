#pragma once

// #include <unordered_map>
#include <vector>

// #include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace automotive {

/// Persistent identifier for a Lane element.
using LightBulbId = TypeSpecificIdentifier<class Lane>;

/// Describes a light bulb in the world. This includes information that
class LightBulb {
 public:
  /// The possible light bulb colors.
  enum class Color {
    kGreen,
    kYellow,
    kRed,
    kOff,
    kUnknown,
  };

  /// The possible light bulb shapes.
  enum class Shape {
    kCircle,
    kArrow,
    kX,
    kPedestrianStop,
    kPedestrianWalk,
  };

  class Type {
    Type();


  }

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LightBulb)

  LightBulb(const LightBulbId& id, const Eigen::Isometry3d& world_pose,
            const std::vector<Color>& light_bulb_color,
            const std::vector<Type>&);

  ~LightBulb() final = default;

  /// Returns the ID of this light bulb. The ID remains constant for the life of
  /// this object.
  const LightBulbId& id() const;

  /// Returns the pose of this light bulb in the world frame. Note that the pose
  /// is static and thus cannot capture variations in the light bulb's pose
  /// over time, which might happen if the bulb is in a traffic light mounted on
  /// on a cable and blowing in the wind. This was intentional since processes
  /// operating at the symbolic level are not interested in the small movement
  /// details of the light bulb.
  const Eigen::Isometry3d& pose_world() const;

  /// Returns the possible colors that this light bulb can take on.
  const std::vector<BulbColor>& possible_colors() const;

  /// Returns the possible types that this light bulb can take on.
  const std::vector<BulbType>& possible_types() const;

 private:

  const LightBulbId id_;
  const std::vector<LightBulbColor> possible_colors_;
  const std::vector<BulbType> possible_types_;
};

}  // namespace automotive
}  // namespace drake
