#pragma once

#include <map>
#include <vector>

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/type_specific_identifier.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {

/// Defines the possible bulb colors.
enum class BulbColor {
  kRed = 0,
  kYellow,
  kGreen,
};

/// Maps BulbColor enums to string representations.
std::map<BulbColor, const char*> BulbColorMapper();

/// Defines the possible bulb types.
enum class BulbType {
  kRound = 0,
  kArrow,
};

/// Maps BulbType enums to string representations.
std::map<BulbType, const char*> BulbTypeMapper();

/// Models a bulb within a bulb group.
class Bulb final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Bulb);

  /// Unique identifier for a Bulb.
  using Id = TypeSpecificIdentifier<class Bulb>;

  /// Constructs a Bulb instance.
  ///
  /// @param id The bulb group's unique ID.
  ///
  /// @param position_bulb_group The linear offset of this bulb's frame relative
  /// to the frame of the bulb group that contains it. The origin of this bulb's
  /// frame should approximate the bulb group's CoM.
  ///
  /// @param orientation_bulb_group The rotational offset of this bulb's frame
  /// relative to the frame of the bulb group that contains it. The +Z axis
  /// should align with the bulb's "up" direction, and the +X axis should point
  /// in the direction that the bulb is facing. If the bulb type is an arrow,
  /// the +Y axis should point in the same direction as the arrow. Otherwise,
  /// there is no restriction on the +Y axis.
  ///
  /// @param color The color of this bulb.
  ///
  /// @param type The type of this bulb.
  Bulb(const Id& id, const GeoPosition& position_bulb_group,
       const Rotation& orientation_bulb_group, const BulbColor& color,
       const BulbType& type);

  /// Returns this Bulb instance's unique identifier.
  const Id& id() const { return id_; }

  /// Returns the linear offset of this bulb's frame relative to the frame of
  /// the bulb group that contains it.
  const GeoPosition& position_bulb_group() const {
    return position_bulb_group_;
  }

  /// Returns the rotational offset of this bulb's frame relative to the frame
  /// of the bulb group that contains it.
  const Rotation& orientation_bulb_group() const {
    return orientation_bulb_group_;
  }

  /// Returns the color of this bulb.
  const BulbColor& color() const { return color_; }

  /// Returns the Bulb instances contained within this Bulb.
  const BulbType& type() const { return type_; }

 private:
  Id id_;
  GeoPosition position_bulb_group_;
  Rotation orientation_bulb_group_;
  BulbColor color_ = BulbColor::kRed;
  BulbType type_ = BulbType::kRound;
};

// /// Models a group of bulbs within a TrafficLight. All bulbs in a BulbGroup
// /// share the same orientation.
// class BulbGroup final {
// public:
//   DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BulbGroup);

//   /// Unique identifier for a BulbGroup.
//   using Id = TypeSpecificIdentifier<BulbGroup>;

//   /// Constructs a BulbGroup instance.
//   ///
//   /// @param id The bulb group's unique ID.
//   ///
//   /// @param position_traffic_light The linear offset of this bulb group's frame
//   /// relative to the frame of the traffic light that contains it. The origin of
//   /// this bulb group's frame should approximate the bulb group's CoM.
//   ///
//   /// @param orientation_traffic_light The rotational offset of this bulb
//   /// group's frame relative to the frame of the traffic light that contains it.
//   /// The +Z axis should align with the bulb group's "up" direction, and the +X
//   /// axis should point in the direction that the bulb group is facing.
//   ///
//   /// @param bulbs The bulbs that are part of this BulbGroup.
//   BulbGroup(const Id& id, const GeoPosition& position_traffic_light,
//             const Rotation& orientation_traffic_light,
//             const std::vector<Bulb>& bulbs);

//   /// Returns this BulbGroup instance's unique identifier.
//   const Id& id() const { return id_; }

//   /// Returns the Bulb instances contained within this Bulb.
//   const std::vector<Bulb>& bulbs() const { return bulbs_; }

//  private:
//   const Id id_;
//   const GeoPosition position_traffic_light_;
//   const Rotation orientation_traffic_light_;
//   const std::vector<Bulb> bulbs_;
// };

// /// Models a traffic light. A traffic light is a physical signaling device
// /// frequently located at road intersections. It contains one or more groups of
// /// light bulbs with varying colors and shapes. The lighting patterns of the
// /// bulbs signify right-of-way information to the agents (i.e., vehicles,
// /// bicyclists, pedestrians, etc.) navigating the intersection. Typically, an
// /// intersection will be managed by multiple traffic lights.
// class TrafficLight final {
//  public:
//   DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TrafficLight);

//   /// Unique identifier for a traffic light.
//   using Id = TypeSpecificIdentifier<TrafficLight>;

//   /// Constructs a TrafficLight instance.
//   ///
//   /// @param id The traffic light's unique ID.
//   ///
//   /// @param position_road_network The linear offset of the traffic light's
//   /// frame relative to the road network's frame. The traffic light frame's
//   /// origin should approximate the traffic light's CoM.
//   ///
//   /// @param orientation_road_network The rotational offset of the traffic
//   /// light's frame relative to the road network's frame. The traffic light's
//   /// frame's +Z axis points in the traffic light's "up" direction. No
//   /// constraints are placed on the orientations of the +X and +Z axes. However,
//   /// it's recommended that they correspond, if possible, to the orientations of
//   /// the bulb group frames within this traffic light.
//   ///
//   /// @param bulb_groups The bulb groups that are part of this traffic light.
//   TrafficLight(const Id& id, const GeoPosition& position_road_network,
//                const Rotation& orientation_road_network,
//                const std::vector<BulbGroup>& bulb_groups);

//   /// Returns this traffic light's unique identifier.
//   const Id& id() const { return id_; }

//   /// Returns this traffic light's frame's position within the road network's
//   /// frame.
//   const GeoPosition& position_road_network() const {
//     return position_road_network_;
//   }

//   const Rotation& orientation_road_network() const {
//     return orientation_road_network_;
//   }

//   /// Returns the bulb groups contained within this traffic light.
//   const std::vector<BulbGroup>& bulb_groups() const { return bulb_groups_; }

//  private:
//   const Id id_;
//   const GeoPosition position_road_network_;
//   const Rotation orientation_road_network_;
//   const std::vector<BulbGroup> bulb_groups_;
// };

}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
