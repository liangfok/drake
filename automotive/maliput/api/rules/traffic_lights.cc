#include "drake/automotive/maliput/api/rules/traffic_lights.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {

std::map<BulbColor, const char*> BulbColorMapper() {
  return {
      {BulbColor::kRed, "Red"},
      {BulbColor::kYellow, "Yellow"},
      {BulbColor::kGreen, "Green"},
  };
}

std::map<BulbType, const char*> BulbTypeMapper() {
  return {
      {BulbType::kRound, "Round"},
      {BulbType::kArrow, "Arrow"},
  };
}

Bulb::Bulb(const Bulb::Id& id, const GeoPosition& position_bulb_group,
           const Rotation& orientation_bulb_group, const BulbColor& color,
           const BulbType& type)
    : id_(id),
      position_bulb_group_(position_bulb_group),
      orientation_bulb_group_(orientation_bulb_group),
      color_(color),
      type_(type) {}

// BulbGroup::BulbGroup(const BulbGroup::Id& id,
//  const GeoPosition& position_traffic_light,
//  const Rotation& orientation_traffic_light, const std::vector<Bulb>& bulbs)
//     : id_(id), position_traffic_light_(position_traffic_light_),
//       orientation_traffic_light_(orientation_traffic_light),
//       bulbs_(bulbs) {}

// TrafficLight::TrafficLight(const TrafficLight::Id& id,
//  const GeoPosition& position_road_network,
//  const Rotation& orientation_road_network,
//  const std::vector<BulbGroup>& bulb_groups)
//     : id_(id), position_road_network_(position_road_network),
//       orientation_road_network_(orientation_road_network),
//       bulb_groups_(bulb_groups) {}

}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
