#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/automotive/maliput/api/intersection.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/rules/road_rulebook.h"
#include "drake/automotive/maliput/base/intersection.h"

namespace drake {
namespace maliput {

/// Instantiates Intersection instances based on the specified @p road_geometry,
/// @p rulebook, and @p input document.
///
/// @param road_geometry The road geometry that includes the intersections to be
/// loaded.
///
/// @param rulebook Contains the RightOfWayRules that are referenced by the
/// Intersection instances to be loaded.
///
/// @param input The YAML Intersections document.
///
/// @return The loaded Intersection objects.
std::vector<std::unique_ptr<const Intersection>> LoadIntersections(
    const api::RoadGeometry* road_geometry,
    const api::rules::RoadRulebook* rulebook, const std::string& input);

/// Instantiates Intersection instances based on the specified @p road_geometry,
/// @p rulebook, and @p filename.
///
/// @param road_geometry The road geometry that includes the intersections to be
/// loaded.
///
/// @param rulebook Contains the RightOfWayRules that are referenced by the
/// Intersection instances to be loaded.
///
/// @param filename The YAML file that contains an Intersections document.
///
/// @return The loaded Intersection objects.
std::vector<std::unique_ptr<const Intersection>> LoadIntersectionsFromFile(
    const api::RoadGeometry* road_geometry,
    const api::rules::RoadRulebook* rulebook, const std::string& filename);

}  // namespace maliput
}  // namespace drake
