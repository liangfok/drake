#pragma once

#include <memory>
#include <string>

#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/base/intersection.h"

namespace drake {
namespace maliput {

/// Loads an Intersection.
///
/// @param rules_file The name of the YAML file containing the road rules.
///
/// @param road_geometry The api::RoadGeometry that contains the intersection to
/// be loaded.
///
/// @return The newly created Intersection instance.
std::unique_ptr<const Intersection> LoadIntersection(
    const std::string& rules_file, const api::RoadGeometry& road_geometry);

}  // namespace maliput
}  // namespace drake
