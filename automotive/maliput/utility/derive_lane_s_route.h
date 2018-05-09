#pragma once

#include <vector>

// #include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/lane.h"
// #include "drake/automotive/maliput/api/rules/regions.h"
// #include "drake/common/drake_optional.h"

namespace drake {
namespace maliput {
namespace utility {

/// Finds and returns sequences of lanes that go from a specified start lane to
/// a specified end lane.
///
/// @param start The lane at the start of the sequence.
/// @param end The lane at the end of the sequence.
/// @param max_length The maximum length of each returned lane sequence.
/// @return A vector of LaneId sequences. A vector of length zero is returned if
/// no sequences are found.
std::vector<std::vector<const api::Lane*>> FindLaneSequences(
      const api::Lane& start,
      const api::Lane& end,
      int max_length = 100);

/// Derives and returns an api::LaneSRoute within @p road that goes from
/// @p start to @p end.
// optional<api::rules::LaneSRoute> DeriveLaneSRoute(
//     const api::RoadGeometry& road,
//     const api::RoadPosition& start,
//     const api::RoadPosition& end);

}  // namespace utility
}  // namespace maliput
}  // namespace drake
