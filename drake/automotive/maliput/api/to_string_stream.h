#pragma once

#include <sstream>
#include <string>

#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/segment.h"

namespace drake {
namespace maliput {
namespace api {

/// Streams a string representation of @p road_geometry_id into @p out.
/// Returns the resulting output stream.
std::ostream& operator<<(std::ostream& out,
    const RoadGeometryId& road_geometry_id) {
  return out << road_geometry_id.id;
}

/// Streams a string representation of @p junction_id into @p out. Returns the
/// resulting output stream.
std::ostream& operator<<(std::ostream& out, const JunctionId& junction_id) {
  return out << junction_id.id;
}

/// Streams a string representation of @p segment_id into @p out. Returns the
/// resulting output stream.
std::ostream& operator<<(std::ostream& out, const SegmentId& segment_id) {
  return out << segment_id.id;
}

/// Streams a string representation of @p lane_id into @p out. Returns the
/// resulting output stream.
std::ostream& operator<<(std::ostream& out, const LaneId& lane_id) {
  return out << lane_id.id;
}

/// Streams a string representation of @p road_geometry into @p out. Returns
/// the resulting output stream.
std::ostream& operator<<(std::ostream& out, const RoadGeometry& road_geometry) {
  return out << road_geometry.id();
}

/// Streams a string representation of @p junction into @p out. Returns the
/// resulting output stream.
std::ostream& operator<<(std::ostream& out, const Junction& junction) {
  return out << *junction.road_geometry() << "/" << junction.id();
}

/// Streams a string representation of @p segment into @p out. Returns the
/// resulting output stream.
std::ostream& operator<<(std::ostream& out, const Segment& segment) {
  // return out << "hello world segment";
  return out << *segment.junction() << "/" << segment.id();
}

/// Streams a string representation of @p lane into @p out. Returns the
/// resulting output stream.
std::ostream& operator<<(std::ostream& out, const Lane& lane) {
  return out << *lane.segment() << "/" << lane.id();
}

/// Streams a string representation of @p lane_end into @p out. Returns the
/// resulting output stream.
std::ostream& operator<<(std::ostream& out, const LaneEnd& lane_end) {
  return out << "(lane = " << *lane_end.lane << ", which = "
      << lane_end.end << ")";
}

}  // namespace api
}  // namespace maliput
}  // namespace drake
