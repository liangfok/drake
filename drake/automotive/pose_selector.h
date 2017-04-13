#pragma once

#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "drake/automotive/lane_direction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/rendering/pose_bundle.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace automotive {
namespace pose_selector {

/// Contains the position of the vehicle with respect to a lane in a road, along
/// with its velocity vector in the world frame.
template <typename T>
struct RoadOdometry {
  /// Default constructor.
  RoadOdometry() = default;
  /// Fully-parameterized constructor.
  RoadOdometry(const maliput::api::RoadPosition& road_position,
               const systems::rendering::FrameVelocity<T>& frame_velocity)
      : lane(road_position.lane), pos(road_position.pos), vel(frame_velocity) {}

  const maliput::api::Lane* lane{};
  maliput::api::LanePosition pos{};
  systems::rendering::FrameVelocity<T> vel{};  // TODO(jadecastro): Make this
                                               // IsoLaneVelocity, or a
                                               // body-frame velocity?
};

/// Specifies whether to assess the cars ahead or behind the ego car at its
/// current orientation with respect to its lane.
enum class WhichSide {kAhead = 0, kBehind = 1};

/// Returns the leading and trailing cars that have closest `s`-coordinates in a
/// given @p traffic_lane to an ego car as if the ego car had moved along the
/// `r`-direction into @p traffic_lane at its current `s`-position.  The ego
/// car's pose @ego_pose and the poses of the traffic cars (@p traffic_poses)
/// are provided.  If @p traffic_lane is `nullptr`, the ego car's current lane
/// is used (this is derived from a call to CalcRoadPosition).  If no
/// leading/trailing cars are seen within @p traffic_lane, car `s`-positions are
/// taken to be at infinite distances away from the ego car.
///
/// The return values are a pair of leading/trailing RoadOdometries. Note that
/// when no car is detected in front of (resp. behind) the ego car, the
/// respective RoadPosition will contain an `s`-value of positive
/// (resp. negative) infinity (`std::numeric_limits<double>::infinity()`).  Any
/// traffic poses that are redunant with `ego_pose` (i.e. have the same
/// RoadPosition as the ego car) are discarded.
///
/// @p max_scanning_distance is the maximum distance to be scanned forward from
/// the ego pose before declaring the car is at infinite distance ahead
/// (resp. behind).
///
/// N.B. When comparing across lanes, it is assumed that @p road is configured
/// such that a comparison between the `s`-positions of any two cars on the road
/// is meaningful.  For instance, if car A is at `s = 10 m` in lane 0's frame
/// and car B is at `s = 0 m` in lane 1's frame then, if car A moved into lane
/// 1, it would be 10 meters ahead of car B.  Only straight multi-lane roads are
/// supported presently.
///
/// The road network is required to have default branches set.
///
/// Assumes that there is only one default branch ahead. Selects the FIRST
/// ENCOUNTERED default branch when scanning in the backward direction.
///
/// TODO(jadecastro): Support road networks containing multi-lane segments
/// (#4934).
///
/// TODO(jadecastro): Support vehicles traveling in the negative-`s`-direction
/// in a given Lane.
const std::pair<const RoadOdometry<double>, const RoadOdometry<double>>
    FindClosestPair(
        const maliput::api::Lane* const lane,
        const systems::rendering::PoseVector<double>& ego_pose,
        const systems::rendering::FrameVelocity<double>& ego_velocity,
        const systems::rendering::PoseBundle<double>& traffic_poses,
        double scan_ahead_distance);

/// Same as FindClosestPair() except that: (1) it only considers the ego car's
/// lane and (2) it returns a single the RoadOdometry of either the vehicle
/// ahead (kAhead) or behind (kBehind).  Default is kAhead.
///
///  Note that when no car is detected in front of the ego car, the returned
///  RoadOdometry will contain an `s`-value of
///  `std::numeric_limits<double>::infinity()`.
const RoadOdometry<double> FindSingleClosestPose(
    const maliput::api::Lane* const lane,
    const systems::rendering::PoseVector<double>& ego_pose,
    const systems::rendering::FrameVelocity<double>& ego_velocity,
    const systems::rendering::PoseBundle<double>& traffic_poses,
    double scan_ahead_distance, WhichSide side = WhichSide::kAhead);

/// Computes the RoadPosition for a car whose @p pose is located on a given @p
/// road.
const maliput::api::RoadPosition CalcRoadPosition(
    const maliput::api::RoadGeometry& road, const Isometry3<double>& pose);

// Extracts the vehicle's `s`-direction velocity based on its RoadOdometry @p
// road_odom.  Assumes the road has zero elevation and superelevation.
//
// TODO(jadecastro): Generalize to three-dimensional rotations.
const maliput::api::IsoLaneVelocity GetIsoLaneVelocity(
    const maliput::api::RoadPosition& road_position,
    const systems::rendering::FrameVelocity<double>& velocity);

// 
const LaneDirection get_lane_direction(
    const maliput::api::IsoLaneVelocity& iso_velocity,
    const maliput::api::Lane* lane);

// 
bool IsWithinLane(const maliput::api::GeoPosition& geo_position,
                  const maliput::api::Lane* lane);

// 
void get_next_lane(LaneDirection* lane_direction);

}  // namespace pose_selector
}  // namespace automotive
}  // namespace drake
