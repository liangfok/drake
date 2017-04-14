#pragma once

#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "drake/automotive/lane_direction.h"
#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/rendering/pose_bundle.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace automotive {

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
enum class WhichSide { kAhead = 0, kBehind = 1 };

///
// TODO(jadecastro): There is some repetitive computations here, making this
// computation O(m^2 * n), where m is the total number of lanes in the road and
// n is the number of vehicles in the road.
class PoseSelector {
 public:
  typedef typename std::pair<const double, const maliput::api::LaneEnd>
      LaneEndDistance;

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PoseSelector)
  PoseSelector() = delete;

  /// Returns the leading and trailing cars that have closest `s`-coordinates in
  /// a given @p traffic_lane to an ego car as if the ego car had moved along
  /// the `r`-direction into @p traffic_lane at its current `s`-position.  The
  /// ego car's pose @ego_pose and the poses of the traffic cars (@p
  /// traffic_poses) are provided.  If @p traffic_lane is `nullptr`, the ego
  /// car's current lane is used (this is derived from a call to
  /// CalcRoadPosition).  If no leading/trailing cars are seen within @p
  /// traffic_lane, car `s`-positions are taken to be at infinite distances away
  /// from the ego car.
  ///
  /// @return A pair of leading/trailing RoadOdometries. Note that
  /// when no car is detected in front of (resp. behind) the ego car, the
  /// respective RoadPosition will contain an `s`-value of positive
  /// (resp. negative) infinity (`std::numeric_limits<double>::infinity()`).
  /// Any traffic poses that are redunant with `ego_pose` (i.e. have the same
  /// RoadPosition as the ego car) are discarded.
  ///
  /// @p max_scanning_distance is the maximum distance to be scanned forward
  /// from the ego pose before declaring the car is at infinite distance ahead
  /// (resp. behind).
  ///
  /// N.B. When comparing across lanes, it is assumed that @p road is configured
  /// such that a comparison between the `s`-positions of any two cars on the
  /// road is meaningful.  For instance, if car A is at `s = 10 m` in lane 0's
  /// frame and car B is at `s = 0 m` in lane 1's frame then, if car A moved
  /// into lane 1, it would be 10 meters ahead of car B.  Only straight
  /// multi-lane roads are supported presently.
  ///
  /// The road network is required to have default branches set.
  static const std::pair<const RoadOdometry<double>, const RoadOdometry<double>>
  FindClosestPair(const maliput::api::Lane* const lane,
                  const systems::rendering::PoseVector<double>& ego_pose,
                  const systems::rendering::FrameVelocity<double>& ego_velocity,
                  const systems::rendering::PoseBundle<double>& traffic_poses,
                  double scan_ahead_distance);

  ///
  ///
  // TODO(jadecastro): Add support for vehicles behind.
  const RoadOdometry<double> FindSingleClosestIncludingBranches(
      const maliput::api::RoadGeometry& road,
      const systems::rendering::PoseVector<double>& ego_pose,
      const systems::rendering::FrameVelocity<double>& ego_velocity,
      const systems::rendering::PoseBundle<double>& traffic_poses,
      double scan_ahead_distance);

  /// Same as FindClosestPair() except that: (1) it only considers the ego car's
  /// lane and (2) it returns a single the RoadOdometry of either the vehicle
  /// ahead (kAhead) or behind (kBehind).  Default is kAhead.
  ///
  /// Note that when no car is detected in front of the ego car, the returned
  /// RoadOdometry will contain an `s`-value of
  /// `std::numeric_limits<double>::infinity()`.
  static const RoadOdometry<double> FindSingleClosestPose(
      const maliput::api::Lane* const lane,
      const systems::rendering::PoseVector<double>& ego_pose,
      const systems::rendering::FrameVelocity<double>& ego_velocity,
      const systems::rendering::PoseBundle<double>& traffic_poses,
      double scan_ahead_distance, const WhichSide side);

  ///
  static const RoadOdometry<double> FindSingleClosestAcrossBranches(
      const maliput::api::RoadGeometry& road,
      const systems::rendering::PoseBundle<double>& traffic_poses,
      double ego_sigma_v, const std::vector<LaneEndDistance>& branches,
      double prior_headway_distance);

  ///
  ///
  /// @return A LaneEndDistance, a vector of pairs with each element containing
  /// the distance along the s-coordinate from the ego vehicle to the branch and
  /// the branch itself (LaneEnd).
  static const std::vector<LaneEndDistance> FindBranches(
      const maliput::api::Lane* const lane,
      const systems::rendering::PoseVector<double>& ego_pose,
      const systems::rendering::FrameVelocity<double>& ego_velocity,
      double scan_ahead_distance);

  /// Extracts the vehicle's `s`-direction velocity based on its RoadOdometry @p
  /// road_odom.  Assumes the road has zero elevation and superelevation.
  ///
  // TODO(jadecastro): Generalize to three-dimensional rotations.
  static const maliput::api::IsoLaneVelocity GetIsoLaneVelocity(
      const maliput::api::RoadPosition& road_position,
      const systems::rendering::FrameVelocity<double>& velocity);

 private:
  //
  static const LaneDirection get_lane_direction(
      const maliput::api::IsoLaneVelocity& iso_velocity,
      const maliput::api::Lane* lane);

  //
  static bool IsWithinLane(const maliput::api::GeoPosition& geo_position,
                           const maliput::api::Lane* lane);

  //
  static bool IsAdjacent(const maliput::api::LaneEnd& lane_end0,
                         const maliput::api::LaneEnd& lane_end1);

  // Mutates lane_direction in the sequence; lane_direction contains a null
  // lane pointer if no default branch is found.
  static std::unique_ptr<maliput::api::LaneEnd> get_default_ongoing_lane(
      LaneDirection* lane_direction);

  //
  static const maliput::api::LaneEndSet* GetIncomingLaneEnds(
      const LaneDirection& lane_direction);

  // Assign the with default positions extending
  // to, respectively, positive and negative infinity and with zero velocities.
  static const RoadOdometry<double> set_default_odometry(
      const maliput::api::Lane* const Lane, const WhichSide side);
};

}  // namespace automotive
}  // namespace drake
