#include "drake/automotive/pose_selector.h"

#include <cmath>
#include <limits>
#include <sstream>

#include "drake/common/drake_assert.h"
#include "drake/math/quaternion.h"
#include "drake/math/roll_pitch_yaw_not_using_quaternion.h"

namespace drake {
namespace automotive {
namespace pose_selector {

using maliput::api::GeoPosition;
using maliput::api::IsoLaneVelocity;
using maliput::api::Lane;
using maliput::api::LaneEnd;
using maliput::api::LanePosition;
using maliput::api::RoadGeometry;
using maliput::api::RoadPosition;
using systems::rendering::FrameVelocity;
using systems::rendering::PoseBundle;
using systems::rendering::PoseVector;

const std::pair<const RoadOdometry<double>, const RoadOdometry<double>>
    FindClosestPair(const Lane* const lane, const PoseVector<double>& ego_pose,
                    const FrameVelocity<double>& ego_velocity,
                    const PoseBundle<double>& traffic_poses,
                    double scan_ahead_distance) {
  RoadOdometry<double> result_leading =
      FindSingleClosestPose(lane, ego_pose, ego_velocity, traffic_poses,
                            scan_ahead_distance, WhichSide::kAhead);
  RoadOdometry<double> result_trailing =
      FindSingleClosestPose(lane, ego_pose, ego_velocity, traffic_poses,
                            scan_ahead_distance, WhichSide::kBehind);
  return std::make_pair(result_leading, result_trailing);
}

const RoadOdometry<double> FindSingleClosestPose(
    const Lane* const lane, const PoseVector<double>& ego_pose,
    const FrameVelocity<double>& ego_velocity,
    const PoseBundle<double>& traffic_poses, double scan_ahead_distance,
    WhichSide side) {
  // Assign the leading and trailing vehicles with default positions extending
  // to, respectively, positive and negative infinity and with zero velocities.
  const double infinite_distance = (side == WhichSide::kAhead) ?
      std::numeric_limits<double>::infinity()
      : -std::numeric_limits<double>::infinity();
  const RoadPosition default_road_position(lane, {infinite_distance, 0., 0.});
  const RoadOdometry<double>
      default_result(default_road_position, FrameVelocity<double>());

  // Express all data vectors as Maliput.
  const GeoPosition ego_geo_position{ego_pose.get_translation().x(),
        ego_pose.get_translation().y(), ego_pose.get_translation().z()};
  const Vector3<double>& ego_vector{
    ego_geo_position.x, ego_geo_position.y, ego_geo_position.z};
  // Transform the ego car's pose into the lane's frame.
  LanePosition ego_lane_position = lane->ToLanePosition(
      ego_geo_position, nullptr, nullptr);

  // TODO(jadecastro): Fail fast if the ego is not within any lane.

  // Get the ego car's velocity and lane direction in the trial lane.
  const IsoLaneVelocity ego_lane_velocity =
      GetIsoLaneVelocity({lane, ego_lane_position}, ego_velocity);
  LaneDirection lane_direction =
      get_lane_direction(ego_lane_velocity, lane);

  // Compute the s-direction of the ego car and its direction of travel.
  double distance_scanned = (lane_direction.with_s) ? -ego_lane_position.s
      : ego_lane_position.s - lane->length();
  while (distance_scanned < scan_ahead_distance) {
    RoadOdometry<double> result = default_result;
    for (int i = 0; i < traffic_poses.get_num_poses(); ++i) {

      const Isometry3<double> traffic_iso = traffic_poses.get_pose(i);
      // Transform this pose into the lane's frame.

      const GeoPosition traffic_geo_position(traffic_iso.translation().x(),
                                             traffic_iso.translation().y(),
                                             traffic_iso.translation().z());
      if (!IsWithinLane(traffic_geo_position, lane_direction.lane)) continue;

      const Vector3<double>& traffic_vector{
        traffic_geo_position.x, traffic_geo_position.y, traffic_geo_position.z};
      if (traffic_vector == ego_vector) continue;

      const LanePosition traffic_lane_position =
          lane_direction.lane->ToLanePosition(
              traffic_geo_position, nullptr, nullptr);
      switch (side) {
        case WhichSide::kAhead: {
          // Ignore traffic behind the ego car.
          if ((lane_direction.with_s &&
               traffic_lane_position.s < ego_lane_position.s) ||
              (!lane_direction.with_s &&
               traffic_lane_position.s > ego_lane_position.s)) {
            continue;
          }
          // Keep positions that are closer than any other found so far.
          if ((lane_direction.with_s &&
               traffic_lane_position.s < result.pos.s) ||
              (!lane_direction.with_s &&
               traffic_lane_position.s > result.pos.s)) {
            result = RoadOdometry<double>(
              {lane_direction.lane, traffic_lane_position},
              traffic_poses.get_velocity(i));
          }
        }
        case WhichSide::kBehind: {
          // Ignore traffic ahead of the ego car.
          if ((lane_direction.with_s &&
               traffic_lane_position.s > ego_lane_position.s) ||
              (!lane_direction.with_s &&
               traffic_lane_position.s < ego_lane_position.s)) {
            continue;
          }
          // Keep positions that are closer than any other found so far.
          if ((traffic_lane_position.s > result.pos.s) ||
              (!lane_direction.with_s &&
               traffic_lane_position.s < result.pos.s)) {
            result = RoadOdometry<double>(
              {lane_direction.lane, traffic_lane_position},
              traffic_poses.get_velocity(i));
          }
        }
      }
      // TODO(jadecastro): Add a fine-grained check to figure out whether or not
      // traffic_lane_position.s is within distance_scanned.
    }
    if (result.pos.s < std::numeric_limits<double>::infinity()) return result;
    get_next_lane(&lane_direction);
    if (lane_direction.lane == nullptr) return result;  // And return infinity.

    // Increment distance_scanned.
    distance_scanned += lane_direction.lane->length();
  }
  return default_result;
}

/*
const std::pair<const RoadOdometry<double>, const RoadOdometry<double>>
    FindClosestPairAcrossBranch() {
}

const RoadOdometry<double> FindClosestAheadAcrossBranch() {
  // 1) Get all incoming default lanes to nontrivial branch points up to some
  //    scanning distance.
  // 2) Run FindClosestPose forward .
  // 3) For all branch points, run a version of FindClosestPose backward that
  //    checks the car ahead.
  // 4) Cars in OTHER lanes that are slow should probably not be considered.
  //    The scan-behind distance should be, for each traffic car, a function of
  //    speed and saturated at zero.  Slow cars in front of faster cars behind
  //    should probably take precidence, and so this approach will be slightly
  //    conservative.

  this_lane_direction = get_lane_direction(ego_odometry);

  RoadOdometry<double> result = FindSingleClosestPose(ego_pose, traffic_poses);

  //    ......
  // Do a search to populate a vector of non-trivial branch points.
  //    ......

  for () {
    RoadOdometry<double> result_other_lane =
        FindSingleClosestPoseBackward(ego_pose, traffic_poses,
                                      branch_position);
    if (result_other_lane.pos.s < result.pos.s) result = result_other_lane;
  }
  return result;
}
*/

const RoadPosition CalcRoadPosition(const RoadGeometry& road,
                                    const Isometry3<double>& pose) {
  return road.ToRoadPosition(
      maliput::api::GeoPosition(pose.translation().x(), pose.translation().y(),
                                pose.translation().z()),
      nullptr, nullptr, nullptr);
}

// TODO: Should something like this be added to Maliput's api?
// TODO: Outputs only sigma_v at the moment.
const IsoLaneVelocity GetIsoLaneVelocity(
    const RoadPosition& road_position, const FrameVelocity<double>& velocity) {
  const maliput::api::Rotation rot =
      road_position.lane->GetOrientation(road_position.pos);
  const Vector3<double>& vel = velocity.get_velocity().translational();
  const Vector3<double>& rpy{rot.roll, rot.pitch, rot.yaw};
  const Vector4<double>& axis = math::quat2axis(math::rpy2quat(rpy));
  // TODO: Using a deprecated function here  ^^
  const auto isovel = IsoLaneVelocity{vel.dot(axis.head<3>()), 0., 0.};
  //std::cerr << " sigma_v " << isovel.sigma_v << std::endl;
  //std::cerr << " rho_v " << isovel.rho_v << std::endl;
  //std::cerr << " eta_v " << isovel.eta_v << std::endl;
  //std::cerr << " vel.dot(axis.head<3>()) " << vel.dot(axis.head<3>()) << std::endl;
  return isovel;
}

// TODO: Put this into lane_direction.h?
const LaneDirection get_lane_direction(const IsoLaneVelocity& iso_velocity,
                                       const Lane* lane) {
  const bool with_s = iso_velocity.sigma_v >= 0.;
  return LaneDirection(lane, with_s);
}

bool IsWithinLane(const GeoPosition& geo_position, const Lane* const lane) {
  const LanePosition lane_position =
      lane->ToLanePosition(geo_position, nullptr, nullptr);
  if (lane_position.s < 0. || lane->length() < lane_position.s) {
    return false;
  }
  if (lane_position.r <= lane->lane_bounds(lane_position.s).r_min ||
      lane->lane_bounds(lane_position.s).r_max <= lane_position.r) {
    return false;
  }
  return true;
}

void get_next_lane(LaneDirection* lane_direction) {
  const Lane* lane{lane_direction->lane};
  const bool with_s{lane_direction->with_s};
  std::unique_ptr<LaneEnd> branch = (with_s) ?
      lane->GetDefaultBranch(LaneEnd::kFinish) :
      lane->GetDefaultBranch(LaneEnd::kStart);
  if (branch == nullptr) {
    lane_direction->lane = nullptr;
    lane_direction->with_s = true;
  }
  lane_direction->lane = branch->lane;
  lane_direction->with_s = (branch->end == LaneEnd::kStart) ? true : false;
}

}  // namespace pose_selector
}  // namespace automotive
}  // namespace drake
