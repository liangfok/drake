#include "drake/automotive/pose_selector.h"

#include <cmath>
#include <limits>
#include <sstream>

#include "drake/common/drake_assert.h"
#include "drake/math/rotation_matrix.h"


namespace drake {
namespace automotive {
namespace pose_selector {

using maliput::api::IsoLaneVelocity;
using maliput::api::Lane;
using maliput::api::RoadGeometry;
using maliput::api::RoadPosition;
using systems::rendering::FrameVelocity;
using systems::rendering::PoseBundle;
using systems::rendering::PoseVector;

const std::pair<const RoadOdometry<double>, const RoadOdometry<double>>
    FindClosestPair(const PoseVector<double>& ego_pose,
                    const PoseBundle<double>& traffic_poses,
                    double scan_ahead_distance,
                    double lane_qual_distance,
                    const Lane* const traffic_lane) {
  RoadOdometry<double> result_leading =
      FindSingleClosestPose(ego_pose, traffic_poses, WhichSide::kAhead);
  RoadOdometry<double> result_trailing =
      FindSingleClosestPoseBackward(ego_pose, traffic_poses);
  return std::make_pair(result_leading, result_trailing);
}

const RoadOdometry<double> FindSingleClosestPose(
    const RoadOdometry<double>& ego_odometry,
    const PoseBundle<double>& traffic_poses, const Lane* const lane = nullptr,
    double scan_ahead_distance, double lane_qual_distance) {
  DRAKE_DEMAND(ego_odometry.lane != nullptr);
  // Take the ego car's lane by default.
  const Lane* const lane =
      (traffic_lane == nullptr) ? ego_odometry.lane : traffic_lane;
  const LanePosition ego_position_other_lane{ego_odometry.pos};
  if (traffic_lane != nullptr) {
    // Use traffic_lane's frame for determining position.
    ego_position_other_lane = traffic_lane.ToLanePosition(
        {ego_odometry.pos.s, ego_odometry.pos.r, ego_odometry.pos.h},
        nullptr, nullptr);
  }
  // Get the ego car's lane direction in the query lane.
  LaneDirection ego_lane_direction = GetLaneDirection(ego_odometry, lane);

  // Compute the s-direction of the ego car and its direction of travel.
  double = s_scanned{0.};
  while (s_scanned < max_scanning_distance) {
    for (int i = 0; i < traffic_poses.get_num_poses(); ++i) {

      if (traffic_position.lane->id().id != lane->id().id) continue;

      const Vector3& traffic_vector{
        traffic_position.pos.s, traffic_position.pos.r, traffic_position.pos.h};

      // 2) from this with_s, infer the polarity of the neigboring lane.
      // 2a) if in opposite direction in a neighboring lane, ignore it.
      //    If lane is ego's lane, keep it!
      // 3) loop through all the cars in the proper direction (forward/backward)
      //    and find the closest one. We need to infer the travel direction of
      //    the found car to see if it qualifies.
      // 4) If none found, increment (decrement) the lane
      //    and repeat, while the count remains less than max_lanes.
      // 5) Threshold for deciding a target is within a lane should be a
      //    multiple of lane_width.
      // 6) Cars in OTHER lanes that are slow should probably not be considered.
      //    Should the scan-behind distance be a function of speed and
      //    scan-ahead distance?

      if (ego_position.lane->id().id == lane->id().id &&
          traffic_vector == ego_vector) {
        continue;
      }
      if (result_trailing.pos.s < traffic_pos(0) &&
          traffic_pos(0) < result_leading.pos.s) {
        // N.B. The ego car and traffic may reside in different lanes.
        return RoadOdometry<double>(traffic_position,
                                    traffic_poses.get_velocity(i));
      }
    }
    get_next_lane(this_lane_direction);
    if (this_lane_direction == nullptr) break;  // And return infinity.
    // TODO: What to do in backward direction??

    // Increment s_scanned
    s_scanned += tr
  }
  // Default the leading and trailing vehicles with positions extending to,
  // respectively, positive and negative infinity and with zero velocities.
  return RoadPosition(
      lane, {std::numeric_limits<double>::infinity(), 0., 0.});
}

const std::pair<const RoadOdometry<double>, const RoadOdometry<double>>
    FindClosestPairAcrossBranch() {
}

const RoadOdometry<double> FindClosestAheadAcrossBranch() {
  // 1) Get all incoming default lanes to nontrivial branch points up to some
  //    scanning distance.
  // 2) Run FindClosestPose forward .
  // 3) For all branch points, run a version of FindClosestPose backward that
  //    checks the car ahead.


  this_lane_direction = GetLaneDirection(ego_odometry);

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

const RoadPosition CalcRoadPosition(const RoadGeometry& road,
                                    const Isometry3<double>& pose) {
  return road.ToRoadPosition(
      maliput::api::GeoPosition(pose.translation().x(), pose.translation().y(),
                                pose.translation().z()),
      nullptr, nullptr, nullptr);
}

// TODO: Should this be added to Maliput's api?
IsoLaneVelocity GetIsoLaneVelocity(const RoadOdometry<double>& road_odometry,
                                   const Lane* lane) {
  const maliput::api::Rotation rot = lane->GetOrientation(road_odometry.pos);
  const Vector3& velocity = road_odometry.vel.get_velocity().translational();
  const Vector4& axis = quat2axis(rpy2quat({rot.roll, rot.pitch, rot.yaw}));
  return velocity.dot(axis.head<3>());
}

LaneDirection GetLaneDirection(const RoadOdometry<double>& road_odometry,
                               const Lane* lane) {
  const IsoLaneVelocity ego_lane_velocity = GetIsoLaneVelocity(ego_odometry,
                                                               lane);
  // TODO(jadecastro): Intermediary conversion to MotionDerivatives?
  const bool with_s = ego_lane_velocity.sigma_v >= 0.;
  return LaneDirection(lane, with_s);
}

void get_next_lane(LaneDirection* lane_direction) {
  const Lane* lane{lane_direction->lane};
  const bool with_s{lane_direction->with_s};
  branch = cond(with_s, lane->GetDefaultBranch(LaneEnd::kFinish),
                lane->GetDefaultBranch(LaneEnd::kStart));
  if (branch == nullptr) return nullptr;
  lane_direction->lane = branch->lane;
  lane_direction->with_s = (branch->end == LaneEnd::kStart) ? true : false;
}

}  // namespace pose_selector
}  // namespace automotive
}  // namespace drake
