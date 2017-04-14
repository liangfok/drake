#include "drake/automotive/pose_selector.h"

#include <cmath>
#include <limits>
#include <sstream>

#include "drake/common/drake_assert.h"
#include "drake/math/quaternion.h"
#include "drake/math/roll_pitch_yaw_not_using_quaternion.h"

namespace drake {
namespace automotive {

using maliput::api::GeoPosition;
using maliput::api::IsoLaneVelocity;
using maliput::api::Lane;
using maliput::api::LaneEnd;
using maliput::api::LaneEndSet;
using maliput::api::LanePosition;
using maliput::api::RoadGeometry;
using maliput::api::RoadPosition;
using systems::rendering::FrameVelocity;
using systems::rendering::PoseBundle;
using systems::rendering::PoseVector;

const std::pair<const RoadOdometry<double>, const RoadOdometry<double>>
PoseSelector::FindClosestPair(const Lane* const lane,
                              const PoseVector<double>& ego_pose,
                              const FrameVelocity<double>& ego_velocity,
                              const PoseBundle<double>& traffic_poses,
                              double scan_ahead_distance) {
  const RoadOdometry<double> result_leading =
      FindSingleClosestPose(lane, ego_pose, ego_velocity, traffic_poses,
                            scan_ahead_distance, WhichSide::kAhead);
  const RoadOdometry<double> result_trailing =
      FindSingleClosestPose(lane, ego_pose, ego_velocity, traffic_poses,
                            scan_ahead_distance, WhichSide::kBehind);
  return std::make_pair(result_leading, result_trailing);
}

// TODO(jadecastro): Add support for cars in back.
const RoadOdometry<double> PoseSelector::FindSingleClosestIncludingBranches(
    const RoadGeometry& road, const PoseVector<double>& ego_pose,
    const FrameVelocity<double>& ego_velocity,
    const PoseBundle<double>& traffic_poses, double scan_ahead_distance) {
  const GeoPosition geo_position{ego_pose.get_translation().x(),
                                 ego_pose.get_translation().y(),
                                 ego_pose.get_translation().z()};
  const Lane* const lane =
      road.ToRoadPosition(geo_position, nullptr, nullptr, nullptr).lane;
  const RoadOdometry<double> result_in_lane =
      FindSingleClosestPose(lane, ego_pose, ego_velocity, traffic_poses,
                            scan_ahead_distance, WhichSide::kAhead);
  const std::vector<LaneEndDistance> branches =
      FindBranches(lane, ego_pose, ego_velocity, scan_ahead_distance);
  if (branches.size() == 0) return result_in_lane;
  const double prior_headway_distance = 1;
  const double ego_sigma_v = 1;
  const RoadOdometry<double> result_in_branch = FindSingleClosestAcrossBranches(
      road, traffic_poses, ego_sigma_v, branches, prior_headway_distance);
  // TODO: compare in branch vs. out of branch.
  return result_in_branch;
}

const RoadOdometry<double> PoseSelector::FindSingleClosestPose(
    const Lane* const lane, const PoseVector<double>& ego_pose,
    const FrameVelocity<double>& ego_velocity,
    const PoseBundle<double>& traffic_poses, double scan_ahead_distance,
    const WhichSide side) {
  DRAKE_DEMAND(lane != nullptr);
  const RoadOdometry<double> default_result = set_default_odometry(lane, side);
  const GeoPosition ego_geo_position{ego_pose.get_translation().x(),
                                     ego_pose.get_translation().y(),
                                     ego_pose.get_translation().z()};
  // Transform the ego car's pose into the lane's frame.
  const LanePosition ego_lane_position =
      lane->ToLanePosition(ego_geo_position, nullptr, nullptr);

  // Get the ego car's velocity and lane direction in the trial lane.
  const IsoLaneVelocity ego_lane_velocity =
      GetIsoLaneVelocity({lane, ego_lane_position}, ego_velocity);
  LaneDirection lane_direction(lane,
                               ego_lane_velocity.sigma_v >= 0. /* with_s */);

  // Compute the s-direction of the ego car and its direction of travel.
  double distance_scanned = (lane_direction.with_s)
                                ? -ego_lane_position.s
                                : ego_lane_position.s - lane->length();
  while (distance_scanned < scan_ahead_distance) {
    RoadOdometry<double> result = default_result;
    // const double distance_to_scan = scan_ahead_distance - distance_scanned;
    // TODO ^^^
    for (int i = 0; i < traffic_poses.get_num_poses(); ++i) {
      const Isometry3<double> traffic_iso = traffic_poses.get_pose(i);
      // Transform this pose into the lane's reference frame.

      const GeoPosition traffic_geo_position(traffic_iso.translation().x(),
                                             traffic_iso.translation().y(),
                                             traffic_iso.translation().z());
      if (!IsWithinLane(traffic_geo_position, lane_direction.lane)) continue;

      if (ego_geo_position.x == traffic_geo_position.x &&
          ego_geo_position.y == traffic_geo_position.y &&
          ego_geo_position.z == traffic_geo_position.z) {
        continue;
      }

      const LanePosition traffic_lane_position =
          lane_direction.lane->ToLanePosition(traffic_geo_position, nullptr,
                                              nullptr);
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
    get_default_ongoing_lane(&lane_direction);
    if (lane_direction.lane == nullptr) return result;  // And return infinity.

    // Increment distance_scanned.
    distance_scanned += lane_direction.lane->length();
  }
  return default_result;
}

const RoadOdometry<double> PoseSelector::FindSingleClosestAcrossBranches(
    const RoadGeometry& road, const PoseBundle<double>& traffic_poses,
    double ego_sigma_v, const std::vector<LaneEndDistance>& branches,
    double prior_headway_distance) {
  RoadOdometry<double> result =
      set_default_odometry(nullptr, WhichSide::kAhead);
  for (int i = 0; i < traffic_poses.get_num_poses(); ++i) {
    const Isometry3<double> isometry = traffic_poses.get_pose(i);
    const GeoPosition geo_position(isometry.translation().x(),
                                   isometry.translation().y(),
                                   isometry.translation().z());
    const RoadPosition road_position =
        road.ToRoadPosition(geo_position, nullptr, nullptr, nullptr);

    const Lane* const lane = road_position.lane;
    const LanePosition lane_position = road_position.pos;
    // Get this traffic vehicle's velocity and travel direction in the lane it
    // is occupying.
    const IsoLaneVelocity lane_velocity = GetIsoLaneVelocity(
        {lane, lane_position}, traffic_poses.get_velocity(i));
    LaneDirection lane_direction(lane,
                                 lane_velocity.sigma_v >= 0. /* with_s */);
    double distance_to_this_branch{};
    LaneEnd branch;
    double distance_scanned = (lane_direction.with_s)
                                  ? -lane_position.s
                                  : lane_position.s - lane->length();
    // Determine if any of the traffic cars eventually lead to a branch within a
    // speed- and branch-dependent influence distance horizon.
    for (auto branch_distance : branches) {
      std::tie(distance_to_this_branch, branch) = branch_distance;
      // The distance ahead needed to scan for intersection is assumed equal to
      // the distance scanned in the ego vehicle's lane modulated by the ratio
      // of s-velocity of the traffic car to that of the ego car.  Cars much
      // slower than the ego car are thus phased out, while those that are
      // faster have a longer influence horizon.
      const double distance_to_scan =
          (lane_velocity.sigma_v / ego_sigma_v) * distance_to_this_branch;
      while (distance_scanned < distance_to_scan) {
        auto lane_end = get_default_ongoing_lane(&lane_direction);
        if (lane_direction.lane == nullptr) break;
        if (IsAdjacent(*lane_end, branch)) {
          // "Effective headway" is the distance between the traffic vehicle and
          // the ego vehicle, compared relative to their positions with respect
          // to their shared branch point.
          const double effective_headway =
              distance_to_this_branch - distance_scanned;
          if (0. < effective_headway &&
              effective_headway < prior_headway_distance) {
            prior_headway_distance = effective_headway;
            result = RoadOdometry<double>({lane_direction.lane, lane_position},
                                          traffic_poses.get_velocity(i));
            break;
          }
        }
        // TODO(jadecastro): Add a fine-grained check to figure out whether or
        // not lane_position.s is within distance_scanned.

        // Increment distance_scanned.
        distance_scanned += lane_direction.lane->length();
      }
    }
  }
  return result;
}

const std::vector<PoseSelector::LaneEndDistance> PoseSelector::FindBranches(
    const maliput::api::Lane* const lane, const PoseVector<double>& ego_pose,
    const FrameVelocity<double>& ego_velocity, double scan_ahead_distance) {
  DRAKE_DEMAND(lane != nullptr);
  const GeoPosition ego_geo_position{ego_pose.get_translation().x(),
                                     ego_pose.get_translation().y(),
                                     ego_pose.get_translation().z()};
  // Transform the ego car's pose into the lane's frame.
  const LanePosition ego_lane_position =
      lane->ToLanePosition(ego_geo_position, nullptr, nullptr);

  // Get the ego car's velocity and lane direction in the trial lane.
  const IsoLaneVelocity ego_lane_velocity =
      GetIsoLaneVelocity({lane, ego_lane_position}, ego_velocity);
  LaneDirection lane_direction(lane,
                               ego_lane_velocity.sigma_v >= 0. /* with_s */);

  // Obtain any branches starting from the ego vehicle's lane, moving along its
  // direction of travel by an amount equal to scan_ahead_distance.
  std::vector<LaneEndDistance> branches;
  double distance_scanned = (lane_direction.with_s)
                                ? -ego_lane_position.s
                                : ego_lane_position.s - lane->length();
  std::unique_ptr<LaneEnd> lane_end;
  while (distance_scanned < scan_ahead_distance) {
    // Increment distance_scanned and collect all non-trivial branches as we go.
    distance_scanned += lane_direction.lane->length();
    const LaneEndSet* ends = GetIncomingLaneEnds(lane_direction);
    if (lane_end != nullptr && ends->size() > 2) {
      for (int i = 0; i < ends->size(); ++i) {
        // Store, from the complete list, the LaneEnds that don't belong to the
        // main trunk (lane sequence containing the ego vehicle).
        if (IsAdjacent(*lane_end, ends->get(i))) {
          branches.emplace_back(std::make_pair(distance_scanned, ends->get(i)));
        }
      }
    }
    lane_end = get_default_ongoing_lane(&lane_direction);
    if (lane_direction.lane == nullptr) break;
  }
  return branches;
}

// TODO: Should something like this be added to Maliput's api?
// TODO: Outputs only sigma_v at the moment.
const IsoLaneVelocity PoseSelector::GetIsoLaneVelocity(
    const RoadPosition& road_position, const FrameVelocity<double>& velocity) {
  const maliput::api::Rotation rot =
      road_position.lane->GetOrientation(road_position.pos);
  const Vector3<double>& vel = velocity.get_velocity().translational();
  const Vector3<double>& rpy{rot.roll, rot.pitch, rot.yaw};
  const Vector4<double>& axis = math::quat2axis(math::rpy2quat(rpy));
  // TODO: Using a deprecated function here  ^^
  // const auto isovel = IsoLaneVelocity{vel.dot(axis.head<3>()), 0., 0.};
  // std::cerr << " sigma_v " << isovel.sigma_v << std::endl;
  // std::cerr << " rho_v " << isovel.rho_v << std::endl;
  // std::cerr << " eta_v " << isovel.eta_v << std::endl;
  // std::cerr << " vel.dot(axis.head<3>()) " << vel.dot(axis.head<3>()) <<
  // std::endl;
  return {vel.dot(axis.head<3>()), 0., 0.};
}

bool PoseSelector::IsWithinLane(const GeoPosition& geo_position,
                                const Lane* const lane) {
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

bool PoseSelector::IsAdjacent(const LaneEnd& lane_end0,
                              const LaneEnd& lane_end1) {
  const LanePosition end0 = (lane_end0.end == LaneEnd::Which::kStart) ?
      LanePosition(0., 0., 0.) : LanePosition(lane_end0.lane->length(), 0., 0.);
  const GeoPosition datum0 = lane_end0.lane->ToGeoPosition(end0);
  const LanePosition end1 = (lane_end1.end == LaneEnd::Which::kStart) ?
      LanePosition(0., 0., 0.) : LanePosition(lane_end1.lane->length(), 0., 0.);
  const GeoPosition datum1 = lane_end1.lane->ToGeoPosition(end1);
  return (datum0.x == datum1.x) && (datum0.y == datum1.y) &&
      (datum0.z == datum1.z);
}

std::unique_ptr<LaneEnd> PoseSelector::get_default_ongoing_lane(
    LaneDirection* lane_direction) {
  const Lane* lane{lane_direction->lane};
  const bool with_s{lane_direction->with_s};
  std::unique_ptr<LaneEnd> branch =
      (with_s) ? lane->GetDefaultBranch(LaneEnd::kFinish)
               : lane->GetDefaultBranch(LaneEnd::kStart);
  if (branch == nullptr) {
    lane_direction->lane = nullptr;
    lane_direction->with_s = true;
    return branch;
  }
  lane_direction->lane = branch->lane;
  lane_direction->with_s = (branch->end == LaneEnd::kStart) ? true : false;
  return branch;
}

const LaneEndSet* PoseSelector::GetIncomingLaneEnds(
    const LaneDirection& lane_direction) {
  const Lane* lane{lane_direction.lane};
  const bool with_s{lane_direction.with_s};
  return (with_s) ? lane->GetOngoingBranches(LaneEnd::kStart)
                  : lane->GetOngoingBranches(LaneEnd::kFinish);
}

const RoadOdometry<double> PoseSelector::set_default_odometry(
    const Lane* const lane, const WhichSide side) {
  const double infinite_distance =
      (side == WhichSide::kAhead) ? std::numeric_limits<double>::infinity()
                                  : -std::numeric_limits<double>::infinity();
  const RoadPosition default_road_position(lane, {infinite_distance, 0., 0.});
  return {default_road_position, FrameVelocity<double>()};
}

}  // namespace automotive
}  // namespace drake
