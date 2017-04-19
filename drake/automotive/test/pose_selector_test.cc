#include "drake/automotive/pose_selector.h"

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/monolane_onramp_merge.h"

namespace drake {
namespace automotive {
namespace {

using maliput::api::GeoPosition;
using maliput::api::IsoLaneVelocity;
using maliput::api::RoadPosition;
using systems::rendering::FrameVelocity;
using systems::rendering::PoseVector;
using systems::rendering::PoseBundle;

constexpr double kLaneLength{100.};
constexpr double kLaneWidth{4.};

constexpr double kEgoSPosition{10.};
constexpr double kEgoRPosition{-0.5 * kLaneWidth};
constexpr double kLeadingSPosition{31.};
constexpr double kTrailingSPosition{7.};
constexpr double kSOffset{4.};
constexpr double kTrafficXVelocity{27.};

constexpr int kFarAheadIndex{0};
constexpr int kJustAheadIndex{1};
constexpr int kJustBehindIndex{2};
constexpr int kFarBehindIndex{3};

static const maliput::api::Lane* GetLaneByJunctionId(
    const maliput::api::RoadGeometry& road, const std::string& lane_id) {
  for (int i = 0; i < road.num_junctions(); ++i) {
    if (road.junction(i)->id().id == lane_id) {
      return road.junction(i)->segment(0)->lane(0);
    }
  }
  throw std::runtime_error("No matching junction name in the road network");
}

static void SetDefaultDragwayPoses(PoseVector<double>* ego_pose,
                                   FrameVelocity<double>* ego_velocity,
                                   PoseBundle<double>* traffic_poses) {
  DRAKE_DEMAND(traffic_poses->get_num_poses() == 4);
  DRAKE_DEMAND(kEgoSPosition > 0. && kLaneLength > kEgoSPosition);
  DRAKE_DEMAND(kLeadingSPosition > kEgoSPosition &&
               kLaneLength > kLeadingSPosition);
  DRAKE_DEMAND(kEgoSPosition > kTrailingSPosition && kTrailingSPosition > 0.);

  // Create poses for four traffic cars and one ego positioned in the right
  // lane, interspersed as follows:
  //
  //     Far Behind   Just Behind     Ego     Just Ahead   Far Ahead
  //   |------o------------o-----------o----------o------------o-------------|
  //  s=0     3            7           10         31           35           100
  ego_pose->set_translation(Eigen::Translation3d(
      kEgoSPosition /* s */, kEgoRPosition /* r */, 0. /* h */));
  Vector6<double> velocity{};
  velocity << 0. /* ωx */, 0. /* ωy */, 0. /* ωz */, 10. /* vx */, 0. /* vy */,
      0. /* vz */;
  ego_velocity->set_velocity(multibody::SpatialVelocity<double>(velocity));

  const Eigen::Translation3d translation_far_ahead(
      kLeadingSPosition + kSOffset /* s */, kEgoRPosition /* r */, 0. /* h */);
  FrameVelocity<double> velocity_far_ahead{};
  velocity_far_ahead.get_mutable_value() << 0. /* ωx */, 0. /* ωy */,
      0. /* ωz */, kTrafficXVelocity /* vx */, 0. /* vy */, 0. /* vz */;
  const Eigen::Translation3d translation_just_ahead(
      kLeadingSPosition /* s */, kEgoRPosition /* r */, 0. /* h */);
  const Eigen::Translation3d translation_just_behind(
      kTrailingSPosition /* s */, kEgoRPosition /* r */, 0. /* h */);
  const Eigen::Translation3d translation_far_behind(
      kTrailingSPosition - kSOffset /* s */, kEgoRPosition /* r */, 0. /* h */);
  traffic_poses->set_pose(kFarAheadIndex,
                          Eigen::Isometry3d(translation_far_ahead));
  traffic_poses->set_velocity(kFarAheadIndex, velocity_far_ahead);
  traffic_poses->set_pose(kJustAheadIndex,
                          Eigen::Isometry3d(translation_just_ahead));
  traffic_poses->set_pose(kJustBehindIndex,
                          Eigen::Isometry3d(translation_just_behind));
  traffic_poses->set_pose(kFarBehindIndex,
                          Eigen::Isometry3d(translation_far_behind));
}

static void SetDefaultOnrampPoses(const GeoPosition& traffic_geo_position,
                                  const double traffic_vy,
                                  PoseVector<double>* ego_pose,
                                  FrameVelocity<double>* ego_velocity,
                                  PoseBundle<double>* traffic_poses) {
  DRAKE_DEMAND(traffic_poses->get_num_poses() == 1);
  DRAKE_DEMAND(kEgoSPosition > kTrailingSPosition && kTrailingSPosition > 0.);

  // Create poses for one ego vehicle in the "onramp0" lane.
  ego_pose->set_translation(
      Eigen::Translation3d(195. /* x */, -50. /* y */, 0. /* z */));
  Vector6<double> velocity{};
  velocity << 0. /* ωx */, 0. /* ωy */, 0. /* ωz */, 0. /* vx */, -10. /* vy */,
      0. /* vz */;
  ego_velocity->set_velocity(multibody::SpatialVelocity<double>(velocity));

  const Eigen::Translation3d translation_ahead(traffic_geo_position.x /* x */,
                                               traffic_geo_position.y /* y */,
                                               traffic_geo_position.z /* z */);
  FrameVelocity<double> velocity_ahead{};
  velocity_ahead.get_mutable_value() << 0. /* ωx */, 0. /* ωy */, 0. /* ωz */,
      traffic_vy /* vx */, 0. /* vy */, 0. /* vz */;
  traffic_poses->set_pose(0, Eigen::Isometry3d(translation_ahead));
  traffic_poses->set_velocity(0, velocity_ahead);
}

GTEST_TEST(PoseSelectorTest, DragwayTest) {
  // Create a straight road, two lanes wide, in which the two s-r
  // Lane-coordinate frames are aligned with the x-y world coordinates, with s_i
  // = 0 for the i-th lane, i ∈ {0, 1}, corresponds to x = 0, and r_i = 0
  // corresponds to y = (i - 0.5) * kLaneWidth.  See sketch below.
  //
  // +y ^
  //    | -  -  -  -  -  -  -  -   <-- lane 1 (y_1)
  //  0 |-----------------------> +x
  //    | -  -  -  -  -  -  -  -   <-- lane 0 (y_0)
  const int kNumLanes{2};
  const maliput::dragway::RoadGeometry road(
      maliput::api::RoadGeometryId({"Test Dragway"}), kNumLanes, kLaneLength,
      kLaneWidth, 0. /* shoulder width */);
  PoseVector<double> ego_pose;
  FrameVelocity<double> ego_velocity;
  PoseBundle<double> traffic_poses(4);
  const double scan_ahead_distance = 50.;

  // Declare variables which contain the found distances to the closest
  // vehicles.
  std::pair<double, double> distances{};
  double distance{};

  // Define the default poses.
  SetDefaultDragwayPoses(&ego_pose, &ego_velocity, &traffic_poses);

  // Calculate the current road position and use it to determine the ego car's
  // lane.
  const GeoPosition geo_position{ego_pose.get_translation().x(),
                                 ego_pose.get_translation().y(),
                                 ego_pose.get_translation().z()};

  const RoadPosition& ego_position =
      road.ToRoadPosition(geo_position, nullptr, nullptr, nullptr);

  RoadOdometry<double> leading_odometry{};
  RoadOdometry<double> trailing_odometry{};
  std::tie(leading_odometry, trailing_odometry) = PoseSelector::FindClosestPair(
      ego_position.lane, ego_pose, ego_velocity, traffic_poses,
      scan_ahead_distance, &distances);

  // Verifies that we are on the road and that the correct car was identified.
  EXPECT_EQ(kLeadingSPosition, leading_odometry.pos.s);
  EXPECT_EQ(kTrailingSPosition, trailing_odometry.pos.s);
  EXPECT_EQ(kLeadingSPosition - kEgoSPosition, distances.first);
  EXPECT_EQ(kEgoSPosition - kTrailingSPosition, distances.second);

  // Test that we get the same result when just the leading car is returned.
  const RoadOdometry<double>& traffic_odometry =
      PoseSelector::FindSingleClosestPose(ego_position.lane, ego_pose,
                                          ego_velocity, traffic_poses,
                                          scan_ahead_distance,
                                          WhichSide::kAhead, &distance);
  EXPECT_EQ(kLeadingSPosition, traffic_odometry.pos.s);
  EXPECT_EQ(kLeadingSPosition - kEgoSPosition, distance);

  // Peer into the adjacent lane to the left.
  std::tie(leading_odometry, trailing_odometry) =
      PoseSelector::FindClosestPair(ego_position.lane->to_left(), ego_pose,
                                    ego_velocity, traffic_poses,
                                    scan_ahead_distance, &distances);

  // Expect to see no cars in the left lane.
  EXPECT_EQ(std::numeric_limits<double>::infinity(), leading_odometry.pos.s);
  EXPECT_EQ(-std::numeric_limits<double>::infinity(), trailing_odometry.pos.s);
  EXPECT_EQ(std::numeric_limits<double>::infinity(), distances.first);
  EXPECT_EQ(-std::numeric_limits<double>::infinity(), distances.second);

  // Bump the "just ahead" car into the lane to the left.
  Isometry3<double> isometry_just_ahead =
      traffic_poses.get_pose(kJustAheadIndex);
  isometry_just_ahead.translation().y() += kLaneWidth;
  traffic_poses.set_pose(kJustAheadIndex, isometry_just_ahead);
  std::tie(leading_odometry, std::ignore) = PoseSelector::FindClosestPair(
      ego_position.lane, ego_pose, ego_velocity, traffic_poses,
      scan_ahead_distance, &distances);

  // Expect the "far ahead" car to be identified and with the correct speed.
  EXPECT_EQ(kLeadingSPosition + kSOffset, leading_odometry.pos.s);
  EXPECT_EQ(kLeadingSPosition + kSOffset - kEgoSPosition, distances.first);
  EXPECT_EQ(kTrafficXVelocity, leading_odometry.vel[3]);

  // Bump the "far ahead" car into the lane to the left.
  Isometry3<double> isometry_far_ahead = traffic_poses.get_pose(kFarAheadIndex);
  isometry_far_ahead.translation().y() += kLaneWidth;
  traffic_poses.set_pose(kFarAheadIndex, isometry_far_ahead);
  std::tie(leading_odometry, std::ignore) = PoseSelector::FindClosestPair(
      ego_position.lane, ego_pose, ego_velocity, traffic_poses,
      scan_ahead_distance, &distances);

  // Looking forward, we expect there to be no car in sight.
  EXPECT_EQ(std::numeric_limits<double>::infinity(), leading_odometry.pos.s);
  EXPECT_EQ(std::numeric_limits<double>::infinity(), distances.first);
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(0., leading_odometry.vel[i]);  // Defaults to zero velocity.
  }

  // Peer into the adjacent lane to the left.
  std::tie(leading_odometry, trailing_odometry) =
      PoseSelector::FindClosestPair(ego_position.lane->to_left(), ego_pose,
                                    ego_velocity, traffic_poses,
                                    scan_ahead_distance, &distances);

  // Expect there to be no car behind on the immediate left and the "just ahead"
  // car to be leading.
  EXPECT_EQ(kLeadingSPosition, leading_odometry.pos.s);
  EXPECT_EQ(-std::numeric_limits<double>::infinity(), trailing_odometry.pos.s);
  EXPECT_EQ(kLeadingSPosition - kEgoSPosition, distances.first);
  EXPECT_EQ(-std::numeric_limits<double>::infinity(), distances.second);
}

GTEST_TEST(PoseSelectorTest, OnrampMergeTest) {
  // Instantiate the onramp merge road.
  std::unique_ptr<MonolaneOnrampMerge> merge_example(new MonolaneOnrampMerge);
  auto road = merge_example->BuildOnramp();

  PoseVector<double> ego_pose;
  FrameVelocity<double> ego_velocity;
  PoseBundle<double> traffic_poses(1);

  // Set a traffic car in lane "post5".
  GeoPosition traffic_geo_post5{0. /* x */, 0. /* y */, 0. /* z */};

  // Define the default poses.
  SetDefaultOnrampPoses(traffic_geo_post5, 10, &ego_pose, &ego_velocity,
                        &traffic_poses);

  // Check that the ego car's pose is in "onramp0".
  const GeoPosition ego_geo_position{ego_pose.get_translation().x(),
                                     ego_pose.get_translation().y(),
                                     ego_pose.get_translation().z()};
  EXPECT_TRUE(PoseSelector::IsWithinLane(
      ego_geo_position, GetLaneByJunctionId(*road, "j:onramp0")));

  // Calculate the current road position and use it to determine the ego car's
  // lane.
  const GeoPosition geo_position{ego_pose.get_translation().x(),
                                 ego_pose.get_translation().y(),
                                 ego_pose.get_translation().z()};
  const RoadPosition& ego_position =
      road->ToRoadPosition(geo_position, nullptr, nullptr, nullptr);

  RoadOdometry<double> leading_odometry{};
  double headway_distance{};
  leading_odometry = PoseSelector::FindSingleClosestPose(
      ego_position.lane, ego_pose, ego_velocity, traffic_poses, 1000.,
      WhichSide::kAhead, &headway_distance);

  // Verifies that we are on the road and that the correct car was identified.
  EXPECT_EQ(0., leading_odometry.pos.s);
  EXPECT_NEAR(265.067, headway_distance, 1e-3);

  // Now, look for cars within the default path and across the branch.
  leading_odometry = PoseSelector::FindSingleClosestAheadAndInBranches(
      *road, ego_pose, ego_velocity, traffic_poses, 1000., &headway_distance);

  // Verifies that we obtain the same result as above.
  EXPECT_EQ(0., leading_odometry.pos.s);
  EXPECT_NEAR(265.067, headway_distance, 1e-3);

  // Set a traffic car in lane "pre0".
  GeoPosition traffic_geo_pre0{180. /* x */, -155. /* y */, 0. /* z */};
  // GeoPosition traffic_geo_pre0{0. /* x */, 0. /* y */, 0. /* z */};

  // Define the default poses.
  SetDefaultOnrampPoses(traffic_geo_pre0, -10, &ego_pose, &ego_velocity,
                        &traffic_poses);

  leading_odometry = PoseSelector::FindSingleClosestAheadAndInBranches(
      *road, ego_pose, ego_velocity, traffic_poses, 1000., &headway_distance);

  // Verifies that we are on the road and that the correct car was identified.
  EXPECT_NEAR(30.064, leading_odometry.pos.s, 1e-3);
  EXPECT_NEAR(95.003, headway_distance, 1e-3);
}

GTEST_TEST(PoseSelectorTest, TestGetIsoVelocity) {
  // Create a single-lane dragway.
  const maliput::dragway::RoadGeometry road(
      maliput::api::RoadGeometryId({"Single-lane dragway"}), 1 /* num_lanes */,
      kLaneLength, kLaneWidth, 0. /* shoulder width */);
  const maliput::api::Lane* lane = road.junction(0)->segment(0)->lane(0);

  RoadPosition position(lane, maliput::api::LanePosition(0., 0., 0.));
  FrameVelocity<double> velocity{};

  // Expect the s-velocity to be zero.
  IsoLaneVelocity iso_velocity =
      PoseSelector::GetIsoLaneVelocity(position, velocity);
  EXPECT_EQ(0., iso_velocity.sigma_v);
  EXPECT_EQ(0., iso_velocity.rho_v);
  EXPECT_EQ(0., iso_velocity.eta_v);

  // Set the velocity to be along the lane's s-coordinate.
  velocity[3] = 10.;
  // Expect the s-velocity to match.
  iso_velocity = PoseSelector::GetIsoLaneVelocity(position, velocity);
  EXPECT_EQ(10., iso_velocity.sigma_v);

  // Set a velocity vector at 45-degrees with the lane's s-coordinate.
  velocity[3] = 10. * std::cos(M_PI / 4.);
  velocity[4] = 10. * std::sin(M_PI / 4.);
  // Expect the s-velocity to be attenuated by sqrt(2) / 2.
  iso_velocity = PoseSelector::GetIsoLaneVelocity(position, velocity);
  EXPECT_NEAR(10. * std::sqrt(2.) / 2., iso_velocity.sigma_v, 1e-12);
  EXPECT_NEAR(10. * std::sqrt(2.) / 2., iso_velocity.rho_v, 1e-12);
}

}  // namespace
}  // namespace automotive
}  // namespace drake
