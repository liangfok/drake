#include "drake/automotive/maliput_railcar.h"

#include <cmath>
#include <memory>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/maliput/monolane/builder.h"
#include "drake/automotive/maliput/utility/generate_obj.h"  // TEMP!!!
#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/math/roll_pitch_yaw_not_using_quaternion.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/system.h"

namespace drake {

using maliput::api::LaneEnd;

using maliput::monolane::ArcOffset;
using maliput::monolane::Connection;
using maliput::monolane::Endpoint;
using maliput::monolane::EndpointXy;
using maliput::monolane::EndpointZ;

using systems::BasicVector;
using systems::LeafContext;
using systems::Parameters;
using systems::rendering::PoseVector;

namespace automotive {
namespace {

class MaliputRailcarTest : public ::testing::Test {
 protected:
  void InitializeDragwayLane(double start_time = 0, bool with_s = true) {
    // Defines the dragway's parameters.
    const int kNumLanes{1};
    const double kDragwayLength{50};
    const double kDragwayLaneWidth{0.5};
    const double kDragwayShoulderWidth{0.25};
    Initialize(
        std::make_unique<const maliput::dragway::RoadGeometry>(
            maliput::api::RoadGeometryId({"RailcarTestDragway"}), kNumLanes,
            kDragwayLength, kDragwayLaneWidth, kDragwayShoulderWidth),
        start_time, with_s);
  }

  void InitializeCurvedMonoLane(double start_time = 0, bool with_s = true) {
    maliput::monolane::Builder builder(
        maliput::api::RBounds(-2, 2),   /* lane_bounds       */
        maliput::api::RBounds(-4, 4),   /* driveable_bounds  */
        0.01,                           /* linear tolerance  */
        0.5 * M_PI / 180.0);            /* angular_tolerance */
    builder.Connect(
        "point.0",                                             /* id    */
        Endpoint(EndpointXy(0, 0, 0), EndpointZ(0, 0, 0, 0)),  /* start */
        ArcOffset(kCurvedRoadRadius, kCurvedRoadTheta),        /* arc   */
        EndpointZ(0, 0, 0, 0));                                /* z_end */
    Initialize(
        builder.Build(maliput::api::RoadGeometryId({"RailcarTestCurvedRoad"})),
        start_time, with_s);
  }

  // Creates a RoadGeometry based on monolane that consists of a straight lane
  // that's then connected to a left turning lane.
  //       ____
  //       ____ \
  //       <.. \ \
  //       __ . \ \      ArcLane that curves 90 degrees.
  //       _ \ . \ \     (The s-curve direction can be flipped by setting
  //        \ \ : \ \     parameter flip_curve_lane to be true.)
  //        | | : | |
  //        | | ^ | |
  //        | | : | |    Straight lane.
  //        | | : | |
  //
  //
  // @param start_time the start time of the MaliputRailcar.
  //
  // @param with_s Whether the Maliputrailcar is traveling with or against the
  // lane's s-curve.
  //
  // @param flip_curve_lane Whether to flip the curved lane's s-axis. This is
  // useful for testing wither MaliputRailcar can traverse lanes that have
  // opposing s-directions.
  void InitializeTwoLaneSegment(double start_time = 0, bool with_s = true,
      bool flip_curve_lane = false) {
    maliput::monolane::Builder builder(
        maliput::api::RBounds(-2, 2),   /* lane_bounds       */
        maliput::api::RBounds(-4, 4),   /* driveable_bounds  */
        0.01,                           /* linear tolerance  */
        0.5 * M_PI / 180.0);            /* angular_tolerance */
    const Connection* straight_lane_connection = builder.Connect(
        "point.0",                                             /* id     */
        Endpoint(EndpointXy(0, 0, 0), EndpointZ(0, 0, 0, 0)),  /* start  */
        kStraightRoadLength,                                   /* length */
        EndpointZ(0, 0, 0, 0));                                /* z_end  */
    if (flip_curve_lane) {
      const Connection* curved_lane_connection = builder.Connect(
          "point.1",                                            /* id     */
          Endpoint(                                             /* start  */
              EndpointXy(10 + kCurvedRoadRadius, kCurvedRoadRadius, 1.5 * M_PI),
              EndpointZ(0, 0, 0, 0)),
          ArcOffset(kCurvedRoadRadius, -kCurvedRoadTheta),      /* arc    */
          EndpointZ(0, 0, 0, 0));                               /* z_end  */
      builder.SetDefaultBranch(
          straight_lane_connection, LaneEnd::kFinish /* in_end */,
          curved_lane_connection, LaneEnd::kFinish   /* out_end */);
      builder.SetDefaultBranch(
          curved_lane_connection, LaneEnd::kFinish   /* in_end */,
          straight_lane_connection, LaneEnd::kFinish /* out_end */);
    } else {
      const Connection* curved_lane_connection = builder.Connect(
          "point.1",                                             /* id     */
          Endpoint(EndpointXy(10, 0, 0), EndpointZ(0, 0, 0, 0)), /* start  */
          ArcOffset(kCurvedRoadRadius, kCurvedRoadTheta),        /* arc    */
          EndpointZ(0, 0, 0, 0));                                /* z_end  */
      builder.SetDefaultBranch(
          straight_lane_connection, LaneEnd::kFinish /* in_end */,
          curved_lane_connection, LaneEnd::kStart    /* out_end */);
      builder.SetDefaultBranch(
          curved_lane_connection, LaneEnd::kStart    /* in_end */,
          straight_lane_connection, LaneEnd::kFinish /* out_end */);
    }

    std::unique_ptr<const maliput::api::RoadGeometry> road =
        builder.Build(maliput::api::RoadGeometryId(
            {"RailcarTestTwoLaneSegmentRoad"}));

    // Verifies that the default branches are set for the lanes.

    // TEMP!
    // Saves an OBJ of the RoadGeometry for visualization / debugging purposes.
    maliput::utility::ObjFeatures features;
    features.max_grid_unit = maliput::utility::ObjFeatures().max_grid_unit;
    features.min_grid_resolution =
        maliput::utility::ObjFeatures().min_grid_resolution;
    maliput::utility::GenerateObjFile(road.get(),
        "/Users/liang/Downloads/test-obj", "foo", features);

    Initialize(
        std::move(road),
        start_time, with_s);
  }

  void Initialize(std::unique_ptr<const maliput::api::RoadGeometry> road,
      double start_time, bool with_s) {
    road_ = std::move(road);
    const maliput::api::Lane* lane = road_->junction(0)->segment(0)->lane(0);
    dut_.reset(new MaliputRailcar<double>(LaneDirection(lane, with_s),
                                          start_time));
    context_ = dut_->CreateDefaultContext();
    output_ = dut_->AllocateOutput(*context_);
    derivatives_ = dut_->AllocateTimeDerivatives();
  }

  void SetInputValue(double desired_acceleration) {
    DRAKE_DEMAND(dut_ != nullptr);
    DRAKE_DEMAND(context_ != nullptr);
    context_->FixInputPort(dut_->command_input().get_index(),
        BasicVector<double>::Make(desired_acceleration));
  }

  MaliputRailcarState<double>* continuous_state() {
    auto result = dynamic_cast<MaliputRailcarState<double>*>(
        context_->get_mutable_continuous_state_vector());
    if (result == nullptr) { throw std::bad_cast(); }
    return result;
  }

  LaneDirection& lane_direction() {
    return context_->template get_mutable_abstract_state<LaneDirection>(0);
  }

  const MaliputRailcarState<double>* state_output() const {
    auto state = dynamic_cast<const MaliputRailcarState<double>*>(
        output_->get_vector_data(dut_->state_output().get_index()));
    DRAKE_DEMAND(state != nullptr);
    return state;
  }

  const PoseVector<double>* pose_output() const {
    auto pose = dynamic_cast<const PoseVector<double>*>(
        output_->get_vector_data(dut_->pose_output().get_index()));
    DRAKE_DEMAND(pose != nullptr);
    return pose;
  }

  // Sets the configuration parameters of the railcar.
  void SetConfig(const MaliputRailcarConfig<double>& config) {
    LeafContext<double>* leaf_context =
        dynamic_cast<LeafContext<double>*>(context_.get());
    ASSERT_NE(leaf_context, nullptr);
    Parameters<double>& parameters = leaf_context->get_mutable_parameters();
    BasicVector<double>* vector_param =
        parameters.get_mutable_numeric_parameter(0);
    ASSERT_NE(vector_param, nullptr);
    MaliputRailcarConfig<double>* railcar_config =
        dynamic_cast<MaliputRailcarConfig<double>*>(vector_param);
    ASSERT_NE(railcar_config, nullptr);
    railcar_config->SetFrom(config);
  }

  // Obtains the lanes created by the called to InitializeTwoLaneSegment().
  // Since a monolane::Builder was used to create the RoadGeometry, we don't
  // know which Junction contains which lane and thus need to figure it out.
  // This is done by checking the lengths of the two lanes. The straight lane
  // has a length of kStraightRoadLength = 10 while the curved lane has a length
  // of approximately 2 * PI * kCurvedRoadRadius /4 = 2 * PI * 10 / 4= 15.078.
  std::pair<const maliput::api::Lane*, const maliput::api::Lane*>
  GetStraightAndCurvedLanes() {
    DRAKE_DEMAND(road_->num_junctions() == 2);
    const maliput::api::Lane* lane_0 =
        road_->junction(0)->segment(0)->lane(0);
    const maliput::api::Lane* lane_1 =
        road_->junction(1)->segment(0)->lane(0);

    const maliput::api::Lane* straight_lane =
        (lane_0->length() == kStraightRoadLength) ? lane_0 : lane_1;
    const maliput::api::Lane* curved_lane =
        (lane_0->length() == kStraightRoadLength) ? lane_1 : lane_0;

    return std::make_pair(straight_lane, curved_lane);
  }

  // The length of the straight lane segment of the road when it is created
  // using InitializeTwoLaneSegment().
  const double kStraightRoadLength{10};

  // The arc radius and theta of the road when it is created using
  // InitializeCurvedMonoLane() and InitializeTwoLaneSegment().
  const double kCurvedRoadRadius{10};
  const double kCurvedRoadTheta{M_PI_2};

  std::unique_ptr<const maliput::api::RoadGeometry> road_;
  std::unique_ptr<MaliputRailcar<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::SystemOutput<double>> output_;
  std::unique_ptr<systems::ContinuousState<double>> derivatives_;
};

TEST_F(MaliputRailcarTest, Topology) {
  EXPECT_NO_FATAL_FAILURE(InitializeDragwayLane());
  ASSERT_EQ(dut_->get_num_input_ports(), 1);

  ASSERT_EQ(dut_->get_num_output_ports(), 3);
  const auto& state_output = dut_->state_output();
  EXPECT_EQ(systems::kVectorValued, state_output.get_data_type());
  EXPECT_EQ(MaliputRailcarStateIndices::kNumCoordinates, state_output.size());

  const auto& pose_output = dut_->pose_output();
  EXPECT_EQ(systems::kVectorValued, pose_output.get_data_type());
  EXPECT_EQ(PoseVector<double>::kSize, pose_output.size());

  EXPECT_FALSE(dut_->HasAnyDirectFeedthrough());
}

TEST_F(MaliputRailcarTest, ZeroInitialOutput) {
  EXPECT_NO_FATAL_FAILURE(InitializeDragwayLane());
  dut_->CalcOutput(*context_, output_.get());
  auto state = state_output();
  EXPECT_EQ(state->s(), MaliputRailcar<double>::kDefaultInitialS);
  EXPECT_EQ(state->speed(), MaliputRailcar<double>::kDefaultInitialSpeed);
  auto pose = pose_output();
  EXPECT_TRUE(CompareMatrices(pose->get_isometry().matrix(),
                              Eigen::Isometry3d::Identity().matrix()));
  Eigen::Translation<double, 3> translation = pose->get_translation();
  EXPECT_EQ(translation.x(), 0);
  EXPECT_EQ(translation.y(), 0);
  EXPECT_EQ(translation.z(), 0);
}

TEST_F(MaliputRailcarTest, StateAppearsInOutputDragway) {
  EXPECT_NO_FATAL_FAILURE(InitializeDragwayLane());
  const double kS{1};
  const double kSpeed{2};

  continuous_state()->set_s(kS);
  continuous_state()->set_speed(kSpeed);
  dut_->CalcOutput(*context_, output_.get());

  auto state = state_output();
  EXPECT_EQ(state->s(), kS);
  EXPECT_EQ(state->speed(), kSpeed);

  auto pose = pose_output();
  Eigen::Isometry3d expected_pose = Eigen::Isometry3d::Identity();
  // For the dragway, the `(s, r, h)` axes in lane-space coorespond to the
  // `(x, y, z)` axes in geo-space. In this case, `s = kS` while `r` and `h` are
  // by default zero.
  expected_pose.translation() = Eigen::Vector3d(kS, 0, 0);
  EXPECT_TRUE(CompareMatrices(pose->get_isometry().matrix(),
                              expected_pose.matrix()));
}

TEST_F(MaliputRailcarTest, StateAppearsInOutputMonolane) {
  EXPECT_NO_FATAL_FAILURE(InitializeCurvedMonoLane());
  const maliput::api::Lane* lane = road_->junction(0)->segment(0)->lane(0);
  const double kSpeed{3.5};

  continuous_state()->set_s(lane->length());
  continuous_state()->set_speed(kSpeed);
  dut_->CalcOutput(*context_, output_.get());

  ASSERT_NE(lane, nullptr);
  auto state = state_output();
  EXPECT_EQ(state->s(), lane->length());
  EXPECT_EQ(state->speed(), kSpeed);

  auto pose = pose_output();
  Eigen::Isometry3d expected_pose = Eigen::Isometry3d::Identity();
  {
    const Eigen::Vector3d rpy(0, 0, kCurvedRoadTheta);
    const Eigen::Vector3d xyz(kCurvedRoadRadius, kCurvedRoadRadius, 0);
    expected_pose.matrix() << drake::math::rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }
  // The following tolerance was determined emperically.
  EXPECT_TRUE(CompareMatrices(pose->get_isometry().matrix(),
                              expected_pose.matrix(), 1e-15 /* tolerance */));
}

TEST_F(MaliputRailcarTest, NonZeroParametersAppearInOutputDragway) {
  EXPECT_NO_FATAL_FAILURE(InitializeDragwayLane());
  const double kR{1.5};
  const double kH{8.2};

  // Sets the parameters to be non-zero values.
  MaliputRailcarConfig<double> config;
  config.set_r(kR);
  config.set_h(kH);
  config.set_initial_speed(1);
  config.set_max_speed(30);
  config.set_velocity_limit_kp(8);
  SetConfig(config);
  dut_->CalcOutput(*context_, output_.get());
  auto pose = pose_output();
  Eigen::Isometry3d expected_pose = Eigen::Isometry3d::Identity();
  expected_pose.translation() = Eigen::Vector3d(0, kR, kH);
  EXPECT_TRUE(CompareMatrices(pose->get_isometry().matrix(),
                              expected_pose.matrix()));
}

TEST_F(MaliputRailcarTest, NonZeroParametersAppearInOutputMonolane) {
  EXPECT_NO_FATAL_FAILURE(InitializeCurvedMonoLane());
  const double kR{1.5};
  const double kH{8.2};

  // Sets the parameters to be non-zero values.
  MaliputRailcarConfig<double> config;
  config.set_r(kR);
  config.set_h(kH);
  config.set_initial_speed(1);
  config.set_max_speed(30);
  config.set_velocity_limit_kp(8);
  SetConfig(config);
  dut_->CalcOutput(*context_, output_.get());
  auto start_pose = pose_output();
  Eigen::Isometry3d expected_start_pose = Eigen::Isometry3d::Identity();
  {
    const Eigen::Vector3d rpy(0, 0, 0);
    const Eigen::Vector3d xyz(0, kR, kH);
    expected_start_pose.matrix()
        << drake::math::rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }
  // The following tolerance was determined empirically.
  EXPECT_TRUE(CompareMatrices(start_pose->get_isometry().matrix(),
                              expected_start_pose.matrix(),
                              1e-15 /* tolerance */));

  // Moves the vehicle to be at the end of the curved lane.
  const maliput::api::Lane* lane = road_->junction(0)->segment(0)->lane(0);
  continuous_state()->set_s(lane->length());

  dut_->CalcOutput(*context_, output_.get());
  auto end_pose = pose_output();
  Eigen::Isometry3d expected_end_pose = Eigen::Isometry3d::Identity();
  {
    const Eigen::Vector3d rpy(0, 0, M_PI_2);
    const Eigen::Vector3d xyz(kCurvedRoadRadius - kR, kCurvedRoadRadius, kH);
    expected_end_pose.matrix() << drake::math::rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }
  // The following tolerance was determined empirically.
  EXPECT_TRUE(CompareMatrices(end_pose->get_isometry().matrix(),
                              expected_end_pose.matrix(),
                              1e-15 /* tolerance */));
}

TEST_F(MaliputRailcarTest, DerivativesDragway) {
  EXPECT_NO_FATAL_FAILURE(InitializeDragwayLane());
  // Grabs a pointer to where the EvalTimeDerivatives results end up.
  const MaliputRailcarState<double>* const result =
      dynamic_cast<const MaliputRailcarState<double>*>(
          derivatives_->get_mutable_vector());
  ASSERT_NE(nullptr, result);

  // Sets the input command.
  SetInputValue(0 /* desired_acceleration */);

  // Checks the derivatives given the default continuous state with r = 0.
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_DOUBLE_EQ(result->s(), MaliputRailcar<double>::kDefaultInitialSpeed);
  EXPECT_DOUBLE_EQ(result->speed(), 0.0);  // Expect zero acceleration.

  // Checks that the acceleration is zero given an acceleration command of zero
  // and a non-default continuous state with r = 0.
  continuous_state()->set_s(3.5);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_DOUBLE_EQ(result->s(), MaliputRailcar<double>::kDefaultInitialSpeed);
  EXPECT_DOUBLE_EQ(result->speed(), 0.0);

  // Checks that the acceleration is zero given an acceleration command of zero
  // and a non-default continuous state with r != 0.
  const double kS{1.5};
  const double kSlowSpeed{2};
  const double kMaxSpeed{30};
  MaliputRailcarConfig<double> config;
  config.set_r(-2);
  config.set_h(0);
  config.set_initial_speed(kSlowSpeed);
  config.set_max_speed(kMaxSpeed);
  config.set_velocity_limit_kp(8);
  SetConfig(config);
  continuous_state()->set_s(kS);
  continuous_state()->set_speed(kSlowSpeed);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_DOUBLE_EQ(result->s(), kSlowSpeed);
  EXPECT_DOUBLE_EQ(result->speed(), 0.0);

  // Checks that a positive acceleration command of `kPosAccelCmd` results in
  // the actual acceleration being equal to the commanded acceleration when the
  // vehicle's current speed of `kSlowSpeed` is far lower than the vehicle's
  // maximum speed of `kMaxSpeed`.
  const double kPosAccelCmd{5};
  SetInputValue(kPosAccelCmd);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_DOUBLE_EQ(result->s(), kSlowSpeed);
  EXPECT_DOUBLE_EQ(result->speed(), kPosAccelCmd);

  // Checks that a negative acceleration command of `kNegAccelCmd` results in
  // the actual acceleration being equal to the commanded acceleration when the
  // vehicle's current speed of `kFastSpeed` is far higher than 0.
  const double kFastSpeed{27};
  const double kNegAccelCmd{-1};
  SetInputValue(kNegAccelCmd);
  continuous_state()->set_speed(kFastSpeed);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_DOUBLE_EQ(result->s(), kFastSpeed);
  EXPECT_DOUBLE_EQ(result->speed(), kNegAccelCmd);
}

TEST_F(MaliputRailcarTest, DerivativesMonolane) {
  EXPECT_NO_FATAL_FAILURE(InitializeCurvedMonoLane());
  // Grabs a pointer to where the EvalTimeDerivatives results end up.
  const MaliputRailcarState<double>* const result =
      dynamic_cast<const MaliputRailcarState<double>*>(
          derivatives_->get_mutable_vector());
  ASSERT_NE(nullptr, result);

  // Sets the input command.
  SetInputValue(0  /* desired acceleration */);

  const double kInitialSpeed{1};
  const double kR{1};

  // Checks the derivatives given a non-default continuous state with r != 0.
  continuous_state()->set_s(1.5);
  MaliputRailcarConfig<double> config;
  config.set_r(kR);
  config.set_h(0);
  config.set_initial_speed(kInitialSpeed);
  config.set_max_speed(30);
  config.set_velocity_limit_kp(8);
  SetConfig(config);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_DOUBLE_EQ(result->s(),
      kInitialSpeed * kCurvedRoadRadius / (kCurvedRoadRadius - kR));
  EXPECT_DOUBLE_EQ(result->speed(), 0);
}

// Tests that connecting the input port is optional, i.e., that it can contain
// a nullptr value.
TEST_F(MaliputRailcarTest, InputPortNotConnected) {
  EXPECT_NO_FATAL_FAILURE(InitializeDragwayLane());
  // Grabs a pointer to where the EvalTimeDerivatives results end up.
  const MaliputRailcarState<double>* const result =
      dynamic_cast<const MaliputRailcarState<double>*>(
          derivatives_->get_mutable_vector());
  ASSERT_NE(nullptr, result);

  ASSERT_NO_FATAL_FAILURE(
      dut_->CalcTimeDerivatives(*context_, derivatives_.get()));
  EXPECT_DOUBLE_EQ(result->s(), MaliputRailcar<double>::kDefaultInitialSpeed);
  EXPECT_DOUBLE_EQ(result->speed(), 0.0);

  // Checks that the derivatives contain the speed set by the configuration and
  // zero acceleration.
  const double kS{2.25};
  const double kInitialSpeed{2};
  MaliputRailcarConfig<double> config;
  config.set_r(-2);
  config.set_h(0);
  config.set_initial_speed(kInitialSpeed);
  config.set_max_speed(30);
  config.set_velocity_limit_kp(8);
  SetConfig(config);
  continuous_state()->set_s(kS);
  continuous_state()->set_speed(kInitialSpeed);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_DOUBLE_EQ(result->s(), kInitialSpeed);
  EXPECT_DOUBLE_EQ(result->speed(), 0.0);
}

// Tests that a MaliputRailcar does not start moving until after the start time
// has passed.
TEST_F(MaliputRailcarTest, NonZeroStartTime) {
  const double kStartTime{10};
  EXPECT_NO_FATAL_FAILURE(InitializeDragwayLane(kStartTime));

  // Grabs a pointer to where the EvalTimeDerivatives results end up.
  const MaliputRailcarState<double>* const result =
      dynamic_cast<const MaliputRailcarState<double>*>(
          derivatives_->get_mutable_vector());
  ASSERT_NE(nullptr, result);

  const double kDesiredAcceleration{1};
  const double kSpeed{3};

  // Verifies that the vehicle has zero derivatives prior to the start time even
  // if the input contains a non-zero acceleration command.
  SetInputValue(kDesiredAcceleration);
  context_->set_time(kStartTime - 1e-8);
  ASSERT_NO_FATAL_FAILURE(
      dut_->CalcTimeDerivatives(*context_, derivatives_.get()));
  EXPECT_DOUBLE_EQ(result->s(), 0);
  EXPECT_DOUBLE_EQ(result->speed(), 0);

  // Verifies that the vehicle has the expected derivatives at the start time.
  context_->set_time(kStartTime);
  continuous_state()->set_speed(kSpeed);
  ASSERT_NO_FATAL_FAILURE(
      dut_->CalcTimeDerivatives(*context_, derivatives_.get()));
  EXPECT_DOUBLE_EQ(result->s(), kSpeed);
  EXPECT_DOUBLE_EQ(result->speed(), kDesiredAcceleration);

  // Verifies that the vehicle has the expected derivatives after the start
  // time.
  context_->set_time(kStartTime + 1e-8);
  ASSERT_NO_FATAL_FAILURE(
      dut_->CalcTimeDerivatives(*context_, derivatives_.get()));
  EXPECT_DOUBLE_EQ(result->s(), kSpeed);
  EXPECT_DOUBLE_EQ(result->speed(), kDesiredAcceleration);
}

// Tests the correctness of the derivatives when the vehicle travels in the
// decreasing `s` direction.
TEST_F(MaliputRailcarTest, DecreasingS) {
  EXPECT_NO_FATAL_FAILURE(InitializeDragwayLane(0 /* start_time */,
                                                false /* with_s */));
  // Grabs a pointer to where the EvalTimeDerivatives results end up.
  const MaliputRailcarState<double>* const result =
      dynamic_cast<const MaliputRailcarState<double>*>(
          derivatives_->get_mutable_vector());
  ASSERT_NE(nullptr, result);

  // Sets the input command.
  const double kAccelCmd{1.5};  // The acceleration command.
  SetInputValue(kAccelCmd);

  // Checks that the time derivative of `s` is the negative of the speed, which
  // happens when the vehicle is traveling in the negative `s` direction.
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());
  EXPECT_DOUBLE_EQ(result->s(), -MaliputRailcar<double>::kDefaultInitialSpeed);
  EXPECT_DOUBLE_EQ(result->speed(), kAccelCmd);
}

// Tests the correctness of MaliputRailcar::DoCalcNextUpdateTime() when the
// road network is a dragway and with_s is true.
TEST_F(MaliputRailcarTest, DoCalcNextUpdateTimeDragwayWithS) {
  EXPECT_NO_FATAL_FAILURE(InitializeDragwayLane(0 /* start_time */,
                                                true /* with_s */));
  const maliput::api::Lane* lane = road_->junction(0)->segment(0)->lane(0);
  const double kSpeed(10);

  systems::UpdateActions<double> actions;

  // Computes the time to reach the end of the lane assuming a speed of kSpeed.
  const double kZeroAccelerationTime = lane->length() / kSpeed;

  // Verifies that the time till reaching the end of the lane is equal to the
  // lane length divided by the vehicle's speed.
  context_->set_time(0);
  continuous_state()->set_s(0);
  continuous_state()->set_speed(kSpeed);
  dut_->CalcNextUpdateTime(*context_, &actions);
  EXPECT_DOUBLE_EQ(actions.time, kZeroAccelerationTime);
  EXPECT_EQ(actions.events.at(0).action,
            systems::DiscreteEvent<double>::kUnrestrictedUpdateAction);

  // Verifies that when the vehicle is mid-way through the lane, the time till
  // it reaches the end is cut in half.
  continuous_state()->set_s(lane->length() / 2);
  dut_->CalcNextUpdateTime(*context_, &actions);
  EXPECT_GE(actions.time, 0);
  EXPECT_DOUBLE_EQ(actions.time, kZeroAccelerationTime / 2);

  // Verifies that when the vehicle is at the end of the lane, the time till it
  // reaches the end is zero.
  continuous_state()->set_s(lane->length());
  dut_->CalcNextUpdateTime(*context_, &actions);
  EXPECT_DOUBLE_EQ(actions.time, 0);
}

// Same as the previous unit test except with_s is false.
TEST_F(MaliputRailcarTest, DoCalcNextUpdateTimeDragwayAgainstS) {
  EXPECT_NO_FATAL_FAILURE(InitializeDragwayLane(0 /* start_time */,
                                                false /* with_s */));
  const maliput::api::Lane* lane = road_->junction(0)->segment(0)->lane(0);
  const double kSpeed(10);

  systems::UpdateActions<double> actions;

  // Computes the time to reach the end of the lane assuming a speed of kSpeed
  // and zero acceleration.
  const double kZeroAccelerationTime = lane->length() / kSpeed;

  // Verifies that the time till reaching the end of the lane is equal to the
  // lane length divided by the vehicle's speed.
  context_->set_time(0);
  continuous_state()->set_s(lane->length());
  continuous_state()->set_speed(kSpeed);
  dut_->CalcNextUpdateTime(*context_, &actions);
  EXPECT_GE(actions.time, 0);
  EXPECT_DOUBLE_EQ(actions.time, kZeroAccelerationTime);

  // Verifies that when the vehicle is mid-way through the lane, the time till
  // it reaches the end is cut in half.
  continuous_state()->set_s(lane->length() / 2);
  dut_->CalcNextUpdateTime(*context_, &actions);
  EXPECT_GE(actions.time, 0);
  EXPECT_DOUBLE_EQ(actions.time, kZeroAccelerationTime / 2);

  // Verifies that when the vehicle is at the end of the lane, the time till it
  // reaches the end is zero.
  continuous_state()->set_s(0);
  dut_->CalcNextUpdateTime(*context_, &actions);
  EXPECT_DOUBLE_EQ(actions.time, 0);
}

// Same as the previous unit test except the road network is a curved monolane
// and with_s is true.
TEST_F(MaliputRailcarTest, DoCalcNextUpdateTimeMonolaneWithS) {
  EXPECT_NO_FATAL_FAILURE(InitializeCurvedMonoLane(0 /* start_time */,
                                                   true /* with_s */));
  const maliput::api::Lane* lane = road_->junction(0)->segment(0)->lane(0);
  const double kSpeed(10);

  systems::UpdateActions<double> actions;

  MaliputRailcarConfig<double> config;
  config.set_r(0);
  config.set_h(0);
  config.set_initial_speed(kSpeed);
  config.set_max_speed(30);
  config.set_velocity_limit_kp(8);
  SetConfig(config);

  // Computes the time to reach the end of the lane assuming a speed of kSpeed
  // and zero acceleration.
  const double kZeroAccelerationTime = lane->length() / kSpeed;

  // Verifies that the time till reaching the end of the lane is equal to the
  // lane length divided by the vehicle's speed.
  context_->set_time(0);
  continuous_state()->set_s(0);
  continuous_state()->set_speed(kSpeed);
  dut_->CalcNextUpdateTime(*context_, &actions);
  EXPECT_DOUBLE_EQ(actions.time, kZeroAccelerationTime);

  // Sets `r` to be positive and verifies that the time to reach the end is
  // shorter.
  config.set_r(1);
  SetConfig(config);
  dut_->CalcNextUpdateTime(*context_, &actions);
  EXPECT_LT(actions.time, kZeroAccelerationTime);

  // Sets `r` to be negative and verifies that the time to reach the end is
  // longer.
  config.set_r(-1);
  SetConfig(config);
  dut_->CalcNextUpdateTime(*context_, &actions);
  EXPECT_GT(actions.time, kZeroAccelerationTime);
}

// Same as the previous unit test except with_s is false.
TEST_F(MaliputRailcarTest, DoCalcNextUpdateTimeMonolaneAgainstS) {
  EXPECT_NO_FATAL_FAILURE(InitializeCurvedMonoLane(0 /* start_time */,
                                                   false /* with_s */));
  const maliput::api::Lane* lane = road_->junction(0)->segment(0)->lane(0);
  const double kSpeed(10);

  systems::UpdateActions<double> actions;

  MaliputRailcarConfig<double> config;
  config.set_r(0);
  config.set_h(0);
  config.set_initial_speed(kSpeed);
  config.set_max_speed(30);
  config.set_velocity_limit_kp(8);
  SetConfig(config);

  // Computes the time to reach the end of the lane assuming a speed of kSpeed
  // and zero acceleration.
  const double kZeroAccelerationTime = lane->length() / kSpeed;

  // Verifies that the time till reaching the end of the lane is equal to the
  // lane length divided by the vehicle's speed.
  context_->set_time(0);
  continuous_state()->set_s(lane->length());
  continuous_state()->set_speed(kSpeed);
  dut_->CalcNextUpdateTime(*context_, &actions);
  EXPECT_DOUBLE_EQ(actions.time, kZeroAccelerationTime);

  // Sets `r` to be positive and verifies that the time to reach the end is
  // shorter.
  config.set_r(1);
  SetConfig(config);
  dut_->CalcNextUpdateTime(*context_, &actions);
  EXPECT_LT(actions.time, kZeroAccelerationTime);

  // Sets `r` to be negative and verifies that the time to reach the end is
  // longer.
  config.set_r(-1);
  SetConfig(config);
  dut_->CalcNextUpdateTime(*context_, &actions);
  EXPECT_GT(actions.time, kZeroAccelerationTime);
}

// Tests the ability for a MaliputRailcar to traverse lane boundaries when the
// s-curve of the continuing lane is consistent (i.e., in the same direction) as
// the s-curve of the initial lane.
TEST_F(MaliputRailcarTest, TraverseLaneBoundaryConsistentS) {
  EXPECT_NO_FATAL_FAILURE(InitializeTwoLaneSegment(
      0 /* start_time */, false /* with_s */, false /* flip_curve_lane */));

  // std::cout << "road_->num_junctions() = " << road_->num_junctions() << std::endl;
  // for (int i = 0; i < road_->num_junctions(); ++i) {
  //   const maliput::api::Junction* junc = road_->junction(i);
  //   std::cout << " - junction " << i << " num_segments = " << junc->num_segments() << std::endl;
  //   for (int j = 0; j < junc->num_segments(); ++j) {
  //     const maliput::api::Segment* seg = junc->segment(j);
  //     std::cout << "  - segment " << j << " num_lanes = " << seg->num_lanes() << std::endl;
  //     for (int k = 0; k < seg->num_lanes(); ++k) {
  //       const maliput::api::Lane* l = seg->lane(k);
  //       std::cout << "    - lane " << k << " length = " << l->length() << std::endl;
  //     }
  //   }
  // }

  // std::cout << "road_->num_branch_points() = " << road_->num_branch_points() << std::endl;

  const maliput::api::Lane* straight_lane{};
  const maliput::api::Lane* curved_lane{};
  std::tie(straight_lane, curved_lane) = GetStraightAndCurvedLanes();

  // Verifies that the end of the straight lane is connected to the start of
  // the curved lane.
  std::unique_ptr<LaneEnd> straight_lane_end =
      straight_lane->GetDefaultBranch(LaneEnd::kFinish);
  ASSERT_NE(straight_lane_end, nullptr);
  EXPECT_EQ(straight_lane_end->end, LaneEnd::kStart);
  EXPECT_EQ(straight_lane_end->lane, curved_lane);

  const double kForwardSpeed(10);

  MaliputRailcarConfig<double> config;
  config.set_r(0);
  config.set_h(0);
  config.set_initial_speed(kForwardSpeed);
  config.set_max_speed(30);
  config.set_velocity_limit_kp(8);
  SetConfig(config);

  context_->set_time(straight_lane->length() / kForwardSpeed);

  // Verifies that MaliputRailcar::DoCalcUnrestrictedUpdate() selects the
  // start of curved lane when it is traveling forward and reaches the end of
  // the straight lane.
  continuous_state()->set_s(straight_lane->length());
  continuous_state()->set_speed(kForwardSpeed);
  lane_direction().lane = straight_lane;
  lane_direction().with_s = true;

  systems::DiscreteEvent<double> event;
  event.action = systems::DiscreteEvent<double>::kUnrestrictedUpdateAction;

  dut_->CalcUnrestrictedUpdate(*context_, event, context_->get_mutable_state());

  EXPECT_EQ(continuous_state()->s(), 0);
  EXPECT_EQ(continuous_state()->speed(), kForwardSpeed);
  EXPECT_EQ(lane_direction().with_s, true);
  EXPECT_EQ(lane_direction().lane, curved_lane);
}

// Tests the ability for a MaliputRailcar to traverse lane boundaries when the
// s-curve of the continuing lane is inconsistent (i.e., in the opposite
// direction) as the s-curve of the initial lane.
TEST_F(MaliputRailcarTest, TraverseLaneBoundaryInconsistentS) {
  EXPECT_NO_FATAL_FAILURE(InitializeTwoLaneSegment(
      0 /* start_time */, false /* with_s */, true /* flip_curve_lane */));
  const maliput::api::Lane* straight_lane{};
  const maliput::api::Lane* curved_lane{};
  std::tie(straight_lane, curved_lane) = GetStraightAndCurvedLanes();

  // Verifies that the end of the straight lane is connected to the end of
  // the curved lane.
  std::unique_ptr<LaneEnd> straight_lane_end =
      straight_lane->GetDefaultBranch(LaneEnd::kFinish);
  ASSERT_NE(straight_lane_end, nullptr);
  EXPECT_EQ(straight_lane_end->end, LaneEnd::kFinish);
  EXPECT_EQ(straight_lane_end->lane, curved_lane);

  const double kForwardSpeed(10);

  MaliputRailcarConfig<double> config;
  config.set_r(0);
  config.set_h(0);
  config.set_initial_speed(kForwardSpeed);
  config.set_max_speed(30);
  config.set_velocity_limit_kp(8);
  SetConfig(config);

  context_->set_time(straight_lane->length() / kForwardSpeed);

  // Verifies that MaliputRailcar::DoCalcUnrestrictedUpdate() selects the
  // start of curved lane when it is traveling forward and reaches the end of
  // the straight lane.
  continuous_state()->set_s(straight_lane->length());
  continuous_state()->set_speed(kForwardSpeed);
  lane_direction().lane = straight_lane;
  lane_direction().with_s = true;

  systems::DiscreteEvent<double> event;
  event.action = systems::DiscreteEvent<double>::kUnrestrictedUpdateAction;

  dut_->CalcUnrestrictedUpdate(*context_, event, context_->get_mutable_state());

  EXPECT_EQ(continuous_state()->s(), curved_lane->length());
  EXPECT_EQ(continuous_state()->speed(), kForwardSpeed);
  EXPECT_EQ(lane_direction().with_s, false);
  EXPECT_EQ(lane_direction().lane, curved_lane);
}

}  // namespace
}  // namespace automotive
}  // namespace drake
