#include "drake/systems/sensors/accelerometer.h"

#include <memory>

#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/model_instance_id_table.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/sensors/accelerometer_output.h"

using Eigen::Vector3d;

using std::make_unique;
using std::move;
using std::string;
using std::stringstream;
using std::unique_ptr;

namespace drake {

using parsers::urdf::AddModelInstanceFromUrdfFileToWorld;

namespace systems {
namespace sensors {
namespace {

const char* const kSensorName = "foo sensor";

// Tests Accelerometer's various accessor and streaming to-string methods.
GTEST_TEST(TestAccelerometer, AccessorsAndToStringTest) {
  auto tree = std::make_unique<RigidBodyTree<double>>();
  RigidBodyFrame<double> frame("foo frame", &tree->world(),
                               Eigen::Isometry3d::Identity());
  RigidBodyPlant<double> plant(move(tree));

  // Defines the Device Under Test (DUT).
  Accelerometer dut(kSensorName, plant, frame);

  stringstream string_buffer;
  string_buffer << dut;
  const string dut_string = string_buffer.str();

  EXPECT_NE(dut_string.find("Accelerometer:"), std::string::npos);
  EXPECT_NE(dut_string.find("name ="), std::string::npos);
  EXPECT_NE(dut_string.find("frame ="), std::string::npos);
  EXPECT_EQ(std::count(dut_string.begin(), dut_string.end(), '\n'), 3);
}

// Tests that the accelerometer attached to a single rigid body floating in
// space can measure the effects of gravity.
GTEST_TEST(TestAccelerometer, TestFreeFall) {
  auto tree = std::make_unique<RigidBodyTree<double>>();

  // Adds a box to the RigidBodyTree and obtains its model instance ID.
  const parsers::ModelInstanceIdTable model_instance_id_table =
      AddModelInstanceFromUrdfFileToWorld(
          GetDrakePath() + "/multibody/models/box.urdf",
          drake::multibody::joints::kQuaternion, tree.get());
  const int model_instance_id = model_instance_id_table.at("box");

  // Adds a frame to the RigidBodyTree called "box frame" that is coincident
  // with the "box" body within the RigidBodyTree.
  auto frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "box frame",
      tree->FindBody("box"), Eigen::Isometry3d::Identity());
  tree->addFrame(frame);

  RigidBodyPlant<double> plant(move(tree));

  // Defines the Device Under Test (DUT).
  Accelerometer dut(kSensorName, plant, *frame);

  unique_ptr<Context<double>> context = dut.CreateDefaultContext();
  EXPECT_EQ(context->get_num_input_ports(), 1);
  EXPECT_EQ(context->get_continuous_state_vector().size(), 0);

  const int num_states =
      plant.model_state_output_port(model_instance_id).size();
  const int num_positions = dut.get_tree().get_num_positions();
  const int num_velocities = dut.get_tree().get_num_velocities();

  // The plant has 13 values in its output vector, 7 generalized positions and
  // 6 generalized velocities.
  EXPECT_EQ(num_states, 13);
  EXPECT_EQ(num_positions, 7);
  EXPECT_EQ(num_velocities, 6);

  context->FixInputPort(
      0, BasicVector<double>::Make({0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));

  auto xc_vector = make_unique<BasicVector<double>>(
      VectorX<double>::Zero(num_states).eval());
  auto xc = make_unique<ContinuousState<double>>(move(xc_vector), num_positions,
                                                 num_velocities,
                                                 0 /* num other variables */);
  context->set_continuous_state(move(xc));

  unique_ptr<SystemOutput<double>> output = dut.AllocateOutput(*context);
  ASSERT_EQ(output->get_num_ports(), 1);
  dut.CalcOutput(*context, output.get());

  const Vector3d expected_acceleration(0, 0, 9.81);
  EXPECT_TRUE(CompareMatrices(output->get_vector_data(0)->get_value(),
                              expected_acceleration, 1e-10,
                              MatrixCompareType::absolute));
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
