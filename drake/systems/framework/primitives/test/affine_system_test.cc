#include "drake/systems/framework/primitives/affine_system.h"

#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/system_output.h"

#include "gtest/gtest.h"

using Eigen::MatrixXd;
using Eigen::Vector2d;

using std::make_unique;
using std::move;
using std::unique_ptr;

namespace drake {
namespace systems {
namespace {

GTEST_TEST(AffineSystemTest, BasicTest) {
  MatrixXd A(2, 2);
  A << 1, 2, 3, 4;

  MatrixXd B(2, 2);
  B << 5, 6, 7, 8;

  Vector2d xdot0(3, 4);

  MatrixXd C(2, 2);
  C << 9, 10, 11, 12;

  MatrixXd D(2, 2);
  D << 13, 14, 15, 16;

  Vector2d y0(17, 18);

  auto affine_system = make_unique<AffineSystem<double>>(A, B, xdot0, C, D, y0);

  auto context = affine_system->CreateDefaultContext();

  // Sets the context's input port.
  Vector2d input_vector(1, 4);
  auto input0  = make_unique<BasicVector<double>>(2 /* size */);
  input0->get_mutable_value() << input_vector;
  context->SetInputPort(0,
      make_unique<FreestandingInputPort>(std::move(input0)));

  // Sets the context's state.
  Vector2d state_eigen_vector(2);
  state_eigen_vector << 12, 13;
  auto state_vector = make_unique<BasicVector<double>>(state_eigen_vector);
  auto state = make_unique<ContinuousState<double>>(move(state_vector));
  context->set_continuous_state(move(state));

  auto derivatives = affine_system->AllocateTimeDerivatives();

  EXPECT_NE(derivatives, nullptr);

  affine_system->EvalTimeDerivatives(*context, derivatives.get());

  Vector2d expected_derivative(70, 131);
  EXPECT_EQ(derivatives->get_state().CopyToVector(), expected_derivative);

  Vector2d output_eigen_vector(2);
  output_eigen_vector << 14, 15;
  auto output_vector = make_unique<BasicVector<double>>(output_eigen_vector);
  auto output_value = make_unique<VectorValue<double>>(move(output_vector));

  LeafSystemOutput<double> sys_output;
  sys_output.add_port(move(output_value));

  affine_system->EvalOutput(*context, &sys_output);

  Vector2d expected_output(324, 385);
  EXPECT_EQ(sys_output.get_port(0).get_vector_data<double>()->CopyToVector(),
      expected_output);

  EXPECT_NO_THROW(affine_system->get_input_port());
  EXPECT_NO_THROW(affine_system->get_output_port());
}

}  // namespace
}  // namespace systems
}  // namespace drake
