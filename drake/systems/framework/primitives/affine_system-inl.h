#pragma once

/// @file
/// Template method implementations for affine_system.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "drake/systems/framework/primitives/affine_system.h"

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_context.h"

namespace drake {
namespace systems {

template <typename T>
AffineSystem<T>::AffineSystem(const MatrixX<T>& A,
                              const MatrixX<T>& B,
                              const VectorX<T>& xdot0,
                              const MatrixX<T>& C,
                              const MatrixX<T>& D,
                              const VectorX<T>& y0)
    : A_(A), B_(B), C_(C), D_(D), xdot0_(xdot0), y0_(y0) {
  this->DeclareInputPort(kVectorValued, xdot0.size(), kContinuousSampling);
  this->DeclareOutputPort(kVectorValued, xdot0.size(), kContinuousSampling);
}

template <typename T>
std::unique_ptr<ContinuousState<T>> AffineSystem<T>::AllocateContinuousState()
    const {
  auto state = std::make_unique<BasicVector<T>>(xdot0_.size());
  auto result = std::make_unique<ContinuousState<T>>(std::move(state));
  return std::move(result);
}

template <typename T>
const SystemPortDescriptor<T>& AffineSystem<T>::get_input_port() const {
  return System<T>::get_input_port(0);
}

template <typename T>
const SystemPortDescriptor<T>& AffineSystem<T>::get_output_port() const {
  return System<T>::get_output_port(0);
}

template <typename T>
void AffineSystem<T>::EvalTimeDerivatives(
    const Context<T>& context, ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));
  DRAKE_DEMAND(derivatives);

  // Obtains the input vector `u`.
  const BasicVector<T>* input = this->EvalVectorInput(context, 0);
  DRAKE_DEMAND(input);
  auto u = input->get_value();

  // TODO(amcastro-tri): provide a nicer accessor to an Eigen representation for
  // LeafSystem state.
  // Obtains the system state vector `x`.
  auto x = dynamic_cast<const BasicVector<T>&>(
      context.get_continuous_state()->get_state()).get_value();

  VectorX<T> xdot = A_ * x + B_ * u + xdot0_;
  derivatives->get_mutable_state()->SetFromVector(xdot);
}

template <typename T>
void AffineSystem<T>::EvalOutput(const Context<T>& context,
                         SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  // Obtains the input vector `u`.
  const BasicVector<T>* input = this->EvalVectorInput(context, 0);
  DRAKE_DEMAND(input);
  auto u = input->get_value();

  // TODO(amcastro-tri): provide a nicer accessor to an Eigen representation for
  // LeafSystem state.
  // Obtains the system state vector `x`.
  auto x = dynamic_cast<const BasicVector<T>&>(
      context.get_continuous_state()->get_state()).get_value();

  VectorX<T> y = C_ * x + D_ * u + y0_;
  System<T>::GetMutableOutputVector(output, 0) = y;
}

}  // namespace systems
}  // namespace drake
