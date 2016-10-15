#pragma once

#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/**
 * An affine system block. Let `x` be the system state, `u` be the system
 * input vector, `y` be the system output vector, and `A`, `B`, `C`, and `D` be
 * state space matrix coefficients. This system implements the following
 * equations:
 *
 * @f[
 *   \dot{x} = Ax + Bu + \dot{x}_0 \\
 *   y = Cx + Du + y_0
 * @f]
 *
 * This class uses Drake's `-inl.h` pattern.  When seeing linker errors from
 * this class, please refer to http://drake.mit.edu/cxx_inl.html.
 *
 * Instantiated templates for the following scalar types @p T are provided:
 * - double
 *
 * They are already available to link against in drakeSystemFramework.
 *
 * To use other specific scalar types see gain-inl.h.
 *
 * @tparam T The element type of the above-mentioned state vectors and
 * coefficient matrices, which must be a valid Eigen scalar.
 *
 * @ingroup primitive_systems
 */
template <typename T>
class AffineSystem : public LeafSystem<T> {
 public:
  /**
   * Constructs an %Affine system. See the class description for details about
   * the parameters.
   */
  AffineSystem(const MatrixX<T>& A, const MatrixX<T>& B,
               const VectorX<T>& xdot0, const MatrixX<T>& C,
               const MatrixX<T>& D, const VectorX<T>& y0);

  // LeafSystem override.
  std::unique_ptr<ContinuousState<T>> AllocateContinuousState() const override;

  /**
   * Sets @p derivatives to contain `xdot` where `xdot` is defined in the class
   * description.
   */
  void EvalTimeDerivatives(const Context<T>& context,
                           ContinuousState<T>* derivatives) const override;

  /**
   * Sets the output port value equal to `y` where `y` is defined in the class
   * description.
   *
   * If number of connected input or output ports differs from one or, the
   * input ports are not the correct size, std::runtime_error will be thrown.
   */
  void EvalOutput(const Context<T>& context,
                  SystemOutput<T>* output) const override;

  /// Returns the input port.
  const SystemPortDescriptor<T>& get_input_port() const;

  /// Returns the output port.
  const SystemPortDescriptor<T>& get_output_port() const;

 private:
  // TODO(liang.fok): Move these parameters to System<T>::Parameter.
  MatrixX<T> A_;
  MatrixX<T> B_;
  MatrixX<T> C_;
  MatrixX<T> D_;
  VectorX<T> xdot0_;
  VectorX<T> y0_;
};

}  // namespace systems
}  // namespace drake
