# pragma once

#include <memory>


#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/plants/RigidBodyTree.h"

namespace drake {
namespace systems {

/// A gravity compensator block with input as the state of a RigidBody System
/// `[q, qdot]` and output `y = G(q)`, where, for a given RigidBodyTree, G(q)
/// is a vector of generalised gravity forces corresponding to a given joint
/// configuration q.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
template <typename T>
class GravityCompensator : public LeafSystem<T> {
 public:
  /// Constructs a gravity compensator (corresponding to a RigidBodyTree) with
  /// the dimension of the input and output port equalling the dimension of the
  /// Degrees of Freedom of the RigidBodyTree (positions of a RigidBodySystem)
  /// @param rigit_body_tree_ptr a shared pointer to a RigidBodyTree object
  /// which in turn is externally derived from a RigidBodySystem object to be
  /// controlled.
  GravityCompensator(const RigidBodyTree& world);

  /// Sets the output port value to the generalised gravity forces corresponding
  /// to a joint configuration as specified in the input.
  /// If number of connected input or output ports differs from one or, the
  /// input ports are not of size length_, std::runtime_error will be thrown.
  void EvalOutput(const ContextBase<T>& context,
                  SystemOutput<T>* output) const override;

 private:
  const RigidBodyTree& mbd_world_;
};

}  // namespace systems
}  // namespace drake
