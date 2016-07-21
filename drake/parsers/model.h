#pragma once

#include "drake/drakeRBM_export.h"
#include "drake/systems/plants/RigidBody.h"
#include "drake/systems/plants/RigidBodyFrame.h"
#include "drake/systems/plants/joints/DrakeJoint.h"

namespace drake {
namespace parsers {

/**
 * A container that holds all elements of a model. This container is used to
 * transfer state between various parsers and the `RigidBodySystem`.
 */
class DRAKERBM_EXPORT Model {
 public:
  /**
   * The constructor.
   *
   * @param model_name The name of the model.
   */
  explicit Model(const std::string& model_name);

  /**
   * Adds a rigid body to this model. Ownership is transferred to this object.
   * Aliases to this rigid body is guaranteed to remain valid for the lifetime
   * of this object.
   */
  void AddRigidBody(std::unique_ptr<RigidBody> rigid_body);

  /**
   * Returns the number of rigid bodies within this model.
   */
  int GetNumberOfRigidBodies() const;

  /**
   * Returns a reference to a rigid body named @p name. If no such rigid body
   * exists, throw a `std::runtime_error`.
   */
  RigidBody& GetRigidBody(const std::string& name) const;

  /**
   * Returns a const reference to a rigid body named @p name. If no such rigid
   * body exists, throw a `std::runtime_error`.
   */
  const RigidBody& GetRigidBody(const std::string& name) const;

  /**
   * Returns a vector of pointers to the `RigidBody` objects within the model.
   */
  std::vector<RigidBody*> GetRigidBodies() const;

 private:
  std::string model_name_ {};

  std::map<const std::string name, std::unique_ptr<RigidBody>> rigid_bodies_;

  // std::vector<std::unique_ptr<DrakeJoint>> joints_;

  // std::vector<unique_ptr<RigidBodyFrame>> frames_;

  // Floating Joint
  // see RigidBodyTree::AddFloatingJoint

  // std::vector<unique_ptr<RigidBodyLoop>> loops_;
};

}  // namespace parsers
}  // namespace drake

