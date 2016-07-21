#pragma once

#include "drake/drakeParsers_export.h"
#include "drake/systems/plants/joints/DrakeJoint.h"
#include "drake/systems/plants/RigidBody.h"
#include "drake/systems/plants/RigidBodyFrame.h"


namespace drake {
namespace parsers {

/**
 * A container that holds all elements of a model. This container is used to
 * transfer state between various parsers and the `RigidBodySystem`.
 */
class DRAKEPARSERS_EXPORT Model {
 public:
  /**
   * The constructor.
   *
   * @param model_name The name of the model.
   */
  explicit Model(const std::string& model_name);

  /**
   * Adds a rigid body to this model. Ownership is transferred to this object.
   * Aliases to this rigid body are guaranteed to remain valid for the lifetime
   * of this object.
   */
  void AddRigidBody(std::unique_ptr<RigidBody> rigid_body);

  /**
   * Returns `true` if and only if this model has a rigid body with a name equal
   * to @p name.
   */
  bool HasRigidBody(const std::string& name) const;

  /**
   * Returns the number of rigid bodies within this model.
   */
  int GetNumberOfRigidBodies() const;

  /**
   * Returns a reference to a rigid body named @p name. If no such rigid body
   * exists, throw a `std::runtime_error`.
   */
  RigidBody& GetMutableRigidBody(const std::string& name) const;

  /**
   * Returns a const reference to a rigid body named @p name. If no such rigid
   * body exists, throw a `std::runtime_error`.
   */
  const RigidBody& GetRigidBody(const std::string& name) const;

  /**
   * Returns a vector of pointers to the `RigidBody` objects within the model.
   */
  std::vector<RigidBody*> GetMutableRigidBodies();

  /**
   * Returns a vector of const pointers to the `RigidBody` objects within the
   * model.
   */
  std::vector<const RigidBody*> GetRigidBodies() const;

  /**
   * Adds a joint to this model. Ownership is transferred to this object.
   * Aliases to this joint are guaranteed to remain valid for the lifetime
   * of this object.
   */
  void AddJoint(std::unique_ptr<DrakeJoint> joint);

  /**
   * Returns `true` if and only if this model has a joint with a name equal
   * to @p name.
   */
  bool HasJoint(const std::string& name) const;

  /**
   * Returns the number of joints within this model.
   */
  int GetNumberOfJoints() const;

  /**
   * Returns a reference to a joint named @p name. If no such joint exists,
   * throw a `std::runtime_error`.
   */
  DrakeJoint& GetMutableJoint(const std::string& name) const;

  /**
   * Returns a const reference to a joint named @p name. If no such joint
   * exists, throw a `std::runtime_error`.
   */
  const DrakeJoint& GetJoint(const std::string& name) const;

  /**
   * Returns a vector of pointers to the `DrakeJoint` objects within the model.
   */
  std::vector<DrakeJoint*> GetMutableJoints();

  /**
   * Returns a vector of const pointers to the `DrakeJoint` objects within the
   * model.
   */
  std::vector<const DrakeJoint*> GetJoints() const;
 private:
  std::string model_name_ {};

  // A map storing the RigidBody objects that are part of this model.
  // The key is the name of a rigid body.
  std::map<const std::string, std::unique_ptr<RigidBody>> rigid_bodies_;

  // A map storing the DrakeJoint objects that are part of this model.
  // The key is the name of a joint.
  std::map<const std::string, std::unique_ptr<DrakeJoint>> joints_;

  // std::vector<unique_ptr<RigidBodyFrame>> frames_;

  // Floating Joint
  // see RigidBodyTree::AddFloatingJoint

  // std::vector<unique_ptr<RigidBodyLoop>> loops_;
};

}  // namespace parsers
}  // namespace drake

