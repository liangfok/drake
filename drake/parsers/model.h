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
   * @param model_name The name of this model.
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
   * Returns a vector of pointers to the `RigidBody` objects within this model.
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
   * Returns a vector of pointers to the `DrakeJoint` objects within this model.
   */
  std::vector<DrakeJoint*> GetMutableJoints();

  /**
   * Returns a vector of const pointers to the `DrakeJoint` objects within the
   * model.
   */
  std::vector<const DrakeJoint*> GetJoints() const;

  /**
   * Adds a frame to this model. Ownership is transferred to this object.
   * Aliases to this frame are guaranteed to remain valid for the lifetime
   * of this object.
   */
  void AddFrame(std::unique_ptr<RigidBodyFrame> frame);

  /**
   * Returns `true` if and only if this model has a frame with a name equal
   * to @p name.
   */
  bool HasFrame(const std::string& name) const;

  /**
   * Returns the number of frames within this model.
   */
  int GetNumberOfFrames() const;

  /**
   * Returns a reference to a frame named @p name. If no such frame exists,
   * throw a `std::runtime_error`.
   */
  RigidBodyFrame& GetMutableFrame(const std::string& name) const;

  /**
   * Returns a const reference to a frame named @p name. If no such frame
   * exists, throw a `std::runtime_error`.
   */
  const RigidBodyFrame& GetFrame(const std::string& name) const;

  /**
   * Returns a vector of pointers to the `RigidBodyFrame` objects within this
   * model.
   */
  std::vector<RigidBodyFrame*> GetMutableFrames();

  /**
   * Returns a vector of const pointers to the `RigidBodyFrame` objects within
   * this model.
   */
  std::vector<const RigidBodyFrame*> GetFrames() const;

  /**
   * Adds a loop joint to this model. Ownership is transferred to this object.
   * Aliases to this loop are guaranteed to remain valid for the lifetime
   * of this object.
   */
  void AddLoopJoint(std::unique_ptr<RigidBodyLoop> loop);

  /**
   * Returns `true` if and only if this model has a loop with a name equal
   * to @p name.
   */
  bool HasLoopJoint(const std::string& name) const;

  /**
   * Returns the number of loop joints within this model.
   */
  int GetNumberOfLoopJoints() const;

  /**
   * Returns a reference to a loop joint named @p name. If no such loop joint
   * exists, throw a `std::runtime_error`.
   */
  RigidBodyLoop& GetMutableLoopJoint(const std::string& name) const;

  /**
   * Returns a const reference to a loop joint named @p name. If no such loop
   * joint exists, throw a `std::runtime_error`.
   */
  const RigidBodyLoop& GetLoopJoint(const std::string& name) const;

  /**
   * Returns a vector of pointers to the `RigidBodyLoop` objects within this
   * model.
   */
  std::vector<RigidBodyLoop*> GetMutableLoopJoints();

  /**
   * Returns a vector of const pointers to the `RigidBodyLoop` objects within
   * this model.
   */
  std::vector<const RigidBodyLoop*> GetLoopJoints() const;
 private:
  std::string model_name_ {};

  // A map storing the RigidBody objects that are part of this model.
  // The key is the name of the rigid body.
  std::map<const std::string, std::unique_ptr<RigidBody>> rigid_bodies_;

  // A map storing the DrakeJoint objects that are part of this model.
  // The key is the name of the joint.
  std::map<const std::string, std::unique_ptr<DrakeJoint>> joints_;

  // A map storing the RigidBodyFrame objects that are part of this model.
  // The key is the name of the frame.
  std::map<const std::string, std::unique_ptr<RigidBodyFrame>> frames_;

  // A map storing the RigidBodyLoop objects that are part of this model.
  // The key is the name of the loop joint.
  std::map<const std::string, std::unique_ptr<RigidBodyLoop>> loop_joints_;
};

}  // namespace parsers
}  // namespace drake

