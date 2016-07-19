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
class DRAKERBM_EXPORT ModelElements {
 public:
  ModelElements() { }

 private:
  std::string model_name_ {};

  std::vector<std::unique_ptr<RigidBody>> rigid_bodies_;

  std::vector<std::unique_ptr<DrakeJoint>> joints_;

  std::vector<unique_ptr<RigidBodyFrame>> frames_;

  // Floating Joint
  // see RigidBodyTree::AddFloatingJoint

  std::vector<unique_ptr<RigidBodyLoop>> loops_;
};

}  // namespace parsers
}  // namespace drake

