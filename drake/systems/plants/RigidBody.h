#pragma once

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <string>

#include "drake/common/eigen_types.h"
#include "drake/drakeRBM_export.h"
#include "drake/systems/plants/collision/DrakeCollision.h"
#include "drake/systems/plants/joints/DrakeJoint.h"

class DRAKERBM_EXPORT RigidBody {
 public:
  RigidBody();

  /**
   * Returns the name of this rigid body.
   */
  const std::string& name() const;

  /**
   * An accessor for the name of the model or robot that this rigid body is
   * a part of.
   *
   * @return The name of the model that this rigid body belongs to.
   */
  const std::string& model_name() const;

  /**
   * Returns the ID of the model to which this rigid body belongs.
   */
  int get_model_id() const;

  /**
   * Sets the ID of the model to which this rigid body belongs.
   */
  void set_model_id(int model_id);

  /**
   * Sets the parent joint through which this rigid body connects to its parent
   * rigid body.
   *
   * @param[in] joint The parent joint of this rigid body. Note that this
   * rigid body assumes ownership of this joint.
   */
  void setJoint(std::unique_ptr<DrakeJoint> joint);

  /**
   * An accessor to this rigid body's parent joint. By "parent joint" we
   * mean the joint through which this rigid body connects to its parent rigid
   * body in the rigid body tree.
   *
   * @return The parent joint of this rigid body.
   */
  const DrakeJoint& getJoint() const;

  bool hasParent() const;

  void set_parent(RigidBody* parent) {
    parent_ = parent;
  }

  RigidBody* get_parent() {
    return parent_;
  }

  void set_body_index(int body_index) {
    body_index_ = body_index;
  }

  int get_body_index() const {
    return body_index_;
  }

  void set_position_num_start(int position_num_start) {
    position_num_start_ = position_num_start;
  }

  int get_position_num_start() const {
    return position_num_start_;
  }

  void set_velocity_num_start(int velocity_num_start) {
    velocity_num_start_ = velocity_num_start;
  }

  int get_velocity_num_start() const {
    return velocity_num_start_;
  }

  void set_mass(double mass) {
    mass_ = mass;
  }

  double get_mass() const {
    return mass_;
  }

  void set_com(Eigen::Vector3d& com) {
    com_ = com;
  }

  const Eigen::Vector3d& get_com() const {
    return com_;
  }

  void set_I(drake::SquareTwistMatrix<double>& I) {
    I_ = I;
  }
  /**
   * Checks if a particular rigid body is the parent of this rigid body.
   *
   * @param[in] other The potential parent of this rigid body.
   * @return true if the supplied rigid body parameter other is the parent of
   * this rigid body.
   */
  bool has_as_parent(const RigidBody& other) const { return parent_ == &other; }

  void addVisualElement(const DrakeShapes::VisualElement& elements);

  const DrakeShapes::VectorOfVisualElements& getVisualElements() const;

  void setCollisionFilter(const DrakeCollision::bitmask& group,
                          const DrakeCollision::bitmask& ignores);

  const DrakeCollision::bitmask& getCollisionFilterGroup() const {
    return collision_filter_group;
  }
  void setCollisionFilterGroup(const DrakeCollision::bitmask& group) {
    this->collision_filter_group = group;
  }

  const DrakeCollision::bitmask& getCollisionFilterIgnores() const {
    return collision_filter_ignores;
  }
  void setCollisionFilterIgnores(const DrakeCollision::bitmask& ignores) {
    this->collision_filter_ignores = ignores;
  }

  void addToCollisionFilterGroup(const DrakeCollision::bitmask& group) {
    this->collision_filter_group |= group;
  }
  void ignoreCollisionFilterGroup(const DrakeCollision::bitmask& group) {
    this->collision_filter_ignores |= group;
  }
  void collideWithCollisionFilterGroup(const DrakeCollision::bitmask& group) {
    this->collision_filter_ignores &= ~group;
  }

  // TODO(amcastro-tri): Change to is_adjacent_to().
  bool adjacentTo(const RigidBody& other) const;

  bool CollidesWith(const RigidBody& other) const {
    bool ignored =
        this == &other || adjacentTo(other) ||
        (collision_filter_group & other.getCollisionFilterIgnores()).any() ||
        (other.getCollisionFilterGroup() & collision_filter_ignores).any();
    return !ignored;
  }

  bool appendCollisionElementIdsFromThisBody(
      const std::string& group_name,
      std::vector<DrakeCollision::ElementId>& ids) const;

  bool appendCollisionElementIdsFromThisBody(
      std::vector<DrakeCollision::ElementId>& ids) const;

  /**
   * Transforms all of the visual, collision, and inertial elements associated
   * with this body to the proper joint frame.  This is necessary, for instance,
   * to support SDF loading where the child frame can be specified independently
   * from the joint frame. In our RigidBodyTree classes, the body frame IS the
   * joint frame.
   *
   * @param transform_body_to_joint The transform from this body's frame to the
   * joint's frame.
   */
  void ApplyTransformToJointFrame(
      const Eigen::Isometry3d& transform_body_to_joint);

  friend std::ostream& operator<<(std::ostream& out, const RigidBody& b);

  // TODO(amcastro-tri): move to a better place (h + cc files).
  class DRAKERBM_EXPORT CollisionElement : public DrakeCollision::Element {
   public:
    CollisionElement(const CollisionElement& other);
    // TODO(amcastro-tri): The RigidBody should be const?
    // TODO(amcastro-tri): It should not be possible to construct a
    // CollisionElement without specifying a geometry. Remove this constructor.
    CollisionElement(const Eigen::Isometry3d& T_element_to_link,
                     const RigidBody* const body);
    CollisionElement(const DrakeShapes::Geometry& geometry,
                     const Eigen::Isometry3d& T_element_to_link,
                     const RigidBody* const body);
    virtual ~CollisionElement() {}

    CollisionElement* clone() const override;

    bool CollidesWith(const DrakeCollision::Element* other) const override;

#ifndef SWIG
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
  };

#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif



 private:
  // The name of this rigid body.
  std::string name_;

  /// The name of the model to which this rigid body belongs.
  std::string model_name_;

  /// A unique ID for each model. It uses 0-index, starts from 0.
  int model_id;
  // note: it's very ugly, but parent, dofnum, and pitch also exist currently
  // (independently) at the RigidBodyTree level to represent the featherstone
  // structure.  this version is for the kinematics.

  std::unique_ptr<DrakeJoint> joint;

  DrakeCollision::bitmask collision_filter_group;

  DrakeCollision::bitmask collision_filter_ignores;

  // TODO(amcastro-tri): Make it private and change to parent_.
  /// The rigid body that's connected to this rigid body's joint.
  RigidBody* parent_;

  /// The index of this rigid body in the rigid body tree.
  int body_index_;

  /// The starting index of this rigid body's joint's position value(s) within
  /// the parent tree's state vector.
  int position_num_start_;

  /// The starting index of this rigid body's joint's velocity value(s) within
  /// the parent tree's state vector.
  int velocity_num_start_;

  /// A list of visual elements for this RigidBody
  DrakeShapes::VectorOfVisualElements visual_elements;

  std::vector<DrakeCollision::ElementId> collision_element_ids;
  std::map<std::string, std::vector<DrakeCollision::ElementId> >
      collision_element_groups;

  Eigen::Matrix3Xd contact_pts;

  /// The mass of this rigid body.
  double mass_;

  /// The center of mass of this rigid body.
  Eigen::Vector3d com_;

  /// The spatial rigid body inertia of this rigid body.
  drake::SquareTwistMatrix<double> I_;
};
