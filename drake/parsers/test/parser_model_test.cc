#include "gtest/gtest.h"

#include "drake/parsers/model.h"
#include "drake/systems/plants/joints/DrakeJoint.h"
#include "drake/systems/plants/joints/RevoluteJoint.h"
#include "drake/systems/plants/RigidBody.h"

using drake::parsers::Model;

// Tests the ability to add and obtain RigidBody objects from a Model object.
GTEST_TEST(ModelTest, TestAddAndGetRigidBody) {
  // Instantiates a Model object.
  const std::string model_name = "BazModel";
  Model model(model_name);

  // Adds a couple RigidBody objects to the Model object.
  const std::string kBodyName1 = "FooBody";
  const std::string kBodyName2 = "BarBody";
  const std::string kModelName1 = "MG{=?82A+_3e/p'Jps6j*Uuc";
  const std::string kModelName2 = ":6p8[+E?j3(frx/GyggS+JW.";
  {
    std::unique_ptr<RigidBody> rigid_body_1(new RigidBody());
    rigid_body_1->name_ = kBodyName1;
    rigid_body_1->model_name_ = kModelName1;

    std::unique_ptr<RigidBody> rigid_body_2(new RigidBody());
    rigid_body_2->name_ = kBodyName2;
    rigid_body_2->model_name_ = kModelName2;

    model.AddRigidBody(std::move(rigid_body_1));
    model.AddRigidBody(std::move(rigid_body_2));
  }

  // Verifies that the number of rigid bodies reported by
  // Model::GetNumberOfRigidBodies() is correct.
  EXPECT_EQ(model.GetNumberOfRigidBodies(), 2);

  // Verifies that the Model's RigidBody accessors are working.
  EXPECT_FALSE(model.HasRigidBody("Non-Existent rigid body"));
  EXPECT_TRUE(model.HasRigidBody(kBodyName1));
  EXPECT_TRUE(model.HasRigidBody(kBodyName2));

  EXPECT_THROW(model.GetRigidBody("Non-Existent rigid body"),
      std::runtime_error);

  RigidBody& body_ref_1 = model.GetMutableRigidBody(kBodyName1);
  EXPECT_EQ(body_ref_1.model_name(), kModelName1);

  RigidBody& body_ref_2 = model.GetMutableRigidBody(kBodyName2);
  EXPECT_EQ(body_ref_2.model_name(), kModelName2);

  const RigidBody& const_body_ref_1 = model.GetRigidBody(kBodyName1);
  EXPECT_EQ(const_body_ref_1.model_name(), kModelName1);

  const RigidBody& const_body_ref_2 = model.GetRigidBody(kBodyName2);
  EXPECT_EQ(const_body_ref_2.model_name(), kModelName2);

  std::vector<RigidBody*> mutable_rigid_bodies = model.GetMutableRigidBodies();
  EXPECT_EQ(mutable_rigid_bodies.size(), model.GetNumberOfRigidBodies());

  for (auto body : mutable_rigid_bodies) {
    EXPECT_TRUE(body->name() == kBodyName1 || body->name() == kBodyName2);
    EXPECT_TRUE(body->model_name() == kModelName1 ||
        body->model_name() == kModelName2);
  }

  std::vector<const RigidBody*> rigid_bodies = model.GetRigidBodies();
  EXPECT_EQ(rigid_bodies.size(), model.GetNumberOfRigidBodies());

  for (auto body : rigid_bodies) {
    EXPECT_TRUE(body->name() == kBodyName1 || body->name() == kBodyName2);
    EXPECT_TRUE(body->model_name() == kModelName1 ||
        body->model_name() == kModelName2);
  }
}

// Tests the ability to add and obtain DrakeJoint objects from a Model object.
GTEST_TEST(ModelTest, TestAddAndGetJoint) {
  // Instantiates a Model object.
  const std::string model_name = "BazModel";
  Model model(model_name);

  // Adds a couple DrakeJoint objects to the Model object.
  const std::string kJointName1 = "FooJoint";
  const std::string kJointName2 = "BarJoint";

  {
    Eigen::Vector3d x_rotation_axis = Eigen::Vector3d::Zero();
    x_rotation_axis[0] = 1;

    std::unique_ptr<DrakeJoint> joint_1(new RevoluteJoint(kJointName1,
        Eigen::Isometry3d::Identity(), x_rotation_axis));

    Eigen::Vector3d joint_2_rotation_axis = Eigen::Vector3d::Zero();
    joint_2_rotation_axis[0] = 1;
    std::unique_ptr<DrakeJoint> joint_2(new RevoluteJoint(kJointName2,
        Eigen::Isometry3d::Identity(), x_rotation_axis));

    model.AddJoint(std::move(joint_1));
    model.AddJoint(std::move(joint_2));
  }

  // Verifies that the number of rigid bodies reported by
  // Model::GetNumberOfRigidBodies() is correct.
  EXPECT_EQ(model.GetNumberOfJoints(), 2);

  // Verifies that the Model's joint accessors are working.
  EXPECT_FALSE(model.HasJoint("Non-Existent joint"));
  EXPECT_TRUE(model.HasJoint(kJointName1));
  EXPECT_TRUE(model.HasJoint(kJointName2));

  EXPECT_THROW(model.GetJoint("Non-Existent joint"),
      std::runtime_error);

  EXPECT_NO_THROW(model.GetMutableJoint(kJointName1));
  EXPECT_NO_THROW(model.GetMutableJoint(kJointName2));

  EXPECT_NO_THROW(model.GetJoint(kJointName1));
  EXPECT_NO_THROW(model.GetJoint(kJointName2));

  std::vector<DrakeJoint*> mutable_joints = model.GetMutableJoints();
  EXPECT_EQ(mutable_joints.size(), model.GetNumberOfJoints());

  for (auto joint : mutable_joints) {
    EXPECT_TRUE(joint->getName() == kJointName1 ||
        joint->getName() == kJointName2);
  }

  std::vector<const DrakeJoint*> joints = model.GetJoints();
  EXPECT_EQ(joints.size(), model.GetNumberOfJoints());

  for (auto joint : joints) {
    EXPECT_TRUE(joint->getName() == kJointName1 ||
        joint->getName() == kJointName2);
  }
}