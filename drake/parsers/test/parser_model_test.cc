#include "gtest/gtest.h"

#include "drake/parsers/model.h"
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

  std::vector<const RigidBody*> rigid_bodies = model.GetRigidBodies();
  EXPECT_EQ(rigid_bodies.size(), model.GetNumberOfRigidBodies());

  for (auto body : rigid_bodies) {
    EXPECT_TRUE(body->name() == kBodyName1 || body->name() == kBodyName2);
    EXPECT_TRUE(body->model_name() == kModelName1 ||
        body->model_name() == kModelName2);
  }
}