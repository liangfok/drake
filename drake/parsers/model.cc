#include "drake/parsers/model.h"

namespace drake {
namespace parsers {

Model::Model(const std::string& model_name) :
    model_name_(model_name) { }

void Model::AddRigidBody(std::unique_ptr<RigidBody> rigid_body) {
  rigid_bodies_[rigid_body->name()] = std::move(rigid_body);
}

int Model::GetNumberOfRigidBodies() const {
  return static_cast<int>(rigid_bodies_.size());
}

bool Model::HasRigidBody(const std::string& name) const {
  return rigid_bodies_.find(name) != rigid_bodies_.end();
}

RigidBody& Model::GetMutableRigidBody(const std::string& name) const {
  if (HasRigidBody(name)) {
    return *(rigid_bodies_.find(name)->second.get());
  } else {
    throw std::runtime_error("Model has no body named \"" + name + "\".");
  }
}

const RigidBody& Model::GetRigidBody(const std::string& name) const {
  if (HasRigidBody(name)) {
    return *(rigid_bodies_.find(name)->second.get());
  } else {
    throw std::runtime_error("Model has no body named \"" + name + "\".");
  }
}

std::vector<RigidBody*> Model::GetMutableRigidBodies() {
  std::vector<RigidBody*> result;
  for (auto& map_entry : rigid_bodies_) {
    RigidBody* rigid_body = (&map_entry.second)->get();
    result.push_back(rigid_body);
  }
  return result;
}

std::vector<const RigidBody*> Model::GetRigidBodies() const {
  std::vector<const RigidBody*> result;
  for (auto& map_entry : rigid_bodies_) {
    const RigidBody* rigid_body = (&map_entry.second)->get();
    result.push_back(rigid_body);
  }
  return result;
}

void Model::AddJoint(std::unique_ptr<DrakeJoint> joint) {
  joints_[joint->getName()] = std::move(joint);
}

int Model::GetNumberOfJoints() const {
  return static_cast<int>(joints_.size());
}

bool Model::HasJoint(const std::string& name) const {
  return joints_.find(name) != joints_.end();
}

DrakeJoint& Model::GetMutableJoint(const std::string& name) const {
  if (HasJoint(name)) {
    return *(joints_.find(name)->second.get());
  } else {
    throw std::runtime_error("Model has no joint named \"" + name + "\".");
  }
}

const DrakeJoint& Model::GetJoint(const std::string& name) const {
  if (HasJoint(name)) {
    return *(joints_.find(name)->second.get());
  } else {
    throw std::runtime_error("Model has no joint named \"" + name + "\".");
  }
}

std::vector<DrakeJoint*> Model::GetMutableJoints() {
  std::vector<DrakeJoint*> result;
  for (auto& map_entry : joints_) {
    DrakeJoint* joint = (&map_entry.second)->get();
    result.push_back(joint);
  }
  return result;
}

std::vector<const DrakeJoint*> Model::GetJoints() const {
  std::vector<const DrakeJoint*> result;
  for (auto& map_entry : joints_) {
    const DrakeJoint* joint = (&map_entry.second)->get();
    result.push_back(joint);
  }
  return result;
}

}  // namespace parsers
}  // namespace drake