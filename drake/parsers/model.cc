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

std::vector<const RigidBody*> Model::GetRigidBodies() const {
  std::vector<const RigidBody*> result;
  for (auto& map_entry : rigid_bodies_) {
    const RigidBody* rigid_body = (&map_entry.second)->get();
    result.push_back(rigid_body);
  }
  return result;
}

}  // namespace parsers
}  // namespace drake