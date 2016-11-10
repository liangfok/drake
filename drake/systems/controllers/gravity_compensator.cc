#include "drake/systems/controllers/gravity_compensator.h"

#include "drake/multibody/kinematics_cache.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace systems {

template <typename T>
GravityCompensator<T>::GravityCompensator(
    const RigidBodyTree<T>& rigid_body_tree)
    : rigid_body_tree_(rigid_body_tree) {
  this->DeclareInputPort(kVectorValued, rigid_body_tree.get_num_positions(),
                         kContinuousSampling);
  this->DeclareOutputPort(kVectorValued, rigid_body_tree_.get_num_actuators(),
                          kContinuousSampling);
}

template <typename T>
void GravityCompensator<T>::EvalOutput(const Context<T>& context,
                                       SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  Eigen::VectorXd q = this->EvalEigenVectorInput(context, 0);

  KinematicsCache<T> cache = rigid_body_tree_.doKinematics(q);
  eigen_aligned_std_unordered_map<RigidBody const*, drake::TwistVector<T>>
      f_ext;
  f_ext.clear();

  Eigen::VectorXd g = rigid_body_tree_.dynamicsBiasTerm(cache, f_ext,
      false /* include velocity terms */);

  // The size of this system's output vector is equal to the number of
  // actuators while the size of `g` is equal to the number of DOFs. Thus, we
  // need to extract from `g` the torque / force commands corresponding to the
  // actuators that are used.
  Eigen::VectorXd actuated_g(rigid_body_tree_.get_num_actuators());
  for (int actuator_index = 0;
      actuator_index < rigid_body_tree_.get_num_actuators(); ++actuator_index) {
    int gravity_index =
        rigid_body_tree_.actuators.at(actuator_index).body_->
            get_position_start_index();
    actuated_g[actuator_index] = g[gravity_index];
  }
  DRAKE_ASSERT(actuated_g.size() ==
               System<T>::GetMutableOutputVector(output, 0).size());
  System<T>::GetMutableOutputVector(output, 0) = actuated_g;
}

template class GravityCompensator<double>;
// TODO(naveenoid): Get the AutoDiff working as in the line below.
// template class GravityCompensator<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
