#include <iostream>

#include <lcm/lcm-cpp.hpp>

#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_simulation.h"
#include "drake/robot_plan_t.hpp"
#include "drake/systems/plants/RigidBodyTree.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

const char* kChannelName = "COMMITTED_ROBOT_PLAN";

/**
 * Generates a joint-space trajectory for the Kuka IIWA robot. This trajectory
 * is saved in a robot_plan_t LCM message. The message is then published on
 * LCM channel kChannelName.
 */
int DoMain(int argc, const char* argv[]) {
  // Waits for the user to type a key.
  std::cout << "Please press any key to continue..." << std::endl;
  getchar();

  // Instantiates a RigidBodyTree containing an IIWA robot instance.
  std::shared_ptr<RigidBodyTree> tree(new RigidBodyTree(
      drake::GetDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf",
      DrakeJoint::FIXED));

  // Generates the joint space trajectory.
  std::cout << "Generating joint space trajectory..." << std::endl;
  Eigen::MatrixXd joint_trajectories;
  std::vector<double> time_stamps;
  GenerateIKDemoJointTrajectory(tree, &joint_trajectories, &time_stamps);

  // Saves the joint space trajectory in to an LCM message.
  std::cout << "Saving joint space trajectory into message..." << std::endl;
  robot_plan_t plan_message;

  plan_message.num_states = time_stamps.size();
  plan_message.plan.resize(time_stamps.size());
  for (int i = 0; i < time_stamps.size(); ++i) {
    bot_core::robot_state_t robot_state;

    // Note that this is multipled by kPlantime in kuka_plan_runner.cc, method
    // HandlePlan().
    robot_state.utime = i;

    robot_state.num_joints = joint_trajectories.num_rows();

    robot_state.joint_name.resize(robot_state.num_joints);
    robot_state.joint_position.resize(robot_state.num_joints);

    for (int j = 0; j < robot_state.num_joints) {
      // The (i + 2) in the line below is to skip the world and root link of the
      // robot.
      const DrakeJoint& joint = tree->get_body(j + 2).getJoint();

      // For now assume each joint only has one position DOF.
      // TODO(liang.fok) Generalize to support multi-DOF joints
      robot_state.joint_name[j] = joint.getName(0);

      // Assume each joint only has 1 DOF.
      robot_state.joint_position[j] = joint_trajectories(j, i);
    }

    // The following are not used by kuka_plan_runner.cc, method HandlePlan().
    // robot_state.pose = ; // position_3d_t
    // robot_state.twist; // bot_core::twist_t
    // robot_state.force_torque = ; // bot_core::force_torque_t
    // robot_state.joint_effort = ; // std::vector< float >
    // robot_state.joint_velocity = ; // std::vector< float >

    plan_message.plan.push_back(robot_state);
  }

  // Publish the LCM message.
  std::cout << "Publishing the message..." << std::endl;
  lcm::LCM lcm;
  lcm.publish(kChannelName, &plan);

  std::cout << "Done..." << std::endl;
  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake


int main(int argc, const char* argv[]) {
  return drake::examples::kuka_iiwa_arm::DoMain(argc, argv);
}
