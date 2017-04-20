#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/rendering/frame_velocity.h"

namespace drake {
namespace systems {
namespace rendering {

// TODO(david-german-tri, SeanCurtis-TRI): Subsume this functionality into
// GeometryWorld/GeometrySystem when they become available.

// TODO(david-german-tri): Consider renaming this to FrameKinematicsBundle,
// since it contains both poses and velocities.

/// PoseBundle is a container for a set of poses, represented by an Isometry3,
/// and corresponding velocities, represented by a FrameVelocity. The poses and
/// velocities are expressed in the world frame: X_WFi, V_WFi. Each pose has a
/// name and a model instance ID.  If two poses in the bundle have the same
/// model instance ID, they must not have the same name.
///
/// This class is explicitly instantiated for the following scalar types. No
/// other scalar types are supported.
/// - double
/// - AutoDiffXd
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///           Only double and AutoDiffXd are supported.
template <typename T>
class PoseBundle {
 public:
  explicit PoseBundle(int num_poses);
  ~PoseBundle();

  int get_num_poses() const;
  const Isometry3<T>& get_pose(int index) const;
  void set_pose(int index, const Isometry3<T>& pose);

  const FrameVelocity<T>& get_velocity(int index) const;
  void set_velocity(int index, const FrameVelocity<T>& velocity);

  const std::string& get_name(int index) const;
  void set_name(int index, const std::string& name);

  int get_model_instance_id(int index) const;
  void set_model_instance_id(int index, int id);

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PoseBundle)

  std::unique_ptr<PoseBundle<AutoDiffXd>> ToAutoDiffXd() const;

  std::string ToString(const Isometry3<T> pose, const std::string& prefix = "")
      const {
    std::stringstream temp_buffer;
    temp_buffer << pose.matrix();
    std::string s;
    std::stringstream result_buffer;
    while (std::getline(temp_buffer, s, '\n')) {
      // std::cout << "*** Read line " << s << std::endl;
      result_buffer << prefix << s << "\n";
    }
    // std::cout << "\n\nResult:\n" << result_buffer.str();
    return result_buffer.str();
  }

  /// Returns a string representation of this class.
  std::string ToString(const std::string& prefix = "") const {
    std::stringstream buffer;
    buffer << prefix << "PoseBundle of size " << get_num_poses() << "\n";
    for (int i = 0; i < get_num_poses(); ++i) {
      buffer << prefix << "  - Index " << i << ":\n"
             << prefix << "    - Name: " << names_.at(i) << "\n"
             << prefix << "    - ID: " << ids_.at(i) << "\n"
             << prefix << "    - Pose:\n"
             << ToString(poses_.at(i), prefix + "      ")
             << prefix << "    - Velocity: " << velocities_.at(i) << "\n";
    }
    return buffer.str();
  }

  friend std::ostream& operator<<(std::ostream& out, const PoseBundle&
      pose_bundle) {
    return out << pose_bundle.ToString();
  }

 private:
  std::vector<Isometry3<T>> poses_;
  std::vector<FrameVelocity<T>> velocities_;
  std::vector<std::string> names_;
  std::vector<int> ids_;
};

}  // namespace rendering
}  // namespace systems
}  // namespace drake
