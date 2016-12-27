#include "drake/systems/sensors/accelerometer.h"

#include "drake/math/quaternion.h"
#include "drake/systems/sensors/accelerometer_output.h"

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using std::make_unique;
using std::move;

namespace drake {

using math::quatRotateVec;

namespace systems {
namespace sensors {

constexpr int Accelerometer::kNumMeasurements;

Accelerometer::Accelerometer(const std::string& name,
                             const RigidBodyPlant<double>& plant,
                             const RigidBodyFrame<double>& frame,
                             bool include_gravity_compensation)
    : name_(name),
      plant_(plant),
      frame_(frame),
      include_gravity_compensation_(include_gravity_compensation) {
  input_port_index_ =
      DeclareInputPort(kVectorValued, get_tree().get_num_positions() +
                                          get_tree().get_num_velocities())
          .get_index();
  output_port_index_ =
      DeclareOutputPort(kVectorValued, kNumMeasurements).get_index();
}

std::unique_ptr<BasicVector<double>> Accelerometer::AllocateOutputVector(
    const OutputPortDescriptor<double>& descriptor) const {
  return std::make_unique<AccelerometerOutput<double>>();
}

const RigidBodyTree<double>& Accelerometer::get_tree() const {
  return plant_.get_rigid_body_tree();
}

void Accelerometer::DoCalcOutput(const systems::Context<double>& context,
                                 systems::SystemOutput<double>* output) const {
  DRAKE_ASSERT_VOID(System<double>::CheckValidContext(context));
  DRAKE_ASSERT_VOID(System<double>::CheckValidOutput(output));

  // Obtains the time derivatives of the position and velocity state. Note that
  // this computes the velocity and acceleration state.
  std::unique_ptr<ContinuousState<double>> derivatives =
      plant_.AllocateTimeDerivatives();
  plant_.CalcTimeDerivatives(context, derivatives.get());
  const auto xdot = derivatives->CopyToVector();
  const auto vdot = xdot.bottomRows(get_tree().get_num_velocities());

  // Computes:
  //
  //  - u: the RigidBodyPlant's state vector. The character `u` is used since it
  //       is this system's input ("u" is the standard variable for system
  //       inputs in the controls community).
  //  - q: the RigidBodyPlant's position state vector.
  //  - v: the RigidBodyPlant's velocity state vector.
  //
  // Note that u = [q, v].
  //
  const VectorXd u = this->EvalEigenVectorInput(context, 0);
  const auto q = u.head(get_tree().get_num_positions());
  const auto v = u.segment(get_tree().get_num_positions(),
                           get_tree().get_num_velocities());
  const KinematicsCache<double> kinematics_cache =
      get_tree().doKinematics(q, v);

  // The sensor's frame coincides with frame_'s origin.
  const Vector3d sensor_origin = Vector3d::Zero();

  const auto J = get_tree().transformPointsJacobian(
      kinematics_cache, sensor_origin, frame_.get_frame_index(),
      RigidBodyTree<double>::kWorldBodyIndex, false /* in_terms_of_qdot */);

  const auto Jdot_times_v = get_tree().transformPointsJacobianDotTimesV(
      kinematics_cache, sensor_origin, frame_.get_frame_index(),
      RigidBodyTree<double>::kWorldBodyIndex);

  const Vector4d quat_world_to_body = get_tree().relativeQuaternion(
      kinematics_cache, RigidBodyTree<double>::kWorldBodyIndex,
      frame_.get_frame_index());

  const Vector3d accel_base = Jdot_times_v + J * vdot;
  Vector3d accel_body = quatRotateVec(quat_world_to_body, accel_base);

  if (include_gravity_compensation_) {
    const Vector3d gravity(0, 0, 9.81);
    accel_body += quatRotateVec(quat_world_to_body, gravity);
  }

  // Saves the acceleration readings into the output port.
  BasicVector<double>* output_vector =
      output->GetMutableVectorData(output_port_index_);
  output_vector->SetFromVector(accel_body);
}

std::ostream& operator<<(std::ostream& out, const Accelerometer& sensor) {
  out << "Accelerometer:\n"
      << "  - name = " << sensor.get_name() << "\n"
      << "  - frame = " << sensor.get_frame().get_name() << "\n";
  return out;
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
