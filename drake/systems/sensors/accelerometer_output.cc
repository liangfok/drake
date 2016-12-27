#include "drake/systems/sensors/accelerometer_output.h"

#include <cmath>

#include "drake/systems/sensors/accelerometer.h"

namespace drake {
namespace systems {
namespace sensors {

template <typename T>
AccelerometerOutput<T>::AccelerometerOutput()
    : BasicVector<double>(Accelerometer::kNumMeasurements) {
  this->SetFromVector(VectorX<double>::Zero(Accelerometer::kNumMeasurements));
}

// template <typename T>
// double ImuSensorOutput<T>::GetAccelerationX() const {
// }

template class AccelerometerOutput<double>;

}  // namespace sensors
}  // namespace systems
}  // namespace drake
