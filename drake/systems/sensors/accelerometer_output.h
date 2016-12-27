#pragma once

#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {
namespace sensors {

/// Specializes BasicVector with specific getters and setters that are useful
/// for consumers of Accelerometer's output.
template <typename T>
class AccelerometerOutput : public BasicVector<T> {
 public:
  /// Default constructor.  Sets all rows to zero.
  AccelerometerOutput();

  /// @name Getters and Setters
  //@{
  /// TODO
  // double GetAcceleration() const;
  //@}
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
