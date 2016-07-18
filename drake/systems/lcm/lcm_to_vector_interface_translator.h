#pragma once

#include <lcm/lcm-cpp.hpp>

#include "drake/drakeLCMSystem2_export.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 * Defines an abstract parent class of all translators that convert from
 * LCM message objects to drake::systems::VectorInterface objects.
 */
class LcmToVectorInterfaceTranslator {
 public:
  /**
   * The constructor.
   *
   * @param[in] size The size of the basic vector.
   */
  explicit LcmToVectorInterfaceTranslator(int size) : size_(size) {
  }

  // Noncopyable and non-comparable.
  LcmToVectorInterfaceTranslator(const LcmToVectorInterfaceTranslator&) = delete;
  LcmToVectorInterfaceTranslator& operator=(const LcmToVectorInterfaceTranslator&)
      = delete;

  /**
   * Returns the size of the vector interface.
   */
  int get_vector_size() const {
    return size_;
  }

  /**
   * Translates an LCM message into a `BasicVector` object.
   *
   * @param[in] rbuf A pointer to a buffer holding the LCM message's data.
   *
   * @param[out] vector_interface A pointer to where the translation of the LCM
   * message should be stored. This pointer must not be nullptr and must remain
   * valid throughout the duration of this method call.
   *
   * @throws runtime_error If a received LCM message failed to be decoded, or
   * if the decoded LCM message's size does not equal the size of
   * @p basic_vector.
   */
  virtual void TranslateLcmToVectorInterface(const ::lcm::ReceiveBuffer* rbuf,
    VectorInterface<double>* vector_interface) const = 0;

 private:
  // The size of the basic vector and vector representation of the LCM message.
  const int size_;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake
