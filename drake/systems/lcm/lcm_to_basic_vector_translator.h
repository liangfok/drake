#pragma once

#include <lcm/lcm-cpp.hpp>

#include "drake/drakeLCMSystem2_export.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 * Defines an abstract parent class of all translators that convert from
 * LCM message objects to drake::systems::BasicVector objects.
 */
class LcmToBasicVectorTranslator {
 public:
  /**
   * The constructor.
   *
   * @param[in] size The size of the basic vector.
   */
  explicit LcmToBasicVectorTranslator(int size) : size_(size) {
  }

  // Noncopyable and non-comparable.
  LcmToBasicVectorTranslator(const LcmToBasicVectorTranslator&) = delete;
  LcmToBasicVectorTranslator& operator=(const LcmToBasicVectorTranslator&)
      = delete;

  /**
   * Returns the size of the basic vector and the vector representation of the
   * LCM message.
   */
  int get_basic_vector_size() const {
    return size_;
  }

  /**
   * Translates an LCM message a `BasicVector` object.
   *
   * @param[in] rbuf A pointer to a buffer holding the LCM message's data.
   *
   * @param[out] basic_vector A pointer to where the translation of the LCM
   * message should be stored. A runtime exception is thrown if this parameter
   * is nullptr.
   *
   * @throws runtime_error If a received LCM message failed to be decoded, or
   * if the decoded LCM message's size does not equal the size of
   * @p basic_vector.
   */
  virtual void TranslateLcmToBasicVector(const ::lcm::ReceiveBuffer* rbuf,
    BasicVector<double>* basic_vector) const = 0;

 private:
  // The size of the basic vector and vector representation of the LCM message.
  const int size_;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake
