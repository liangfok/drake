#pragma once

#include <lcm/lcm-cpp.hpp>

#include "drake/drakeLCMSystem2_export.h"
#include "drake/systems/lcm/lcm_to_vector_interface_translator.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 * Specializes the LcmBasicVectorTranslator to handle the LCM messages of type
 * `drake::lcmt_drake_signal`.
 *
 * Assumes that the order of the values in the LCM Vector and the
 * VectorInterface are identical.
 */
class DRAKELCMSYSTEM2_EXPORT TranslatorToLcmtDrakeSignal
    : public LcmToVectorInterfaceTranslator {
 public:
  /**
   * The constructor.
   *
   * @param[in] size The number of elements in vector.
   */
  explicit TranslatorToLcmtDrakeSignal(int size)
      : LcmToVectorInterfaceTranslator(size) {}

  virtual void TranslateLcmToVectorInterface(const ::lcm::ReceiveBuffer* rbuf,
    VectorInterface<double>* vector_interface) const override;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake
