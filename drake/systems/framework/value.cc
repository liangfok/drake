#include "drake/systems/framework/value.h"

#include <sstream>
#include <string>

#include "drake/common/nice_type_name.h"

namespace drake {
namespace systems {

AbstractValue::~AbstractValue() {}

std::string AbstractValue::ToString(const std::string& prefix) const {
  const void * address = static_cast<const void*>(this);
  std::stringstream buffer;
  buffer << prefix << NiceTypeName.get(*this) << "@" << address;
  return buffer.str();
}

}  // namespace systems
}  // namespace drake
