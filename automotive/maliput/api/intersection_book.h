#pragma once

#include "drake/automotive/maliput/api/intersection.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace maliput {
namespace api {

/// Abstract interface for providing the mapping from Intersection::Id to
/// Intersection.
class IntersectionBook {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IntersectionBook);

  virtual ~IntersectionBook() = default;

  /// Gets the specified Intersection. Returns nullopt if @p id is unrecognized.
  optional<Intersection> GetIntersection(const Intersection::Id& id) const {
    return DoGetIntersection(id);
  }

 protected:
  IntersectionBook() = default;

 private:
  virtual optional<Intersection> DoGetIntersection(
      const Intersection::Id& id) const = 0;
};

}  // namespace api
}  // namespace maliput
}  // namespace drake
