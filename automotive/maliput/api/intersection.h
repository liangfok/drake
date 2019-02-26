#pragma once

// #include <memory>
// #include <string>
#include <vector>

// #include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/rules/regions.h"
#include "drake/automotive/maliput/api/rules/right_of_way_phase_provider.h"
#include "drake/automotive/maliput/api/rules/right_of_way_phase_ring.h"
#include "drake/automotive/maliput/api/type_specific_identifier.h"
#include "drake/automotive/maliput/simple_phase_provider/simple_right_of_way_phase_provider.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
// #include "drake/common/symbolic.h"

namespace drake {
namespace maliput {
namespace api {

/// A convenience data structure for aggregating information about an
/// intersection. Its primary purpose is to serve as a single source of this
/// information and to remove the need to query numerous disparate data
/// structures and state providers.
class Intersection {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Intersection)

  /// Unique identifier for an Intersection.
  using Id = TypeSpecificIdentifier<class Intersection>;

  /// Constructs an Intersection instance.
  ///
  /// @param id The intersection's unique ID.
  ///
  /// @param region The region of the road network that is part of the
  /// intersection.
  ///
  /// @param ring The ring that defines the phases within the intersection. The
  /// pointer must remain valid throughout this class instance's lifetime.
  ///
  /// @param phase_provider Enables the current phase within @p ring to be
  /// specified and obtained. The pointer must remain valid throughout this
  /// class instance's lifetime.
  Intersection(const Id& id, const std::vector<LaneSRange>& region,
      const rules::RightOfWayPhaseRing* ring,
      simple_phase_provider::SimpleRightOfWayPhaseProvider* phase_provider);

  virtual ~Intersection() = default;

  /// Returns the persistent identifier.
  const Id id() const { return id_; }

  /// Returns the current phase.
  const optional<RightOfWayPhaseProvider::Result> Phase() const;

  /// Sets the current phase.
  void SetPhase(const RightOfWayPhase::id& phase_id);

  const std::vector<LaneSRange>& region() const { return region_; }

  // TODO(liang.fok) Add method for obtaining the current bulb states

  // TODO(liang.fok) Add method for obtaining the intersection's bounding box.
 private:
  const Id id_;
  const std::vector<LaneSRange> region_;
  const RightOfWayPhaseRing* ring_{};
  simple_phase_provider::SimpleRightOfWayPhaseProvider* phase_provider_{};
};

}  // namespace api
}  // namespace maliput
}  // namespace drake
