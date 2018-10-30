#include "drake/automotive/maliput/simple_phase_book/simple_right_of_way_phase_book.h"

#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

namespace drake {
namespace maliput {
namespace simple_phase_book {

using api::rules::RightOfWayRule;
using api::rules::RightOfWayPhase;
using api::rules::RightOfWayPhaseRing;

class SimpleRightOfWayPhaseBook::Impl {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Impl)

  Impl() {}
  ~Impl() {}

  void AddPhaseRing(const RightOfWayPhaseRing& ring) {
    auto result = ring_book_.emplace(ring.id(), ring);
    if (!result.second) {
      throw std::logic_error("Attempted to add multiple RightOfWayPhaseRing "
                             "instances with ID " + ring.id().string());
    }
    const RightOfWayPhase phase = ring.phases().begin()->second;
    for (std::pair<RightOfWayRule::Id, RightOfWayRule::State::Id> element :
        phase.rule_states()) {
      auto r = rule_book_.emplace(element.first, ring);
      if (!r.second) {
        throw std::logic_error("RightOfWayRule with ID " +
                               element.first.string() + " is part of more than "
                               "one RightOfWayPhaseRing.");
      }
    }
  }

  optional<RightOfWayPhaseRing> DoGetPhaseRing(
      const RightOfWayPhaseRing::Id& ring_id) const {
    auto it = ring_book_.find(ring_id);
    if (it == ring_book_.end()) {
      return nullopt;
    }
    return it->second;
  }

  optional<RightOfWayPhaseRing> DoFindPhaseRing(
      const RightOfWayRule::Id& rule_id) const {
    auto it = rule_book_.find(rule_id);
    if (it == rule_book_.end()) {
      return nullopt;
    }
    return it->second;
  }

  std::unordered_map<RightOfWayPhaseRing::Id, const RightOfWayPhaseRing&>
      ring_book_; 
  std::unordered_map<RightOfWayRule::Id, const RightOfWayPhaseRing&>
      rule_book_; 
};

SimpleRightOfWayPhaseBook::SimpleRightOfWayPhaseBook()
    : impl_(std::make_unique<Impl>()) {}

SimpleRightOfWayPhaseBook::~SimpleRightOfWayPhaseBook() = default;

void SimpleRightOfWayPhaseBook::AddPhaseRing(const RightOfWayPhaseRing& ring) {
  impl_->AddPhaseRing(ring);
}

optional<RightOfWayPhaseRing> SimpleRightOfWayPhaseBook::DoGetPhaseRing(
    const RightOfWayPhaseRing::Id& ring_id) const {
  return impl_->DoGetPhaseRing(ring_id);
}

optional<RightOfWayPhaseRing> SimpleRightOfWayPhaseBook::DoFindPhaseRing(
    const RightOfWayRule::Id& rule_id) const {
  return impl_->DoFindPhaseRing(rule_id);
}

}  // namespace simple_phase_book
}  // namespace maliput
}  // namespace drake
