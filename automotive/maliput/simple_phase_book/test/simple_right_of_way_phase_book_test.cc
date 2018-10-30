#include "drake/automotive/maliput/simple_phase_book/simple_right_of_way_phase_book.h"

#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/rules/right_of_way_phase.h"
#include "drake/automotive/maliput/api/rules/right_of_way_phase_ring.h"
#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace maliput {
namespace simple_phase_book {
namespace {

using api::rules::RightOfWayRule;
using api::rules::RightOfWayPhase;
using api::rules::RightOfWayPhaseRing;

GTEST_TEST(SimpleRightOfWayPhaseBookTest, BasicTest) {
  const RightOfWayRule::Id rule_id_a("rule_a");
  const RightOfWayRule::Id rule_id_b("rule_b");
  const RightOfWayPhase phase(RightOfWayPhase::Id("phase"),
                              {{rule_id_a, RightOfWayRule::State::Id("a")},
                               {rule_id_b, RightOfWayRule::State::Id("b")}});
  const RightOfWayPhaseRing::Id ring_id("ring");
  const RightOfWayPhaseRing ring(ring_id, {phase});
  SimpleRightOfWayPhaseBook dut;
  EXPECT_NO_THROW(dut.AddPhaseRing(ring));
  EXPECT_THROW(dut.AddPhaseRing(ring), std::logic_error);
  optional<RightOfWayPhaseRing> result = dut.GetPhaseRing(ring_id);
  EXPECT_TRUE(!!result);
  EXPECT_EQ(result->id(), ring_id);
  result = dut.GetPhaseRing(RightOfWayPhaseRing::Id("unknown ring"));
  EXPECT_EQ(result, nullopt);
  for (const auto rule_id : {rule_id_a, rule_id_b}) {
    result = dut.FindPhaseRing(rule_id);
    EXPECT_TRUE(!!result);
    EXPECT_EQ(result->id(), ring_id);
  }
  result = dut.FindPhaseRing(RightOfWayRule::Id("unkown rule"));
  EXPECT_EQ(result, nullopt);
}

}  // namespace
}  // namespace simple_phase_book
}  // namespace maliput
}  // namespace drake
