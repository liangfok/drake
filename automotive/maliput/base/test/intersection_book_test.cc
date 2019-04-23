#include "drake/automotive/maliput/base/intersection_book.h"

#include <gtest/gtest.h>

// #include "drake/automotive/maliput/api/lane_data.h"
// #include "drake/automotive/maliput/api/test_utilities/rules_test_utilities.h"
#include "drake/automotive/maliput/base/intersection.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace maliput {
namespace {

// using api::rules::TrafficLight;

GTEST_TEST(IntersectionBookTest, BasicTest) {
  const Intersection::Id id("my intersection");
  const Intersection intersection(id, ...);
  IntersectionBook dut;
  dut.AddIntersection(traffic_light);
  EXPECT_EQ(dut.GetTrafficLight(TrafficLight::Id("unknown_traffic light")),
            nullopt);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(*dut.GetTrafficLight(id), traffic_light));
}

}  // namespace
}  // namespace maliput
}  // namespace drake
