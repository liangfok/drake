#include "drake/automotive/maliput/base/intersections_loader.h"

// #include <exception>
#include <memory>
#include <string>

#include <gtest/gtest.h>

// #include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/base/right_of_way_rules_loader.h"
#include "drake/automotive/maliput/base/simple_rulebook.h"
#include "drake/automotive/maliput/multilane/builder.h"
#include "drake/automotive/maliput/multilane/loader.h"
#include "drake/common/find_resource.h"

namespace drake {
namespace maliput {
namespace {

class TestLoading2x2IntersectionIntersections : public ::testing::Test {
 protected:
  TestLoading2x2IntersectionIntersections()
      : filepath_(FindResourceOrThrow(
            "drake/automotive/maliput/multilane/2x2_intersection.yaml")),
        road_geometry_(
            multilane::LoadFile(multilane::BuilderFactory(), filepath_)),
        rulebook_(
            LoadRightOfWayRulesFromFile(road_geometry_.get(), filepath_)) {}


  const std::string filepath_;
  const std::unique_ptr<const api::RoadGeometry> road_geometry_;
  const std::unique_ptr<const SimpleRulebook> rulebook_;
};

TEST_F(TestLoading2x2IntersectionIntersections, LoadFromFile) {
  std::vector<std::unique_ptr<const Intersection>> intersections =
      LoadIntersectionsFromFile(road_geometry_.get(), rulebook_.get(),
                                filepath_);
  EXPECT_TRUE(intersections.size() > 0);
}

}  // namespace
}  // namespace maliput
}  // namespace drake
