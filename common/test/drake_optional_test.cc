#include "drake/common/drake_optional.h"

#include <gtest/gtest.h>

namespace drake {
namespace {

GTEST_TEST(OptionalTest, BasicTest) {
  optional<int> foo;
  EXPECT_EQ(!!foo, false);
  EXPECT_FALSE(foo.has_value());

  foo = 1;
  ASSERT_EQ(!!foo, true);
  EXPECT_TRUE(foo.has_value());
  EXPECT_EQ(*foo, 1);

  foo.reset();
  EXPECT_FALSE(foo.has_value());

  constexpr int kBarValue{2};
  optional<int> bar = make_optional<int>(kBarValue);
  EXPECT_EQ(!!bar, true);
  EXPECT_TRUE(bar.has_value());
  EXPECT_EQ(*bar, kBarValue);
}

}  // namespace
}  // namespace drake
