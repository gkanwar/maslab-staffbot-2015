#include "util.h"

#include "gtest/gtest.h"

TEST(UtilTest, ThetaContained) {
  EXPECT_TRUE(isThetaContained(0.1, 0.5, 0.3));
  EXPECT_FALSE(isThetaContained(0.1, 0.5, 0.7));
  EXPECT_TRUE(isThetaContained(0.5, 0.2, 0.7));
  EXPECT_TRUE(isThetaContained(0.5, 0.2, 0.1));
  EXPECT_FALSE(isThetaContained(0.5, 0.2, 0.3));
}
