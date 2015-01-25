#include "prob.h"

#include "gtest/gtest.h"

#define PROB_TOLERANCE 0.0000001

TEST(ProbTest, AndProb) {
  // Neg log of 0.5
  Prob p1(0.6931471805599);
  ASSERT_NEAR(0.5, p1.getProb(), PROB_TOLERANCE);
  // Neg log of 0.25
  Prob p2(1.3862943611199);
  ASSERT_NEAR(0.25, p2.getProb(), PROB_TOLERANCE);

  Prob out = Prob::andProb(p1, p2);
  EXPECT_NEAR(0.125, out.getProb(), PROB_TOLERANCE);
}

TEST(ProbTest, OrProb) {
  // Neg log of 0.5
  Prob p1(0.6931471805599);
  ASSERT_NEAR(0.5, p1.getProb(), PROB_TOLERANCE);
  // Neg log of 0.25
  Prob p2(1.3862943611199);
  ASSERT_NEAR(0.25, p2.getProb(), PROB_TOLERANCE);

  Prob out = Prob::orProb(p1, p2);
  EXPECT_NEAR(0.75, out.getProb(), PROB_TOLERANCE);
}
