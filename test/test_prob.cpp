#include "prob.h"

#include "gtest/gtest.h"

#define PROB_TOLERANCE 0.0000001

TEST(ProbTest, MakeFromLinear) {
  Prob p1 = Prob::makeFromLinear(0.1234567);
  EXPECT_NEAR(0.1234567, p1.getProb(), PROB_TOLERANCE);
  Prob p2 = Prob::makeFromLinear(1.0);
  EXPECT_NEAR(1.0, p2.getProb(), PROB_TOLERANCE);
}

TEST(ProbTest, AndProb) {
  Prob p1 = Prob::makeFromLinear(0.5);
  Prob p2 = Prob::makeFromLinear(0.25);
  Prob out = Prob::andProb(p1, p2);
  EXPECT_NEAR(0.125, out.getProb(), PROB_TOLERANCE);
}

TEST(ProbTest, OrProb) {
  Prob p1 = Prob::makeFromLinear(0.5);
  Prob p2 = Prob::makeFromLinear(0.25);
  Prob out = Prob::orProb(p1, p2);
  EXPECT_NEAR(0.75, out.getProb(), PROB_TOLERANCE);
}

TEST(ProbTest, NormProb) {
  Prob p1 = Prob::makeFromLinear(0.125);
  Prob out = Prob::normProb(p1, -log(0.5));
  EXPECT_NEAR(0.25, out.getProb(), PROB_TOLERANCE);
}

TEST(ProbTest, CompareProb) {
  Prob p1 = Prob::makeFromLinear(0.40);
  Prob p2 = Prob::makeFromLinear(0.41);
  EXPECT_TRUE(p1 < p2);
  EXPECT_TRUE(p2 > p1);
  EXPECT_FALSE(p1 >= p2);
  EXPECT_FALSE(p2 <= p1);
}
