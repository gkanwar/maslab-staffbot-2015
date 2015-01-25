#ifndef PROB_H
#define PROB_H

#include <cassert>
#include <cmath>

// Representation of probability, using negative log weights for precision
// in lower numbers
class Prob {
 private:
  // -ln(prob)
  double negLogProb;

 public:
  Prob(double negLogProb) : negLogProb(negLogProb) {
    assert(negLogProb > 0.0);
  }

  // Extract the linear probability
  double getProb() {
    return exp(-negLogProb);
  }

  // Multiply two probabilites
  static Prob andProb(Prob p1, Prob p2) {
    return Prob(p1.negLogProb + p2.negLogProb);
  }

  // Add two probabilities. Unsafe on probability operations for significantly
  // different scales of probability, as the smaller one will likely be lost in
  // floating point imprecision.
  static Prob orProb(Prob p1, Prob p2) {
    double prob1 = exp(-p1.negLogProb);
    double prob2 = exp(-p2.negLogProb);
    return Prob(-log(prob1 + prob2));
  }
};

#endif
