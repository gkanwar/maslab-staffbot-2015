#ifndef PROB_H
#define PROB_H

#include <cmath>

#include "error.h"

// Representation of probability, using negative log weights for precision
// in lower numbers
class Prob {
 private:
  // -ln(prob)
  double negLogProb;

 public:
  // Default to probability of 1.0, -log(1.0) = 0.0
  Prob() : Prob(0.0) {}

  explicit Prob(double negLogProb) : negLogProb(negLogProb) {
    rassert(negLogProb >= 0.0);
  }

  // Extract the linear probability
  double getProb() const {
    return exp(-negLogProb);
  }
  double getNegLogProb() const {
    return negLogProb;
  }

  static Prob makeFromLinear(double linearProb) {
    rassert(linearProb > 0.0);
    rassert(linearProb <= 1.0);
    return Prob(-log(linearProb));
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

  // Divide two probabilities, to normalize p relative to base.
  static Prob normProb(Prob p, double negLogBase) {
    // Handle floating point error
    if (abs(p.negLogProb - negLogBase) < 0.0000001)
      return Prob(0.0);
    else
      return Prob(p.negLogProb - negLogBase);
  }

  // Compare ops
  friend inline bool operator<(const Prob& lhs, const Prob& rhs) {
    return lhs.negLogProb > rhs.negLogProb;
  }
  friend inline bool operator>(const Prob& lhs, const Prob& rhs) {return rhs < lhs;}
  friend inline bool operator<=(const Prob& lhs, const Prob& rhs) {return !(lhs > rhs);}
  friend inline bool operator>=(const Prob& lhs, const Prob& rhs) {return !(lhs < rhs);}

  friend ostream& operator<<(ostream& os, const Prob& value) {
    os << value.getProb();
    return os;
  }
};

#endif
