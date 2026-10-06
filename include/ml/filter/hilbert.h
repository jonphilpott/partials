// mutablelib — filter/hilbert.h
//
// Hilbert: splits a signal into two copies 90 degrees apart at every
// frequency (an "analytic signal": in-phase I and quadrature Q). It is the
// key part of a frequency shifter (Bode / single-sideband modulation), and
// also gives a smooth amplitude envelope (sqrt(I^2 + Q^2)). Warps uses it
// for its frequency shifter.
//
// How it works: two chains of first-order all-pass filters. All-passes
// change only phase, not level. The chains are designed so their phase
// responses stay exactly 90 degrees apart (within about 1 degree) from
// 10 Hz to 20 kHz. The design is an elliptic half-band filter
// decomposed into all-passes (as in Warps' resource script), computed
// here for the actual sample rate.
//
// Both outputs are delayed and phase-shifted versions of the input, so
// use I and Q together, not I as a stand-in for the input.
//
// Derived from Warps, Copyright 2014 Emilie Gillet. MIT licence.

#ifndef ML_FILTER_HILBERT_H_
#define ML_FILTER_HILBERT_H_

#include <algorithm>
#include <cmath>
#include <vector>

namespace ml {

class Hilbert {
 public:
  static const int kNumFilters = 17;

  // Designs the filters for this rate (a few microseconds of maths).
  void init(float sampleRate) {
    designPoles(sampleRate, coefficient_);
    for (int i = 0; i < kNumFilters; ++i) x_[i] = y_[i] = 0.0f;
    i_ = q_ = 0.0f;
  }

  // Returns I; q() then holds Q, 90 degrees behind it.
  float process(float in) {
    // Filters alternate between the two chains: even ones build I, odd
    // ones Q. The first of each chain takes the input.
    float out[2] = {in, in};
    for (int k = 0; k < kNumFilters; ++k) {
      float& s = out[k & 1];
      float y = coefficient_[k] * (s - y_[k]) + x_[k];
      x_[k] = s;
      y_[k] = y;
      s = y;
    }
    i_ = out[0];
    q_ = out[1];
    return i_;
  }

  float i() const { return i_; }
  float q() const { return q_; }

  // The all-pass coefficients (Warps' design, for any rate). Public so the
  // design can be inspected or tested.
  static void designPoles(float sampleRate, float* coefficients) {
    const double pi = 3.14159265358979323846;
    // 1. Elliptic low-pass prototype poles: half-band (bandwidth 0.495),
    // order 17 (C. Britton Rorabaugh, "Digital Filter Designer's
    // Handbook", p. 94).
    const double cutoff = 0.495;
    const int order = 17;
    double wp = cutoff * pi, ws = pi - wp;
    double k = std::tan(0.5 * wp) / std::tan(0.5 * ws);
    double r = std::pow(1.0 - k * k, 0.25);
    double u = 0.5 * (1.0 - r) / (1.0 + r);
    double q = u + 2.0 * std::pow(u, 5) + 15.0 * std::pow(u, 9) + 150.0 * std::pow(u, 13);
    std::vector<double> lpPoles;
    for (int i = 0; i < (order - 1) / 2; ++i) {
      double w = (i + 1) * pi / order;
      double num = 0.0, den = 0.0;
      for (int m = 0; m < 7; ++m) num += ((m & 1) ? -1.0 : 1.0) * std::pow(q, m * (m + 1)) * std::sin((2 * m + 1) * w);
      for (int m = 1; m < 7; ++m) den += ((m & 1) ? -1.0 : 1.0) * std::pow(q, m * m) * std::cos(2 * m * w);
      double l = 2.0 * std::pow(q, 0.25) * num / (1.0 + 2.0 * den);
      double b = std::sqrt((1.0 - k * l * l) * (1.0 - l * l / k));
      double c = 2.0 * b / (1.0 + l * l);
      lpPoles.push_back((2.0 - c) / (2.0 + c));
    }
    lpPoles.push_back(0.0);

    // 2. Warp them into all-pass poles covering 10 Hz..20 kHz (capped
    // below Nyquist at low sample rates).
    double high = std::min(20000.0, 0.45 * sampleRate);
    double beta = std::sqrt(std::tan(pi * 10.0 / sampleRate) * std::tan(pi * high / sampleRate));
    double bw = (beta - 1.0) / (beta + 1.0);
    std::vector<double> ap;
    for (double pole : lpPoles) {
      double p = std::sqrt(pole);
      if (pole != 0.0) {
        ap.push_back((p + bw) / (p * bw + 1.0));
        ap.push_back((-p + bw) / (-p * bw + 1.0));
      } else {
        ap.push_back(bw);
      }
    }
    // 3. Sorted, as Warps stores them (its table holds the negatives and
    // flips the sign back when loading).
    std::sort(ap.begin(), ap.end());
    for (int i = 0; i < kNumFilters; ++i) coefficients[i] = static_cast<float>(ap[i]);
  }

 private:
  float coefficient_[kNumFilters] = {};
  float x_[kNumFilters] = {}, y_[kNumFilters] = {};
  float i_ = 0.0f, q_ = 0.0f;
};

}  // namespace ml

#endif  // ML_FILTER_HILBERT_H_
