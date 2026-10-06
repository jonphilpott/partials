// mutablelib — mod/hysteresis_quantizer.h
//
// HysteresisQuantizer: turns a continuous 0..1 value (a knob, a CV) into
// one of N steps, without flickering between two steps when the value sits
// near the boundary. Use it for mode selectors, range switches, or picking
// from a list with a CV.
//
// How it works: the boundaries move away from the current step by a
// fraction of a step (the hysteresis). To change step, the value must go a
// little past the halfway point, so small noise near a boundary has no
// effect.
//
// Derived from stmlib dsp/hysteresis_quantizer.h (HysteresisQuantizer2),
// Copyright 2015 Emilie Gillet. MIT licence.

#ifndef ML_MOD_HYSTERESIS_QUANTIZER_H_
#define ML_MOD_HYSTERESIS_QUANTIZER_H_

namespace ml {

class HysteresisQuantizer {
 public:
  // numSteps: how many steps. hysteresis: in steps, e.g. 0.25 = a quarter
  // of a step past halfway. symmetric: false spreads the 0..1 range evenly
  // across the steps (each gets 1/n of the range); true puts step 0 at
  // exactly 0 and the last step at exactly 1.
  void init(int numSteps, float hysteresis = 0.25f, bool symmetric = false) {
    numSteps_ = numSteps < 1 ? 1 : numSteps;
    hysteresis_ = hysteresis;
    scale_ = static_cast<float>(symmetric ? numSteps_ - 1 : numSteps_);
    offset_ = symmetric ? 0.0f : -0.5f;
    value_ = 0;
  }

  // value: 0..1. Returns the step, 0..numSteps-1.
  int process(float value) {
    value = value * scale_ + offset_;
    float sign = value > static_cast<float>(value_) ? -1.0f : 1.0f;
    int q = static_cast<int>(value + sign * hysteresis_ + 0.5f);
    if (q < 0) q = 0;
    if (q > numSteps_ - 1) q = numSteps_ - 1;
    value_ = q;
    return q;
  }

  int value() const { return value_; }
  int numSteps() const { return numSteps_; }

 private:
  int numSteps_ = 2;
  float hysteresis_ = 0.25f;
  float scale_ = 2.0f, offset_ = -0.5f;
  int value_ = 0;
};

}  // namespace ml

#endif  // ML_MOD_HYSTERESIS_QUANTIZER_H_
