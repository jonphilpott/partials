// mutablelib — filter/one_pole.h
//
// One-pole filter: the simplest useful filter, with a gentle 6 dB/octave
// slope. Low-pass smooths control signals and tames harsh highs;
// high-pass removes rumble.
//
// It uses the same zero-delay-feedback (TPT) design as the Svf, so the
// cutoff stays accurate up to Nyquist and can change every sample.
//
// How it works: one integrator with feedback. Each sample computes the
// low-pass output `lp` from the input and a single state variable; the
// high-pass is whatever the low-pass removed (in - lp).
//
// Derived from stmlib dsp/filter.h, Copyright 2014 Emilie Gillet. MIT licence.

#ifndef ML_FILTER_ONE_POLE_H_
#define ML_FILTER_ONE_POLE_H_

#include "ml/filter/svf.h"  // for TanApprox and tanApprox()

namespace ml {

struct OnePoleOut {
  float lp, hp;
};

class OnePole {
 public:
  // Stores the sample rate (needed only by setFrequency) and clears state.
  void init(float sampleRate) {
    sampleTime_ = 1.0f / sampleRate;
    setCoefficients(0.01f, TanApprox::Dirty);
    reset();
  }

  void reset() { state_ = 0.0f; }

  // Cutoff in Hz: the -3 dB point.
  void setFrequency(float hz) { setCoefficients(hz * sampleTime_, TanApprox::Exact); }

  // Lower-level form: f is cutoff / sampleRate.
  void setCoefficients(float f, TanApprox approx) {
    g_ = tanApprox(f, approx);
    gi_ = 1.0f / (1.0f + g_);
  }

  OnePoleOut process(float in) {
    OnePoleOut o;
    o.lp = (g_ * in + state_) * gi_;
    state_ = g_ * (in - o.lp) + o.lp;
    o.hp = in - o.lp;
    return o;
  }

 private:
  float sampleTime_ = 1.0f / 48000.0f;
  float g_ = 0.0f, gi_ = 1.0f;
  float state_ = 0.0f;
};

}  // namespace ml

#endif  // ML_FILTER_ONE_POLE_H_
