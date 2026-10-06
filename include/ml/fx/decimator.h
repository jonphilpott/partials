// mutablelib — fx/decimator.h
//
// Decimator (sample-rate reducer): the lo-fi crunch of an old sampler or
// game console, from Plaits' sample-rate reducer. The input is
// sampled-and-held at a lower rate.
//
// How it works: a phase accumulator ticks at the target rate. At each tick
// it grabs a new input value and holds it. Done naively, the steps between
// held values land on whole samples, which adds harsh, inharmonic aliasing
// that depends on the host rate. Here each step is placed at its exact
// fractional position: the input is interpolated at that instant, and the
// step's edge is smoothed with a PolyBLEP correction. What remains is the
// intended "stair-step" aliasing, which is the effect, without the extra
// grit. Costs one sample of latency while active.
//
// Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.

#ifndef ML_FX_DECIMATOR_H_
#define ML_FX_DECIMATOR_H_

#include "ml/core/math.h"
#include "ml/osc/polyblep.h"

namespace ml {

class Decimator {
 public:
  void init(float sampleRate) {
    sampleRate_ = sampleRate;
    phase_ = 0.0f;
    sample_ = nextSample_ = previousSample_ = 0.0f;
    setRate(rateHz_);
  }

  // Target sample rate in Hz. At or above the host rate the signal passes
  // through unchanged.
  void setRate(float hz) {
    rateHz_ = hz;
    frequency_ = clamp(hz / sampleRate_, 0.0f, 1.0f);
  }

  float process(float in) {
    // At or above the host rate: pass through untouched (no latency).
    if (frequency_ >= 1.0f) {
      sample_ = nextSample_ = previousSample_ = in;
      return in;
    }
    float thisSample = nextSample_;
    nextSample_ = 0.0f;
    phase_ += frequency_;
    if (phase_ >= 1.0f) {
      phase_ -= 1.0f;
      // t: how far back (as a fraction of the decimated period, in host
      // samples) the tick really happened; 0 = exactly now.
      float t = phase_ / frequency_;
      // Interpolate the input at the tick's true position.
      float newSample = previousSample_ + (in - previousSample_) * (1.0f - t);
      // Spread the step over this sample and the next (PolyBLEP).
      float discontinuity = newSample - sample_;
      thisSample += discontinuity * thisBlepSample(t);
      nextSample_ += discontinuity * nextBlepSample(t);
      sample_ = newSample;
    }
    nextSample_ += sample_;
    previousSample_ = in;
    return thisSample;
  }

 private:
  float sampleRate_ = 48000.0f;
  float rateHz_ = 8000.0f;
  float frequency_ = 1.0f;
  float phase_ = 0.0f;
  float sample_ = 0.0f, nextSample_ = 0.0f, previousSample_ = 0.0f;
};

}  // namespace ml

#endif  // ML_FX_DECIMATOR_H_
