// mutablelib — noise/clocked_noise.h
//
// ClockedNoise: sample-and-hold noise. Random values held for one clock
// period each: stepped "computer" noise at low rates, then a pitched,
// crunchy buzz, then white noise. Plaits' clocked-noise mode.
//
// How it works: a phase accumulator ticks at the set rate; at each tick a
// new random value is held. The steps land between samples, so each one
// gets a PolyBLEP correction at its exact position. That keeps the tone
// clean and pitched instead of aliased. Near and above a quarter of the
// sample rate it crossfades into plain white noise. A sync input restarts
// the clock, so a pattern of steps can be locked to a rhythm.
//
// Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.

#ifndef ML_NOISE_CLOCKED_NOISE_H_
#define ML_NOISE_CLOCKED_NOISE_H_

#include "ml/core/math.h"
#include "ml/core/random.h"
#include "ml/osc/polyblep.h"

namespace ml {

class ClockedNoise {
 public:
  void init(float sampleRate) {
    sampleTime_ = 1.0f / sampleRate;
    phase_ = sample_ = nextSample_ = 0.0f;
  }

  // New values per second.
  void setFrequency(float hz) { frequency_ = clamp(hz * sampleTime_, 0.0f, 1.0f); }
  void seed(uint32_t s) { random_.seed(s); }

  // sync: true restarts the clock, so a new value is drawn immediately.
  // Returns ±1, one sample late (BLEP look-ahead).
  float process(bool sync = false) {
    if (sync) phase_ = 1.0f;
    float thisSample = nextSample_;
    float nextSample = 0.0f;
    const float raw = random_.uniform() * 2.0f - 1.0f;
    const float rawAmount = clamp(4.0f * (frequency_ - 0.25f), 0.0f, 1.0f);
    phase_ += frequency_;
    if (phase_ >= 1.0f) {
      phase_ -= 1.0f;
      float t = phase_ / frequency_;
      float discontinuity = raw - sample_;
      thisSample += discontinuity * thisBlepSample(t);
      nextSample += discontinuity * nextBlepSample(t);
      sample_ = raw;
    }
    nextSample += sample_;
    nextSample_ = nextSample;
    return thisSample + rawAmount * (raw - thisSample);
  }

 private:
  float sampleTime_ = 1.0f / 48000.0f;
  float frequency_ = 0.001f;
  float phase_ = 0.0f, sample_ = 0.0f, nextSample_ = 0.0f;
  Random random_;
};

}  // namespace ml

#endif  // ML_NOISE_CLOCKED_NOISE_H_
