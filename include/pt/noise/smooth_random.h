// partials — noise/smooth_random.h
//
// SmoothRandom: a smoothly wandering random signal, a random LFO. At each
// tick of its clock it picks a new random target (±1) and glides there
// along an S-curve, so there are no corners. Plaits uses it for slow
// modulation (drift, wobble); use it for vibrato that never repeats,
// "living" filter sweeps, or random pans.
//
// How it works: a phase accumulator at the set rate; between ticks, the
// output moves from the old value to the new one along smoothstep(phase)
// (3t^2 - 2t^3), which starts and ends with zero slope.
//
// Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.

#ifndef PT_NOISE_SMOOTH_RANDOM_H_
#define PT_NOISE_SMOOTH_RANDOM_H_

#include "pt/core/random.h"

namespace pt {

class SmoothRandom {
 public:
  void init(float sampleRate) {
    sampleTime_ = 1.0f / sampleRate;
    phase_ = from_ = interval_ = 0.0f;
  }

  // New targets per second.
  void setFrequency(float hz) { frequency_ = hz * sampleTime_; }
  void seed(uint32_t s) { random_.seed(s); }

  // Returns ±1.
  float process() {
    phase_ += frequency_;
    if (phase_ >= 1.0f) {
      phase_ -= 1.0f;
      from_ += interval_;
      interval_ = random_.uniform() * 2.0f - 1.0f - from_;
    }
    float t = phase_ * phase_ * (3.0f - 2.0f * phase_);
    return from_ + interval_ * t;
  }

 private:
  float sampleTime_ = 1.0f / 48000.0f;
  float frequency_ = 0.0f;
  float phase_ = 0.0f, from_ = 0.0f, interval_ = 0.0f;
  Random random_;
};

}  // namespace pt

#endif  // PT_NOISE_SMOOTH_RANDOM_H_
