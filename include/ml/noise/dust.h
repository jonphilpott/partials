// mutablelib — noise/dust.h
//
// Dust: sparse random impulses ("crackle"), each with a random height,
// arriving at a set average rate. At low densities it is vinyl crackle or
// rain; at high densities, a hiss. Plaits' noise engines and particle
// exciter use it; it is also a good exciter for the resonators.
//
// How it works: each sample, a random number u (0..1) is drawn. If u is
// below the probability of an impulse in this sample (density / sample
// rate), an impulse of height u / probability (0..1) is emitted;
// otherwise silence. Reusing u for the height costs no extra random
// number.
//
// Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.

#ifndef ML_NOISE_DUST_H_
#define ML_NOISE_DUST_H_

#include "ml/core/random.h"

namespace ml {

class Dust {
 public:
  void init(float sampleRate) {
    sampleTime_ = 1.0f / sampleRate;
    setDensity(density_);
  }

  // Average impulses per second.
  void setDensity(float hz) {
    density_ = hz;
    probability_ = hz * sampleTime_;
    if (probability_ > 1.0f) probability_ = 1.0f;
    inverse_ = probability_ > 0.0f ? 1.0f / probability_ : 0.0f;
  }

  void seed(uint32_t s) { random_.seed(s); }

  // Returns an impulse (0..1) or 0.
  float process() {
    float u = random_.uniform();
    return u < probability_ ? u * inverse_ : 0.0f;
  }

 private:
  float sampleTime_ = 1.0f / 48000.0f;
  float density_ = 100.0f, probability_ = 0.0f, inverse_ = 0.0f;
  Random random_;
};

}  // namespace ml

#endif  // ML_NOISE_DUST_H_
