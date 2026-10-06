// mutablelib — noise/particle.h
//
// Particle: random impulses ("dust") each ringing a resonant filter tuned
// to a slightly different, random pitch around a centre frequency. At low
// densities, droplets, pings, or a Geiger counter; at high densities, a
// shimmering cloud. Plaits' particle engine sums several of these and adds
// a diffuser.
//
// How it works: each sample, a random number decides whether an impulse
// fires (probability = density / sample rate). When one does, the
// band-pass filter is retuned to the centre frequency shifted randomly by
// up to ±spread semitones, and the impulse rings it. The filter's level is
// compensated for its Q and the density, so the output stays at a steady
// loudness.
//
// Difference from Plaits: Plaits retunes the filter at most once per
// 12-sample block; this version retunes on every impulse.
//
// Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.

#ifndef ML_NOISE_PARTICLE_H_
#define ML_NOISE_PARTICLE_H_

#include <cmath>

#include "ml/core/random.h"
#include "ml/core/units.h"
#include "ml/filter/svf.h"

namespace ml {

class Particle {
 public:
  void init(float sampleRate) {
    sampleRate_ = sampleRate;
    filter_.init(sampleRate);
    preGain_ = 0.0f;
    impulse_ = 0.0f;
  }

  // Average impulses per second.
  void setDensity(float hz) { density_ = hz / sampleRate_; }
  // Centre frequency of the pings, in Hz.
  void setFrequency(float hz) { frequency_ = hz / sampleRate_; }
  // Random detuning range in semitones (e.g. 0 = all the same pitch, 24 =
  // up to two octaves either way).
  void setSpread(float semitones) { spread_ = semitones; }
  // Filter resonance: 1 = soft clicks, 100+ = long pings.
  void setQ(float q) { q_ = q; }
  void seed(uint32_t s) { random_.seed(s); }

  // Fire an impulse now (e.g. from a trigger input).
  void trigger() { forced_ = true; }

  // Returns the ringing output; impulse() then holds the raw impulse.
  float process() {
    float u = random_.uniform();
    if (forced_) {
      u = density_ < 1.0f ? density_ : 0.999f;
      forced_ = false;
    }
    float s = 0.0f;
    if (u <= density_) {
      s = u;
      // A new random pitch for this particle. The gain compensates the
      // filter's Q and the density (written against 48 kHz).
      float f = std::fmin(semitonesToRatio(spread_ * (2.0f * random_.uniform() - 1.0f)) * frequency_, 0.25f);
      float density48 = std::fmax(density_ * sampleRate_ / 48000.0f, 1e-6f);
      float f48 = f * sampleRate_ / 48000.0f;
      preGain_ = 0.5f / std::sqrt(q_ * f48 * std::sqrt(density48));
      filter_.setCoefficients(f, q_, TanApprox::Dirty);
    }
    impulse_ = s;
    return filter_.process(preGain_ * s).bp;
  }

  float impulse() const { return impulse_; }

 private:
  float sampleRate_ = 48000.0f;
  float density_ = 0.001f, frequency_ = 0.02f, spread_ = 12.0f, q_ = 10.0f;
  float preGain_ = 0.0f, impulse_ = 0.0f;
  bool forced_ = false;
  Random random_;
  Svf filter_;
};

}  // namespace ml

#endif  // ML_NOISE_PARTICLE_H_
