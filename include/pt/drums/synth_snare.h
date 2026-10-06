// partials — drums/synth_snare.h
//
// SynthSnare: a model of the TR-909 snare. Plaits' "synthetic snare drum".
//
// How it works
// - The drum: two soft-clipped triangle oscillators a ratio of 1.47 apart,
//   with a fast pitch drop at the start (`fmAmount`) and a long decaying
//   tail. A quirk of the 909 is copied too: the oscillators reset each
//   other slightly, which adds a gritty intermodulation at high FM
//   settings.
// - The snares: white noise through a band-pass (a high-pass and a
//   resonant low-pass), with an envelope that holds for 40-70 ms before
//   decaying. That hold is what gives the 909 snare its body.
// - `snappy` balances drum against snares (equal-power).
// - Sustain mode holds both at a steady level.
//
// Sample-rate independence: all times are in seconds; filter frequencies
// follow the pitch in Hz.
//
// Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.

#ifndef PT_DRUMS_SYNTH_SNARE_H_
#define PT_DRUMS_SYNTH_SNARE_H_

#include <cmath>

#include "pt/core/math.h"
#include "pt/core/random.h"
#include "pt/core/units.h"
#include "pt/filter/one_pole.h"
#include "pt/filter/svf.h"

namespace pt {

class SynthSnare {
 public:
  void init(float sampleRate) {
    sampleRate_ = sampleRate;
    drumLp_.init(sampleRate);
    snareHp_.init(sampleRate);
    snareLp_.init(sampleRate);
    phase_[0] = phase_[1] = 0.0f;
    drumAmplitude_ = snareAmplitude_ = fm_ = 0.0f;
    holdCounter_ = 0;
    tailToggle_ = 0;
    update();
  }

  // Pitch of the drum in Hz. 909 snares sit around 150-250 Hz.
  void setFrequency(float hz) { hz_ = hz; update(); }
  // 0..1. Depth of the pitch drop at the attack (and the 909's oscillator
  // intermodulation at low pitches).
  void setFmAmount(float x) { fmAmount_ = clamp(x, 0.0f, 1.0f); update(); }
  // 0..1. Decay of drum and snares.
  void setDecay(float x) { decay_ = clamp(x, 0.0f, 1.0f); update(); }
  // 0..1. Balance: 0 = all drum, 1 = all snares.
  void setSnappy(float x) { snappy_ = clamp(x, 0.0f, 1.0f); update(); }
  // 0..1. How hard the next hit is (and the sustain-mode level).
  void setAccent(float x) { accent_ = clamp(x, 0.0f, 1.0f); }
  void setSustain(bool on) { sustain_ = on; }
  void seed(uint32_t s) { random_.seed(s); }

  void trigger() {
    snareAmplitude_ = drumAmplitude_ = 0.3f + 0.7f * accent_;
    fm_ = 1.0f;
    phase_[0] = phase_[1] = 0.0f;
    holdCounter_ = static_cast<int>((0.04f + decay_ * 0.03f) * sampleRate_);
  }

  float process() {
    // 1. Envelopes. The drum's tail, once quiet, decays at half speed (the
    // 909's long tail); the snares hold, then decay.
    if (sustain_) {
      snareAmplitude_ = drumAmplitude_ = accent_ * decay_;
      fm_ = 0.0f;
    } else {
      bool odd = (tailToggle_++ & 1) != 0;
      if (drumAmplitude_ > 0.03f || odd) drumAmplitude_ *= drumDecay_;
      if (holdCounter_) {
        --holdCounter_;
      } else {
        snareAmplitude_ *= snareDecay_;
      }
      fm_ *= fmDecay_;
    }

    // 2. The 909's oscillator coupling: each oscillator's reset point
    // wobbles with the other's state, at low pitches and high FM.
    float resetNoise = 0.0f;
    resetNoise += phase_[0] > 0.5f ? -1.0f : 1.0f;
    resetNoise += phase_[1] > 0.5f ? -1.0f : 1.0f;
    resetNoise *= resetNoiseAmount_ * 0.025f;

    // 3. Two oscillators with a shared pitch drop.
    float f = f0_ * (1.0f + fmAmount2_ * (4.0f * fm_));
    phase_[0] += f;
    phase_[1] += f * 1.47f;
    if (resetNoiseAmount_ > 0.1f) {
      if (phase_[0] >= 1.0f + resetNoise) phase_[0] = 1.0f - phase_[0];
      if (phase_[1] >= 1.0f + resetNoise) phase_[1] = 1.0f - phase_[1];
    } else {
      if (phase_[0] >= 1.0f) phase_[0] -= 1.0f;
      if (phase_[1] >= 1.0f) phase_[1] -= 1.0f;
    }
    float drum = -0.1f;
    drum += distortedSine(phase_[0]) * 0.60f;
    drum += distortedSine(phase_[1]) * 0.25f;
    drum *= drumAmplitude_ * drumLevel_;
    drum = drumLp_.process(drum).lp;

    // 4. Snares: band-passed noise.
    float noise = random_.uniform();
    float snare = snareLp_.process(noise).lp;
    snare = snareHp_.process(snare).hp;
    snare = (snare + 0.1f) * (snareAmplitude_ + fm_) * snareLevel_;

    return snare + drum;  // "It's a snare, it's a drum, it's a snare drum."
  }

 private:
  static float distortedSine(float phase) {
    float triangle = (phase < 0.5f ? phase : 1.0f - phase) * 4.0f - 1.3f;
    return 2.0f * triangle / (1.0f + std::fabs(triangle));
  }

  void update() {
    f0_ = hz_ / sampleRate_;
    const float f048 = hz_ / 48000.0f;
    const float decayXt = decay_ * (1.0f + decay_ * (decay_ - 1.0f));
    fmAmount2_ = fmAmount_ * fmAmount_;
    drumDecay_ = 1.0f - 1.0f / (0.015f * sampleRate_) *
                            semitonesToRatio(-decayXt * 72.0f - fmAmount2_ * 12.0f + snappy_ * 7.0f);
    snareDecay_ = 1.0f - 1.0f / (0.01f * sampleRate_) *
                             semitonesToRatio(-decay_ * 60.0f - snappy_ * 7.0f);
    fmDecay_ = 1.0f - 1.0f / (0.007f * sampleRate_);

    float snappy = clamp(snappy_ * 1.1f - 0.05f, 0.0f, 1.0f);
    drumLevel_ = std::sqrt(1.0f - snappy);
    snareLevel_ = std::sqrt(snappy);

    snareHp_.setCoefficients(std::fmin(10.0f * f0_, 0.5f), TanApprox::Fast);
    snareLp_.setCoefficients(std::fmin(35.0f * f0_, 0.5f), 0.5f + 2.0f * snappy, TanApprox::Fast);
    drumLp_.setCoefficients(3.0f * f0_, TanApprox::Fast);

    // Coupling only matters at low pitches (written against 48 kHz).
    resetNoiseAmount_ = clamp((0.125f - f048) * 8.0f, 0.0f, 1.0f);
    resetNoiseAmount_ *= resetNoiseAmount_ * fmAmount2_;
  }

  float sampleRate_ = 48000.0f;
  float hz_ = 200.0f, fmAmount_ = 0.5f, decay_ = 0.5f, snappy_ = 0.5f, accent_ = 0.8f;
  bool sustain_ = false;

  float f0_ = 0.0f, fmAmount2_ = 0.25f;
  float drumDecay_ = 0.99f, snareDecay_ = 0.99f, fmDecay_ = 0.99f;
  float drumLevel_ = 0.7f, snareLevel_ = 0.7f, resetNoiseAmount_ = 0.0f;

  float phase_[2] = {0.0f, 0.0f};
  float drumAmplitude_ = 0.0f, snareAmplitude_ = 0.0f, fm_ = 0.0f;
  int holdCounter_ = 0;
  unsigned tailToggle_ = 0;

  Random random_;
  OnePole drumLp_, snareHp_;
  Svf snareLp_;
};

}  // namespace pt

#endif  // PT_DRUMS_SYNTH_SNARE_H_
