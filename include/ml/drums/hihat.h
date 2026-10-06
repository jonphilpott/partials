// mutablelib — drums/hihat.h
//
// HiHat: a model of the TR-808 hi-hat, plus a ring-modulated variant.
// Plaits' hi-hat engine, which offers both (on OUT and AUX).
//
// How it works
// 1. Metallic noise. Two sources:
//    - SQUARE_808: six square-wave oscillators at unrelated ratios, summed,
//      as on the 808 (and its cymbal). The clash of inharmonic partials
//      sounds metallic.
//    - RING_MOD: three pairs of oscillators ring-modulated together, for a
//      harsher, more electronic sound.
// 2. A band-pass filter colours the noise (`tone`). With some
//    `noisiness`, the metallic noise is blended with clocked random noise,
//    for washier, cymbal-like sounds.
// 3. A VCA with a decaying envelope. SQUARE_808 uses the 808's lopsided
//    "swing" VCA and one decay; RING_MOD uses a clean VCA and a two-stage
//    decay (an open hat that chokes into a short tail).
// 4. A high-pass filter removes the low end.
//
// Sample-rate independence: oscillator and filter frequencies are in Hz;
// envelope coefficients are converted from Plaits' 48 kHz.
//
// Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.

#ifndef ML_DRUMS_HIHAT_H_
#define ML_DRUMS_HIHAT_H_

#include <cmath>
#include <cstdint>

#include "ml/core/math.h"
#include "ml/core/random.h"
#include "ml/core/units.h"
#include "ml/filter/svf.h"
#include "ml/osc/basic.h"

namespace ml {

class HiHat {
 public:
  enum Model { SQUARE_808, RING_MOD };

  void init(float sampleRate, Model model = SQUARE_808) {
    sampleRate_ = sampleRate;
    rateRatio_ = 48000.0f / sampleRate;
    model_ = model;
    for (int i = 0; i < 6; ++i) {
      squarePhase_[i] = 0;
      ringOsc_[i].init(sampleRate);
      ringOsc_[i].setShape(i % 2 == 0 ? BasicOscillator::SQUARE : BasicOscillator::SAW);
    }
    coloration_.init(sampleRate);
    hpf_.init(sampleRate);
    envelope_ = noiseClock_ = noiseSample_ = 0.0f;
    update();
  }

  // Base pitch of the metallic oscillators in Hz. Around 400 Hz is the 808.
  void setFrequency(float hz) { hz_ = hz; update(); }
  // 0..1. Colour: the band-pass and high-pass cutoff, from dark to sizzling.
  void setTone(float x) { tone_ = clamp(x, 0.0f, 1.0f); update(); }
  // 0..1. Closed (short) to open (long).
  void setDecay(float x) { decay_ = clamp(x, 0.0f, 1.0f); update(); }
  // 0..1. Blend of metallic noise and plain clocked noise.
  void setNoisiness(float x) { noisiness_ = clamp(x, 0.0f, 1.0f); update(); }
  // 0..1. How hard the next hit is (and the sustain-mode level).
  void setAccent(float x) { accent_ = clamp(x, 0.0f, 1.0f); }
  void setSustain(bool on) { sustain_ = on; }
  void seed(uint32_t s) { random_.seed(s); }

  void trigger() {
    envelope_ = (1.5f + 0.5f * (1.0f - decay_)) * (0.3f + 0.7f * accent_);
  }

  float process() {
    // 1. Metallic noise.
    float s = model_ == SQUARE_808 ? squareNoise() : ringModNoise();

    // 2. Colour, then blend in clocked noise.
    s = coloration_.process(s).bp;
    noiseClock_ += noiseF_;
    if (noiseClock_ >= 1.0f) {
      noiseClock_ -= 1.0f;
      noiseSample_ = random_.uniform() - 0.5f;
    }
    s += noisiness2_ * (noiseSample_ - s);

    // 3. VCA. The ring-mod model's envelope switches to a faster "cut"
    // decay once it falls below 0.5.
    bool twoStage = model_ == RING_MOD;
    envelope_ *= envelope_ > 0.5f || !twoStage ? envelopeDecay_ : cutDecay_;
    float gain = sustain_ ? accent_ * decay_ : envelope_;
    if (model_ == SQUARE_808) {
      // The 808's VCA: lopsided and saturating.
      s *= s > 0.0f ? 4.0f : 0.1f;
      s = s / (1.0f + std::fabs(s));
      s = (s + 0.1f) * gain;
    } else {
      s *= gain;
    }

    // 4. High-pass.
    return hpf_.process(s).hp;
  }

 private:
  // Six square waves at the 808's inharmonic ratios, summed. Phases are
  // 32-bit integers: overflow wraps them, and the top bit is the square.
  float squareNoise() {
    uint32_t sum = 0;
    for (int i = 0; i < 6; ++i) {
      squarePhase_[i] += squareIncrement_[i];
      sum += squarePhase_[i] >> 31;
    }
    return 0.33f * static_cast<float>(sum) - 1.0f;
  }

  // Three square x saw pairs, each ring-modulated (multiplied).
  float ringModNoise() {
    float out = 0.0f;
    for (int i = 0; i < 6; i += 2) {
      out += ringOsc_[i].process(ringF_[i], 0.5f) * ringOsc_[i + 1].process(ringF_[i + 1], 0.5f);
    }
    return out;
  }

  void update() {
    const float f0 = hz_ / sampleRate_;
    const float f048 = hz_ / 48000.0f;

    // 1. Metallic oscillators (the 808 ratios, nominally 414 Hz). Plaits
    // feeds them twice the note frequency.
    static const float kRatios[6] = {1.0f, 1.304f, 1.466f, 1.787f, 1.932f, 2.536f};
    for (int i = 0; i < 6; ++i) {
      float f = std::fmin(2.0f * f0 * kRatios[i], 0.499f);
      squareIncrement_[i] = static_cast<uint32_t>(f * 4294967296.0f);
    }
    // Ring-mod pairs: fixed frequencies in Hz, scaled by pitch.
    const float ratio = 2.0f * f048 / (0.01f + 2.0f * f048);
    static const float kRingHz[6] = {200.0f, 7530.0f, 510.0f, 8075.0f, 730.0f, 10500.0f};
    for (int i = 0; i < 6; ++i) ringF_[i] = kRingHz[i] / sampleRate_ * ratio;

    // 2. Filters and clocked noise.
    float cutoff = clamp(150.0f * semitonesToRatio(tone_ * 72.0f), 0.0f, 16000.0f) / sampleRate_;
    coloration_.setCoefficients(cutoff, model_ == SQUARE_808 ? 3.0f + 3.0f * tone_ : 1.0f, TanApprox::Accurate);
    hpf_.setCoefficients(cutoff, 0.5f, TanApprox::Accurate);
    noisiness2_ = noisiness_ * noisiness_;
    noiseF_ = clamp(f0 * (16.0f + 16.0f * (1.0f - noisiness2_)), 0.0f, 0.5f);

    // 3. Envelope rates.
    envelopeDecay_ = 1.0f - rescaleCoefficient(0.003f * semitonesToRatio(-decay_ * 84.0f), rateRatio_);
    cutDecay_ = 1.0f - rescaleCoefficient(0.0025f * semitonesToRatio(-decay_ * 36.0f), rateRatio_);
  }

  float sampleRate_ = 48000.0f, rateRatio_ = 1.0f;
  Model model_ = SQUARE_808;
  float hz_ = 400.0f, tone_ = 0.5f, decay_ = 0.5f, noisiness_ = 0.0f, accent_ = 0.8f;
  bool sustain_ = false;

  uint32_t squarePhase_[6] = {0, 0, 0, 0, 0, 0};
  uint32_t squareIncrement_[6] = {0, 0, 0, 0, 0, 0};
  float ringF_[6] = {0, 0, 0, 0, 0, 0};
  float noisiness2_ = 0.0f, noiseF_ = 0.0f;
  float envelopeDecay_ = 0.99f, cutDecay_ = 0.99f;
  float envelope_ = 0.0f, noiseClock_ = 0.0f, noiseSample_ = 0.0f;

  Random random_;
  BasicOscillator ringOsc_[6];
  Svf coloration_, hpf_;
};

}  // namespace ml

#endif  // ML_DRUMS_HIHAT_H_
