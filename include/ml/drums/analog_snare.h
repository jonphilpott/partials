// mutablelib — drums/analog_snare.h
//
// AnalogSnare: a circuit model of the Roland TR-808 snare drum, extended.
// Plaits' "analog snare drum".
//
// How it works
// - The drum "shell" is a set of resonant band-pass filters ("modes")
//   struck by a short pulse. The 808 has two (at 1x and 2x the pitch).
//   Above two-thirds of the `tone` range, three more inharmonic modes fade
//   in: what the 808 could have been with extra modes.
// - The "snares" (the rattling wires) are half-wave-rectified noise
//   through a band-pass, with their own decaying envelope.
// - `snappy` balances shell against snares.
// - Sustain mode replaces the filters with sine oscillators for a steady
//   tone.
//
// Sample-rate independence: times are in seconds, Q values and per-sample
// coefficients are converted from Plaits' 48 kHz.
//
// Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.

#ifndef ML_DRUMS_ANALOG_SNARE_H_
#define ML_DRUMS_ANALOG_SNARE_H_

#include <cmath>

#include "ml/core/math.h"
#include "ml/core/random.h"
#include "ml/core/units.h"
#include "ml/filter/svf.h"
#include "ml/osc/sine.h"

namespace ml {

class AnalogSnare {
 public:
  static const int kNumModes = 5;

  void init(float sampleRate) {
    sampleRate_ = sampleRate;
    rateRatio_ = 48000.0f / sampleRate;
    for (int i = 0; i < kNumModes; ++i) {
      resonator_[i].init(sampleRate);
      oscillator_[i].init(sampleRate);
    }
    noiseFilter_.init(sampleRate);
    pulseRemaining_ = 0;
    pulse_ = pulseHeight_ = pulseLp_ = noiseEnvelope_ = 0.0f;
    pulseLpCoefficient_ = rescaleCoefficient(0.75f, rateRatio_);
    update();
  }

  // Pitch of the shell in Hz. 808 snares sit around 150-250 Hz.
  void setFrequency(float hz) { hz_ = hz; update(); }
  // 0..1. Below 2/3: the 808's two modes, from low/round to brighter.
  // Above: three extra modes fade in.
  void setTone(float x) { tone_ = clamp(x, 0.0f, 1.0f); update(); }
  // 0..1. Ring time of shell and snares.
  void setDecay(float x) { decay_ = clamp(x, 0.0f, 1.0f); update(); }
  // 0..1. Balance: 0 = all shell, 1 = all snares.
  void setSnappy(float x) { snappy_ = clamp(x, 0.0f, 1.0f); update(); }
  // 0..1. How hard the next hit is (and the sustain-mode level).
  void setAccent(float x) { accent_ = clamp(x, 0.0f, 1.0f); }
  void setSustain(bool on) { sustain_ = on; }
  void seed(uint32_t s) { random_.seed(s); }

  void trigger() {
    pulseRemaining_ = static_cast<int>(1.0e-3f * sampleRate_);
    pulseHeight_ = 3.0f + 7.0f * accent_;
    noiseEnvelope_ = 2.0f;
  }

  float process() {
    // 1. Trigger pulse (Q45 / Q46).
    float pulse;
    if (pulseRemaining_) {
      --pulseRemaining_;
      pulse = pulseRemaining_ ? pulseHeight_ : pulseHeight_ - 1.0f;
      pulse_ = pulse;
    } else {
      pulse_ *= 1.0f - 1.0f / (0.1e-3f * sampleRate_);
      pulse = pulse_;
    }
    const float sustainGain = accent_ * decay_;

    // 2. Shell: excite each mode (R189 / C57 / R190 + C58 / C59 / R197 /
    // R196 / IC14). The first mode gets the differentiated pulse, the
    // others a little of the raw pulse.
    onePole(pulseLp_, pulse, pulseLpCoefficient_);
    float shell = 0.0f;
    for (int i = 0; i < kNumModes; ++i) {
      float excitation = i == 0 ? (pulse - pulseLp_) + 0.006f * pulse : 0.026f * pulse;
      shell += gain_[i] * (sustain_
          ? oscillator_[i].next(f_[i]) * sustainGain * 0.25f
          : resonator_[i].process(excitation).bp + excitation * exciterLeak_);
    }
    shell = softClip(shell);

    // 3. Snares: rectified noise, decaying envelope, band-pass (C56 / R194
    // / Q48 / C54 / R188 / D54, then C66 / R201 / C67 / R202 / R203 / Q49).
    float noise = 2.0f * random_.uniform() - 1.0f;
    if (noise < 0.0f) noise = 0.0f;
    noiseEnvelope_ *= noiseEnvelopeDecay_;
    noise *= (sustain_ ? sustainGain : noiseEnvelope_) * snappyGain_ * 2.0f;
    noise = noiseFilter_.process(noise).bp;

    // 4. Mix (IC13).
    return noise + shell * (1.0f - snappyGain_);
  }

 private:
  void update() {
    const float f0 = hz_ / sampleRate_;
    const float f048 = hz_ / 48000.0f;

    // 1. Mode tuning and decay. Q is written against the 48 kHz frequency
    // so ring times hold at any rate.
    static const float kModeFrequencies[kNumModes] = {1.00f, 2.00f, 3.18f, 4.16f, 5.62f};
    const float decayXt = decay_ * (1.0f + decay_ * (decay_ - 1.0f));
    const float q = 2000.0f * semitonesToRatio(decayXt * 84.0f);
    for (int i = 0; i < kNumModes; ++i) {
      f_[i] = std::fmin(f0 * kModeFrequencies[i], 0.499f);
      float f48 = std::fmin(f048 * kModeFrequencies[i], 0.499f);
      resonator_[i].setCoefficients(f_[i], 1.0f + f48 * (i == 0 ? q : q * 0.25f), TanApprox::Fast);
    }

    // 2. Mode levels from the tone knob.
    float tone = tone_;
    if (tone < 0.666667f) {
      // 808-style: two modes.
      tone *= 1.5f;
      gain_[0] = 1.5f + (1.0f - tone) * (1.0f - tone) * 4.5f;
      gain_[1] = 2.0f * tone + 0.15f;
      for (int i = 2; i < kNumModes; ++i) gain_[i] = 0.0f;
    } else {
      // Extra modes fade in.
      tone = (tone - 0.666667f) * 3.0f;
      gain_[0] = 1.5f - tone * 0.5f;
      gain_[1] = 2.15f - tone * 0.7f;
      for (int i = 2; i < kNumModes; ++i) {
        gain_[i] = tone;
        tone *= tone;
      }
    }

    // 3. Snares: envelope speed, filter, balance.
    noiseEnvelopeDecay_ = 1.0f - rescaleCoefficient(
        0.0017f * semitonesToRatio(-decay_ * (50.0f + snappy_ * 10.0f)), rateRatio_);
    exciterLeak_ = snappy_ * (2.0f - snappy_) * 0.1f;
    snappyGain_ = clamp(snappy_ * 1.1f - 0.05f, 0.0f, 1.0f);
    float fNoise = clamp(f0 * 16.0f, 0.0f, 0.499f);
    float fNoise48 = clamp(f048 * 16.0f, 0.0f, 0.499f);
    noiseFilter_.setCoefficients(fNoise, 1.0f + fNoise48 * 1.5f, TanApprox::Fast);
  }

  float sampleRate_ = 48000.0f, rateRatio_ = 1.0f;
  float hz_ = 200.0f, tone_ = 0.5f, decay_ = 0.5f, snappy_ = 0.5f, accent_ = 0.8f;
  bool sustain_ = false;

  float f_[kNumModes] = {0, 0, 0, 0, 0};
  float gain_[kNumModes] = {0, 0, 0, 0, 0};
  float noiseEnvelopeDecay_ = 0.99f, exciterLeak_ = 0.0f, snappyGain_ = 0.5f;
  float pulseLpCoefficient_ = 0.75f;

  int pulseRemaining_ = 0;
  float pulse_ = 0.0f, pulseHeight_ = 0.0f, pulseLp_ = 0.0f, noiseEnvelope_ = 0.0f;

  Random random_;
  Svf resonator_[kNumModes];
  Svf noiseFilter_;
  SineOscillator oscillator_[kNumModes];
};

}  // namespace ml

#endif  // ML_DRUMS_ANALOG_SNARE_H_
