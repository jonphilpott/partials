// mutablelib — drums/analog_kick.h
//
// AnalogKick: a circuit model of the Roland TR-808 bass drum, with extras.
// Plaits' "analog bass drum" (the first drum model on its last bank).
//
// How it works (the comments in process() name the 808's own components)
// - A trigger fires a short pulse into a resonant band-pass filter tuned
//   to the drum's pitch. Like the 808's "bridged-T" network, the filter
//   rings and decays: that ringing *is* the drum. `decay` sets the
//   filter's Q, so it sets how long it rings.
// - A second, longer pulse briefly raises the pitch at the start (the
//   808's pitch "knock"; `attackFm`), and the drum can modulate its own
//   pitch by its output level (`selfFm`), for a punchier, pitch-dropping
//   thump.
// - A diode clips the excitation and a low-pass filter shapes the tone.
//   `tone` also lets some of the click leak through.
// - In sustain mode the filter is replaced by a sine oscillator, giving a
//   continuous 808-style tone (Plaits does this when no trigger is
//   patched).
//
// Sample-rate independence: pulse lengths and decay times are in seconds,
// the filter's Q and the tone filter are converted from Plaits' 48 kHz, so
// the drum sounds the same at any rate.
//
// Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.

#ifndef ML_DRUMS_ANALOG_KICK_H_
#define ML_DRUMS_ANALOG_KICK_H_

#include <cmath>

#include "ml/core/math.h"
#include "ml/core/units.h"
#include "ml/filter/svf.h"
#include "ml/osc/sine.h"

namespace ml {

class AnalogKick {
 public:
  void init(float sampleRate) {
    sampleRate_ = sampleRate;
    rateRatio_ = 48000.0f / sampleRate;
    qScale_ = sampleRate / 48000.0f;
    resonator_.init(sampleRate);
    oscillator_.init(sampleRate);
    pulseRemaining_ = fmPulseRemaining_ = 0;
    pulse_ = pulseHeight_ = pulseLp_ = fmPulseLp_ = retrigPulse_ = 0.0f;
    lpOut_ = toneLp_ = 0.0f;
    update();
  }

  // Pitch in Hz. 808 kicks sit around 40-60 Hz.
  void setFrequency(float hz) { hz_ = hz; update(); }
  // 0..1. Brightness: the tone filter's cutoff, and how much click leaks.
  void setTone(float x) { tone_ = clamp(x, 0.0f, 1.0f); update(); }
  // 0..1. Ring time, from a short thud to a long boom.
  void setDecay(float x) { decay_ = clamp(x, 0.0f, 1.0f); update(); }
  // 0..1. Pitch knock at the attack.
  void setAttackFm(float x) { attackFm_ = clamp(x, 0.0f, 1.0f); }
  // 0..1. Pitch rising with the output level: punchier, with a drop.
  void setSelfFm(float x) { selfFm_ = clamp(x, 0.0f, 1.0f); }
  // 0..1. How hard the next hit is (and the sustain-mode level).
  void setAccent(float x) { accent_ = clamp(x, 0.0f, 1.0f); }
  // Continuous tone instead of triggered hits.
  void setSustain(bool on) { sustain_ = on; }

  void trigger() {
    pulseRemaining_ = static_cast<int>(1.0e-3f * sampleRate_);
    fmPulseRemaining_ = static_cast<int>(6.0e-3f * sampleRate_);
    pulseHeight_ = 3.0f + 7.0f * accent_;
    lpOut_ = 0.0f;
  }

  float process() {
    const float pulseDecayTime = 0.2e-3f * sampleRate_;
    const float pulseFilterTime = 0.1e-3f * sampleRate_;
    const float retrigPulseDuration = 0.05f * sampleRate_;

    // 1. Trigger pulse (Q39 / Q40): a flat top, then an exponential tail.
    float pulse;
    if (pulseRemaining_) {
      --pulseRemaining_;
      pulse = pulseRemaining_ ? pulseHeight_ : pulseHeight_ - 1.0f;
      pulse_ = pulse;
    } else {
      pulse_ *= 1.0f - 1.0f / pulseDecayTime;
      pulse = pulse_;
    }
    if (sustain_) pulse = 0.0f;

    // 2. Differentiate and clip the pulse (C40 / R163 / R162 / D83).
    onePole(pulseLp_, pulse, 1.0f / pulseFilterTime);
    pulse = diode((pulse - pulseLp_) + pulse * 0.044f);

    // 3. Pitch-knock pulse (Q41 / Q42), and the small negative
    // "retrigger" pulse at its end (C39 / C52).
    float fmPulse = 0.0f;
    if (fmPulseRemaining_) {
      --fmPulseRemaining_;
      fmPulse = 1.0f;
      retrigPulse_ = fmPulseRemaining_ ? 0.0f : -0.8f;
    } else {
      retrigPulse_ *= 1.0f - 1.0f / retrigPulseDuration;
    }
    if (sustain_) fmPulse = 0.0f;
    onePole(fmPulseLp_, fmPulse, 1.0f / pulseFilterTime);

    // 4. Pitch: base + knock + self-modulation (Q43 / R165 / R170).
    float punch = 0.7f + diode(10.0f * lpOut_ - 1.0f);
    float attackFm = fmPulseLp_ * 1.7f * attackFm_;
    float selfFm = punch * 0.08f * selfFm_;
    float f = clamp(f0_ * (1.0f + attackFm + selfFm), 0.0f, 0.4f);

    // 5. The ringing filter (or the sine, in sustain mode). Q is written
    // against the 48 kHz-normalised frequency so decay times hold at any
    // rate.
    float resonatorOut;
    if (sustain_) {
      oscillator_.next(f, accent_ * decay_, resonatorOut, lpOut_);
    } else {
      resonator_.setCoefficients(f, 1.0f + q_ * f * qScale_, TanApprox::Dirty);
      SvfOut o = resonator_.process((pulse - retrigPulse_ * 0.2f) * scale_);
      resonatorOut = o.bp;
      lpOut_ = o.lp;
    }

    // 6. Tone filter, with some excitation leaking through.
    onePole(toneLp_, pulse * exciterLeak_ + resonatorOut, toneCoefficient_);
    return toneLp_;
  }

 private:
  static float diode(float x) {
    if (x >= 0.0f) return x;
    x *= 2.0f;
    return 0.7f * x / (1.0f + std::fabs(x));
  }

  void update() {
    f0_ = hz_ / sampleRate_;
    float f048 = hz_ / 48000.0f;
    // The excitation is scaled by 1/f so the level doesn't depend on pitch
    // (written against Plaits' 48 kHz frequency).
    scale_ = 0.001f / f048;
    q_ = 1500.0f * semitonesToRatio(decay_ * 80.0f);
    toneCoefficient_ = rescaleCoefficient(
        std::fmin(4.0f * f048 * semitonesToRatio(tone_ * 108.0f), 1.0f), rateRatio_);
    exciterLeak_ = 0.08f * (tone_ + 0.25f);
  }

  float sampleRate_ = 48000.0f, rateRatio_ = 1.0f, qScale_ = 1.0f;
  float hz_ = 50.0f, tone_ = 0.5f, decay_ = 0.5f;
  float attackFm_ = 0.5f, selfFm_ = 0.5f, accent_ = 0.8f;
  bool sustain_ = false;

  float f0_ = 0.0f, scale_ = 1.0f, q_ = 1500.0f;
  float toneCoefficient_ = 1.0f, exciterLeak_ = 0.0f;

  int pulseRemaining_ = 0, fmPulseRemaining_ = 0;
  float pulse_ = 0.0f, pulseHeight_ = 0.0f, pulseLp_ = 0.0f;
  float fmPulseLp_ = 0.0f, retrigPulse_ = 0.0f;
  float lpOut_ = 0.0f, toneLp_ = 0.0f;

  Svf resonator_;
  SineOscillator oscillator_;
};

}  // namespace ml

#endif  // ML_DRUMS_ANALOG_KICK_H_
