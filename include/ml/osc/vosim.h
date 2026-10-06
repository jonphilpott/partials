// mutablelib — osc/vosim.h
//
// VosimOscillator: VOSIM ("VOice SIMulation") synthesis, a 1970s technique
// for vowel-like sounds. Two formant sines are restarted at the pitch rate
// and shaped by a decaying window, giving two spectral peaks, as a voice
// has. Plaits' VOSIM mode (in its "grain" engine).
//
// How it works: at each pitch cycle both formant sines restart. A raised
// cosine window (one per cycle) fades them, so each cycle is a short burst
// of two sines. Pitch comes from the restart rate; the vowel colour comes
// from the two formant frequencies. `shape` changes the sines' starting
// phase, from smooth to buzzy.
//
// Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.

#ifndef ML_OSC_VOSIM_H_
#define ML_OSC_VOSIM_H_

#include <cmath>

#include "ml/osc/sine.h"

namespace ml {

class VosimOscillator {
 public:
  void init(float sampleRate) {
    sampleTime_ = 1.0f / sampleRate;
    carrierPhase_ = formant1Phase_ = formant2Phase_ = 0.0f;
    sineTable();
  }

  // The pitch, in Hz.
  void setCarrierFrequency(float hz) { carrier_ = std::fmin(hz * sampleTime_, 0.25f); }
  // The two formants (spectral peaks), in Hz. Vowels sit around 300-900 Hz
  // for the first and 900-2500 Hz for the second.
  void setFormantFrequencies(float hz1, float hz2) {
    formant1_ = std::fmin(hz1 * sampleTime_, 0.25f);
    formant2_ = std::fmin(hz2 * sampleTime_, 0.25f);
  }
  // 0..1. Starting phase of the formant bursts: smooth (0) to buzzy (1).
  void setShape(float x) { shape_ = x; }

  // Returns about ±1. (No look-ahead: the restarts are smooth enough
  // without correction.)
  float process() {
    carrierPhase_ += carrier_;
    if (carrierPhase_ >= 1.0f) {
      carrierPhase_ -= 1.0f;
      float resetTime = carrierPhase_ / carrier_;
      formant1Phase_ = resetTime * formant1_;
      formant2Phase_ = resetTime * formant2_;
    } else {
      formant1Phase_ += formant1_;
      if (formant1Phase_ >= 1.0f) formant1Phase_ -= 1.0f;
      formant2Phase_ += formant2_;
      if (formant2Phase_ >= 1.0f) formant2Phase_ -= 1.0f;
    }
    // The window: half a cosine per cycle, 2 at the start to 0 at the end.
    float window = sineFromPhase(carrierPhase_ * 0.5f + 0.25f) + 1.0f;
    float resetPhase = 0.75f - 0.25f * shape_;
    float resetAmplitude = sineFromPhase(resetPhase);
    float f1 = sineFromPhase(formant1Phase_ + resetPhase) - resetAmplitude;
    float f2 = sineFromPhase(formant2Phase_ + resetPhase) - resetAmplitude;
    return window * (f1 + f2) * 0.25f + resetAmplitude;
  }

 private:
  float sampleTime_ = 1.0f / 48000.0f;
  float carrier_ = 0.0f, formant1_ = 0.0f, formant2_ = 0.0f, shape_ = 0.0f;
  float carrierPhase_ = 0.0f, formant1Phase_ = 0.0f, formant2Phase_ = 0.0f;
};

}  // namespace ml

#endif  // ML_OSC_VOSIM_H_
