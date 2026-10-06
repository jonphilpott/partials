// mutablelib — osc/z_osc.h
//
// ZOscillator: a resonant, filter-sweep-like tone in the style of the
// Casio CZ synths' "resonance" waveforms. Plaits' Z mode (in its
// "grain" engine).
//
// How it works: the CZ trick for resonance without a filter.
// - A "formant" sine runs at the resonant frequency and is restarted twice
//   per carrier cycle.
// - It is multiplied by a window that fades out over each half cycle
//   ("ramp down"), and by a contour shaped by `shape`.
// - The window makes the formant sound like the ringing of a resonant
//   filter excited once per cycle, and sweeping the formant sounds like
//   sweeping the cutoff.
// - `mode` picks the window and offset style (three CZ-like variants).
// - Restarts are PolyBLEP-corrected.
//
// Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.

#ifndef ML_OSC_Z_OSC_H_
#define ML_OSC_Z_OSC_H_

#include <cmath>

#include "ml/osc/polyblep.h"
#include "ml/osc/sine.h"

namespace ml {

class ZOscillator {
 public:
  void init(float sampleRate) {
    sampleTime_ = 1.0f / sampleRate;
    carrierPhase_ = discontinuityPhase_ = formantPhase_ = nextSample_ = 0.0f;
    sineTable();
  }

  // Pitch in Hz (limited to an eighth of the sample rate).
  void setCarrierFrequency(float hz) { carrier_ = std::fmin(hz * sampleTime_, 0.125f); }
  // The "resonance" frequency in Hz.
  void setFormantFrequency(float hz) { formant_ = std::fmin(hz * sampleTime_, 0.25f); }
  // 0..1. Carrier contour: below 0.5 it narrows each cycle's window; above,
  // it shifts its phase.
  void setShape(float x) { shape_ = x; }
  // 0..1. Three window/offset styles, at 0..1/3, 1/3..2/3, 2/3..1.
  void setMode(float x) { mode_ = x; }

  // Returns about ±1, one sample late (BLEP look-ahead).
  float process() {
    const float f0 = carrier_, f1 = formant_;
    float thisSample = nextSample_;
    float nextSample = 0.0f;
    // The discontinuity phase runs at twice the carrier: two restarts per
    // carrier cycle.
    discontinuityPhase_ += 2.0f * f0;
    carrierPhase_ += f0;
    if (discontinuityPhase_ >= 1.0f) {
      discontinuityPhase_ -= 1.0f;
      float resetTime = discontinuityPhase_ / (2.0f * f0);
      float carrierBefore = carrierPhase_ >= 1.0f ? 1.0f : 0.5f;
      float carrierAfter = carrierPhase_ >= 1.0f ? 0.0f : 0.5f;
      float before = z(carrierBefore, 1.0f, formantPhase_ + (1.0f - resetTime) * f1, shape_, mode_);
      float after = z(carrierAfter, 0.0f, 0.0f, shape_, mode_);
      float discontinuity = after - before;
      thisSample += discontinuity * thisBlepSample(resetTime);
      nextSample += discontinuity * nextBlepSample(resetTime);
      formantPhase_ = resetTime * f1;
      if (carrierPhase_ > 1.0f) carrierPhase_ = discontinuityPhase_ * 0.5f;
    } else {
      formantPhase_ += f1;
      if (formantPhase_ >= 1.0f) formantPhase_ -= 1.0f;
    }
    if (carrierPhase_ >= 1.0f) carrierPhase_ -= 1.0f;
    nextSample += z(carrierPhase_, discontinuityPhase_, formantPhase_, shape_, mode_);
    nextSample_ = nextSample;
    return thisSample;
  }

 private:
  // One sample of the windowed formant. c: carrier phase, d: position in
  // the current half cycle, f: formant phase.
  static float z(float c, float d, float f, float shape, float mode) {
    float rampDown = 0.5f * (1.0f + sineFromPhase(0.5f * d + 0.25f));
    float offset, phaseShift;
    if (mode < 0.333f) {
      offset = 1.0f;
      phaseShift = 0.25f + mode * 1.50f;
    } else if (mode < 0.666f) {
      phaseShift = 0.7495f - (mode - 0.33f) * 0.75f;
      offset = -sineFromPhase(phaseShift);
    } else {
      phaseShift = 0.7495f - (mode - 0.33f) * 0.75f;
      offset = 0.001f;
    }
    float discontinuity = sineFromPhase(f + phaseShift);
    float contour;
    if (shape < 0.5f) {
      shape *= 2.0f;
      if (c >= 0.5f) rampDown *= shape;
      contour = 1.0f + (sineFromPhase(c + 0.25f) - 1.0f) * shape;
    } else {
      contour = sineFromPhase(c + shape * 0.5f);
    }
    return (rampDown * (offset + discontinuity) - offset) * contour;
  }

  float sampleTime_ = 1.0f / 48000.0f;
  float carrier_ = 0.0f, formant_ = 0.0f, shape_ = 0.0f, mode_ = 0.0f;
  float carrierPhase_ = 0.0f, discontinuityPhase_ = 0.0f, formantPhase_ = 0.0f, nextSample_ = 0.0f;
};

}  // namespace ml

#endif  // ML_OSC_Z_OSC_H_
