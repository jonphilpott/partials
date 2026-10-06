// partials — osc/grainlet.h
//
// GrainletOscillator: a stream of tiny sound grains, one per pitch cycle:
// a formant sine shaped by a carrier window. Like the formant oscillator,
// but the window shape morphs from a short blip to a long smooth bell,
// for sounds from glassy and percussive to round and vocal. Plaits'
// "grainlet" (in its "grain" engine).
//
// How it works
// - Each carrier cycle restarts the formant sine and multiplies it by a
//   window (a stretched or squeezed half-sine) chosen by `shape`.
// - `bleed` lets the window itself through as well, adding the carrier's
//   fundamental and body.
// - The restart is PolyBLEP-corrected.
//
// Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.

#ifndef PT_OSC_GRAINLET_H_
#define PT_OSC_GRAINLET_H_

#include <cmath>

#include "pt/osc/polyblep.h"
#include "pt/osc/sine.h"

namespace pt {

class GrainletOscillator {
 public:
  void init(float sampleRate) {
    sampleTime_ = 1.0f / sampleRate;
    carrierPhase_ = formantPhase_ = nextSample_ = 0.0f;
    sineTable();
  }

  // Pitch in Hz (limited to an eighth of the sample rate).
  void setCarrierFrequency(float hz) { carrier_ = std::fmin(hz * sampleTime_, 0.125f); }
  // Grain (formant) frequency in Hz.
  void setFormantFrequency(float hz) { formant_ = std::fmin(hz * sampleTime_, 0.25f); }
  // 0..1. Window: short blip, then a skewed bell, then a long smooth bell.
  void setShape(float x) { shape_ = x; }
  // 0..1+. How much of the window itself is heard.
  void setBleed(float x) { bleed_ = x; }

  // Returns about ±1, one sample late (BLEP look-ahead).
  float process() {
    float thisSample = nextSample_;
    float nextSample = 0.0f;
    carrierPhase_ += carrier_;
    if (carrierPhase_ >= 1.0f) {
      carrierPhase_ -= 1.0f;
      float resetTime = carrierPhase_ / carrier_;
      float before = grainlet(1.0f, formantPhase_ + (1.0f - resetTime) * formant_);
      float after = grainlet(0.0f, 0.0f);
      float discontinuity = after - before;
      thisSample += discontinuity * thisBlepSample(resetTime);
      nextSample += discontinuity * nextBlepSample(resetTime);
      formantPhase_ = resetTime * formant_;
    } else {
      formantPhase_ += formant_;
      if (formantPhase_ >= 1.0f) formantPhase_ -= 1.0f;
    }
    nextSample += grainlet(carrierPhase_, formantPhase_);
    nextSample_ = nextSample;
    return thisSample;
  }

 private:
  // The window, 0..1, over one carrier cycle. Three regions of `shape`:
  // a squeezed half-sine (short blip), a half-sine with a moving peak, and
  // a stretched one (long bell).
  float carrier(float phase) const {
    float shape = shape_ * 3.0f;
    int region = static_cast<int>(shape);
    float t = 1.0f - (shape - static_cast<float>(region));
    if (region == 0) {
      phase = phase * (1.0f + t * t * t * 15.0f);
      if (phase >= 1.0f) phase = 1.0f;
      phase += 0.75f;
    } else if (region == 1) {
      float breakpoint = 0.001f + 0.499f * t * t * t;
      phase = phase < breakpoint ? phase * (0.5f / breakpoint)
                                 : 0.5f + (phase - breakpoint) * 0.5f / (1.0f - breakpoint);
      phase += 0.75f;
    } else {
      t = 1.0f - t;
      phase = 0.25f + phase * (0.5f + t * t * t * 14.5f);
      if (phase >= 0.75f) phase = 0.75f;
    }
    return (sineFromPhase(phase) + 1.0f) * 0.25f;
  }

  float grainlet(float carrierPhase, float formantPhase) const {
    return carrier(carrierPhase) * (sineFromPhase(formantPhase) + bleed_) / (1.0f + bleed_);
  }

  float sampleTime_ = 1.0f / 48000.0f;
  float carrier_ = 0.0f, formant_ = 0.0f, shape_ = 0.0f, bleed_ = 0.0f;
  float carrierPhase_ = 0.0f, formantPhase_ = 0.0f, nextSample_ = 0.0f;
};

}  // namespace pt

#endif  // PT_OSC_GRAINLET_H_
