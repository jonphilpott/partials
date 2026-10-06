// mutablelib — osc/formant.h
//
// FormantOscillator: a sine at the "formant" frequency, restarted at the
// "carrier" (pitch) frequency. The result has the pitch of the carrier but
// a peak in its spectrum at the formant: a vocal or nasal tone. Sweeping
// the formant sounds like a filter sweep with no filter. Plaits' formant
// mode.
//
// How it works: two phase counters. Each time the carrier wraps, the
// formant sine restarts from zero phase (hard sync). The restart is a
// jump, so it gets a PolyBLEP correction for an alias-free result.
// `phaseShift` offsets the formant sine's phase, changing the waveform
// shape at the restart.
//
// Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.

#ifndef ML_OSC_FORMANT_H_
#define ML_OSC_FORMANT_H_

#include <cmath>

#include "ml/osc/polyblep.h"
#include "ml/osc/sine.h"

namespace ml {

class FormantOscillator {
 public:
  void init(float sampleRate) {
    sampleTime_ = 1.0f / sampleRate;
    carrierPhase_ = formantPhase_ = nextSample_ = 0.0f;
    sineTable();
  }

  // The pitch you hear, in Hz.
  void setCarrierFrequency(float hz) { carrier_ = std::fmin(hz * sampleTime_, 0.25f); }
  // The spectral peak, in Hz. Usually above the carrier.
  void setFormantFrequency(float hz) { formant_ = std::fmin(hz * sampleTime_, 0.25f); }
  // Phase offset of the formant sine, in cycles (0..1).
  void setPhaseShift(float x) { phaseShift_ = x; }

  // Returns about ±1, one sample late (BLEP look-ahead).
  float process() {
    float thisSample = nextSample_;
    float nextSample = 0.0f;
    carrierPhase_ += carrier_;
    if (carrierPhase_ >= 1.0f) {
      // Restart the formant at the exact moment the carrier wrapped.
      carrierPhase_ -= 1.0f;
      float resetTime = carrierPhase_ / carrier_;
      float before = sineFromPhase(formantPhase_ + (1.0f - resetTime) * formant_ + phaseShift_);
      float after = sineFromPhase(phaseShift_);
      float discontinuity = after - before;
      thisSample += discontinuity * thisBlepSample(resetTime);
      nextSample += discontinuity * nextBlepSample(resetTime);
      formantPhase_ = resetTime * formant_;
    } else {
      formantPhase_ += formant_;
      if (formantPhase_ >= 1.0f) formantPhase_ -= 1.0f;
    }
    nextSample += sineFromPhase(formantPhase_ + phaseShift_);
    nextSample_ = nextSample;
    return thisSample;
  }

 private:
  float sampleTime_ = 1.0f / 48000.0f;
  float carrier_ = 0.0f, formant_ = 0.01f, phaseShift_ = 0.0f;
  float carrierPhase_ = 0.0f, formantPhase_ = 0.0f, nextSample_ = 0.0f;
};

}  // namespace ml

#endif  // ML_OSC_FORMANT_H_
