// partials — physical/plucker.h
//
// Plucker: a short burst of noise shaped like a pick or finger, ready to
// excite a String or ModalResonator. Call trigger() on each note.
//
// Origin: Rings' internal exciter.
//
// How it works
// - Each trigger emits white noise for a fraction of one period of the
//   note. The fraction depends on `position`.
// - The burst goes through a feedback comb filter. A pick near the end of
//   a string excites harmonics differently from one in the middle, and the
//   comb imitates that.
// - A low-pass filter sets the burst's brightness: soft finger or hard
//   pick.
//
// Derived from Rings, Copyright 2015 Emilie Gillet. MIT licence.

#ifndef PT_PHYSICAL_PLUCKER_H_
#define PT_PHYSICAL_PLUCKER_H_

#include <cstddef>

#include "pt/core/random.h"
#include "pt/filter/svf.h"
#include "pt/fx/delay_line.h"

namespace pt {

class Plucker {
 public:
  void init(float sampleRate) {
    sampleRate_ = sampleRate;
    // Rings: comb delay up to 255 samples at 48 kHz.
    maxComb_ = 255.0f * sampleRate / 48000.0f;
    comb_.init(static_cast<size_t>(maxComb_) + 1);
    svf_.init(sampleRate);
    remaining_ = 0;
    combPeriod_ = 0.0f;
    combGain_ = 0.0f;
  }

  // Pitch of the note being plucked, in Hz (sets the burst length).
  void setFrequency(float hz) { frequency_ = hz / sampleRate_; }
  // Low-pass cutoff of the burst in Hz: low = soft, high = bright pick.
  void setCutoff(float hz) { cutoff_ = hz / sampleRate_; }
  // 0..1. Pluck position along the string.
  void setPosition(float x) { position_ = x; }

  void seed(uint32_t s) { random_.seed(s); }

  // Start a new burst using the current settings.
  void trigger() {
    float ratio = position_ * 0.9f + 0.05f;
    float period = 1.0f / frequency_ * ratio;
    remaining_ = static_cast<size_t>(period);
    while (period >= maxComb_) period *= 0.5f;
    combPeriod_ = period;
    combGain_ = (1.0f - position_) * 0.8f;
    svf_.setCoefficients(cutoff_ < 0.499f ? cutoff_ : 0.499f, 1.0f, TanApprox::Dirty);
  }

  float process() {
    float in = 0.0f;
    if (remaining_) {
      in = 2.0f * random_.uniform() - 1.0f;
      --remaining_;
    }
    float s = in + combGain_ * comb_.read(combPeriod_);
    comb_.write(s);
    return svf_.process(s).lp;
  }

 private:
  float sampleRate_ = 48000.0f;
  float maxComb_ = 255.0f;
  float frequency_ = 220.0f / 48000.0f;
  float cutoff_ = 0.1f;
  float position_ = 0.5f;

  size_t remaining_ = 0;
  float combPeriod_ = 0.0f;
  float combGain_ = 0.0f;
  Random random_;
  DelayLine comb_;
  Svf svf_;
};

}  // namespace pt

#endif  // PT_PHYSICAL_PLUCKER_H_
