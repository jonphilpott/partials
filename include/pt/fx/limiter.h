// partials — fx/limiter.h
//
// Limiter: keeps a signal from exceeding full scale, smoothly, with no
// hard clipping. Rings' and Warps' output limiter. Put it last in a chain
// that might get loud: resonators, feedback, stacked voices.
//
// How it works
// - A peak follower tracks the signal level. It rises fast (attack about
//   0.5 ms) and falls very slowly (release about 1 s), so the gain doesn't
//   pump on every waveform cycle.
// - When the peak goes above 1, the gain becomes 1/peak, so loud passages
//   are turned down to full scale.
// - A soft clipper (scaled by 0.8 first) rounds off any fast transients
//   the follower is too slow to catch.
// - In stereo, both channels share one gain, which also watches the
//   left-right difference, so the stereo image doesn't wobble.
//
// Sample-rate independence: the follower's coefficients were per-sample at
// 48 kHz; they are converted for the actual rate.
//
// Derived from Rings and Warps, Copyright 2014-2015 Emilie Gillet. MIT licence.

#ifndef PT_FX_LIMITER_H_
#define PT_FX_LIMITER_H_

#include <cmath>

#include "pt/core/math.h"

namespace pt {

class Limiter {
 public:
  void init(float sampleRate) {
    float ratio = 48000.0f / sampleRate;
    attack_ = rescaleCoefficient(0.05f, ratio);
    release_ = rescaleCoefficient(0.00002f, ratio);
    peak_ = 0.5f;
  }

  // Gain applied before limiting (1 = unity). Raise it to make quiet
  // material louder: the limiter will catch the peaks.
  void setPreGain(float gain) { preGain_ = gain; }

  float process(float in) {
    float pre = in * preGain_;
    slope(peak_, std::fabs(pre), attack_, release_);
    return softLimit(pre * gain() * 0.8f);
  }

  void process(float& left, float& right) {
    float l = left * preGain_;
    float r = right * preGain_;
    float peak = std::fmax(std::fmax(std::fabs(l), std::fabs(r)), std::fabs(r - l));
    slope(peak_, peak, attack_, release_);
    float g = gain() * 0.8f;
    left = softLimit(l * g);
    right = softLimit(r * g);
  }

 private:
  float gain() const { return peak_ <= 1.0f ? 1.0f : 1.0f / peak_; }

  float attack_ = 0.05f, release_ = 0.00002f;
  float peak_ = 0.5f;
  float preGain_ = 1.0f;
};

}  // namespace pt

#endif  // PT_FX_LIMITER_H_
