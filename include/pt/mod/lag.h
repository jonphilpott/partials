// partials — mod/lag.h
//
// Lag: clock-synchronised glide between stepped values, from a sharp step
// to a smooth glide or a sine-like curve, all in time with a phase ramp.
// Marbles' "steps" control (lag processor).
//
// How it works
// - The glide time follows the clock. The phase ramp (0..1 per step, e.g.
//   from ClockToRamp or your sequencer) tells the lag how long a step is,
//   so the same setting glides proportionally at any tempo.
// - Smoothness 0..0.6: a low-pass glide, from instant (0) to about half a
//   step (0.6).
// - 0.6..1: crossfades into an interpolation along the step, from S-curved
//   (raised cosine) to linear, so the output traces a smooth curve
//   through the values instead of lagging behind them.
//
// Derived from Marbles, Copyright 2015 Emilie Gillet. MIT licence.

#ifndef PT_MOD_LAG_H_
#define PT_MOD_LAG_H_

#include <cmath>

#include "pt/core/math.h"
#include "pt/core/units.h"

namespace pt {

class Lag {
 public:
  void init() {
    rampStart_ = rampValue_ = lpState_ = previousPhase_ = 0.0f;
  }

  // Call at each new step (when the value changes), so the interpolation
  // starts from where the output is now.
  void resetRamp() { rampStart_ = rampValue_; }

  // value: the current step's target. smoothness: 0..1. phase: position
  // within the step, 0..1, rising (wrapping back to 0 at the next step).
  float process(float value, float smoothness, float phase) {
    // 1. Step rate from the phase ramp.
    float frequency = phase - previousPhase_;
    if (frequency < 0.0f) frequency += 1.0f;
    previousPhase_ = phase;

    // 2. Low-pass glide: from half the step rate up to 7 octaves above;
    // near 0 smoothness, no lag at all.
    frequency *= 0.25f;
    frequency *= semitonesToRatio(84.0f * (1.0f - smoothness));
    if (frequency >= 1.0f) frequency = 1.0f;
    if (smoothness <= 0.05f) frequency += 20.0f * (0.05f - smoothness) * (1.0f - frequency);
    onePole(lpState_, value, frequency);

    // 3. Interpolation along the step: raised-cosine (S-curve) blending to
    // linear.
    float interpAmount = clamp((smoothness - 0.6f) * 5.0f, 0.0f, 1.0f);
    float linearity = clamp((1.0f - smoothness) * 5.0f, 0.0f, 1.0f);
    float warped = 0.5f - 0.5f * std::cos(phase * 3.14159265f);
    float interpPhase = crossfade(warped, phase, linearity);
    float interp = crossfade(rampStart_, value, interpPhase);
    rampValue_ = interp;
    return crossfade(lpState_, interp, interpAmount);
  }

 private:
  float rampStart_ = 0.0f, rampValue_ = 0.0f, lpState_ = 0.0f, previousPhase_ = 0.0f;
};

}  // namespace pt

#endif  // PT_MOD_LAG_H_
