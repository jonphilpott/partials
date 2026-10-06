// partials — core/math.h
//
// Small maths helpers used by nearly every component. These replace the
// macros and inline functions of stmlib's dsp/dsp.h and stmlib.h.
//
// Derived from stmlib, Copyright 2012-2014 Emilie Gillet. MIT licence.
// See LICENSE in the repository root.

#ifndef PT_CORE_MATH_H_
#define PT_CORE_MATH_H_

#include <cmath>
#include <cstdint>

namespace pt {

// Keep x within [lo, hi]. (std::clamp only arrived in C++17; Rack plugins
// build as C++11.)
inline float clamp(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

// Read a lookup table at a fractional position, blending the two nearest
// entries ("linear interpolation").
//
// `index` is normalised 0..1 and `size` is the number of table *steps*, so
// the table must hold size + 1 entries (the extra guard point saves a wrap
// check on the last step). Gillet's tables are all built this way.
inline float interpolate(const float* table, float index, float size) {
  index *= size;
  int32_t i = static_cast<int32_t>(index);  // whole part picks the entry...
  float frac = index - static_cast<float>(i);  // ...fractional part blends
  float a = table[i];
  float b = table[i + 1];
  return a + (b - a) * frac;
}

// As interpolate(), but `index` may be any value: only its fractional part
// is used, so 1.25 reads the same place as 0.25. Handy for periodic tables
// such as one cycle of a sine wave driven by an ever-growing phase.
inline float interpolateWrap(const float* table, float index, float size) {
  index -= static_cast<float>(static_cast<int32_t>(index));
  return interpolate(table, index, size);
}

// 4-point Hermite (cubic) interpolation. Smoother than linear: it fits a
// curve through the two neighbours on either side, so it doesn't add the
// high-frequency "corners" linear interpolation does. Needs one guard entry
// before the read position and two after.
inline float interpolateHermite(const float* table, float index, float size) {
  index *= size;
  int32_t i = static_cast<int32_t>(index);
  float f = index - static_cast<float>(i);
  const float xm1 = table[i - 1];
  const float x0 = table[i];
  const float x1 = table[i + 1];
  const float x2 = table[i + 2];
  // The polynomial coefficients, arranged so the evaluation below costs
  // only a few multiplies (Horner's method: ((a*f + b)*f + c)*f + d).
  const float c = (x1 - xm1) * 0.5f;
  const float v = x0 - x1;
  const float w = c + v;
  const float a = w + v + (x2 - x0) * 0.5f;
  const float bNeg = w + a;
  return (((a * f) - bNeg) * f + c) * f + x0;
}

// Linear blend: fade = 0 gives a, fade = 1 gives b.
inline float crossfade(float a, float b, float fade) {
  return a + (b - a) * fade;
}

// S-shaped ease curve on 0..1 (zero slope at both ends).
inline float smoothStep(float x) {
  return x * x * (3.0f - 2.0f * x);
}

// Gentle saturation: a rational approximation of tanh(). Linear near zero,
// reaches exactly ±1 at x = ±3 with zero slope there. Cheap enough to run
// on every sample.
inline float softLimit(float x) {
  return x * (27.0f + x * x) / (27.0f + 9.0f * x * x);
}

// softLimit() is only valid on ±3, so clamp outside that range.
inline float softClip(float x) {
  if (x < -3.0f) return -1.0f;
  if (x > 3.0f) return 1.0f;
  return softLimit(x);
}

// One-pole low-pass step: move `state` a fraction `coefficient` (0..1) of
// the way towards `target`. Replaces stmlib's ONE_POLE macro. With a
// coefficient c applied every sample, the time constant is about
// 1 / (c * sampleRate) seconds.
inline void onePole(float& state, float target, float coefficient) {
  state += coefficient * (target - state);
}

// One-pole step with separate rise and fall speeds, e.g. an envelope
// follower with fast attack and slow release. Replaces stmlib's SLOPE.
inline void slope(float& state, float target, float rise, float fall) {
  float error = target - state;
  state += (error > 0.0f ? rise : fall) * error;
}

// Move `state` towards `target` by at most `maxDelta` per call: a
// fixed-rate slew limiter. Replaces stmlib's SLEW.
inline void slew(float& state, float target, float maxDelta) {
  state += clamp(target - state, -maxDelta, maxDelta);
}

// Convert a one-pole coefficient tuned for one sample rate to another.
//
// Much of Gillet's code applies a fixed coefficient c once per sample at
// the firmware's rate. Run at a different rate, the same c would make the
// filter or envelope faster or slower. After n samples a one-pole has
// covered 1 - (1 - c)^n of the distance to its target, so matching the
// time in seconds means raising (1 - c) to the power
// ratio = originalRate / newRate.
inline float rescaleCoefficient(float c, float ratio) {
  if (ratio == 1.0f) return c;  // exact at the original rate
  return 1.0f - std::pow(1.0f - c, ratio);
}

}  // namespace pt

#endif  // PT_CORE_MATH_H_
