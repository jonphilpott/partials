// partials — osc/polyblep.h
//
// PolyBLEP correction: the standard trick for removing aliasing from
// signals with sudden jumps (saw and square waves, hard sync, sample-and-
// hold, bit reduction).
//
// Why: a jump that happens *between* two samples can't be represented
// exactly, so it is rounded to the nearest sample. That rounding error
// contains frequencies above Nyquist, which fold back down as inharmonic
// aliasing (the "digital" whine of naive oscillators).
//
// How: we know where the jump really happened (a fraction t of a sample
// ago) and how big it was. A band-limited jump differs from a hard one only
// near the discontinuity, so we add a small two-sample correction: one
// part to this sample, one to the next. "BLEP" = band-limited step. The
// polynomial versions here are cheap approximations of that correction.
//
// Usage pattern (t in 0..1 = how far past the jump this sample is, from 0
// "the jump is exactly now" to 1 "it happened a whole sample ago"):
//   thisSample += jump * thisBlepSample(t);
//   nextSample += jump * nextBlepSample(t);
// which means delaying the output by one sample.
//
// The "integrated" versions correct a jump in slope (a corner, as in a
// triangle wave) rather than in value.
//
// Derived from stmlib dsp/polyblep.h, Copyright 2017 Emilie Gillet.
// MIT licence.

#ifndef PT_OSC_POLYBLEP_H_
#define PT_OSC_POLYBLEP_H_

namespace pt {

inline float thisBlepSample(float t) { return 0.5f * t * t; }

inline float nextBlepSample(float t) {
  t = 1.0f - t;
  return -0.5f * t * t;
}

inline float nextIntegratedBlepSample(float t) {
  const float t1 = 0.5f * t;
  const float t2 = t1 * t1;
  const float t4 = t2 * t2;
  return 0.1875f - t1 + 1.5f * t2 - t4;
}

inline float thisIntegratedBlepSample(float t) {
  return nextIntegratedBlepSample(1.0f - t);
}

}  // namespace pt

#endif  // PT_OSC_POLYBLEP_H_
