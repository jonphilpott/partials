// mutablelib — core/random.h
//
// Fast pseudo-random numbers for noise and randomised behaviour.
//
// This is a "linear congruential generator" (LCG): the next state is
// state * a + c, letting the 32-bit integer overflow wrap around. It is
// very cheap and statistically fine for audio noise, though not for
// cryptography.
//
// stmlib's version kept one global state shared by every module. Here each
// component owns its own Random, so instances don't disturb each other and
// a seeded instance always produces the same sequence (useful in tests).
//
// Derived from stmlib, Copyright 2012 Emilie Gillet. MIT licence.

#ifndef ML_CORE_RANDOM_H_
#define ML_CORE_RANDOM_H_

#include <cstdint>

namespace ml {

struct Random {
  uint32_t state = 0x21;

  void seed(uint32_t s) { state = s; }

  // Next raw 32-bit value. The constants are the classic ones from
  // Numerical Recipes, the same as stmlib's.
  uint32_t word() {
    state = state * 1664525u + 1013904223u;
    return state;
  }

  // Uniform float in [0, 1).
  float uniform() { return static_cast<float>(word()) / 4294967296.0f; }

  // Uniform float in [-1, 1): white noise.
  float bipolar() { return uniform() * 2.0f - 1.0f; }
};

}  // namespace ml

#endif  // ML_CORE_RANDOM_H_
