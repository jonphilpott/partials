// partials — mod/random_sequence.h
//
// RandomSequence: random values that can loop. Marbles' "déjà vu" control:
// at 0 every value is new; turning it up makes the sequence repeat a loop
// of recent values, more and more faithfully; at 0.5 it locks to the loop;
// past 0.5 it starts jumping randomly around inside the loop.
//
// How it works
// - A loop buffer holds the last `length` values.
// - At each step, a random "mutation" happens with probability
//   (2*dejaVu - 1)^2: high at both ends, zero in the middle.
//   - Below 0.5, a mutation replaces the loop's next value with a fresh
//     random one, so the loop slowly evolves.
//   - Above 0.5, a mutation jumps to a random position in the loop.
//   - Without a mutation, the sequence steps through the loop in order.
// - next(input) instead records an external value into the loop (like a
//   looping sample-and-hold, or a shift register).
//
// Derived from Marbles, Copyright 2015 Emilie Gillet. MIT licence.

#ifndef PT_MOD_RANDOM_SEQUENCE_H_
#define PT_MOD_RANDOM_SEQUENCE_H_

#include "pt/core/random.h"

namespace pt {

class RandomSequence {
 public:
  static const int kMaxLength = 16;

  void init(uint32_t seed = 0x21) {
    random_.seed(seed);
    for (int i = 0; i < kMaxLength; ++i) loop_[i] = random_.uniform();
    writeHead_ = 0;
    length_ = 8;
    step_ = 0;
  }

  // 0..1. 0 = always new, 0.5 = locked loop, 1 = random jumps in the loop.
  void setDejaVu(float x) { dejaVu_ = x; }
  // 1..16. Loop length in steps.
  void setLength(int n) {
    if (n < 1 || n > kMaxLength) return;
    length_ = n;
    step_ %= n;
  }
  // Restart the loop from its first step.
  void reset() { step_ = length_ - 1; }

  // Next random value, 0..1.
  float next() { return advance(false, 0.0f); }

  // Next value, recording `input` (0..1) into the loop when it mutates
  // instead of a random value: a shift register that repeats what it was
  // fed.
  float next(float input) { return advance(true, input); }

 private:
  float advance(bool external, float input) {
    const float pSqrt = 2.0f * dejaVu_ - 1.0f;
    const bool mutate = random_.uniform() < pSqrt * pSqrt;
    if (mutate && dejaVu_ <= 0.5f) {
      // New value at the end of the loop.
      loop_[writeHead_] = external ? input : random_.uniform();
      writeHead_ = (writeHead_ + 1) % kMaxLength;
      step_ = length_ - 1;
    } else if (mutate) {
      // Jump somewhere in the loop.
      step_ = static_cast<int>(random_.uniform() * static_cast<float>(length_));
    } else {
      step_ = (step_ + 1) % length_;
    }
    return loop_[(writeHead_ + kMaxLength - length_ + step_) % kMaxLength];
  }

  Random random_;
  float loop_[kMaxLength] = {};
  int writeHead_ = 0, length_ = 8, step_ = 0;
  float dejaVu_ = 0.0f;
};

}  // namespace pt

#endif  // PT_MOD_RANDOM_SEQUENCE_H_
