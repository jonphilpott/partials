// partials — spectral/fm_operators.h
//
// FmOperators: a 4- or 6-operator FM (phase modulation) voice in the
// style of Yamaha's DX synths, with all 8 four-operator (DX100) or 32
// six-operator (DX7) algorithms. There are no patches or envelopes: you
// set each operator's frequency ratio and level (drive the levels from
// your own envelopes), and pick an algorithm. From Plaits' FM engine.
//
// How it works
// - Each operator is a sine oscillator whose phase can be pushed around
//   by another operator's output ("phase modulation", which is what the
//   DX synths call FM). A modulator's level sets how far: it is the
//   modulation index, i.e. how many harmonics appear.
// - An algorithm says, for each operator, where its modulation comes from
//   (nothing, feedback, or a bus written by earlier operators) and where
//   its output goes (a modulation bus or the audio output). Operators run
//   from the highest-numbered down to operator 1, all within one sample.
// - Feedback: one operator (marked in the algorithm) can modulate
//   itself, or an operator earlier in the chain, with its last two
//   outputs averaged, as on the DX7, from smooth to noisy.
//
// Operators are numbered as on the DX synths: operator 1 is usually a
// carrier (heard), higher numbers usually modulators.
//
// Derived from Plaits, Copyright 2021 Emilie Gillet. MIT licence.

#ifndef PT_SPECTRAL_FM_OPERATORS_H_
#define PT_SPECTRAL_FM_OPERATORS_H_

#include <cmath>
#include <cstdint>

#include "pt/core/math.h"
#include "pt/core/tables.h"

namespace pt {

class FmOperators {
 public:
  static const int kMaxOperators = 6;

  // numOperators: 4 (DX100-style, 8 algorithms) or 6 (DX7-style, 32).
  void init(float sampleRate, int numOperators = 6) {
    sampleTime_ = 1.0f / sampleRate;
    numOperators_ = numOperators == 4 ? 4 : 6;
    for (int i = 0; i < kMaxOperators; ++i) {
      phase_[i] = 0;
      ratio_[i] = 1.0f;
      fixedHz_[i] = 0.0f;
      level_[i] = 0.0f;
    }
    feedback_[0] = feedback_[1] = 0.0f;
    algorithm_ = 0;
    sineTable();
    update();
  }

  // Algorithm number, 1-based as printed on the synths: 1..32 (6 operators)
  // or 1..8 (4 operators).
  void setAlgorithm(int n) {
    int count = numOperators_ == 4 ? 8 : 32;
    algorithm_ = n < 1 ? 0 : (n > count ? count - 1 : n - 1);
  }
  // Base pitch in Hz.
  void setFrequency(float hz) { hz_ = hz; update(); }
  // Operator `op` (1..N) runs at base pitch x ratio (1, 2, 0.5, 3.5...).
  void setRatio(int op, float ratio) {
    if (!valid(op)) return;
    ratio_[index(op)] = ratio;
    fixedHz_[index(op)] = 0.0f;
    update();
  }
  // Or at a fixed frequency, ignoring the pitch (for bells, noises).
  void setFixedFrequency(int op, float hz) {
    if (!valid(op)) return;
    fixedHz_[index(op)] = hz;
    update();
  }
  // Operator output level, 0..1 for carriers (loudness); for modulators,
  // 0..4 is the modulation depth (about 1 gives a bright, brassy tone).
  void setLevel(int op, float level) {
    if (valid(op)) level_[index(op)] = std::fmin(level, 4.0f);
  }
  // 0..1. Feedback amount (the DX7's 0..7).
  void setFeedback(float x) {
    float fb = clamp(x, 0.0f, 1.0f) * 7.0f;
    feedbackScale_ = fb > 0.0f ? std::exp2(fb) / 512.0f : 0.0f;
  }
  // Restart all operators at phase 0 (on note-on, for a consistent attack).
  void reset() {
    for (uint32_t& p : phase_) p = 0;
  }

  float process() {
    const uint8_t* opcodes = numOperators_ == 4 ? algorithms4(algorithm_) : algorithms6(algorithm_);
    // Buses: 0 = audio output, 1 and 2 = modulation.
    float bus[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < numOperators_; ++i) {
      const uint8_t op = opcodes[i];
      // 1. Phase modulation input.
      int source = (op & kSourceMask) >> 4;
      float pm = 0.0f;
      if (source == 3) {
        pm = (feedback_[0] + feedback_[1]) * feedbackScale_;
      } else if (source) {
        pm = bus[source];
      }
      // 2. The operator: a sine of (phase + pm), pm in cycles.
      phase_[i] += increment_[i];
      float out = sinePM(phase_[i], pm) * level_[i];
      // 3. Remember the feedback source's last two outputs.
      if (op & kFeedbackSourceFlag) {
        feedback_[1] = feedback_[0];
        feedback_[0] = out;
      }
      // 4. Write (or add) to the destination bus.
      int destination = op & kDestinationMask;
      bus[destination] = (op & kAdditiveFlag) ? bus[destination] + out : out;
    }
    return bus[0];
  }

 private:
  // Opcode layout (Plaits' algorithms.cc): bits 0-1 destination bus, bit 2
  // add instead of replace, bits 4-5 modulation source (0 none, 1-2 a bus,
  // 3 feedback), bit 6 this operator feeds the feedback path.
  static const uint8_t kDestinationMask = 0x03;
  static const uint8_t kAdditiveFlag = 0x04;
  static const uint8_t kSourceMask = 0x30;
  static const uint8_t kFeedbackSourceFlag = 0x40;

  bool valid(int op) const { return op >= 1 && op <= numOperators_; }
  // Opcodes run from the highest operator down: operator N is index 0.
  int index(int op) const { return numOperators_ - op; }

  void update() {
    for (int i = 0; i < numOperators_; ++i) {
      float hz = fixedHz_[i] > 0.0f ? fixedHz_[i] : hz_ * ratio_[i];
      float f = std::fmin(hz * sampleTime_, 0.5f);
      increment_[i] = static_cast<uint32_t>(f * 4294967296.0f);
    }
  }

  // Sine of a 32-bit phase shifted by pm cycles (up to +-32), read from
  // the shared table (Plaits' SinePM).
  static float sinePM(uint32_t phase, float pm) {
    const float scale = 4294967296.0f / 64.0f;
    phase += static_cast<uint32_t>((pm + 32.0f) * scale) * 64u;
    const float* t = sineTable();
    uint32_t i = phase >> (32 - 9);
    float frac = static_cast<float>(phase << 9) / 4294967296.0f;
    return t[i] + (t[i + 1] - t[i]) * frac;
  }

  // The algorithms, operator N first. Generated from Plaits'
  // algorithms.cc. (Function-local statics keep the header safe to include
  // from several .cpp files.)
  static const uint8_t* algorithms4(int a) {
    static const uint8_t kTable[8][4] = {
        {0x71, 0x11, 0x11, 0x14},  // Algorithm 1: 4 -> 3 -> 2 -> 1
        {0x71, 0x05, 0x11, 0x14},  // Algorithm 2: 4 + 3 -> 2 -> 1
        {0x71, 0x02, 0x25, 0x14},  // Algorithm 3: 4 + (3 -> 2) -> 1
        {0x71, 0x11, 0x05, 0x14},  // Algorithm 4: (4 -> 3) + 2 -> 1
        {0x71, 0x14, 0x01, 0x14},  // Algorithm 5: (4 -> 3) + (2 -> 1)
        {0x71, 0x14, 0x14, 0x14},  // Algorithm 6: (4 -> 3) + (4 -> 2) + (4 -> 1)
        {0x71, 0x14, 0x04, 0x04},  // Algorithm 7: (4 -> 3) + 2 + 1
        {0x74, 0x04, 0x04, 0x04},  // Algorithm 8: 4 + 3 + 2 + 1
    };
    return kTable[a];
  }
  static const uint8_t* algorithms6(int a) {
    static const uint8_t kTable[32][6] = {
        {0x71, 0x11, 0x11, 0x14, 0x01, 0x14},  // Algorithm 1
        {0x01, 0x11, 0x11, 0x14, 0x71, 0x14},  // Algorithm 2
        {0x71, 0x11, 0x14, 0x01, 0x11, 0x14},  // Algorithm 3
        {0x31, 0x11, 0x54, 0x01, 0x11, 0x14},  // Algorithm 4
        {0x71, 0x14, 0x01, 0x14, 0x01, 0x14},  // Algorithm 5
        {0x31, 0x54, 0x01, 0x14, 0x01, 0x14},  // Algorithm 6
        {0x71, 0x11, 0x05, 0x14, 0x01, 0x14},  // Algorithm 7
        {0x01, 0x11, 0x75, 0x14, 0x01, 0x14},  // Algorithm 8
        {0x01, 0x11, 0x05, 0x14, 0x71, 0x14},  // Algorithm 9
        {0x01, 0x05, 0x14, 0x71, 0x11, 0x14},  // Algorithm 10
        {0x71, 0x05, 0x14, 0x01, 0x11, 0x14},  // Algorithm 11
        {0x01, 0x05, 0x05, 0x14, 0x71, 0x14},  // Algorithm 12
        {0x71, 0x05, 0x05, 0x14, 0x01, 0x14},  // Algorithm 13
        {0x71, 0x05, 0x11, 0x14, 0x01, 0x14},  // Algorithm 14
        {0x01, 0x05, 0x11, 0x14, 0x71, 0x14},  // Algorithm 15
        {0x71, 0x11, 0x02, 0x25, 0x05, 0x14},  // Algorithm 16
        {0x01, 0x11, 0x02, 0x25, 0x75, 0x14},  // Algorithm 17
        {0x01, 0x11, 0x11, 0x75, 0x05, 0x14},  // Algorithm 18
        {0x71, 0x14, 0x14, 0x01, 0x11, 0x14},  // Algorithm 19
        {0x01, 0x05, 0x14, 0x71, 0x14, 0x14},  // Algorithm 20
        {0x01, 0x14, 0x14, 0x71, 0x14, 0x14},  // Algorithm 21
        {0x71, 0x14, 0x14, 0x14, 0x01, 0x14},  // Algorithm 22
        {0x71, 0x14, 0x14, 0x01, 0x14, 0x04},  // Algorithm 23
        {0x71, 0x14, 0x14, 0x14, 0x04, 0x04},  // Algorithm 24
        {0x71, 0x14, 0x14, 0x04, 0x04, 0x04},  // Algorithm 25
        {0x71, 0x05, 0x14, 0x01, 0x14, 0x04},  // Algorithm 26
        {0x01, 0x05, 0x14, 0x71, 0x14, 0x04},  // Algorithm 27
        {0x04, 0x71, 0x11, 0x14, 0x01, 0x14},  // Algorithm 28
        {0x71, 0x14, 0x01, 0x14, 0x04, 0x04},  // Algorithm 29
        {0x04, 0x71, 0x11, 0x14, 0x04, 0x04},  // Algorithm 30
        {0x71, 0x14, 0x04, 0x04, 0x04, 0x04},  // Algorithm 31
        {0x74, 0x04, 0x04, 0x04, 0x04, 0x04},  // Algorithm 32
    };
    return kTable[a];
  }

  float sampleTime_ = 1.0f / 48000.0f;
  int numOperators_ = 6;
  int algorithm_ = 0;
  float hz_ = 220.0f;
  float ratio_[kMaxOperators] = {}, fixedHz_[kMaxOperators] = {}, level_[kMaxOperators] = {};
  uint32_t increment_[kMaxOperators] = {}, phase_[kMaxOperators] = {};
  float feedback_[2] = {0.0f, 0.0f};
  float feedbackScale_ = 0.0f;
};

}  // namespace pt

#endif  // PT_SPECTRAL_FM_OPERATORS_H_
