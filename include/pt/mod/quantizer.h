// partials — mod/quantizer.h
//
// Quantizer: snaps a pitch CV to a scale, with a control that gradually
// thins the scale down to its most important notes. Marbles' quantizer.
//
// How it works
// - A scale is a list of degrees (pitches within one octave, in volts),
//   each with a weight 0..255: how important it is. Marbles' C major gives
//   C 255, G 192, E and B 128, D and A 96, F 64, and the other notes less.
// - `amount` (0..1) picks a threshold. At 0 nothing is quantised. Turning
//   it up quantises to every note (chromatic), then drops the
//   least-important notes in turn: the major scale, then a pentatonic-like
//   subset, then the triad, then root and fifth, and finally the root only.
// - The input snaps to the nearest allowed note. Optional hysteresis stops
//   it flickering between two notes when the input sits halfway.
//
// Derived from Marbles, Copyright 2015 Emilie Gillet. MIT licence.

#ifndef PT_MOD_QUANTIZER_H_
#define PT_MOD_QUANTIZER_H_

#include <cstdint>

#include "pt/mod/hysteresis_quantizer.h"

namespace pt {

class Quantizer {
 public:
  static const int kMaxDegrees = 16;

  struct Degree {
    float voltage;   // within the base interval, e.g. 0.5833 = 7 semitones
    uint8_t weight;  // 0..255 importance
  };

  struct Scale {
    float baseInterval;  // usually 1 V (an octave)
    int numDegrees;
    Degree degree[kMaxDegrees];
  };

  // Marbles' six preset scales.
  enum Preset { MAJOR, MINOR, PENTATONIC, PELOG, RAAG_BHAIRAV, RAAG_SHRI };

  static const Scale& preset(Preset p) {
    static const Scale kScales[6] = {
        // C major
        {1.0f, 12, {{0.0000f, 255}, {0.0833f, 16}, {0.1667f, 96}, {0.2500f, 24}, {0.3333f, 128}, {0.4167f, 64},
                    {0.5000f, 8}, {0.5833f, 192}, {0.6667f, 16}, {0.7500f, 96}, {0.8333f, 24}, {0.9167f, 128}}},
        // C minor
        {1.0f, 12, {{0.0000f, 255}, {0.0833f, 16}, {0.1667f, 96}, {0.2500f, 128}, {0.3333f, 8}, {0.4167f, 64},
                    {0.5000f, 4}, {0.5833f, 192}, {0.6667f, 96}, {0.7500f, 16}, {0.8333f, 128}, {0.9167f, 16}}},
        // Pentatonic
        {1.0f, 12, {{0.0000f, 255}, {0.0833f, 4}, {0.1667f, 96}, {0.2500f, 4}, {0.3333f, 4}, {0.4167f, 140},
                    {0.5000f, 4}, {0.5833f, 192}, {0.6667f, 4}, {0.7500f, 96}, {0.8333f, 4}, {0.9167f, 4}}},
        // Pelog (7 notes, not 12-tone)
        {1.0f, 7, {{0.0000f, 255}, {0.1275f, 128}, {0.2625f, 32}, {0.4600f, 8}, {0.5883f, 192}, {0.7067f, 64},
                   {0.8817f, 16}}},
        // Raag Bhairav That
        {1.0f, 12, {{0.0000f, 255}, {0.0752f, 128}, {0.1699f, 4}, {0.2630f, 4}, {0.3219f, 128}, {0.4150f, 64},
                    {0.4918f, 4}, {0.5850f, 192}, {0.6601f, 64}, {0.7549f, 4}, {0.8479f, 4}, {0.9069f, 64}}},
        // Raag Shri
        {1.0f, 12, {{0.0000f, 255}, {0.0752f, 4}, {0.1699f, 128}, {0.2630f, 64}, {0.3219f, 4}, {0.4150f, 128},
                    {0.4918f, 4}, {0.5850f, 192}, {0.6601f, 4}, {0.7549f, 64}, {0.8479f, 128}, {0.9069f, 4}}},
    };
    return kScales[p];
  }

  void init(const Scale& scale = preset(MAJOR)) {
    int n = scale.numDegrees;
    if (n < 1 || n > kMaxDegrees || scale.baseInterval == 0.0f) return;
    numDegrees_ = n;
    baseInterval_ = scale.baseInterval;

    // 1. Weight thresholds. If the second most important note is very
    // important (> 192), raise the second-to-last level so it keeps only
    // the top two notes.
    uint8_t secondLargest = 0;
    for (int i = 0; i < n; ++i) {
      voltage_[i] = scale.degree[i].voltage;
      if (scale.degree[i].weight != 255 && scale.degree[i].weight >= secondLargest) {
        secondLargest = scale.degree[i].weight;
      }
    }
    uint8_t thresholds[kNumThresholds] = {0, 16, 32, 64, 128, 192, 255};
    if (secondLargest > 192) thresholds[kNumThresholds - 2] = secondLargest;

    // 2. For each level, which degrees pass (a bitmask), and the first
    // and last of them.
    for (int t = 0; t < kNumThresholds; ++t) {
      uint16_t mask = 0;
      int first = -1, last = 0;
      for (int i = 0; i < n; ++i) {
        if (scale.degree[i].weight >= thresholds[t]) {
          mask |= 1 << i;
          if (first < 0) first = i;
          last = i;
        }
      }
      level_[t].bitmask = mask;
      level_[t].first = first < 0 ? 0 : first;
      level_[t].last = last;
      feedback_[t] = 0.0f;
    }
    levelQuantizer_.init(kNumThresholds + 1, 0.25f, true);
  }

  // value: pitch in volts. amount: 0 = off, then progressively fewer notes.
  // hysteresis: avoid flickering between two notes.
  float process(float value, float amount, bool hysteresis = true) {
    int level = levelQuantizer_.process(amount);
    if (level == 0) return value;
    level -= 1;
    float raw = value;
    if (hysteresis) value += feedback_[level];

    // 1. Split into octave (note) and position within it.
    float note = value / baseInterval_;
    int octave = static_cast<int>(note);
    float within = note - static_cast<float>(octave);
    if (value < 0.0f) {
      octave -= 1;
      within += 1.0f;
    }
    within *= baseInterval_;

    // 2. Find the allowed notes just below and above (the ends wrap to the
    // neighbouring octaves).
    const Level& l = level_[level];
    float a = voltage_[l.last] - baseInterval_;
    float b = voltage_[l.first] + baseInterval_;
    uint16_t mask = l.bitmask;
    for (int i = 0; i < numDegrees_; ++i) {
      if (mask & 1) {
        if (within > voltage_[i]) {
          a = voltage_[i];
        } else {
          b = voltage_[i];
          break;
        }
      }
      mask >>= 1;
    }

    // 3. Nearest, back in absolute volts. The feedback biases the next
    // decision towards the current note (hysteresis).
    float quantized = (within < (a + b) * 0.5f ? a : b) + static_cast<float>(octave) * baseInterval_;
    feedback_[level] = (quantized - raw) * 0.25f;
    return quantized;
  }

 private:
  static const int kNumThresholds = 7;
  struct Level {
    uint16_t bitmask;
    int first, last;
  };
  float voltage_[kMaxDegrees] = {};
  Level level_[kNumThresholds] = {};
  float feedback_[kNumThresholds] = {};
  float baseInterval_ = 1.0f;
  int numDegrees_ = 1;
  HysteresisQuantizer levelQuantizer_;
};

}  // namespace pt

#endif  // PT_MOD_QUANTIZER_H_
