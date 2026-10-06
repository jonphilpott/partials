// partials — mod/envelope.h
//
// Envelope: a multi-segment envelope generator, with ADSR, AD, AR and
// looping presets, or fully custom segments. Peaks' envelope mode.
//
// How it works
// - An envelope is a list of segments. Segment i ramps from level[i] to
//   level[i+1] over time[i] seconds, following a curve shape: linear,
//   exponential (fast then slow) or quartic (slow then fast).
// - A rising gate restarts at segment 0, from the current level (so a
//   retrigger doesn't click) unless hard reset is on.
// - The sustain point is a segment the envelope holds at while the gate
//   stays high; a falling gate jumps to it ("release").
// - A loop (start..end) makes the envelope cycle through those segments,
//   like an LFO with an arbitrary shape.
//
// Original: Peaks, 16-bit fixed point with lookup tables at 48 kHz. This
// port is float with times in seconds, so it runs at any sample rate.
//
// Derived from Peaks, Copyright 2013 Emilie Gillet. MIT licence.

#ifndef PT_MOD_ENVELOPE_H_
#define PT_MOD_ENVELOPE_H_

#include <cmath>

#include "pt/core/math.h"

namespace pt {

class Envelope {
 public:
  enum Shape { LINEAR, EXPONENTIAL, QUARTIC };
  static const int kMaxSegments = 6;

  void init(float sampleRate) {
    sampleTime_ = 1.0f / sampleRate;
    setAdsr(0.002f, 0.1f, 0.5f, 0.3f);
    segment_ = numSegments_;  // idle
    phase_ = 0.0f;
    increment_ = 0.0f;
    startValue_ = value_ = 0.0f;
    previousGate_ = false;
  }

  // --- Presets (times in seconds, levels 0..1) --------------------------
  // Peaks uses a quartic attack and exponential decay/release for ADSR,
  // exponential AD, and linear shapes for the others.

  void setAdsr(float attack, float decay, float sustain, float release) {
    set3(attack, decay, sustain, release, 2, QUARTIC, EXPONENTIAL, EXPONENTIAL, 0, 0);
  }
  // Attack-decay: ignores the gate length.
  void setAd(float attack, float decay) { set2(attack, decay, 0, EXPONENTIAL, 0); }
  // Attack-release: holds at full level while the gate is high.
  void setAr(float attack, float release) { set2(attack, release, 1, LINEAR, 0); }
  // Attack, decay to sustain level, then release, ignoring the gate length.
  void setAdr(float attack, float decay, float sustain, float release) {
    set3(attack, decay, sustain, release, 0, LINEAR, LINEAR, LINEAR, 0, 0);
  }
  // Looping versions: cycle while running, like an LFO.
  void setAdLoop(float attack, float decay) { set2(attack, decay, 0, LINEAR, 2); }
  void setAdrLoop(float attack, float decay, float sustain, float release) {
    set3(attack, decay, sustain, release, 0, LINEAR, LINEAR, LINEAR, 0, 3);
  }

  // --- Custom segments ---------------------------------------------------

  // Segment i starts at level(i) and ends at level(i + 1), so a 3-segment
  // envelope uses levels 0..3.
  void setLevel(int i, float level) { level_[i] = level; }
  void setTime(int i, float seconds) { time_[i] = seconds; }
  void setShape(int i, Shape shape) { shape_[i] = shape; }
  void setNumSegments(int n) { numSegments_ = n; }
  // Hold at the start of this segment while the gate is high (0 = none).
  void setSustainPoint(int segment) { sustainPoint_ = segment; }
  // Loop from segment `start` when segment `end` is reached (end = 0: off).
  void setLoop(int start, int end) {
    loopStart_ = start;
    loopEnd_ = end;
  }
  // On: every trigger restarts from level 0. Off: from the current level.
  void setHardReset(bool on) { hardReset_ = on; }

  // gate: true while the key/gate is held. Returns the envelope, 0..1.
  float process(bool gate) {
    bool rising = gate && !previousGate_;
    bool falling = !gate && previousGate_;
    previousGate_ = gate;

    // 1. Events: trigger, release, or the end of a segment.
    if (rising) {
      startValue_ = (segment_ == numSegments_ || hardReset_) ? level_[0] : value_;
      segment_ = 0;
      phase_ = 0.0f;
    } else if (falling && sustainPoint_) {
      startValue_ = value_;
      segment_ = sustainPoint_;
      phase_ = 0.0f;
    } else if (phase_ >= 1.0f) {
      startValue_ = level_[segment_ + 1];
      ++segment_;
      phase_ = 0.0f;
      if (segment_ == loopEnd_) segment_ = loopStart_;
    }

    // 2. Speed: zero when finished or sustaining.
    bool done = segment_ == numSegments_;
    bool sustained = sustainPoint_ && segment_ == sustainPoint_ && gate;
    increment_ = sustained || done ? 0.0f : sampleTime_ / std::fmax(time_[segment_], 1e-5f);

    // 3. Value along the segment's curve, then advance.
    float t = curve(shape_[segment_], std::fmin(phase_, 1.0f));
    value_ = startValue_ + (level_[segment_ + 1] - startValue_) * t;
    phase_ += increment_;
    return value_;
  }

  float value() const { return value_; }
  // True while the envelope is running (not finished).
  bool active() const { return segment_ < numSegments_; }

 private:
  // Peaks' curves on 0..1: linear; exponential 1 - e^(-4x) (normalised),
  // fast then slow; quartic x^3.32, slow then fast.
  static float curve(Shape shape, float x) {
    switch (shape) {
      case EXPONENTIAL: {
        static const float kNorm = 1.0f / (1.0f - std::exp(-4.0f));
        return (1.0f - std::exp(-4.0f * x)) * kNorm;
      }
      case QUARTIC:
        return std::pow(x, 3.32f);
      case LINEAR:
      default:
        return x;
    }
  }

  void set2(float a, float d, int sustainPoint, Shape shape, int loopEnd) {
    numSegments_ = 2;
    sustainPoint_ = sustainPoint;
    level_[0] = 0.0f; level_[1] = 1.0f; level_[2] = 0.0f;
    time_[0] = a; time_[1] = d;
    shape_[0] = shape_[1] = shape;
    loopStart_ = 0;
    loopEnd_ = loopEnd;
  }

  void set3(float a, float d, float s, float r, int sustainPoint,
            Shape sa, Shape sd, Shape sr, int loopStart, int loopEnd) {
    numSegments_ = 3;
    sustainPoint_ = sustainPoint;
    level_[0] = 0.0f; level_[1] = 1.0f; level_[2] = s; level_[3] = 0.0f;
    time_[0] = a; time_[1] = d; time_[2] = r;
    shape_[0] = sa; shape_[1] = sd; shape_[2] = sr;
    loopStart_ = loopStart;
    loopEnd_ = loopEnd;
  }

  float sampleTime_ = 1.0f / 48000.0f;
  float level_[kMaxSegments + 1] = {0, 0, 0, 0, 0, 0, 0};
  float time_[kMaxSegments] = {0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f};
  Shape shape_[kMaxSegments] = {LINEAR, LINEAR, LINEAR, LINEAR, LINEAR, LINEAR};
  int numSegments_ = 3, sustainPoint_ = 2, loopStart_ = 0, loopEnd_ = 0;
  bool hardReset_ = false;

  int segment_ = 3;
  float phase_ = 0.0f, increment_ = 0.0f;
  float startValue_ = 0.0f, value_ = 0.0f;
  bool previousGate_ = false;
};

}  // namespace pt

#endif  // PT_MOD_ENVELOPE_H_
