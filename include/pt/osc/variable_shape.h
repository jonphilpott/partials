// partials — osc/variable_shape.h
//
// VariableShapeOscillator: one alias-free oscillator that morphs smoothly
// from triangle to saw to square, with variable pulse width and optional
// hard sync. Plaits' virtual-analog workhorse.
//
// How it works
// - The naive waveform is a blend of three shapes computed from the same
//   phase: saw, square and a triangle whose peak sits at the pulse width.
//   `shape` 0 is all triangle, 0.5 all saw, 1 all square.
// - Each corner (triangle) and jump (saw, square) gets a PolyBLEP
//   correction, weighted by how much of that shape is in the blend, so
//   every in-between shape is also alias-free.
// - Hard sync: a second, "master" phase resets this oscillator each cycle.
//   The jump at the reset is corrected too, so sync sweeps are clean.
//
// Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.

#ifndef PT_OSC_VARIABLE_SHAPE_H_
#define PT_OSC_VARIABLE_SHAPE_H_

#include <cmath>

#include "pt/core/math.h"
#include "pt/osc/polyblep.h"

namespace pt {

class VariableShapeOscillator {
 public:
  void init(float sampleRate) {
    sampleTime_ = 1.0f / sampleRate;
    masterPhase_ = slavePhase_ = 0.0f;
    nextSample_ = 0.0f;
    previousPw_ = 0.5f;
    high_ = false;
  }

  // Pitch in Hz. With sync on, this is the synced (slave) pitch: sweep it
  // for the classic sync sound.
  void setFrequency(float hz) { frequency_ = hz * sampleTime_; }
  // Hard sync: the oscillator restarts at this rate (Hz). 0 turns sync off.
  void setSyncFrequency(float hz) { masterFrequency_ = hz * sampleTime_; }
  // 0..1. Pulse width for the square, peak position for the triangle.
  void setPulseWidth(float pw) { pw_ = pw; }
  // 0..1. 0 = triangle, 0.5 = saw, 1 = square, morphing in between.
  void setShape(float x) { shape_ = clamp(x, 0.0f, 1.0f); }

  // Returns about ±1, one sample late (BLEP look-ahead).
  float process() {
    const bool sync = masterFrequency_ > 0.0f;
    const float masterFrequency = std::fmin(masterFrequency_, 0.25f);
    const float frequency = std::fmin(frequency_, 0.25f);
    float pw = frequency >= 0.25f ? 0.5f : clamp(pw_, frequency * 2.0f, 1.0f - 2.0f * frequency);

    bool reset = false;
    bool transitionDuringReset = false;
    float resetTime = 0.0f;
    float thisSample = nextSample_;
    float nextSample = 0.0f;

    const float squareAmount = std::fmax(shape_ - 0.5f, 0.0f) * 2.0f;
    const float triangleAmount = std::fmax(1.0f - shape_ * 2.0f, 0.0f);
    const float slopeUp = 1.0f / pw;
    const float slopeDown = 1.0f / (1.0f - pw);

    // 1. Sync: when the master wraps, note where the slave was and correct
    // for the jump back to the start.
    if (sync) {
      masterPhase_ += masterFrequency;
      if (masterPhase_ >= 1.0f) {
        masterPhase_ -= 1.0f;
        resetTime = masterPhase_ / masterFrequency;
        float slavePhaseAtReset = slavePhase_ + (1.0f - resetTime) * frequency;
        reset = true;
        if (slavePhaseAtReset >= 1.0f) {
          slavePhaseAtReset -= 1.0f;
          transitionDuringReset = true;
        }
        if (!high_ && slavePhaseAtReset >= pw) transitionDuringReset = true;
        float value = naiveSample(slavePhaseAtReset, pw, slopeUp, slopeDown, triangleAmount, squareAmount);
        thisSample -= value * thisBlepSample(resetTime);
        nextSample -= value * nextBlepSample(resetTime);
      }
    }

    // 2. Advance, correcting the edge at the pulse width and the reset at
    // the end of the cycle (with weights for the square and triangle
    // parts).
    slavePhase_ += frequency;
    while (transitionDuringReset || !reset) {
      if (!high_) {
        if (slavePhase_ < pw) break;
        float t = (slavePhase_ - pw) / (previousPw_ - pw + frequency);
        float triangleStep = (slopeUp + slopeDown) * frequency * triangleAmount;
        thisSample += squareAmount * thisBlepSample(t);
        nextSample += squareAmount * nextBlepSample(t);
        thisSample -= triangleStep * thisIntegratedBlepSample(t);
        nextSample -= triangleStep * nextIntegratedBlepSample(t);
        high_ = true;
      }
      if (high_) {
        if (slavePhase_ < 1.0f) break;
        slavePhase_ -= 1.0f;
        float t = slavePhase_ / frequency;
        float triangleStep = (slopeUp + slopeDown) * frequency * triangleAmount;
        thisSample -= (1.0f - triangleAmount) * thisBlepSample(t);
        nextSample -= (1.0f - triangleAmount) * nextBlepSample(t);
        thisSample += triangleStep * thisIntegratedBlepSample(t);
        nextSample += triangleStep * nextIntegratedBlepSample(t);
        high_ = false;
      }
    }
    if (sync && reset) {
      slavePhase_ = resetTime * frequency;
      high_ = false;
    }

    nextSample += naiveSample(slavePhase_, pw, slopeUp, slopeDown, triangleAmount, squareAmount);
    previousPw_ = pw;
    nextSample_ = nextSample;
    return 2.0f * thisSample - 1.0f;
  }

 private:
  // The uncorrected blend of saw, square and triangle at this phase (0..1).
  static float naiveSample(float phase, float pw, float slopeUp, float slopeDown,
                           float triangleAmount, float squareAmount) {
    float saw = phase;
    float square = phase < pw ? 0.0f : 1.0f;
    float triangle = phase < pw ? phase * slopeUp : 1.0f - (phase - pw) * slopeDown;
    saw += (square - saw) * squareAmount;
    saw += (triangle - saw) * triangleAmount;
    return saw;
  }

  float sampleTime_ = 1.0f / 48000.0f;
  float frequency_ = 0.01f, masterFrequency_ = 0.0f, pw_ = 0.5f, shape_ = 0.5f;
  float masterPhase_ = 0.0f, slavePhase_ = 0.0f, nextSample_ = 0.0f, previousPw_ = 0.5f;
  bool high_ = false;
};

}  // namespace pt

#endif  // PT_OSC_VARIABLE_SHAPE_H_
