// mutablelib — osc/basic.h
//
// BasicOscillator: Plaits' alias-free saw / square / triangle oscillator,
// with variable pulse width and a "slope" shape (a triangle whose rise and
// fall times can differ). Mostly plumbing for other components (the
// ring-mod hi-hat, wavefolder inputs), but a solid general oscillator too.
//
// How it works: a phase accumulator (0..1 per cycle) drives a naive
// waveform. Wherever the naive waveform jumps (saw reset, square edges) or
// bends sharply (triangle corners), a PolyBLEP correction is added at the
// exact fractional time of the event. That removes the aliasing a naive
// oscillator would have (see osc/polyblep.h). The corrections need one
// sample of look-ahead, so the output is one sample late.
//
// Changes from Plaits: SQUARE_DARK is scaled to about ±1 like the other
// shapes (it was ±1.5), and the start-up state is set from the shape,
// which fixes a spike on the first sample of SLOPE and IMPULSE_TRAIN.
//
// Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.

#ifndef ML_OSC_BASIC_H_
#define ML_OSC_BASIC_H_

#include <cmath>

#include "ml/core/math.h"
#include "ml/osc/polyblep.h"

namespace ml {

class BasicOscillator {
 public:
  enum Shape {
    SAW,
    TRIANGLE,
    SLOPE,            // triangle with variable symmetry (pulse width)
    SQUARE,           // variable pulse width
    SQUARE_BRIGHT,    // square through a high-pass: thin, nasal
    SQUARE_DARK,      // square through a low-pass: soft, hollow
    SQUARE_TRIANGLE,  // integrated square: a smooth triangle-like wave
    IMPULSE_TRAIN     // band-limited pulse train
  };

  void init(float sampleRate) {
    sampleTime_ = 1.0f / sampleRate;
    phase_ = 0.5f;  // Plaits' starting phase
    nextSample_ = 0.0f;
    lpState_ = 0.0f;
    hpState_ = 0.0f;
    needsSync_ = true;
  }

  void setShape(Shape s) {
    if (s != shape_) needsSync_ = true;
    shape_ = s;
  }
  void setFrequency(float hz) { frequency_ = hz * sampleTime_; }
  // 0..1. Pulse width for SQUARE shapes, symmetry for SLOPE.
  void setPulseWidth(float pw) { pw_ = pw; }

  float process() { return process(frequency_, pw_); }

  // Lower-level form: frequency as cycles per sample (Hz / sampleRate).
  float process(float frequency, float pw) {
    // 1. Limits: above a quarter of the sample rate the BLEP corrections
    // overlap and stop working; the pulse width must leave room for both
    // edges.
    frequency = clamp(frequency, 0.000001f, 0.25f);
    pw = clamp(pw, frequency * 2.0f, 1.0f - 2.0f * frequency);
    if (shape_ == TRIANGLE || shape_ == SQUARE_TRIANGLE) pw = 0.5f;

    // After init or a shape change, set the edge tracker to match the
    // current phase, so the first sample doesn't see a false edge. (Its
    // meaning differs: "past the pulse width" for squares, "before the
    // corner" for triangles.)
    if (needsSync_) {
      high_ = (shape_ == TRIANGLE || shape_ == SLOPE) ? phase_ < pw : phase_ >= pw;
      needsSync_ = false;
    }

    float thisSample = nextSample_;
    float nextSample = 0.0f;
    float out;
    phase_ += frequency;

    if (shape_ == SAW || shape_ == IMPULSE_TRAIN) {
      // 2a. Saw: one jump of -1 per cycle.
      if (phase_ >= 1.0f) {
        phase_ -= 1.0f;
        float t = phase_ / frequency;
        thisSample -= thisBlepSample(t);
        nextSample -= nextBlepSample(t);
      }
      nextSample += phase_;
      if (shape_ == SAW) {
        out = 2.0f * thisSample - 1.0f;
      } else {
        // The impulse train is the saw's derivative, gently smoothed.
        lpState_ += 0.25f * ((hpState_ - thisSample) - lpState_);
        out = 4.0f * lpState_;
        hpState_ = thisSample;
      }
    } else if (shape_ == TRIANGLE || shape_ == SLOPE) {
      // 2b. Triangle/slope: no jumps, but corners (jumps in slope), fixed
      // with the integrated BLEP.
      float slopeUp = 2.0f, slopeDown = 2.0f;
      if (shape_ == SLOPE) {
        slopeUp = 1.0f / pw;
        slopeDown = 1.0f / (1.0f - pw);
      }
      if (high_ ^ (phase_ < pw)) {
        float t = (phase_ - pw) / frequency;
        float discontinuity = (slopeUp + slopeDown) * frequency;
        thisSample -= thisIntegratedBlepSample(t) * discontinuity;
        nextSample -= nextIntegratedBlepSample(t) * discontinuity;
        high_ = phase_ < pw;
      }
      if (phase_ >= 1.0f) {
        phase_ -= 1.0f;
        float t = phase_ / frequency;
        float discontinuity = (slopeUp + slopeDown) * frequency;
        thisSample += thisIntegratedBlepSample(t) * discontinuity;
        nextSample += nextIntegratedBlepSample(t) * discontinuity;
        high_ = true;
      }
      nextSample += high_ ? phase_ * slopeUp : 1.0f - (phase_ - pw) * slopeDown;
      out = 2.0f * thisSample - 1.0f;
    } else {
      // 2c. Square: a jump up at the pulse width, a jump down at the reset.
      if (high_ ^ (phase_ >= pw)) {
        float t = (phase_ - pw) / frequency;
        thisSample += thisBlepSample(t);
        nextSample += nextBlepSample(t);
        high_ = phase_ >= pw;
      }
      if (phase_ >= 1.0f) {
        phase_ -= 1.0f;
        float t = phase_ / frequency;
        thisSample -= thisBlepSample(t);
        nextSample -= nextBlepSample(t);
        high_ = false;
      }
      nextSample += phase_ < pw ? 0.0f : 1.0f;

      // Filtered variants: the one-pole coefficient follows the frequency,
      // so the tone is the same at every pitch.
      if (shape_ == SQUARE_TRIANGLE) {
        thisSample = 128.0f * (thisSample - 0.5f);
        lpState_ += frequency * 0.0625f * (thisSample - lpState_);
        out = lpState_;
      } else if (shape_ == SQUARE_DARK) {
        thisSample = 4.0f * (thisSample - 0.5f);
        lpState_ += frequency * 2.0f * (thisSample - lpState_);
        out = lpState_ * 0.67f;  // Plaits' level was about ±1.5
      } else if (shape_ == SQUARE_BRIGHT) {
        thisSample = 2.0f * thisSample - 1.0f;
        lpState_ += frequency * 2.0f * (thisSample - lpState_);
        out = (thisSample - lpState_) * 0.5f;
      } else {
        out = 2.0f * thisSample - 1.0f;
      }
    }
    nextSample_ = nextSample;
    return out;
  }

 private:
  float sampleTime_ = 1.0f / 48000.0f;
  Shape shape_ = SAW;
  float frequency_ = 0.001f;
  float pw_ = 0.5f;
  float phase_ = 0.5f;
  float nextSample_ = 0.0f;
  float lpState_ = 0.0f;
  float hpState_ = 0.0f;
  bool high_ = false;
  bool needsSync_ = true;
};

}  // namespace ml

#endif  // ML_OSC_BASIC_H_
