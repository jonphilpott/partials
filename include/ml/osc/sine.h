// mutablelib — osc/sine.h
//
// Sine oscillators: sineFromPhase() and SineOscillator (Plaits' table
// sine), and the recursive CosineOscillator (stmlib).
//
// CosineOscillator: generates the series cos(0), cos(w), cos(2w), ... one
// value per next() call, using only one multiply and one subtract per step.
//
// It is rarely used as an audio oscillator. Gillet uses it to compute a
// set of per-harmonic gains cheaply: the modal resonator, for instance,
// weights mode k by 0.5 + 0.5*cos(2*pi*k*position) to imitate where along
// a string or bar it is struck or picked up.
//
// How it works: cosines obey the recurrence
//   cos((n+1)w) = 2cos(w) * cos(nw) - cos((n-1)w)
// so once 2cos(w) is known, each new value needs only the previous two.
//
// Derived from stmlib dsp/cosine_oscillator.h (Copyright 2014) and Plaits
// sine_oscillator.h (Copyright 2016), Emilie Gillet. MIT licence.

#ifndef ML_OSC_SINE_H_
#define ML_OSC_SINE_H_

#include <cmath>

#include "ml/core/math.h"
#include "ml/core/tables.h"

namespace ml {

// Sine from a 0..1 phase using the shared table (Plaits' Sine()). Cheaper
// than std::sin and accurate to about -100 dB. Any phase works; only its
// fractional part counts.
inline float sineFromPhase(float phase) {
  return interpolateWrap(sineTable(), phase, kSineTableSize);
}

// SineOscillator: a plain table sine oscillator (Plaits). Use it for
// LFOs, test tones, FM carriers, or anywhere a pure tone is needed.
class SineOscillator {
 public:
  void init(float sampleRate) {
    sampleTime_ = 1.0f / sampleRate;
    phase_ = 0.0f;
    sineTable();  // build the table now, not on the audio thread
  }

  void setFrequency(float hz) { frequency_ = hz * sampleTime_; }
  void reset() { phase_ = 0.0f; }

  float process() { return next(frequency_); }

  // Lower-level form: frequency as cycles per sample (Hz / sampleRate).
  float next(float frequency) {
    if (frequency >= 0.5f) frequency = 0.5f;
    phase_ += frequency;
    if (phase_ >= 1.0f) phase_ -= 1.0f;
    return interpolate(sineTable(), phase_, kSineTableSize);
  }

  // Sine and cosine together (90 degrees apart), scaled by amplitude.
  void next(float frequency, float amplitude, float& sine, float& cosine) {
    if (frequency >= 0.5f) frequency = 0.5f;
    phase_ += frequency;
    if (phase_ >= 1.0f) phase_ -= 1.0f;
    sine = amplitude * interpolate(sineTable(), phase_, kSineTableSize);
    cosine = amplitude * interpolate(sineTable(), phase_ + 0.25f, kSineTableSize);
  }

 private:
  float sampleTime_ = 1.0f / 48000.0f;
  float frequency_ = 0.0f;
  float phase_ = 0.0f;
};

class CosineOscillator {
 public:
  // frequency: cycles per step (0..1). Uses exact cos().
  void init(float frequency) {
    coefficient_ = 2.0f * std::cos(2.0f * 3.14159265358979f * frequency);
    initialAmplitude_ = coefficient_ * 0.25f;
    start();
  }

  // Same, with stmlib's cheap parabolic approximation of 2cos(2*pi*f). Used
  // where the original used it, so the output matches.
  void initApproximate(float frequency) {
    float sign = 16.0f;
    frequency -= 0.25f;
    if (frequency < 0.0f) {
      frequency = -frequency;
    } else if (frequency > 0.5f) {
      frequency -= 0.5f;
    } else {
      sign = -16.0f;
    }
    coefficient_ = sign * frequency * (1.0f - 2.0f * frequency);
    initialAmplitude_ = coefficient_ * 0.25f;
    start();
  }

  // Rewind to the first term.
  void start() {
    y1_ = initialAmplitude_;
    y0_ = 0.5f;
  }

  // The most recent value returned by next().
  float value() const { return y1_ + 0.5f; }

  // Returns 0.5 + 0.5 * cos(n * w) for n = 0, 1, 2, ... (range 0..1).
  float next() {
    float temp = y0_;
    y0_ = coefficient_ * y0_ - y1_;
    y1_ = temp;
    return temp + 0.5f;
  }

 private:
  float y1_ = 0.0f, y0_ = 0.5f;
  float coefficient_ = 0.0f, initialAmplitude_ = 0.0f;
};

}  // namespace ml

#endif  // ML_OSC_SINE_H_
