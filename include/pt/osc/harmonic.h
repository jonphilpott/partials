// partials — osc/harmonic.h
//
// HarmonicOscillator: additive synthesis, a sum of up to 32 harmonics of
// one pitch, each with its own amplitude. Set the amplitudes from knobs,
// CVs, or computed spectra (organ drawbars, formants, a spectral tilt).
// Plaits' additive engine uses three of these.
//
// How it works: instead of computing 32 sines, it computes one sine,
// x = sin(phase), and gets the rest from the Chebyshev recurrence
//   T(k+1) = 2x T(k) - T(k-1)
// which costs a multiply and an add per harmonic. Each harmonic's
// amplitude is reduced as it nears Nyquist, and silenced above it, so
// nothing aliases.
//
// Harmonic k comes out with a phase offset compared with sin(k*phase):
// that changes the waveform's shape but not its sound.
//
// Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.

#ifndef PT_OSC_HARMONIC_H_
#define PT_OSC_HARMONIC_H_

#include <cmath>

#include "pt/core/math.h"
#include "pt/osc/sine.h"

namespace pt {

class HarmonicOscillator {
 public:
  static const int kMaxHarmonics = 32;

  void init(float sampleRate) {
    sampleTime_ = 1.0f / sampleRate;
    phase_ = 0.0f;
    for (float& a : amplitude_) a = 0.0f;
    amplitude_[0] = 1.0f;
    numHarmonics_ = 1;
    sineTable();
    update();
  }

  // Fundamental in Hz.
  void setFrequency(float hz) {
    frequency_ = std::fmin(hz * sampleTime_, 0.5f);
    update();
  }
  // Amplitude of harmonic k (1 = fundamental). Sum of all amplitudes about
  // 1 for a full-scale output.
  void setAmplitude(int k, float a) {
    if (k < 1 || k > kMaxHarmonics) return;
    amplitude_[k - 1] = a;
    if (k > numHarmonics_) numHarmonics_ = k;
    update();
  }
  // Set harmonics 1..n at once.
  void setAmplitudes(const float* a, int n) {
    numHarmonics_ = n < kMaxHarmonics ? n : kMaxHarmonics;
    for (int i = 0; i < numHarmonics_; ++i) amplitude_[i] = a[i];
    for (int i = numHarmonics_; i < kMaxHarmonics; ++i) amplitude_[i] = 0.0f;
    update();
  }

  float process() {
    phase_ += frequency_;
    if (phase_ >= 1.0f) phase_ -= 1.0f;
    // Chebyshev recurrence from x = sin(phase).
    const float twoX = 2.0f * interpolate(sineTable(), phase_, kSineTableSize);
    float previous = 1.0f;
    float current = twoX * 0.5f;
    float sum = 0.0f;
    for (int i = 0; i < numHarmonics_; ++i) {
      sum += gain_[i] * current;
      float temp = current;
      current = twoX * current - previous;
      previous = temp;
    }
    return sum;
  }

 private:
  // Fade each harmonic out as it approaches Nyquist (0.5).
  void update() {
    for (int i = 0; i < numHarmonics_; ++i) {
      float f = std::fmin(frequency_ * static_cast<float>(i + 1), 0.5f);
      gain_[i] = amplitude_[i] * (1.0f - f * 2.0f);
    }
  }

  float sampleTime_ = 1.0f / 48000.0f;
  float frequency_ = 0.0f, phase_ = 0.0f;
  int numHarmonics_ = 1;
  float amplitude_[kMaxHarmonics] = {};
  float gain_[kMaxHarmonics] = {};
};

}  // namespace pt

#endif  // PT_OSC_HARMONIC_H_
