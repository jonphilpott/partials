// mutablelib — osc/wavetable.h
//
// WavetableOscillator: plays your own single-cycle waveforms, morphing
// smoothly between them, with Plaits' cheap and effective anti-aliasing.
// Load any number of waves (all the same length) once, then sweep through
// them with setMorph().
//
// How it works (the "integrate then differentiate" trick)
// - loadWaves() normalises each wave and stores its running sum (its
//   integral) instead of the wave itself.
// - At play time, the difference between successive reads of the integral
//   gives back the wave, averaged over everything the read head skipped
//   since the last sample. That averaging is a box filter whose width
//   follows the pitch: the higher the note, the more it smooths, which
//   removes most of the aliasing that plain table reads produce.
// - A gentle low-pass, also tracking the pitch, finishes the job, and the
//   level is reduced near Nyquist.
//
// Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.

#ifndef ML_OSC_WAVETABLE_H_
#define ML_OSC_WAVETABLE_H_

#include <cmath>
#include <vector>

#include "ml/core/math.h"

namespace ml {

class WavetableOscillator {
 public:
  void init(float sampleRate) {
    sampleTime_ = 1.0f / sampleRate;
    phase_ = 0.0f;
    previous_ = differentiated_ = lp_ = 0.0f;
    primed_ = false;
  }

  // Load `numWaves` single-cycle waves of `waveSize` samples each, stored
  // one after another. Each is normalised to ±1. Allocates memory: call
  // from the constructor or onSampleRateChange, not process().
  void loadWaves(const float* waves, int waveSize, int numWaves) {
    waveSize_ = waveSize;
    numWaves_ = numWaves;
    // Each stored wave: its integral, plus one guard point (the cycle's
    // integral returns to its start because the wave has no DC).
    table_.assign(static_cast<size_t>(numWaves) * (waveSize + 1), 0.0f);
    for (int w = 0; w < numWaves; ++w) {
      const float* x = waves + static_cast<size_t>(w) * waveSize;
      // 1. Remove DC and normalise.
      double mean = 0.0, peak = 0.0;
      for (int i = 0; i < waveSize; ++i) mean += x[i];
      mean /= waveSize;
      for (int i = 0; i < waveSize; ++i) peak = std::fmax(peak, std::fabs(x[i] - mean));
      if (peak == 0.0) peak = 1.0;
      // 2. Running sum, with its own mean removed so it stays centred.
      std::vector<double> integral(waveSize + 1, 0.0);
      for (int i = 0; i < waveSize; ++i) integral[i + 1] = integral[i] + (x[i] - mean) / peak;
      double integralMean = 0.0;
      for (int i = 0; i < waveSize; ++i) integralMean += integral[i];
      integralMean /= waveSize;
      float* out = &table_[static_cast<size_t>(w) * (waveSize + 1)];
      for (int i = 0; i <= waveSize; ++i) out[i] = static_cast<float>(integral[i] - integralMean);
    }
    primed_ = false;
  }

  void setFrequency(float hz) { frequency_ = clamp(hz * sampleTime_, 0.0000001f, 0.25f); }
  // 0..1. Position across the loaded waves, blending neighbours.
  void setMorph(float x) { morph_ = clamp(x, 0.0f, 1.0f); }

  float process() {
    if (table_.empty()) return 0.0f;
    const float f0 = frequency_;
    // 1. Advance; the filters' cutoff follows the read speed through the
    // table.
    const float cutoff = std::fmin(static_cast<float>(waveSize_) * f0, 1.0f);
    phase_ += f0;
    if (phase_ >= 1.0f) phase_ -= 1.0f;

    // 2. Read the two neighbouring waves' integrals and blend.
    float w = morph_ * (static_cast<float>(numWaves_) - 1.0001f);
    int wi = static_cast<int>(w);
    float wf = w - static_cast<float>(wi);
    if (numWaves_ == 1) {
      wi = 0;
      wf = 0.0f;
    }
    float p = phase_ * static_cast<float>(waveSize_);
    int pi = static_cast<int>(p);
    float pf = p - static_cast<float>(pi);
    float x0 = read(wi, pi, pf);
    float x1 = numWaves_ > 1 ? read(wi + 1, pi, pf) : x0;
    float integral = x0 + (x1 - x0) * wf;
    // The very first read has no previous one to difference against;
    // start from it, so there is no spike. (Plaits had a start-up
    // transient here.)
    if (!primed_) {
      previous_ = integral;
      primed_ = true;
    }

    // 3. Differentiate: the change in the integral per sample is the wave
    // averaged over that sample. Dividing by the read speed restores the
    // level.
    float s = (integral - previous_) / (static_cast<float>(waveSize_) * f0);
    previous_ = integral;
    onePole(differentiated_, s, cutoff);
    onePole(lp_, differentiated_, cutoff);
    return lp_ * (1.0f - 2.0f * f0);
  }

 private:
  float read(int wave, int i, float frac) const {
    const float* t = &table_[static_cast<size_t>(wave) * (waveSize_ + 1)];
    return t[i] + (t[i + 1] - t[i]) * frac;
  }

  float sampleTime_ = 1.0f / 48000.0f;
  std::vector<float> table_;
  int waveSize_ = 0, numWaves_ = 0;
  float frequency_ = 0.001f, morph_ = 0.0f;
  float phase_ = 0.0f, previous_ = 0.0f, differentiated_ = 0.0f, lp_ = 0.0f;
  bool primed_ = false;
};

}  // namespace ml

#endif  // ML_OSC_WAVETABLE_H_
