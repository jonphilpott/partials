// partials — analysis/onset_detector.h
//
// OnsetDetector: tells you when a new note or hit starts in an audio
// signal. Rings uses it to "strum" its internal exciter from audio input;
// use it to trigger envelopes, drums or sequencers from a guitar, a drum
// loop or a voice.
//
// How it works
// 1. Automatic gain control evens out the level, so quiet and loud
//    playing both register.
// 2. The signal is split into low, mid and high bands, and each band's
//    energy is tracked.
// 3. The "onset function" is the sum of each band's energy increase
//    (decreases are ignored), lightly smoothed.
// 4. An onset fires when that function jumps well above its recent
//    average (more than one standard deviation, a "z-score" test), the
//    total energy is above a threshold left by the previous onset, and
//    the minimum time between onsets has passed.
//
// The analysis runs in blocks of about 0.5 ms (as Rings' did, once per
// audio block), so detection lags the actual onset by up to one block.
//
// Derived from Rings, Copyright 2015 Emilie Gillet. MIT licence.

#ifndef PT_ANALYSIS_ONSET_DETECTOR_H_
#define PT_ANALYSIS_ONSET_DETECTOR_H_

#include <cmath>
#include <vector>

#include "pt/core/math.h"

namespace pt {

class OnsetDetector {
 public:
  // minInterval: shortest time between onsets, in seconds. The default,
  // 50 ms, gives one onset per note. Rings used 10 ms; with that, a sharp
  // attack can register several times in its first 30 ms, as its energy
  // keeps rising past the re-arm threshold.
  void init(float sampleRate, float minInterval = 0.05f) {
    // 1. Block size: about 0.5 ms, like Rings' 24 samples at 48 kHz.
    blockSize_ = static_cast<int>(sampleRate / 2000.0f + 0.5f);
    if (blockSize_ < 4) blockSize_ = 4;
    block_.assign(blockSize_, 0.0f);
    fill_ = 0;
    const float blockRate = sampleRate / blockSize_;

    // 2. Rings' settings: crossovers at 160 Hz and 1.6 kHz; per-band
    // detectors; gain control and z-score speeds tied to minInterval.
    const float low = 8.0f / sampleRate, lowMid = 160.0f / sampleRate, midHigh = 1600.0f / sampleRate;
    const float ioiF = 1.0f / (minInterval * blockRate);
    agcAttack_ = ioiF * 10.0f;
    agcDecay_ = ioiF * 0.05f;
    agcSkew_ = 1.0f / 40.0f;
    agcLevel_ = 0.0f;
    lowMid_.set(lowMid);
    midHigh_.set(midHigh);
    for (int i = 0; i < 3; ++i) {
      attack_[i] = lowMid;
      decay_[i] = low * 0.25f;
      envelope_[i] = energy_[i] = 0.0f;
    }
    zCoefficient_ = ioiF * 0.05f;
    zMean_ = zVariance_ = 0.0f;
    inhibitTime_ = static_cast<int>(minInterval * blockRate);
    inhibitDecay_ = 1.0f / (minInterval * blockRate);
    inhibitThreshold_ = 0.0f;
    inhibitCounter_ = 0;
    onsetDf_ = 0.0f;
    onset_ = false;
  }

  // Feed one sample. Returns true on the sample where an onset is
  // detected (at the end of an analysis block).
  bool process(float in) {
    // Automatic gain control, per sample.
    slope(agcLevel_, std::fabs(in), agcAttack_, agcDecay_);
    block_[fill_++] = in / (agcSkew_ + agcLevel_);
    onset_ = false;
    if (fill_ == blockSize_) {
      fill_ = 0;
      onset_ = analyse();
    }
    return onset_;
  }

  bool onset() const { return onset_; }

 private:
  bool analyse() {
    // 1. Energy per band. The low band's envelope is updated every 4th
    // sample, the mid band's every 2nd (they change slowly).
    float onsetDf = 0.0f, totalEnergy = 0.0f;
    float sums[3] = {0.0f, 0.0f, 0.0f};
    for (int j = 0; j < blockSize_; ++j) {
      midHigh_.process(block_[j]);
      float high = midHigh_.hp;
      lowMid_.process(midHigh_.lp);
      float bands[3] = {lowMid_.lp, lowMid_.hp, high};
      for (int i = 0; i < 3; ++i) {
        int increment = 4 >> i;
        if (j % increment == 0) {
          slope(envelope_[i], bands[i] * bands[i], attack_[i], decay_[i]);
          sums[i] += envelope_[i];
        }
      }
    }
    // 2. The onset function: total energy rise across bands.
    for (int i = 0; i < 3; ++i) {
      float energy = std::sqrt(sums[i]) * static_cast<float>(4 >> i);
      float derivative = energy - energy_[i];
      onsetDf += derivative + std::fabs(derivative);
      energy_[i] = energy;
      totalEnergy += energy;
    }
    onsetDf_ += 0.05f * (onsetDf - onsetDf_);

    // 3. Outlier test (z-score), energy threshold, and inhibition.
    float centered = onsetDf_ - zMean_;
    zMean_ += zCoefficient_ * centered;
    zVariance_ += zCoefficient_ * (centered * centered - zVariance_);
    bool outlier = centered > std::sqrt(zVariance_) * 1.0f && centered > 0.01f;
    bool loudEnough = totalEnergy >= inhibitThreshold_;
    bool onset = outlier && loudEnough && !inhibitCounter_;
    if (onset) {
      inhibitThreshold_ = totalEnergy * 1.5f;
      inhibitCounter_ = inhibitTime_;
    } else {
      inhibitThreshold_ -= inhibitDecay_ * inhibitThreshold_;
      if (inhibitCounter_) --inhibitCounter_;
    }
    return onset;
  }

  // Chamberlin state-variable filter (stmlib's NaiveSvf), q = 0.5.
  struct NaiveSvf {
    float f = 0.1f, lp = 0.0f, bp = 0.0f, hp = 0.0f;
    void set(float frequency) { f = 2.0f * 3.14159265f * std::fmin(frequency, 0.158f); }
    void process(float in) {
      float notch = in - bp * 2.0f;
      lp += f * bp;
      hp = notch - lp;
      bp += f * hp;
    }
  };

  int blockSize_ = 24, fill_ = 0;
  std::vector<float> block_;
  float agcAttack_ = 0.0f, agcDecay_ = 0.0f, agcSkew_ = 0.025f, agcLevel_ = 0.0f;
  NaiveSvf lowMid_, midHigh_;
  float attack_[3] = {}, decay_[3] = {}, envelope_[3] = {}, energy_[3] = {};
  float onsetDf_ = 0.0f;
  float zCoefficient_ = 0.0f, zMean_ = 0.0f, zVariance_ = 0.0f;
  float inhibitThreshold_ = 0.0f, inhibitDecay_ = 0.0f;
  int inhibitTime_ = 0, inhibitCounter_ = 0;
  bool onset_ = false;
};

}  // namespace pt

#endif  // PT_ANALYSIS_ONSET_DETECTOR_H_
