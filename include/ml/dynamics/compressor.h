// mutablelib — dynamics/compressor.h
//
// Compressor: an RMS feed-forward compressor with optional sidechain,
// soft knee and makeup gain. Streams' "compressor" mode, in float.
//
// How it works
// 1. Detector: the input (or the sidechain, if given) is squared and
//    smoothed with separate attack and release rates. That tracks the
//    signal's power (RMS level) rather than its peaks, which sounds more
//    natural.
// 2. Gain computer, in decibels: above the threshold, each dB of input
//    rise gives only 1/ratio dB of output rise. The soft knee rounds the
//    corner at the threshold.
// 3. The gain (with makeup) is applied to the input.
//
// Original: Streams, fixed-point log/exp arithmetic driving an analog VCA.
// This port works in float dB, with times in seconds.
//
// Derived from Streams, Copyright 2014 Emilie Gillet. MIT licence.

#ifndef ML_DYNAMICS_COMPRESSOR_H_
#define ML_DYNAMICS_COMPRESSOR_H_

#include <cmath>

#include "ml/core/math.h"

namespace ml {

class Compressor {
 public:
  void init(float sampleRate) {
    sampleRate_ = sampleRate;
    detector_ = 0.0f;
    gainReductionDb_ = 0.0f;
    update();
  }

  // Level (dB, 0 = full scale) above which compression starts. Streams:
  // -30..0 dB.
  void setThreshold(float db) { thresholdDb_ = db; }
  // Compression ratio, 1 (none) to 25 or more; very high = limiting.
  void setRatio(float ratio) { ratio_ = ratio < 1.0f ? 1.0f : ratio; }
  // Attack and release in seconds. Streams' defaults: 0.2 ms / 150 ms
  // (hard knee), 2 ms / 70 ms (soft knee).
  void setAttack(float seconds) { attack_ = seconds; update(); }
  void setRelease(float seconds) { release_ = seconds; update(); }
  // Gain added after compression, in dB.
  void setMakeup(float db) { makeupDb_ = db; }
  // Round the corner at the threshold over about 6 dB.
  void setSoftKnee(bool on) { softKnee_ = on; }

  // Compress `in`, detecting the level from `in` itself.
  float process(float in) { return process(in, in); }

  // Compress `in`, detecting the level from `sidechain` (e.g. a kick, for
  // ducking).
  float process(float in, float sidechain) {
    // 1. RMS detector.
    float energy = sidechain * sidechain;
    float error = energy - detector_;
    detector_ += error * (error > 0.0f ? attackCoefficient_ : releaseCoefficient_);

    // 2. Gain computer. The level is in dB of RMS: 10*log10 of the power.
    float levelDb = 10.0f * std::log10(detector_ + 1e-20f);
    float over = levelDb - thresholdDb_;
    float attenuation = 0.0f;
    if (over > 0.0f) {
      attenuation = over * (1.0f - 1.0f / ratio_);
      // Soft knee (Streams' cubic curve over the first ~6 dB of
      // attenuation): gentler gain reduction near the threshold.
      if (softKnee_ && attenuation < 6.02f) {
        float t = attenuation / 6.02f;
        float knee = t * t * t * 6.02f;
        attenuation += (knee - attenuation) * (1.0f - t);
      }
    }
    gainReductionDb_ = attenuation;

    // 3. Apply.
    float gain = std::pow(10.0f, (makeupDb_ - attenuation) / 20.0f);
    return in * gain;
  }

  // Current gain reduction in dB (positive = turned down), for a meter.
  float gainReduction() const { return gainReductionDb_; }

 private:
  void update() {
    attackCoefficient_ = 1.0f - std::exp(-1.0f / (attack_ * sampleRate_));
    releaseCoefficient_ = 1.0f - std::exp(-1.0f / (release_ * sampleRate_));
  }

  float sampleRate_ = 48000.0f;
  float thresholdDb_ = -12.0f, ratio_ = 4.0f, makeupDb_ = 0.0f;
  float attack_ = 0.0002f, release_ = 0.15f;
  bool softKnee_ = false;
  float attackCoefficient_ = 1.0f, releaseCoefficient_ = 0.001f;
  float detector_ = 0.0f, gainReductionDb_ = 0.0f;
};

}  // namespace ml

#endif  // ML_DYNAMICS_COMPRESSOR_H_
