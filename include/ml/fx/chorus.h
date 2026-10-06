// mutablelib — fx/chorus.h
//
// Stereo chorus: Rings' built-in chorus. Gentle, slow and wide.
//
// How it works: the input (left + right) goes into one delay line. Four
// read heads, each swept slowly by a sine or cosine LFO, read it back at
// 15-30 ms. A delayed copy whose delay keeps changing is slightly
// detuned, so mixing it with the dry signal gives the "several players at
// once" thickening. Left and right use different heads and LFO phases,
// which spreads the sound across the stereo field.
//
// Sample-rate independence: delay times and LFO rates are in seconds and
// Hz, converted from Rings' 48 kHz settings.
//
// Derived from Rings, Copyright 2015 Emilie Gillet. MIT licence.

#ifndef ML_FX_CHORUS_H_
#define ML_FX_CHORUS_H_

#include <cmath>

#include "ml/fx/fx_engine.h"

namespace ml {

class Chorus {
 public:
  // Allocates the delay memory: call from onSampleRateChange, not process.
  void init(float sampleRate) {
    scale_ = sampleRate / 48000.0f;
    engine_.reset();
    line_ = engine_.addDelay(static_cast<int32_t>(2047 * scale_ + 0.5f));
    engine_.allocate();
    // Rings' per-sample LFO increments at 48 kHz: 0.2 Hz and 0.26 Hz.
    increment1_ = 4.17e-06f / scale_;
    increment2_ = 5.417e-06f / scale_;
    phase1_ = phase2_ = 0.0f;
  }

  // 0..1. Wet amount. The dry signal is turned down by half the wet amount.
  void setAmount(float x) { amount_ = x; }
  // 0..1. Sweep depth, up to ±8 ms.
  void setDepth(float x) { depth_ = x * 384.0f; }

  void clear() { engine_.clear(); }

  void process(float& left, float& right) {
    FxEngine& c = engine_;
    c.start();
    const float dryAmount = 1.0f - amount_ * 0.5f;

    // 1. Two slow LFOs, each with a sine and a cosine (90° apart).
    phase1_ += increment1_;
    if (phase1_ >= 1.0f) phase1_ -= 1.0f;
    phase2_ += increment2_;
    if (phase2_ >= 1.0f) phase2_ -= 1.0f;
    const float twoPi = 6.28318530718f;
    float sin1 = std::sin(twoPi * phase1_);
    float cos1 = std::cos(twoPi * phase1_);
    float sin2 = std::sin(twoPi * phase2_);
    float cos2 = std::cos(twoPi * phase2_);

    // 2. Mono sum into the delay line.
    c.read(left, 0.5f);
    c.read(right, 0.5f);
    c.write(line_, 0.0f);

    // 3. Two swept heads per side (25 ms and 16.7 ms centres).
    float depth = depth_ * scale_;
    float wet;
    c.interpolate(line_, sin1 * depth + 1200.0f * scale_, 0.5f);
    c.interpolate(line_, sin2 * depth + 800.0f * scale_, 0.5f);
    c.write(wet, 0.0f);
    left = wet * amount_ + left * dryAmount;

    c.interpolate(line_, cos1 * depth + 800.0f * scale_, 0.5f);
    c.interpolate(line_, cos2 * depth + 1200.0f * scale_, 0.5f);
    c.write(wet, 0.0f);
    right = wet * amount_ + right * dryAmount;
  }

 private:
  FxEngine engine_;
  FxEngine::Delay line_;
  float scale_ = 1.0f;
  float amount_ = 0.5f;
  float depth_ = 0.5f * 384.0f;
  float phase1_ = 0.0f, phase2_ = 0.0f;
  float increment1_ = 0.0f, increment2_ = 0.0f;
};

}  // namespace ml

#endif  // ML_FX_CHORUS_H_
