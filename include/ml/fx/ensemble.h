// mutablelib — fx/ensemble.h
//
// Stereo ensemble: the lush, shimmering chorus of 1970s string machines
// (Solina, ARP String Ensemble). Rings' built-in ensemble effect.
//
// How it works: like a chorus, but with three delay taps per side whose
// modulation is spread 120° apart, and each tap swept by two LFOs at once:
// a slow one (0.75 Hz) for the broad "swirl" and a fast one (6.6 Hz) for
// vibrato-like shimmer. Three taps at three phases means at any moment some
// are going sharp while others go flat, so the average pitch stays steady
// while the sound thickens.
//
// Sample-rate independence: delay times and LFO rates are converted from
// Rings' 48 kHz settings.
//
// Derived from Rings, Copyright 2015 Emilie Gillet. MIT licence.

#ifndef ML_FX_ENSEMBLE_H_
#define ML_FX_ENSEMBLE_H_

#include <cmath>
#include <cstdint>

#include "ml/fx/fx_engine.h"

namespace ml {

class Ensemble {
 public:
  // Allocates the delay memory: call from onSampleRateChange, not process.
  void init(float sampleRate) {
    scale_ = sampleRate / 48000.0f;
    engine_.reset();
    lineL_ = engine_.addDelay(static_cast<int32_t>(2047 * scale_ + 0.5f));
    lineR_ = engine_.addDelay(static_cast<int32_t>(2047 * scale_ + 0.5f));
    engine_.allocate();
    increment1_ = 1.57e-05f / scale_;  // 0.75 Hz
    increment2_ = 1.37e-04f / scale_;  // 6.6 Hz
    phase1_ = phase2_ = 0.0f;
  }

  // 0..1. Wet amount. The dry signal is turned down by half the wet amount.
  void setAmount(float x) { amount_ = x; }
  // 0..1. Modulation depth, up to about ±3 ms.
  void setDepth(float x) { depth_ = x * 128.0f; }

  void clear() { engine_.clear(); }

  void process(float& left, float& right) {
    FxEngine& c = engine_;
    c.start();
    const float dryAmount = 1.0f - amount_ * 0.5f;

    // 1. Slow and fast LFOs, each at 0°, 120° and 240°. Rings read a
    // 4096-step sine table without interpolation; computing sin() of the
    // same quantised phase gives identical values.
    phase1_ += increment1_;
    if (phase1_ >= 1.0f) phase1_ -= 1.0f;
    phase2_ += increment2_;
    if (phase2_ >= 1.0f) phase2_ -= 1.0f;
    int32_t phi1 = static_cast<int32_t>(phase1_ * 4096.0f);
    int32_t phi2 = static_cast<int32_t>(phase2_ * 4096.0f);
    float slow0 = sine4096(phi1), slow120 = sine4096(phi1 + 1365), slow240 = sine4096(phi1 + 2730);
    float fast0 = sine4096(phi2), fast120 = sine4096(phi2 + 1365), fast240 = sine4096(phi2 + 2730);

    // 2. Each tap's sweep: mostly slow, a little fast.
    float a = depth_ * scale_;
    float b = depth_ * 0.1f * scale_;
    float mod1 = slow0 * a + fast0 * b;
    float mod2 = slow120 * a + fast120 * b;
    float mod3 = slow240 * a + fast240 * b;
    float centre = 1024.0f * scale_;  // about 21 ms

    // 3. Write both channels, then read three taps per side: two from its
    // own line, one from the other side's.
    float wet;
    c.read(left, 1.0f);
    c.write(lineL_, 0.0f);
    c.read(right, 1.0f);
    c.write(lineR_, 0.0f);

    c.interpolate(lineL_, mod1 + centre, 0.33f);
    c.interpolate(lineL_, mod2 + centre, 0.33f);
    c.interpolate(lineR_, mod3 + centre, 0.33f);
    c.write(wet, 0.0f);
    left = wet * amount_ + left * dryAmount;

    c.interpolate(lineR_, mod1 + centre, 0.33f);
    c.interpolate(lineR_, mod2 + centre, 0.33f);
    c.interpolate(lineL_, mod3 + centre, 0.33f);
    c.write(wet, 0.0f);
    right = wet * amount_ + right * dryAmount;
  }

 private:
  static float sine4096(int32_t i) {
    return static_cast<float>(std::sin(6.283185307179586 * (i & 4095) / 4096.0));
  }

  FxEngine engine_;
  FxEngine::Delay lineL_, lineR_;
  float scale_ = 1.0f;
  float amount_ = 0.5f;
  float depth_ = 0.5f * 128.0f;
  float phase1_ = 0.0f, phase2_ = 0.0f;
  float increment1_ = 0.0f, increment2_ = 0.0f;
};

}  // namespace ml

#endif  // ML_FX_ENSEMBLE_H_
