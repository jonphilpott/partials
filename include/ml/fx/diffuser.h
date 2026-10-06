// mutablelib — fx/diffuser.h
//
// Stereo diffuser: four all-pass filters per channel that smear a sound in
// time without changing its spectrum. Each click turns into a short, dense
// wash, like the very start of a reverb with no tail. Clouds uses it to
// blur grains together ("texture"); it also softens harsh transients and
// thickens sparse sounds.
//
// How it works: an all-pass filter delays different frequencies by
// different amounts while passing all of them at equal level. Four in a
// row, with unrelated lengths, scatter the energy of each transient over
// a few tens of milliseconds. The left and right chains use different
// lengths, so a mono input comes out with a wide stereo image.
//
// Sample-rate independence: delay lengths are scaled from Clouds' 32 kHz.
//
// Derived from Clouds, Copyright 2014 Emilie Gillet. MIT licence.

#ifndef ML_FX_DIFFUSER_H_
#define ML_FX_DIFFUSER_H_

#include "ml/fx/fx_engine.h"

namespace ml {

class Diffuser {
 public:
  // Allocates the delay memory: call from onSampleRateChange, not process.
  void init(float sampleRate) {
    static const int32_t kLengths[8] = {126, 180, 269, 444, 151, 205, 245, 405};
    float scale = sampleRate / 32000.0f;
    engine_.reset();
    for (int i = 0; i < 8; ++i) {
      lines_[i] = engine_.addDelay(static_cast<int32_t>(kLengths[i] * scale + 0.5f));
    }
    engine_.allocate();
  }

  // 0..1. Wet/dry mix.
  void setAmount(float x) { amount_ = x; }
  // 0..1. All-pass coefficient; higher = more smearing. Clouds: 0.625.
  void setDiffusion(float x) { diffusion_ = x; }

  void clear() { engine_.clear(); }

  void process(float& left, float& right) {
    const float kap = diffusion_;
    FxEngine& c = engine_;
    float wet;
    c.start();
    // Left: four all-passes in series.
    c.read(left);
    for (int i = 0; i < 4; ++i) {
      c.readTail(lines_[i], kap);
      c.writeAllPass(lines_[i], -kap);
    }
    c.write(wet, 0.0f);
    left += amount_ * (wet - left);
    // Right: four more, with different lengths.
    c.read(right);
    for (int i = 4; i < 8; ++i) {
      c.readTail(lines_[i], kap);
      c.writeAllPass(lines_[i], -kap);
    }
    c.write(wet, 0.0f);
    right += amount_ * (wet - right);
  }

 private:
  FxEngine engine_;
  FxEngine::Delay lines_[8];
  float amount_ = 1.0f;
  float diffusion_ = 0.625f;
};

}  // namespace ml

#endif  // ML_FX_DIFFUSER_H_
