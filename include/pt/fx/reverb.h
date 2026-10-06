// partials — fx/reverb.h
//
// Stereo plate-style reverb: the reverb built into Rings, Elements and
// Clouds, with each module's tuning available as a preset.
//
// How it works (the "Griesinger" topology from Jon Dattorro's 1997 paper
// on reverb design)
// 1. The input (left + right summed) goes through four all-pass filters in
//    series. All-passes don't colour the sound, but they smear each click
//    into a dense spray of echoes ("diffusion").
// 2. That feeds a figure-of-eight loop of two halves. Each half has two
//    more all-passes, a long delay, and a low-pass filter (high
//    frequencies die first, as in a real room). Each half feeds the other.
// 3. The left output is tapped from one half and the right from the other,
//    which gives a wide, decorrelated stereo image.
// 4. Slow LFOs wobble some delay taps, which blurs metallic resonances and
//    adds a gentle shimmer.
//
// Presets (they differ in delay lengths, which delays are modulated, and
// the rate they were tuned at):
// - Rings: brighter and shorter (Elements' delays run at 48 kHz).
// - Elements: larger, with modulation inside the input diffuser.
// - Clouds: smaller and denser (delays 75% of Elements').
//
// Sample-rate independence: delay lengths and modulation depths are scaled
// by sampleRate / the preset's original rate, and the damping filter's
// coefficient is converted, so each preset sounds the same at any rate.
//
// Derived from Rings, Elements and Clouds, Copyright 2014-2015 Emilie
// Gillet. MIT licence.

#ifndef PT_FX_REVERB_H_
#define PT_FX_REVERB_H_

#include "pt/core/math.h"
#include "pt/fx/fx_engine.h"

namespace pt {

class Reverb {
 public:
  enum Preset { RINGS, ELEMENTS, CLOUDS };

  // Allocates the delay memory: call from onSampleRateChange, not process.
  void init(float sampleRate, Preset preset = ELEMENTS) {
    preset_ = preset;
    // 1. The preset's original rate and delay lengths.
    static const int32_t kElementsLengths[10] = {150, 214, 319, 527, 2182, 2690, 4501, 2525, 2197, 6312};
    static const int32_t kCloudsLengths[10] = {113, 162, 241, 399, 1653, 2038, 3411, 1913, 1663, 4782};
    const int32_t* lengths = preset == CLOUDS ? kCloudsLengths : kElementsLengths;
    float originalRate = preset == RINGS ? 48000.0f : 32000.0f;
    scale_ = sampleRate / originalRate;
    lpRateRatio_ = originalRate / sampleRate;

    // 2. Lay out the ten delay lines, scaled to this sample rate.
    engine_.reset();
    for (int i = 0; i < 10; ++i) {
      lines_[i] = engine_.addDelay(static_cast<int32_t>(lengths[i] * scale_ + 0.5f));
    }
    engine_.allocate();
    engine_.setLfoFrequency(0, 0.5f / sampleRate);
    engine_.setLfoFrequency(1, 0.3f / sampleRate);

    // 3. Preset-specific modulation: the loop tap on del2 (read at a fixed
    // point, swept by LFO 2), plus optional modulation of del1 or of the
    // first input all-pass.
    switch (preset) {
      case RINGS:
        del2Tap_ = 6261.0f; del2Depth_ = 50.0f;
        del1Tap_ = 4460.0f; del1Depth_ = 40.0f;
        ap1Depth_ = 0.0f;
        break;
      case ELEMENTS:
        del2Tap_ = 6211.0f; del2Depth_ = 100.0f;
        del1Tap_ = 0.0f; del1Depth_ = 0.0f;
        ap1Depth_ = 80.0f;
        break;
      case CLOUDS:
        del2Tap_ = 4680.0f; del2Depth_ = 100.0f;
        del1Tap_ = 0.0f; del1Depth_ = 0.0f;
        ap1Depth_ = 60.0f;
        break;
    }
    lpDecay1_ = lpDecay2_ = 0.0f;
    setLp(lp_);
  }

  // 0..1. Wet/dry mix: 0 = dry only, 1 = wet only.
  void setAmount(float x) { amount_ = x; }
  // Input level into the reverb. The modules used 0.2: the loop has a lot
  // of gain, so keep this low.
  void setInputGain(float x) { inputGain_ = x; }
  // 0..1. Decay: the gain of each trip round the loop. 0.5 is a small room,
  // 0.9 a hall, 0.98 near-infinite.
  void setTime(float x) { time_ = x; }
  // 0..1. All-pass coefficient: how smeared the echoes are. 0.625 default.
  void setDiffusion(float x) { diffusion_ = x; }
  // 0..1. Damping filter in the loop, as the modules set it: 1 = bright,
  // lower values darker (high frequencies die faster).
  void setLp(float x) {
    lp_ = x;
    lpCoefficient_ = rescaleCoefficient(clamp(x, 0.0f, 1.0f), lpRateRatio_);
  }

  void clear() { engine_.clear(); }

  // Stereo in, stereo out, in place.
  void process(float& left, float& right) {
    FxEngine& c = engine_;
    const Line& ap1 = lines_[0];
    const Line& ap2 = lines_[1];
    const Line& ap3 = lines_[2];
    const Line& ap4 = lines_[3];
    const Line& dap1a = lines_[4];
    const Line& dap1b = lines_[5];
    const Line& del1 = lines_[6];
    const Line& dap2a = lines_[7];
    const Line& dap2b = lines_[8];
    const Line& del2 = lines_[9];
    const float kap = diffusion_;
    const float klp = lpCoefficient_;
    const float krt = time_;
    float wet;
    float apout = 0.0f;
    c.start();

    // 1. Optionally smear the first all-pass by rewriting part of it from
    // an LFO-swept position (Elements, Clouds).
    if (ap1Depth_ > 0.0f) {
      c.interpolate(ap1, 10.0f * scale_, 0, ap1Depth_ * scale_, 1.0f);
      c.write(ap1, static_cast<int32_t>(100 * scale_), 0.0f);
    }

    // 2. Input diffusion: four all-passes in series.
    c.read(left + right, inputGain_);
    c.readTail(ap1, kap);
    c.writeAllPass(ap1, -kap);
    c.readTail(ap2, kap);
    c.writeAllPass(ap2, -kap);
    c.readTail(ap3, kap);
    c.writeAllPass(ap3, -kap);
    c.readTail(ap4, kap);
    c.writeAllPass(ap4, -kap);
    c.write(apout);

    // 3. First half of the loop, fed by the end of the second half (del2).
    c.load(apout);
    c.interpolate(del2, del2Tap_ * scale_, 1, del2Depth_ * scale_, krt);
    c.lp(lpDecay1_, klp);
    c.readTail(dap1a, -kap);
    c.writeAllPass(dap1a, kap);
    c.readTail(dap1b, kap);
    c.writeAllPass(dap1b, -kap);
    c.write(del1, 2.0f);
    c.write(wet, 0.0f);
    left += (wet - left) * amount_;

    // 4. Second half, fed by the end of the first (del1).
    c.load(apout);
    if (del1Depth_ > 0.0f) {
      c.interpolate(del1, del1Tap_ * scale_, 0, del1Depth_ * scale_, krt);
    } else {
      c.readTail(del1, krt);
    }
    c.lp(lpDecay2_, klp);
    c.readTail(dap2a, kap);
    c.writeAllPass(dap2a, -kap);
    c.readTail(dap2b, -kap);
    c.writeAllPass(dap2b, kap);
    c.write(del2, 2.0f);
    c.write(wet, 0.0f);
    right += (wet - right) * amount_;
  }

 private:
  typedef FxEngine::Delay Line;

  FxEngine engine_;
  Line lines_[10];
  Preset preset_ = ELEMENTS;
  float scale_ = 1.0f;
  float lpRateRatio_ = 1.0f;
  float del2Tap_ = 6211.0f, del2Depth_ = 100.0f;
  float del1Tap_ = 0.0f, del1Depth_ = 0.0f;
  float ap1Depth_ = 80.0f;

  float amount_ = 0.5f;
  float inputGain_ = 0.2f;
  float time_ = 0.7f;
  float diffusion_ = 0.625f;
  float lp_ = 0.7f;
  float lpCoefficient_ = 0.7f;
  float lpDecay1_ = 0.0f, lpDecay2_ = 0.0f;
};

}  // namespace pt

#endif  // PT_FX_REVERB_H_
