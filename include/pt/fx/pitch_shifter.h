// partials — fx/pitch_shifter.h
//
// Pitch shifter: shifts audio up or down by any ratio in real time, with a
// slightly grainy, "tape-splice" character. Clouds' pitch shifter.
//
// How it works (a "delay-line" or "rotating tape head" shifter)
// - The input is written into a short delay line.
// - A read head moves along it at a different speed from the write head:
//   reading faster than writing raises the pitch, slower lowers it. That
//   is how varispeed tape works.
// - The read head soon runs into the write head, so there are two heads
//   half a window apart, each faded in and out by a triangle window. One
//   always plays while the other jumps back, so the jumps are hidden.
// - `size` sets the window length. Short windows track transients well
//   but sound buzzy; long windows are smoother but smear and echo.
//
// Sample-rate independence: the window is set in time, scaled from Clouds'
// 32 kHz.
//
// Derived from Clouds, Copyright 2014 Emilie Gillet. MIT licence.

#ifndef PT_FX_PITCH_SHIFTER_H_
#define PT_FX_PITCH_SHIFTER_H_

#include "pt/core/math.h"
#include "pt/fx/fx_engine.h"

namespace pt {

class PitchShifter {
 public:
  // Allocates the delay memory: call from onSampleRateChange, not process.
  void init(float sampleRate) {
    scale_ = sampleRate / 32000.0f;
    maxSize_ = 2047.0f * scale_;
    minSize_ = 128.0f * scale_;
    engine_.reset();
    line_ = engine_.addDelay(static_cast<int32_t>(maxSize_ + 0.5f));
    engine_.allocate();
    // Clouds smoothed size changes once per 1 kHz block with coefficient
    // 0.05 (about 20 ms); convert that to a per-sample coefficient.
    sizeCoefficient_ = rescaleCoefficient(0.05f, 1000.0f / sampleRate);
    phase_ = 0.0f;
    size_ = targetSize_ = maxSize_;
  }

  // Frequency ratio: 2 = up an octave, 0.5 = down an octave. Use
  // pt::semitonesToRatio() for intervals.
  void setRatio(float ratio) { ratio_ = ratio; }
  // 0..1. Window length, about 4 ms to 64 ms (curved, more resolution at
  // the short end). Changes are smoothed.
  void setSize(float x) {
    x = clamp(x, 0.0f, 1.0f);
    targetSize_ = minSize_ + (maxSize_ - minSize_) * x * x * x;
  }

  void clear() { engine_.clear(); }

  float process(float in) {
    FxEngine& c = engine_;
    c.start();
    onePole(size_, targetSize_, sizeCoefficient_);

    // 1. Move the read heads relative to the write head. The phase runs
    // 0..1 through the window; its speed sets the pitch change.
    phase_ += (1.0f - ratio_) / size_;
    if (phase_ >= 1.0f) phase_ -= 1.0f;
    if (phase_ <= 0.0f) phase_ += 1.0f;

    // 2. Triangle crossfade between two heads half a window apart: each is
    // silent when it jumps.
    float tri = 2.0f * (phase_ >= 0.5f ? 1.0f - phase_ : phase_);
    float phase = phase_ * size_;
    float half = phase + size_ * 0.5f;
    if (half >= size_) half -= size_;

    float out;
    c.read(in, 1.0f);
    c.write(line_, 0.0f);
    c.interpolate(line_, phase, tri);
    c.interpolate(line_, half, 1.0f - tri);
    c.write(out, 0.0f);
    return out;
  }

 private:
  FxEngine engine_;
  FxEngine::Delay line_;
  float scale_ = 1.0f;
  float maxSize_ = 2047.0f, minSize_ = 128.0f;
  float sizeCoefficient_ = 0.05f;
  float phase_ = 0.0f;
  float ratio_ = 1.0f;
  float size_ = 2047.0f, targetSize_ = 2047.0f;
};

}  // namespace pt

#endif  // PT_FX_PITCH_SHIFTER_H_
