// mutablelib — fx/fx_engine.h
//
// FxEngine: a toolkit for building delay-based effects (reverbs, choruses,
// diffusers, pitch shifters) from many short delay lines sharing one
// buffer. The reverb, diffuser, chorus, ensemble and pitch shifter here are
// all written with it, and you can write your own the same way.
//
// How it works
// - All delay lines live in one circular buffer. A single write pointer
//   moves backwards one step per sample, and each line is a fixed region
//   ahead of it, so advancing one pointer advances every line at once.
// - Processing works like a tiny programmable DSP chip: an "accumulator"
//   holds the signal being built. You load or read values into it, scale
//   and filter it, and write it into delay lines. One effect is a short
//   program run once per sample.
// - Two slow cosine LFOs are built in, for modulating delay reads (chorus,
//   reverb shimmer).
//
// Differences from the original (Gillet's FxEngine template):
// - Delay lines are declared at run time with addDelay(), sized from the
//   sample rate, instead of through nested templates fixed at compile time.
// - Storage is always float. The originals often packed samples into 12
//   or 16 bits to save RAM, which adds a little noise.
// - The accumulator lives in the engine: there is no separate Context.
//
// Typical use:
//   init:     line = engine.addDelay(n); ...; engine.allocate();
//   process:  engine.start(); engine.read(in); ...; engine.write(line, 0.f);
//
// Derived from Clouds/Rings/Elements fx_engine.h, Copyright 2014 Emilie
// Gillet. MIT licence.

#ifndef ML_FX_FX_ENGINE_H_
#define ML_FX_FX_ENGINE_H_

#include <algorithm>
#include <cstdint>
#include <vector>

#include "ml/osc/sine.h"

namespace ml {

class FxEngine {
 public:
  // A delay line: a region of the shared buffer.
  struct Delay {
    int32_t base = 0;
    int32_t length = 0;
  };

  // Pass as the offset to read/write the far end of a line (its oldest
  // sample), as the original's TAIL macro did.
  static const int32_t kTail = -1;

  // 1. Forget all delay lines. Call before declaring a new layout.
  void reset() {
    total_ = 0;
    buffer_.clear();
  }

  // 2. Declare a delay line of `length` samples. Lines are laid out one
  // after another with a one-sample gap, as in the original.
  Delay addDelay(int32_t length) {
    Delay d;
    d.base = total_;
    d.length = length;
    total_ += length + 1;
    return d;
  }

  // 3. Allocate the buffer for everything declared (rounded up to a power
  // of two so the index wraps with a bit mask) and clear it.
  void allocate() {
    int32_t size = 1;
    while (size < total_) size <<= 1;
    buffer_.assign(size, 0.0f);
    mask_ = size - 1;
    clear();
  }

  // Silence every line.
  void clear() {
    std::fill(buffer_.begin(), buffer_.end(), 0.0f);
    writePtr_ = 0;
  }

  // LFO rate in cycles per sample (Hz / sampleRate). The LFOs are only
  // updated every 32 samples, which is plenty for slow modulation.
  void setLfoFrequency(int index, float frequency) {
    lfo_[index].initApproximate(frequency * 32.0f);
  }

  // Begin one sample: advance all lines, clear the accumulator.
  void start() {
    writePtr_ = (writePtr_ - 1) & mask_;
    accumulator_ = 0.0f;
    previousRead_ = 0.0f;
    // Step the LFOs once every 32 samples (when the write pointer is a
    // multiple of 32, as in the original); in between, hold their value.
    if ((writePtr_ & 31) == 0) {
      lfoValue_[0] = lfo_[0].next();
      lfoValue_[1] = lfo_[1].next();
    } else {
      lfoValue_[0] = lfo_[0].value();
      lfoValue_[1] = lfo_[1].value();
    }
  }

  // --- Accumulator operations (all apply to the current sample) ---------

  // accumulator = value
  void load(float value) { accumulator_ = value; }
  // accumulator += value * scale
  void read(float value, float scale = 1.0f) { accumulator_ += value * scale; }
  // value = accumulator, then accumulator *= scale
  void write(float& value, float scale = 1.0f) {
    value = accumulator_;
    accumulator_ *= scale;
  }

  // accumulator += line[offset] * scale. offset counts samples into the
  // line; kTail reads its oldest sample.
  void read(const Delay& d, int32_t offset, float scale) {
    float r = buffer_[index(d, offset)];
    previousRead_ = r;
    accumulator_ += r * scale;
  }
  void readTail(const Delay& d, float scale) { read(d, kTail, scale); }

  // line[offset] = accumulator, then accumulator *= scale.
  void write(const Delay& d, int32_t offset, float scale) {
    buffer_[index(d, offset)] = accumulator_;
    accumulator_ *= scale;
  }
  void write(const Delay& d, float scale) { write(d, 0, scale); }

  // The write half of an all-pass filter. Read the line's tail with gain g
  // first, then writeAllPass(line, -g): together they form
  //   y = -g*x + delayed,  stored = x + g*delayed.
  void writeAllPass(const Delay& d, float scale) {
    write(d, 0, scale);
    accumulator_ += previousRead_;
  }

  // One-pole low-pass / high-pass of the accumulator through `state`.
  void lp(float& state, float coefficient) {
    state += coefficient * (accumulator_ - state);
    accumulator_ = state;
  }
  void hp(float& state, float coefficient) {
    state += coefficient * (accumulator_ - state);
    accumulator_ -= state;
  }

  // Fractional read with linear interpolation: accumulator += line[offset]
  // * scale, where offset can be a fraction.
  void interpolate(const Delay& d, float offset, float scale) {
    int32_t i = static_cast<int32_t>(offset);
    float frac = offset - static_cast<float>(i);
    float a = buffer_[(writePtr_ + i + d.base) & mask_];
    float b = buffer_[(writePtr_ + i + d.base + 1) & mask_];
    float x = a + (b - a) * frac;
    previousRead_ = x;
    accumulator_ += x * scale;
  }

  // As above, with the offset modulated by LFO `lfo` (0 or 1) by
  // `amplitude` samples.
  void interpolate(const Delay& d, float offset, int lfo, float amplitude, float scale) {
    interpolate(d, offset + amplitude * lfoValue_[lfo], scale);
  }

 private:
  int32_t index(const Delay& d, int32_t offset) const {
    int32_t o = offset == kTail ? d.length - 1 : offset;
    return (writePtr_ + d.base + o) & mask_;
  }

  std::vector<float> buffer_;
  int32_t total_ = 0;
  int32_t mask_ = 0;
  int32_t writePtr_ = 0;
  float accumulator_ = 0.0f;
  float previousRead_ = 0.0f;
  float lfoValue_[2] = {0.0f, 0.0f};
  CosineOscillator lfo_[2];
};

}  // namespace ml

#endif  // ML_FX_FX_ENGINE_H_
