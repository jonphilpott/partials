// mutablelib — fx/delay_line.h
//
// A circular buffer you write one sample at a time into and read back from
// any distance in the past, with optional fractional (interpolated) reads.
// The basic part of every echo, comb filter, string model and reverb.
//
// Differences from stmlib's DelayLine<T, N>:
// - The size is chosen at run time in init(), so it can grow with the
//   sample rate (a 20 ms delay needs twice the samples at 96 kHz).
// - The buffer is rounded up to a power of two. Wrapping the index is then
//   a bitwise AND with (size - 1) instead of a slower % division.
//
// Derived from stmlib dsp/delay_line.h, Copyright 2014 Emilie Gillet.
// MIT licence.

#ifndef ML_FX_DELAY_LINE_H_
#define ML_FX_DELAY_LINE_H_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace ml {

class DelayLine {
 public:
  // Allocates room for delays up to maxDelay samples (plus a few guard
  // samples for interpolated reads). Call from init(), never from process().
  void init(size_t maxDelay) {
    size_t size = 1;
    while (size < maxDelay + 4) size <<= 1;
    buffer_.assign(size, 0.0f);
    mask_ = size - 1;
    reset();
  }

  void reset() {
    std::fill(buffer_.begin(), buffer_.end(), 0.0f);
    writeIndex_ = 0;
  }

  // Usable delay in samples.
  size_t size() const { return buffer_.size(); }

  // The write position moves *backwards* through the buffer, so a read
  // "delay samples ago" is simply writeIndex + delay. read(1) returns the
  // most recently written sample. (This matches stmlib and keeps the read
  // arithmetic to one addition.)
  void write(float x) {
    buffer_[writeIndex_] = x;
    writeIndex_ = (writeIndex_ - 1) & mask_;
  }

  float read(size_t delay) const {
    return buffer_[(writeIndex_ + delay) & mask_];
  }

  // Fractional delay, blending the two nearest samples (linear
  // interpolation). Cheap, but slightly dulls high frequencies.
  float read(float delay) const {
    size_t i = static_cast<size_t>(delay);
    float frac = delay - static_cast<float>(i);
    float a = read(i);
    float b = read(i + 1);
    return a + (b - a) * frac;
  }

  // Fractional delay with 4-point Hermite interpolation: brighter and less
  // smeared than linear, for tuned delays such as strings.
  float readHermite(float delay) const {
    size_t i = static_cast<size_t>(delay);
    float f = delay - static_cast<float>(i);
    const float xm1 = read(i - 1);
    const float x0 = read(i);
    const float x1 = read(i + 1);
    const float x2 = read(i + 2);
    const float c = (x1 - xm1) * 0.5f;
    const float v = x0 - x1;
    const float w = c + v;
    const float a = w + v + (x2 - x0) * 0.5f;
    const float bNeg = w + a;
    return (((a * f) - bNeg) * f + c) * f + x0;
  }

  // All-pass filter built on the delay line: passes every frequency at
  // equal level but delays them by different amounts. Used for dispersion
  // (stiff strings) and for diffusing reverbs.
  float allpass(float in, size_t delay, float coefficient) {
    float delayed = read(delay);
    float w = in + coefficient * delayed;
    write(w);
    return -w * coefficient + delayed;
  }

 private:
  std::vector<float> buffer_;
  size_t mask_ = 0;
  size_t writeIndex_ = 0;
};

}  // namespace ml

#endif  // ML_FX_DELAY_LINE_H_
