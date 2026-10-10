// partials — sample/audio_buffer.h
//
// AudioBuffer: fixed-size audio storage you record into, load files into
// and read back at fractional positions. Pure Data's array: write() is
// tabwrite~, read() is tabread4~, and decodeWav() (sample/wav.h) is
// soundfiler.
//
// How it works
// - One allocation holds every channel, one after another ("planar"), not
//   interleaved L R L R as in a WAV file. Each channel is wrapped in guard
//   points for the 4-point Hermite read, which needs one sample before the
//   read position and two after:
//
//     [guard | frame 0 .. frame N-1 | guard guard]   channel 0
//     [guard | frame 0 .. frame N-1 | guard guard]   channel 1 ...
//
// - One-shot (the default): the guards are silence, so reads at either
//   end fade towards zero rather than reading garbage.
// - Looping: the guards hold copies of the other end (frame N-1 before,
//   frames 0 and 1 after), so a read across the loop point is seamless.
//   write() keeps those copies up to date, so you can record into a loop
//   while it plays.
//
// Positions are in frames, as in tabread4~, and are doubles. A float holds
// only 24 bits of precision: at frame 480,000 (10 s at 48 kHz) its smallest
// step is 1/32 of a frame, which roughens slow playback of long samples.
// A double keeps sub-sample accuracy for hours. Keep playback positions in
// doubles too.
//
// Only init() allocates memory. Call it from onSampleRateChange() or the
// interface thread, never from process(); everything else is safe there.

#ifndef PT_SAMPLE_AUDIO_BUFFER_H_
#define PT_SAMPLE_AUDIO_BUFFER_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "pt/core/math.h"

namespace pt {

class AudioBuffer {
 public:
  // Allocate `frames` frames of silence per channel. The sample rate is
  // the rate the audio is (or will be) recorded at, not necessarily the
  // host's: play back at sampleRate() / hostRate frames per sample to keep
  // the original pitch.
  void init(size_t frames, int channels = 1, float sampleRate = 48000.0f) {
    frames_ = frames;
    channels_ = std::max(channels, 1);
    sampleRate_ = sampleRate;
    stride_ = frames + 3;  // one guard before, two after
    data_.assign(stride_ * channels_, 0.0f);
    looping_ = false;
  }

  size_t frames() const { return frames_; }
  int channels() const { return channels_; }
  float sampleRate() const { return sampleRate_; }
  bool looping() const { return looping_; }

  // Pointer to frame 0 of a channel (just past its leading guard). Channels
  // past the last one read the last, so a mono buffer serves as "right" too.
  // After writing through this pointer, call setLooping() again to refresh
  // the guards.
  float* channel(int c) { return &data_[index(c)]; }
  const float* channel(int c) const { return &data_[index(c)]; }

  // tabwrite~: store one sample. Out-of-range frames are ignored.
  void write(int c, size_t frame, float x) {
    if (frame >= frames_) return;
    channel(c)[frame] = x;
    // Only the first two and the last frame have copies in the guards.
    if (looping_ && (frame < 2 || frame + 1 == frames_)) wrapGuards(c);
  }

  // tabread4~: Hermite read at a fractional frame. Looping buffers wrap
  // the position (both directions); one-shot buffers clamp it to the ends.
  float read(int c, double frame) const {
    if (frames_ == 0) return 0.0f;
    const double n = static_cast<double>(frames_);
    if (looping_) {
      frame -= std::floor(frame / n) * n;
      if (frame >= n) frame = 0.0;  // rounding can land exactly on n
    } else {
      frame = std::min(std::max(frame, 0.0), n - 1.0);
    }
    // Split into a whole frame and a fraction, then let interpolateHermite
    // read the four points around that frame (size 1: the index is the
    // fraction itself). Doing the split in double is what keeps the
    // precision on long buffers.
    size_t i = static_cast<size_t>(frame);
    float frac = static_cast<float>(frame - static_cast<double>(i));
    return interpolateHermite(channel(c) + i, frac, 1.0f);
  }

  // One-shot (silent guards) or looping (wrapped guards).
  void setLooping(bool loop) {
    looping_ = loop;
    for (int c = 0; c < channels_; ++c) {
      if (loop) {
        wrapGuards(c);
      } else {
        float* p = channel(c);
        p[-1] = p[frames_] = p[frames_ + 1] = 0.0f;
      }
    }
  }

  // Zero frames [begin, end) on every channel. Wiping a long buffer in one
  // go can take longer than one audio block; call this a chunk at a time
  // from process() instead, as with DelayLine::clear().
  void clear(size_t begin, size_t end) {
    end = std::min(end, frames_);
    if (begin >= end) return;
    for (int c = 0; c < channels_; ++c) {
      std::fill(channel(c) + begin, channel(c) + end, 0.0f);
      if (looping_ && (begin < 2 || end == frames_)) wrapGuards(c);
    }
  }

 private:
  size_t index(int c) const {
    c = std::min(std::max(c, 0), channels_ - 1);
    return static_cast<size_t>(c) * stride_ + 1;
  }

  // Copy the far ends into the guards. The modulo only matters for buffers
  // of one frame, where "frame 1" is frame 0 again.
  void wrapGuards(int c) {
    if (frames_ == 0) return;
    float* p = channel(c);
    p[-1] = p[frames_ - 1];
    p[frames_] = p[0];
    p[frames_ + 1] = p[1 % frames_];
  }

  std::vector<float> data_;
  size_t frames_ = 0, stride_ = 0;
  int channels_ = 1;
  float sampleRate_ = 48000.0f;
  bool looping_ = false;
};

}  // namespace pt

#endif  // PT_SAMPLE_AUDIO_BUFFER_H_
