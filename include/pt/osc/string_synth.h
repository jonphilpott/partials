// partials — osc/string_synth.h
//
// StringSynthOscillator: a "divide-down" oscillator like those in string
// machines and combo organs. One master oscillator gives saw and square
// waves at four octaves at once (Plaits' string machine and chord engines
// use it), mixed by a "registration", as on an organ's stops. Follow it
// with an Ensemble for the classic string-machine sound.
//
// How it works: a single phase counter runs from 0 to 8 per cycle of the
// lowest octave. Saws at 1x, 2x, 4x and 8x the pitch are all read from
// that one counter (like a chain of frequency dividers), so they stay
// perfectly locked. A square at one octave is the difference of two saws
// an octave apart, so the registration's saw and square mix becomes four
// saw gains. Every saw reset gets a PolyBLEP correction.
//
// Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.

#ifndef PT_OSC_STRING_SYNTH_H_
#define PT_OSC_STRING_SYNTH_H_

#include "pt/core/math.h"
#include "pt/osc/polyblep.h"

namespace pt {

class StringSynthOscillator {
 public:
  void init(float sampleRate) {
    sampleTime_ = 1.0f / sampleRate;
    phase_ = 0.0f;
    nextSample_ = 0.0f;
    segment_ = 0;
    setRegistration(0.0f);
  }

  // Pitch of the lowest octave, in Hz.
  void setFrequency(float hz) {
    hz_ = hz;
    update();
  }
  // Seven registration levels (0..1 each), low to high: saw 1x, square 1x,
  // saw 2x, square 2x, saw 4x, square 4x, saw 8x.
  void setRegistration(const float r[7]) {
    for (int i = 0; i < 7; ++i) registration_[i] = r[i];
    update();
  }
  // 0..1. One knob sweeping Plaits' string-machine presets, from a single
  // saw through fuller saw and saw-plus-square mixes to a single square.
  void setRegistration(float x) {
    static const float kPresets[11][6] = {
        {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // saw
        {0.5f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f},  // saw + saw
        {0.4f, 0.0f, 0.2f, 0.0f, 0.4f, 0.0f},  // full saw
        {0.3f, 0.0f, 0.0f, 0.3f, 0.0f, 0.4f},  // full saw + square hybrid
        {0.3f, 0.0f, 0.0f, 0.0f, 0.0f, 0.7f},  // saw + high square harmonic
        {0.2f, 0.0f, 0.0f, 0.2f, 0.0f, 0.6f},  // weird hybrid
        {0.0f, 0.2f, 0.1f, 0.0f, 0.2f, 0.5f},  // saw-square, high harmonics
        {0.0f, 0.3f, 0.0f, 0.3f, 0.0f, 0.4f},  // square, high harmonics
        {0.0f, 0.4f, 0.0f, 0.3f, 0.0f, 0.3f},  // full square
        {0.0f, 0.5f, 0.0f, 0.5f, 0.0f, 0.0f},  // square + square
        {0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // square
    };
    float p = clamp(x, 0.0f, 1.0f) * (11 - 1.001f);
    int i = static_cast<int>(p);
    float frac = p - static_cast<float>(i);
    float r[7];
    for (int k = 0; k < 6; ++k) r[k] = crossfade(kPresets[i][k], kPresets[i + 1][k], frac);
    r[6] = 0.0f;
    setRegistration(r);
  }
  // Output level.
  void setGain(float g) {
    gain_ = g;
    update();
  }

  // Returns the mix (about ±1 at gain 1), one sample late.
  float process() {
    if (silent_) return 0.0f;
    float thisSample = nextSample_;
    float nextSample = 0.0f;
    phase_ += frequency_;
    int nextSegment = static_cast<int>(phase_);
    if (nextSegment != segment_) {
      // Which of the four saws reset at this segment boundary?
      float discontinuity = 0.0f;
      if (nextSegment == 8) {
        phase_ -= 8.0f;
        nextSegment -= 8;
        discontinuity -= saw8_;
      }
      if ((nextSegment & 3) == 0) discontinuity -= saw4_;
      if ((nextSegment & 1) == 0) discontinuity -= saw2_;
      discontinuity -= saw1_;
      if (discontinuity != 0.0f) {
        float t = (phase_ - static_cast<float>(nextSegment)) / frequency_;
        thisSample += thisBlepSample(t) * discontinuity;
        nextSample += nextBlepSample(t) * discontinuity;
      }
    }
    segment_ = nextSegment;
    nextSample += (phase_ - 4.0f) * saw8_ * 0.125f;
    nextSample += (phase_ - static_cast<float>(segment_ & 4) - 2.0f) * saw4_ * 0.25f;
    nextSample += (phase_ - static_cast<float>(segment_ & 6) - 1.0f) * saw2_ * 0.5f;
    nextSample += (phase_ - static_cast<float>(segment_ & 7) - 0.5f) * saw1_;
    nextSample_ = nextSample;
    return 2.0f * thisSample;
  }

 private:
  void update() {
    // 1. Very high notes: shift everything down 1-2 octaves and drop the
    // top registers, rather than alias.
    float frequency = hz_ * sampleTime_ * 8.0f;
    int shift = 0;
    while (frequency > 0.5f) {
      shift += 2;
      frequency *= 0.5f;
    }
    silent_ = shift >= 8;
    frequency_ = frequency;
    float r[7] = {0, 0, 0, 0, 0, 0, 0};
    for (int i = shift; i < 7; ++i) r[i] = registration_[i - shift];

    // 2. Registration to saw gains (a square = saw minus the saw an octave
    // up, scaled).
    saw8_ = (r[0] + 2.0f * r[1]) * gain_;
    saw4_ = (r[2] - r[1] + 2.0f * r[3]) * gain_;
    saw2_ = (r[4] - r[3] + 2.0f * r[5]) * gain_;
    saw1_ = (r[6] - r[5]) * gain_;
  }

  float sampleTime_ = 1.0f / 48000.0f;
  float hz_ = 110.0f, gain_ = 1.0f;
  float registration_[7] = {1, 0, 0, 0, 0, 0, 0};
  float frequency_ = 0.01f;
  bool silent_ = false;
  float saw8_ = 0.0f, saw4_ = 0.0f, saw2_ = 0.0f, saw1_ = 0.0f;
  float phase_ = 0.0f, nextSample_ = 0.0f;
  int segment_ = 0;
};

}  // namespace pt

#endif  // PT_OSC_STRING_SYNTH_H_
