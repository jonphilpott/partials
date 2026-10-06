// partials — drums/fm_drum.h
//
// FmDrum: a sine-wave drum with pitch envelopes, noise and overdrive, from
// Peaks' "FM drum" mode. Kicks, toms, zaps and noisy snares from one
// voice, with an X/Y "morph" control that sweeps through Gillet's presets.
//
// How it works
// - A sine oscillator is swept down in pitch by two envelopes: a
//   configurable FM envelope (`fmAmount`, part of `decay`) and a short
//   fixed "aux" envelope that adds a thump to low notes.
// - The output also feeds back into the pitch, which roughens it slightly
//   as it gets louder.
// - An amplitude envelope with an exponential shape shapes the hit.
// - `noise` above 0.5 mixes in white noise (snare-like); below 0.5 it
//   applies a tanh overdrive instead (harder kicks).
//
// Original: fixed-point (16/32-bit integers) at 48 kHz, with lookup tables
// for pitch, envelope times and curves. This port uses float and computes
// those curves directly, in seconds and Hz, so it runs at any sample rate.
//
// Derived from Peaks, Copyright 2013 Emilie Gillet. MIT licence.

#ifndef PT_DRUMS_FM_DRUM_H_
#define PT_DRUMS_FM_DRUM_H_

#include <cmath>

#include "pt/core/math.h"
#include "pt/core/random.h"
#include "pt/core/units.h"
#include "pt/osc/sine.h"

namespace pt {

class FmDrum {
 public:
  void init(float sampleRate) {
    sampleRate_ = sampleRate;
    rateRatio_ = 48000.0f / sampleRate;
    // Peaks updated the pitch every 4 samples at 48 kHz (12 kHz).
    pitchPeriod_ = static_cast<int>(sampleRate / 12000.0f + 0.5f);
    if (pitchPeriod_ < 1) pitchPeriod_ = 1;
    pitchCounter_ = 0;
    phase_ = 0.0f;
    fmEnvPhase_ = amEnvPhase_ = auxEnvPhase_ = 1.0f;  // finished: silent
    increment_ = 0.0f;
    previousSample_ = 0.0f;
    // The aux envelope always lasts about 20 ms.
    auxIncrement_ = 4473924.0f / 4294967296.0f * rateRatio_;
    sineTable();
    update();
  }

  // Pitch in Hz, about 33 Hz (C1) to 1047 Hz (C6).
  void setFrequency(float hz) {
    // Peaks' knob covered MIDI notes 24..96; the aux envelope's strength
    // depends on the knob position, so work it out from the pitch.
    note_ = 69.0f + 12.0f * std::log2(hz / 440.0f);
    knob_ = clamp((note_ - 24.0f) / 72.0f, 0.0f, 1.0f);
  }
  // 0..1. Depth of the pitch sweep (up to 96 semitones).
  void setFmAmount(float x) { fmAmount_ = clamp(x, 0.0f, 1.0f) * 96.0f; }
  // 0..1. Length of the amplitude and FM envelopes.
  void setDecay(float x) { decay_ = clamp(x, 0.0f, 1.0f); update(); }
  // 0..1. Below 0.5: overdrive (most at 0). Above 0.5: noise (most at 1).
  void setNoise(float x) {
    x = clamp(x, 0.0f, 1.0f);
    float n = x >= 0.5f ? (x - 0.5f) * 2.0f : 0.0f;
    noise_ = n * n * 0.625f;
    float o = x <= 0.5f ? (0.5f - x) * 2.0f : 0.0f;
    overdrive_ = o * o;
  }

  // Sweep all four parameters through Peaks' presets on an x/y pad
  // (0..1 each). snare = false: kicks and toms; true: snares and zaps.
  void morph(float x, float y, bool snare) {
    // Rows: {frequency knob, fm amount, decay, noise}, in pairs (y = 0,
    // y = 1) for five x positions.
    static const float kBassDrum[10][4] = {
        {4096, 0, 65535, 32768}, {12288, 0, 65535, 32768},
        {8192, 4096, 49512, 32768}, {8192, 16384, 40960, 32768},
        {10240, 4096, 24576, 32768}, {10240, 16384, 24576, 16384},
        {8192, 8192, 32768, 16384}, {8192, 24576, 49152, 8192},
        {4096, 16384, 40960, 16384}, {8192, 24576, 49152, 0}};
    static const float kSnareDrum[10][4] = {
        {24576, 0, 24576, 36864}, {24576, 0, 16384, 65535},
        {28672, 0, 16384, 36864}, {28672, 0, 16384, 65535},
        {20488, 0, 32768, 57344}, {28672, 0, 24576, 65535},
        {20488, 0, 24576, 65535}, {28672, 0, 32768, 65535},
        {20488, 65535, 16384, 0}, {65535, 0, 8192, 32768}};
    const float(*map)[4] = snare ? kSnareDrum : kBassDrum;
    // Bilinear interpolation between the four nearest presets.
    float xs = clamp(x, 0.0f, 0.9999f) * 4.0f;
    int xi = static_cast<int>(xs);
    float xf = xs - xi;
    float p[4];
    for (int i = 0; i < 4; ++i) {
      float e = crossfade(map[2 * xi][i], map[2 * xi + 2 > 9 ? 9 : 2 * xi + 2][i], xf);
      float f = crossfade(map[2 * xi + 1][i], map[2 * xi + 3 > 9 ? 9 : 2 * xi + 3][i], xf);
      p[i] = crossfade(e, f, clamp(y, 0.0f, 1.0f)) / 65535.0f;
    }
    knob_ = p[0];
    note_ = 24.0f + 72.0f * p[0];
    setFmAmount(p[1]);
    setDecay(p[2]);
    setNoise(p[3]);
  }

  void seed(uint32_t s) { random_.seed(s); }

  void trigger() {
    fmEnvPhase_ = amEnvPhase_ = auxEnvPhase_ = 0.0f;
    // A tiny phase offset proportional to the FM amount, as in Peaks.
    phase_ = 16383.0f * (fmAmount_ * 128.0f) / 65536.0f / 4294967296.0f;
  }

  float process() {
    // 1. Advance the envelope phases (0 = start, 1 = finished).
    fmEnvPhase_ = std::fmin(fmEnvPhase_ + fmIncrement_, 1.0f);
    auxEnvPhase_ = std::fmin(auxEnvPhase_ + auxIncrement_, 1.0f);

    // 2. Pitch, at the control rate: base note + FM sweep + aux thump +
    // output feedback, in semitones.
    if (++pitchCounter_ >= pitchPeriod_) {
      pitchCounter_ = 0;
      float auxStrength = knob_ <= 0.25f ? 1024.0f : (knob_ <= 0.5f ? 2048.0f - knob_ * 4096.0f : 0.0f);
      float semitones = note_ + envelope(fmEnvPhase_) * fmAmount_ +
                        envelope(auxEnvPhase_) * auxStrength / 64.0f + previousSample_ * 4.0f;
      semitones = std::fmin(semitones, 127.99f);
      increment_ = midiToHz(semitones) / sampleRate_;
    }
    phase_ += increment_;
    phase_ -= static_cast<float>(static_cast<int>(phase_));

    // 3. Sine, plus noise.
    float mix = sineFromPhase(phase_);
    if (noise_ > 0.0f) mix = crossfade(mix, random_.bipolar(), noise_);

    // 4. Amplitude envelope, then overdrive.
    amEnvPhase_ = std::fmin(amEnvPhase_ + amIncrement_, 1.0f);
    mix *= envelope(amEnvPhase_);
    if (overdrive_ > 0.0f) {
      static const float kTanh5 = std::tanh(5.0f);
      mix = crossfade(mix, std::tanh(5.0f * mix) / kTanh5, overdrive_);
    }
    previousSample_ = mix;
    return mix;
  }

 private:
  // The decaying envelope shape: 1 at phase 0, 0 at phase 1, falling
  // fast at first (1 - Peaks' "expo" curve).
  static float envelope(float phase) {
    static const float kNorm = 1.0f / (1.0f - std::exp(-4.0f));
    return 1.0f - (1.0f - std::exp(-4.0f * phase)) * kNorm;
  }

  // Envelope speed (fraction of the envelope per sample) for a 0..1 time
  // knob: Peaks' curve from 0.5 ms to 8 s, at 48 kHz, converted.
  float envelopeIncrement(float k) const {
    const double gamma = 0.175;
    const double maxIncrement = 1.0 / (0.0005 * 48000.0);
    const double minIncrement = 1.0 / (8.0 * 48000.0);
    double a = std::pow(maxIncrement, -gamma), b = std::pow(minIncrement, -gamma);
    double rate = a + (b - a) * k;
    return static_cast<float>(std::pow(rate, -1.0 / gamma) * rateRatio_);
  }

  void update() {
    amIncrement_ = envelopeIncrement(0.25f + decay_ * 0.5f);
    fmIncrement_ = envelopeIncrement(0.125f + decay_ * 0.25f);
  }

  float sampleRate_ = 48000.0f, rateRatio_ = 1.0f;
  float note_ = 36.0f, knob_ = 1.0f / 6.0f;
  float fmAmount_ = 0.0f, decay_ = 0.5f, noise_ = 0.0f, overdrive_ = 0.0f;
  float amIncrement_ = 0.0f, fmIncrement_ = 0.0f, auxIncrement_ = 0.0f;

  int pitchPeriod_ = 4, pitchCounter_ = 0;
  float phase_ = 0.0f, increment_ = 0.0f;
  float fmEnvPhase_ = 1.0f, amEnvPhase_ = 1.0f, auxEnvPhase_ = 1.0f;
  float previousSample_ = 0.0f;
  Random random_;
};

}  // namespace pt

#endif  // PT_DRUMS_FM_DRUM_H_
