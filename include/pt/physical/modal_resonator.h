// partials — physical/modal_resonator.h
//
// Modal resonator: the sound of a struck or bowed object (string, bar,
// plate, bell, glass) made by summing up to 64 resonant band-pass filters,
// one per vibration "mode". Feed it a short burst (a click, noise, a pluck)
// and it rings.
//
// Origin: Rings' modal resonator, plus Elements' optional bowed modes and
// stereo "side" output.
//
// How it works
// - Each mode is an SVF band-pass tuned to one partial. Partial k sits at
//   k * frequency * stretch, where the stretch grows with `structure`: none
//   for a harmonic string, more for stiff bars, compressed for bells.
// - Each mode's Q sets how long it rings. Higher partials lose Q faster
//   (`brightness` controls how fast), which is how real objects behave.
// - `position` imitates where the object is struck: mode k is weighted by
//   0.5 + 0.5*cos(2*pi*k*position). That is a comb filter applied directly
//   to the mode gains instead of to the audio, which avoids flanging when
//   position is modulated.
// - With bowed modes on, the first 8 modes also get a delay-line loop
//   driven by a nonlinear "bow table" (Elements' banded waveguides), so a
//   continuous bow pressure produces a sustained, scratchy tone.
//
// Cost: up to 64 filters per sample. Coefficients are recomputed at 2 kHz
// (every sampleRate/2000 samples), as the original did once per block.
//
// Sample-rate independence: the original ran at a fixed 48 kHz (Rings) or
// 32 kHz (Elements). Mode frequencies are already relative to the sample
// rate; the Q values are scaled by sampleRate / original rate so decay
// times stay the same in seconds.
//
// Derived from Rings and Elements, Copyright 2015 Emilie Gillet. MIT licence.

#ifndef PT_PHYSICAL_MODAL_RESONATOR_H_
#define PT_PHYSICAL_MODAL_RESONATOR_H_

#include <cmath>
#include <cstddef>

#include "pt/core/math.h"
#include "pt/core/tables.h"
#include "pt/filter/svf.h"
#include "pt/fx/delay_line.h"
#include "pt/osc/sine.h"

namespace pt {

class ModalResonator {
 public:
  static const int kMaxModes = 64;
  static const int kBowedModes = 8;

  void init(float sampleRate) {
    sampleRate_ = sampleRate;
    // 1. Control rate: recompute filters 2000 times a second.
    controlPeriod_ = static_cast<int>(sampleRate / 2000.0f + 0.5f);
    if (controlPeriod_ < 1) controlPeriod_ = 1;
    counter_ = 0;
    // 2. Rate scaling for Q (see header). Rings was tuned at 48 kHz, the
    // bowed modes (from Elements) at 32 kHz.
    qScale_ = sampleRate / 48000.0f;
    bowQScale_ = sampleRate / 32000.0f;
    // 3. Bowed-mode delay lines hold one period of each mode. Elements had
    // 1024 samples at 32 kHz; scale up so low notes still fit at high rates.
    bowDelaySize_ = static_cast<size_t>(1024.0f * bowQScale_);
    for (int i = 0; i < kMaxModes; ++i) modes_[i].init(sampleRate);
    for (int i = 0; i < kBowedModes; ++i) {
      bowFilters_[i].init(sampleRate);
      bowDelays_[i].init(bowDelaySize_);
      bowPeriod_[i] = 1;
    }
    lut4Decades();  // build tables now, not on the audio thread
    lutStiffness();
    previousPosition_ = 0.0f;
    positionIncrement_ = 0.0f;
    bowSignal_ = 0.0f;
    lfoPhase_ = 0.0f;
    numModes_ = 0;
    odd_ = even_ = side_ = 0.0f;
  }

  // Pitch of the fundamental in Hz.
  void setFrequency(float hz) { frequency_ = hz / sampleRate_; }
  // 0..1. Inharmonicity. Below 0.25: bell-like (partials squeezed), 0.25..0.3:
  // harmonic string, above: stiffer bars and plates, near 1: metallic.
  void setStructure(float x) { structure_ = clamp(x, 0.0f, 1.0f); }
  // 0..1. How long the high partials ring relative to the low ones.
  void setBrightness(float x) { brightness_ = clamp(x, 0.0f, 1.0f); }
  // 0..1. Overall decay: 0 is a dead thud, 1 rings for ~30 seconds.
  void setDamping(float x) { damping_ = clamp(x, 0.0f, 1.0f); }
  // 0..1. Excitation/pickup point; 0.5 is the middle (odd harmonics only).
  void setPosition(float x) { position_ = clamp(x, 0.0f, 1.0f); }
  // Number of modes, 1..64. Fewer is cheaper and simpler-sounding.
  void setModes(int n) { resolution_ = n < 1 ? 1 : (n > kMaxModes ? kMaxModes : n); }
  // Elements' bowed modes. Off by default (Rings' sound).
  void setBowedModes(bool on) { bowed_ = on; }
  // Side-channel position LFO (Elements): rate in Hz and offset (0..1).
  void setSideLfo(float hz, float offset) {
    lfoFrequency_ = hz / sampleRate_;
    lfoOffset_ = offset;
  }

  // in: excitation (strikes, noise, audio). bowStrength: 0..1, bow pressure
  // for the bowed modes (ignored unless setBowedModes(true)).
  // Returns the sum of all modes. odd(), even() and side() give the other
  // outputs of this sample.
  float process(float in, float bowStrength = 0.0f) {
    // 1. Control-rate update: retune all filters, and plan a linear ramp of
    // the position over the next period (position is very sensitive to
    // zipper noise).
    if (counter_ == 0) {
      counter_ = controlPeriod_;
      computeFilters();
      positionIncrement_ = (position_ - previousPosition_) / controlPeriod_;
    }
    --counter_;
    previousPosition_ += positionIncrement_;

    // 2. Triangle LFO (0..0.5) that sweeps the side output's pickup point.
    lfoPhase_ += lfoFrequency_;
    if (lfoPhase_ >= 1.0f) lfoPhase_ -= 1.0f;
    float lfo = lfoPhase_ > 0.5f ? 1.0f - lfoPhase_ : lfoPhase_;

    // 3. Per-mode gains from the position: 0.5 + 0.5*cos(2*pi*k*position).
    CosineOscillator amplitudes, sideAmplitudes;
    amplitudes.initApproximate(previousPosition_);
    sideAmplitudes.initApproximate(lfoOffset_ + lfo);

    // 4. Run the modes. They are summed in two groups, odd and even mode
    // numbers (Rings sends these to its two outputs); the side sum uses
    // the LFO-swept gains.
    float input = in * 0.125f;
    float odd = 0.0f, even = 0.0f, side = 0.0f;
    for (int i = 0; i < numModes_; ++i) {
      float s = modes_[i].process(input).bp;
      float a = amplitudes.next();
      if (i & 1) {
        even += s * a;
      } else {
        odd += s * a;
      }
      side += s * sideAmplitudes.next();
    }
    float center = odd + even;
    side_ = side - center;

    // 5. Bowed modes: each is a delay line one period long, closed through
    // a narrow band-pass, all excited by the bow-table nonlinearity. The
    // bow signal is the sum of the loops, fed back next sample.
    if (bowed_) {
      int numBowed = numModes_ < kBowedModes ? numModes_ : kBowedModes;
      float bowSum = 0.0f;
      input += bowSignal_;
      amplitudes.start();
      for (int i = 0; i < numBowed; ++i) {
        float s = 0.99f * bowDelays_[i].read(bowPeriod_[i]);
        bowSum += s;
        Svf& f = bowFilters_[i];
        s = f.process(input + s).bp * f.r();
        bowDelays_[i].write(s);
        center += s * amplitudes.next() * 8.0f;
      }
      bowSignal_ = bowTable(bowSum, bowStrength);
    }

    odd_ = odd;
    even_ = even;
    return center;
  }

  float odd() const { return odd_; }
  float even() const { return even_; }
  // Difference between an LFO-swept pickup and the main one: use
  // (center + side, center - side) for a moving stereo image.
  float side() const { return side_; }

 private:
  // Bow friction curve: steep for small bow/string velocity differences
  // (the bow grips), flat for large ones (it slips). The stick-slip
  // switching is what makes a bowed tone.
  static float bowTable(float x, float velocity) {
    x = 0.13f * velocity - x;
    float bow = std::fabs(x * 6.0f) + 0.75f;
    bow *= bow;
    bow *= bow;
    bow = 0.25f / bow;
    bow = clamp(bow, 0.0025f, 0.245f);
    return x * bow;
  }

  void computeFilters() {
    // 1. Stretch per partial from the structure knob.
    float stiffness = interpolate(lutStiffness(), structure_, 256.0f);
    float harmonic = frequency_;
    float stretchFactor = 1.0f;
    // 2. Base Q: 4 decades of range, scaled to keep decay times constant
    // across sample rates.
    float q = 500.0f * interpolate(lut4Decades(), damping_, 256.0f) * qScale_;
    // 3. Brightness: each partial's Q is the previous one's times qLoss.
    // At very low structure (bell) the brightness range is reduced to avoid
    // clipping.
    float attenuation = 1.0f - structure_;
    attenuation *= attenuation;
    attenuation *= attenuation;
    attenuation *= attenuation;
    float brightness = brightness_ * (1.0f - 0.2f * attenuation);
    float qLoss = brightness * (2.0f - brightness) * 0.85f + 0.15f;
    float qLossDampingRate = structure_ * (2.0f - structure_) * 0.1f;

    numModes_ = 0;
    for (int i = 0; i < resolution_; ++i) {
      // Partials above Nyquist are pinned there and not counted.
      float partial = harmonic * stretchFactor;
      if (partial >= 0.49f) {
        partial = 0.49f;
      } else {
        numModes_ = i + 1;
      }
      modes_[i].setCoefficients(partial, 1.0f + partial * q, TanApprox::Fast);
      if (i < kBowedModes) {
        size_t period = static_cast<size_t>(1.0f / partial);
        while (period >= bowDelaySize_) period >>= 1;  // fold down octaves
        bowPeriod_[i] = period;
        bowFilters_[i].setGQ(modes_[i].g(), 1.0f + partial * 1500.0f * bowQScale_);
      }
      stretchFactor += stiffness;
      // Negative stiffness shrinks so partials never fold below zero;
      // positive stiffness shrinks slowly, adding a few high partials.
      stiffness *= stiffness < 0.0f ? 0.93f : 0.98f;
      // Stops the highest partials decaying too fast.
      qLoss += qLossDampingRate * (1.0f - qLoss);
      harmonic += frequency_;
      q *= qLoss;
    }
  }

  float sampleRate_ = 48000.0f;
  float frequency_ = 220.0f / 48000.0f;
  float structure_ = 0.25f;
  float brightness_ = 0.5f;
  float damping_ = 0.3f;
  float position_ = 0.999f;
  int resolution_ = kMaxModes;
  bool bowed_ = false;
  float lfoFrequency_ = 0.5f / 48000.0f;
  float lfoOffset_ = 0.1f;

  int controlPeriod_ = 24;
  int counter_ = 0;
  int numModes_ = 0;
  float qScale_ = 1.0f, bowQScale_ = 1.5f;
  float previousPosition_ = 0.0f, positionIncrement_ = 0.0f;
  float lfoPhase_ = 0.0f;
  float bowSignal_ = 0.0f;
  float odd_ = 0.0f, even_ = 0.0f, side_ = 0.0f;

  Svf modes_[kMaxModes];
  Svf bowFilters_[kBowedModes];
  DelayLine bowDelays_[kBowedModes];
  size_t bowPeriod_[kBowedModes];
  size_t bowDelaySize_ = 1536;
};

}  // namespace pt

#endif  // PT_PHYSICAL_MODAL_RESONATOR_H_
