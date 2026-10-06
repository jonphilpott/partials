// partials — physical/tube.h
//
// Reed and tube: a simple wind-instrument model. Blow noise (or any
// "breath" signal) into it and it produces a reedy, clarinet-like tone at
// the tube's pitch.
//
// Origin: Elements' "blow" exciter path.
//
// How it works
// - A delay line one period long stands in for the air column: pressure
//   waves travel down it and come back.
// - At the mouthpiece, the difference between breath pressure and the
//   returning wave goes through a "reed" curve (a simple quadratic). The
//   reed lets more air through as the pressure difference grows, then
//   chokes it off. That nonlinear switching sustains the oscillation.
// - A one-pole low-pass on the output sets the tone (`timbre`).
// - Like a clarinet it produces mostly odd harmonics.
//
// Sample-rate independence: the loop's one-sample averaging filter and the
// reed's behaviour depend on the sample rate (at 96 kHz the tube jumps to a
// higher mode). So the model always runs at Elements' 32 kHz internally,
// and its output is linearly interpolated to the host rate. The output is
// low-passed by the tone filter, so the interpolation is inaudible.
//
// Derived from Elements, Copyright 2014 Emilie Gillet. MIT licence.

#ifndef PT_PHYSICAL_TUBE_H_
#define PT_PHYSICAL_TUBE_H_

#include <cstdint>
#include <vector>

#include "pt/core/math.h"

namespace pt {

class Tube {
 public:
  static constexpr float kInternalRate = 32000.0f;

  void init(float sampleRate) {
    // Internal steps per host sample, e.g. 2/3 at 48 kHz.
    step_ = kInternalRate / sampleRate;
    phase_ = 0.0f;
    previous_ = current_ = 0.0f;
    delayLine_.assign(2048, 0.0f);  // a power of two: wraps with a bit mask
    delayPtr_ = 0;
    zeroState_ = 0.0f;
    poleState_ = 0.0f;
    setFrequency(220.0f);
  }

  // Sounding pitch in Hz. Below the lowest note that fits, the tube jumps
  // up by octaves.
  //
  // Like a clarinet, a reed tube sounds an octave below its delay loop:
  // the reflection at the mouthpiece inverts the wave, so one full cycle
  // takes two trips round the loop (which is also why it has mostly odd
  // harmonics). Elements' tube therefore sounded an octave below the
  // frequency it was given; here we tune the loop an octave up so `hz` is
  // the pitch you hear.
  void setFrequency(float hz) {
    frequency_ = 2.0f * hz / kInternalRate;
    // The loop's two-tap averaging filter (zeroState_) adds half a sample
    // of delay; take it off so the tube is in tune. (Elements didn't, and
    // was about 10 cents flat.)
    float delay = 1.0f / frequency_ - 0.5f;
    while (delay >= static_cast<float>(delayLine_.size())) delay *= 0.5f;
    delayIntegral_ = static_cast<int32_t>(delay);
    delayFractional_ = delay - static_cast<float>(delayIntegral_);
    updateLpf();
  }
  // 0..1. Higher values scale the breath down: a softer, more muted tone.
  // (Elements fed its resonator damping knob in here.)
  void setDamping(float x) { damping_ = 3.6f - clamp(x, 0.0f, 1.0f) * 1.8f; }
  // 0..1. Brightness of the output.
  void setTimbre(float x) {
    timbre_ = clamp(x, 0.0f, 1.0f);
    updateLpf();
  }

  // breath: excitation (Elements used filtered noise, about ±1).
  // envelope: 0..1 blowing pressure, e.g. from an envelope generator.
  // Returns the tube output scaled by the envelope.
  float process(float breath, float envelope) {
    // Run as many internal 32 kHz steps as fall within this host sample
    // (zero or one above 32 kHz), then interpolate between the last two.
    phase_ += step_;
    while (phase_ >= 1.0f) {
      phase_ -= 1.0f;
      previous_ = current_;
      current_ = tick(breath, envelope);
    }
    return previous_ + (current_ - previous_) * phase_;
  }

 private:
  // One step of the model at 32 kHz.
  float tick(float breath, float envelope) {
    if (envelope > 1.0f) envelope = 1.0f;
    const size_t mask = delayLine_.size() - 1;
    // 1. Pressure difference across the reed. The returning wave
    // (interpolated read) is averaged with the previous one by zeroState_
    // (a simple loss filter).
    float b = breath * damping_ + 0.8f;
    float x0 = delayLine_[(delayPtr_ + delayIntegral_) & mask];
    float x1 = delayLine_[(delayPtr_ + delayIntegral_ + 1) & mask];
    float in = x0 + (x1 - x0) * delayFractional_;
    float pressureDelta = -0.95f * (in * envelope + zeroState_) - b;
    zeroState_ = in;

    // 2. Reed: the opening shrinks as the pressure difference grows.
    float reed = pressureDelta * -0.2f + 0.8f;
    float out = clamp(pressureDelta * reed + b, -5.0f, 5.0f);

    // 3. Send the new wave down the tube (write pointer runs backwards).
    delayLine_[delayPtr_] = out * 0.5f;
    delayPtr_ = (delayPtr_ - 1) & mask;

    // 4. Tone filter.
    poleState_ += lpfCoefficient_ * (out - poleState_);
    return envelope * poleState_;
  }

  void updateLpf() {
    lpfCoefficient_ = frequency_ * (1.0f + timbre_ * timbre_ * 256.0f);
    if (lpfCoefficient_ >= 0.995f) lpfCoefficient_ = 0.995f;
  }

  float step_ = 1.0f;
  float phase_ = 0.0f;
  float previous_ = 0.0f, current_ = 0.0f;
  float frequency_ = 220.0f / kInternalRate;
  float damping_ = 3.6f;
  float timbre_ = 0.5f;
  float lpfCoefficient_ = 0.0f;
  int32_t delayIntegral_ = 0;
  float delayFractional_ = 0.0f;

  std::vector<float> delayLine_;
  size_t delayPtr_ = 0;
  float zeroState_ = 0.0f;
  float poleState_ = 0.0f;
};

}  // namespace pt

#endif  // PT_PHYSICAL_TUBE_H_
