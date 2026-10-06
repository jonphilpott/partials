// mutablelib — dynamics/vactrol.h
//
// Vactrol: a detailed model of a vactrol (an LED shining on a
// light-dependent resistor), the part behind Buchla-style low-pass gates.
// Feed it a control signal; it returns the gain and filter cutoff a real
// vactrol would give, to drive your own VCA and filter. Streams' "vactrol"
// mode.
//
// Compared with the envelope inside LowPassGate (from Plaits), this model
// adds:
// - Memory. A vactrol that has been lit a lot becomes more sensitive (it
//   reacts faster), and recovers over about a minute.
// - Second-order response, with a little overshoot on fast rises.
// - A saturating ("Gompertz") curve from light to gain, so the gain snaps
//   open and has a long, soft tail.
// - A plucked mode: each rising edge "plucks" the vactrol, for repeatable
//   percussive decays.
//
// How it works: the LED's light (the input) charges a first-order lag
// (state0) and a second lag (state1), with attack and decay rates chosen by
// whether the light is rising or falling and by how sensitised the cell is
// (state2). The lag state then goes through the saturating curve to give
// the gain; the cutoff follows the square of the state.
//
// Original: Streams, fixed-point at 31.089 kHz, driving an analog VCA and
// filter. This port is float, with time constants set in seconds, so it
// runs at any rate. It returns gain and cutoff (0..1) instead of driving
// hardware.
//
// Derived from Streams, Copyright 2014 Emilie Gillet. MIT licence.

#ifndef ML_DYNAMICS_VACTROL_H_
#define ML_DYNAMICS_VACTROL_H_

#include <cmath>

#include "ml/core/math.h"

namespace ml {

class Vactrol {
 public:
  void init(float sampleRate) {
    sampleRate_ = sampleRate;
    for (float& s : state_) s = 0.0f;
    excite_ = 0.0f;
    gate_ = false;
    gain_ = cutoff_ = 0.0f;
    // Streams' fixed rates, per sample at 31089 Hz, converted to seconds.
    const float ratio = 31089.0f / sampleRate;
    overshootCoefficient_ = rescaleCoefficient(67976239.0f / 2147483648.0f, ratio);
    sensitiseCoefficient_ = rescaleCoefficient(138132.0f / 2147483648.0f, ratio);
    desensitiseCoefficient_ = rescaleCoefficient(1151.0f / 2147483648.0f, ratio);
    update();
  }

  // Attack and decay times in seconds. Streams' range: attack 10-500 ms,
  // decay 50 ms-5 s.
  void setAttack(float seconds) { attack_ = seconds; update(); }
  void setDecay(float seconds) { decay_ = seconds; update(); }
  // Plucked mode: each rising edge of the input starts a full decay, like
  // striking a low-pass gate. Off: the vactrol follows the input level.
  void setPlucked(bool on) { plucked_ = on; update(); }

  // control: the light level, 0..1 (negative values count as 0). Returns
  // the gain (0..1); cutoff() then holds the matching filter cutoff, 0..1.
  float process(float control) {
    float excite = control > 0.0f ? control : 0.0f;
    if (plucked_) {
      processPlucked(excite);
    } else {
      processFollow(excite);
    }
    return gain_;
  }

  float gain() const { return gain_; }
  // 0..1. Map to Hz however suits your filter, e.g.
  // 20 * std::pow(1000.f, cutoff) for 20 Hz to 20 kHz.
  float cutoff() const { return cutoff_; }

 private:
  void processFollow(float excite) {
    // 1. Smooth falling edges of the input, so a short trigger still lights
    // the LED long enough for the cell to react: the input can be a
    // trigger or a slow CV.
    float error = excite - excite_;
    excite_ += error * (error > 0.0f ? 0.5f : 2.0f * decay_c_);
    // Full light (1.0) drives the cell to exactly full gain. (Streams
    // scaled its input by the filter settings, typically to about half.)
    float input = excite_ * 0.5f;

    // 2. Overshoot helper: a fast lag of the input.
    state_[3] += (input - state_[3]) * overshootCoefficient_;

    // 3. First- and second-order lags. Rising: slower when the cell is not
    // sensitised; falling: slow tail.
    error = input - state_[0];
    float c;
    if (error > 0.0f) {
      if (state_[1] > 0.0f) {
        c = attack_c_ * (1.0f + (1.0f - state_[2]) * (255.0f / 64.0f));
      } else {
        c = fastAttack_c_;
      }
    } else {
      c = state_[1] < 0.0f ? decay_c_ : fastDecay_c_;
    }
    state_[0] += error * c;
    state_[1] += (error - state_[1]) * c;

    // 4. Memory: brightly lit cells become sensitised within about a
    // second, and recover over a minute.
    float sensitivity = state_[0] > 0.125f ? 1.0f : state_[0] * 8.0f;
    error = sensitivity - state_[2];
    state_[2] += error * (error > 0.0f ? sensitiseCoefficient_ : desensitiseCoefficient_);

    // 5. Saturating response, with a little overshoot.
    float index = clamp(state_[0] * 0.5f + state_[3] * state_[1], 0.0f, 0.4999f);
    gain_ = index < 0.25f ? gompertz(index * 4.0f) : 1.0f;
    cutoff_ = std::fmin(index * 4.0f, 1.0f);
    cutoff_ *= cutoff_;
  }

  void processPlucked(float excite) {
    // 1. Schmitt trigger on the input (thresholds as in Streams).
    if (!gate_) {
      if (excite > kThreshold) {
        gate_ = true;
        state_[0] = state_[1] = 1.0f;
      }
    } else if (excite < kThreshold * 0.5f) {
      gate_ = false;
    }
    // 2. Two decaying pulses: a fast one for the filter, a slower one for
    // the VCA.
    state_[0] -= state_[0] * fastDecay_c_;
    state_[1] -= state_[1] * decay_c_;
    float error = state_[0] - state_[2];
    state_[2] += error * (error > 0.0f ? fastAttack_c_ : fastDecay_c_);
    // 3. VCA envelope; the tail lengthens as it gets quieter.
    error = state_[1] - state_[3];
    float c = error > 0.0f ? fastAttack_c_ : decay_c_;
    c = c * 0.5f + c * std::fabs(error);
    state_[3] += error * c;
    cutoff_ = state_[2];
    gain_ = gompertz(state_[3] * 0.375f);
  }

  // Light-to-gain curve: an S-shaped (Gompertz) curve, normalised to 0..1.
  static float gompertz(float x) {
    static const float kMin = curve(0.0f), kMax = curve(1.0f);
    return (curve(clamp(x, 0.0f, 1.0f)) - kMin) / (kMax - kMin);
  }
  static float curve(float x) {
    float y = std::exp(-10.0f * std::exp(-13.0f * x));
    return y + 0.1f * std::pow(y, 0.05f);
  }

  // One-pole coefficient for a time constant in seconds.
  float coefficient(float seconds) const {
    return 1.0f - std::exp(-1.0f / (seconds * sampleRate_));
  }

  void update() {
    // As in Streams: the "fast" rates are 10x the normal ones; the decay is
    // doubled in follow mode, and the fast attack sharpened when plucked.
    attack_c_ = coefficient(attack_);
    fastAttack_c_ = coefficient(attack_ * 0.1f);
    decay_c_ = coefficient(decay_);
    fastDecay_c_ = coefficient(decay_ * 0.1f);
    if (plucked_) {
      fastAttack_c_ = std::fmin(fastAttack_c_ * 16.0f, 1.0f);
    } else {
      decay_c_ *= 0.5f;
    }
  }

  static constexpr float kThreshold = 0.1f;  // Streams: about 0.83 V of 10 V

  float sampleRate_ = 48000.0f;
  float attack_ = 0.01f, decay_ = 0.5f;
  bool plucked_ = false;

  float attack_c_ = 0.0f, fastAttack_c_ = 0.0f, decay_c_ = 0.0f, fastDecay_c_ = 0.0f;
  float overshootCoefficient_ = 0.0f, sensitiseCoefficient_ = 0.0f, desensitiseCoefficient_ = 0.0f;
  float state_[4] = {0, 0, 0, 0};
  float excite_ = 0.0f;
  bool gate_ = false;
  float gain_ = 0.0f, cutoff_ = 0.0f;
};

}  // namespace ml

#endif  // ML_DYNAMICS_VACTROL_H_
