// partials — dynamics/lpg.h
//
// Low-pass gate (LPG) with its vactrol envelope: a combined VCA and
// low-pass filter that opens and closes together, the "bongo" sound of
// Buchla-style West Coast synthesis. Ping it with trigger() for a natural
// percussive decay, or drive it with a level CV.
//
// Origin: Plaits' output LPG and LPG envelope.
//
// How it works
// - A real LPG uses a vactrol: an LED shining on a light-dependent
//   resistor. The resistor responds fast to light coming on and slowly,
//   with a long tail, to light going off. The envelope here imitates that:
//   a fast rise, then a decay that is short at first and slows as the
//   level falls.
// - The envelope's single state drives three things at once: the gain,
//   the filter cutoff (which falls faster than the gain, since it follows
//   the 4th power of the state), and an "HF bleed" that lets some
//   unfiltered signal through. As a note decays it gets quieter and duller
//   together, which is the point of an LPG.
// - `colour` shifts the balance from pure VCA-like (0) to more filtering
//   and bleed (1).
//
// Cost: the envelope and filter cutoff update 4000 times a second (once
// per Plaits block); the gain ramps smoothly between updates.
//
// Sample-rate independence: Plaits ran at 48 kHz with 12-sample blocks.
// Envelope rates are scaled by the actual update period, and the filter
// cutoff Plaits produced at 48 kHz is converted to Hz and re-warped for
// the actual rate.
//
// Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.

#ifndef PT_DYNAMICS_LPG_H_
#define PT_DYNAMICS_LPG_H_

#include <cmath>

#include "pt/core/math.h"
#include "pt/core/units.h"
#include "pt/filter/svf.h"

namespace pt {

class LowPassGate {
 public:
  void init(float sampleRate) {
    sampleRate_ = sampleRate;
    controlPeriod_ = static_cast<int>(sampleRate / 4000.0f + 0.5f);
    if (controlPeriod_ < 1) controlPeriod_ = 1;
    counter_ = 0;
    filter_.init(sampleRate);
    vactrol_ = 0.0f;
    rampUp_ = false;
    gain_ = 0.0f;
    gainIncrement_ = 0.0f;
    hfBleed_ = 0.0f;
  }

  // 0..1. Decay time, from a click to several seconds.
  void setDecay(float x) { decay_ = clamp(x, 0.0f, 1.0f); }
  // 0..1. 0 = mostly VCA; 1 = more filtering, brighter attack, more bleed.
  void setColour(float x) { colour_ = clamp(x, 0.0f, 1.0f); }
  // Attack speed for trigger(). Plaits ties it to the note's pitch (higher
  // notes snap open faster): pass your oscillator's frequency in Hz.
  void setAttackPitch(float hz) { attackHz_ = hz; }

  // Ping the gate: open quickly, then decay.
  void trigger() { rampUp_ = true; }

  // Ping mode: pass the audio, use trigger() to strike.
  float process(float in) { return run(in, false, 0.0f); }

  // Level mode: the gate follows a CV (0..1) through the vactrol response,
  // like patching Plaits' LEVEL input.
  float process(float in, float level) { return run(in, true, level); }

 private:
  float run(float in, bool levelMode, float level) {
    // 1. Control-rate update of the vactrol envelope.
    if (counter_ == 0) {
      counter_ = controlPeriod_;
      updateEnvelope(levelMode, level);
    }
    --counter_;

    // 2. Ramp the gain, filter, and mix back some unfiltered signal.
    gain_ += gainIncrement_;
    const float s = in * gain_;
    const float lp = filter_.process(s).lp;
    return lp + (s - lp) * hfBleed_;
  }

  void updateEnvelope(bool levelMode, float level) {
    // 1. Envelope rates, expressed per update as in Plaits (where one
    // update = one 12-sample block at 48 kHz).
    const float updateTime = controlPeriod_ / sampleRate_;
    const float hf = colour_;
    const float shortDecay = 200.0f * updateTime * semitonesToRatio(-96.0f * decay_);
    const float decayTail =
        20.0f * updateTime * semitonesToRatio(-72.0f * decay_ + 12.0f * hf) - shortDecay;

    // 2. Target: the CV in level mode (soft-compressed as Plaits does), or
    // a fast ramp to 1 after a trigger in ping mode.
    float target;
    if (levelMode) {
      target = 1.3f * level / (0.3f + std::fabs(level));
    } else {
      if (rampUp_) {
        vactrol_ += attackHz_ * 2.0f * updateTime;
        if (vactrol_ >= 1.0f) {
          vactrol_ = 1.0f;
          rampUp_ = false;
        }
      }
      target = rampUp_ ? vactrol_ : 0.0f;
    }

    // 3. Vactrol response: fast rise (0.6 per update), decay that slows
    // down as the state falls (the decayTail term grows as state^4 drops).
    float error = target - vactrol_;
    float state2 = vactrol_ * vactrol_;
    float state4 = state2 * state2;
    float tail = 1.0f - vactrol_;
    float tail2 = tail * tail;
    float coefficient = error > 0.0f ? 0.6f : shortDecay + (1.0f - state4) * decayTail;
    vactrol_ += coefficient * error;

    // 4. Outputs. The cutoff formula gives a frequency normalised to
    // Plaits' 48 kHz.
    float frequency48k = 0.003f + 0.3f * vactrol_ * vactrol_ * vactrol_ * vactrol_ + hf * 0.04f;
    hfBleed_ = (tail2 + (1.0f - tail2) * hf) * hf * hf;
    gainIncrement_ = (vactrol_ - gain_) / controlPeriod_;
    setCutoff(frequency48k);
  }

  // Plaits used the cheap "dirty" tan() approximation, which puts the
  // cutoff lower than nominal at high settings. To keep that sound at any
  // rate: g = dirty(f) is the filter coefficient Plaits had at 48 kHz,
  // atan(g) gives back the true analog cutoff it produced, and tan() of
  // that at the actual rate gives the coefficient we need.
  void setCutoff(float frequency48k) {
    float g = tanApprox(frequency48k, TanApprox::Dirty);
    if (sampleRate_ != 48000.0f) {
      float angle = std::atan(g) * (48000.0f / sampleRate_);
      g = std::tan(angle < 1.56f ? angle : 1.56f);
    }
    filter_.setGQ(g, 0.4f);
  }

  float sampleRate_ = 48000.0f;
  float decay_ = 0.5f;
  float colour_ = 0.5f;
  float attackHz_ = kFreqC4;

  int controlPeriod_ = 12;
  int counter_ = 0;
  float vactrol_ = 0.0f;
  bool rampUp_ = false;
  float gain_ = 0.0f, gainIncrement_ = 0.0f;
  float hfBleed_ = 0.0f;
  Svf filter_;
};

}  // namespace pt

#endif  // PT_DYNAMICS_LPG_H_
