// mutablelib — spectral/vocoder.h
//
// Vocoder: imposes the spectral envelope of one sound (the modulator,
// usually a voice) on another (the carrier, usually a rich synth tone): the
// classic "talking synth". Warps' vocoder.
//
// How it works
// 1. Both signals are split into 20 bands, a third of an octave apart,
//    from about 87 Hz to 7 kHz (plus a low-pass and a high-pass at the
//    ends).
// 2. An envelope follower measures the modulator's level in each band.
//    Higher bands get faster followers.
// 3. Each carrier band is turned up or down by that level, and the bands
//    are summed. Wherever the voice has energy, the synth comes through.
// 4. `formantShift` away from 0.5 crossfades to a static version: the
//    modulator's band levels, shifted up or down the bank, applied to the
//    carrier. Shifting gives the "chipmunk" or "giant" effect. At 1 or 0,
//    the shift is about ±2 octaves.
// 5. A limiter keeps the output in range.
//
// Difference from Warps: Warps ran its low and mid bands at 1/12 and 1/3
// of its 96 kHz rate, through resampling filters, with band filters
// designed offline for that rate. Here every band runs at the host rate,
// with state-variable filters designed at run time, so it works at any
// sample rate and is simpler, at a higher CPU cost (160 filter sections
// per sample).
//
// Derived from Warps, Copyright 2014 Emilie Gillet. MIT licence.

#ifndef ML_SPECTRAL_VOCODER_H_
#define ML_SPECTRAL_VOCODER_H_

#include <cmath>

#include "ml/core/math.h"
#include "ml/core/units.h"
#include "ml/filter/svf.h"
#include "ml/fx/limiter.h"

namespace ml {

class Vocoder {
 public:
  static const int kNumBands = 20;
  // Filter sections per band: Warps cascades two 4th-order filters (8th
  // order in all); four 2nd-order sections give the same steepness.
  static const int kSections = 4;

  void init(float sampleRate) {
    sampleRate_ = sampleRate;
    // 1. Band centres: a third of an octave apart, the first a third of
    // an octave below 110 Hz (Warps).
    const float interval = std::cbrt(2.0f);
    float centre = 110.0f / interval;
    for (int b = 0; b < kNumBands; ++b) {
      for (int s = 0; s < kSections; ++s) {
        modulator_[b][s].init(sampleRate);
        carrier_[b][s].init(sampleRate);
      }
      // The end bands are a low-pass and a high-pass; the rest
      // band-passes about a third of an octave wide (Q 1.88 per section
      // gives that for four identical sections in series).
      float f = std::fmin(centre / sampleRate, 0.45f);
      float q = (b == 0 || b == kNumBands - 1) ? 0.707f : 1.88f;
      for (int s = 0; s < kSections; ++s) {
        modulator_[b][s].setCoefficients(f, q, TanApprox::Exact);
        carrier_[b][s].setCoefficients(f, q, TanApprox::Exact);
      }
      envelope_[b] = peak_[b] = 0.0f;
      centre *= interval;
    }
    limiter_.init(sampleRate);
    limiter_.setPreGain(1.4f);
    controlPeriod_ = static_cast<int>(sampleRate / 1000.0f + 0.5f);  // 1 ms
    if (controlPeriod_ < 1) controlPeriod_ = 1;
    counter_ = 0;
    for (int b = 0; b < kNumBands; ++b) {
      blockPeak_[b] = 0.0f;
      carrierGain_[b] = 0.0f;
    }
    vocoderGain_ = 1.0f;
    update();
  }

  // 0..1. Envelope release, from fast (articulate) to slow (smeared). At
  // 1, the envelopes freeze, holding the current spectrum.
  void setRelease(float x) { release_ = clamp(x, 0.0f, 1.0f); update(); }
  // 0..1. 0.5 = normal vocoding. Away from 0.5, crossfades to the
  // modulator's spectrum shifted down (below) or up (above) the bank.
  void setFormantShift(float x) { formantShift_ = clamp(x, 0.0f, 1.0f); }

  // modulator: the voice (±1). carrier: the synth (±1). Returns the
  // vocoded signal.
  float process(float modulator, float carrier) {
    if (counter_ == 0) {
      counter_ = controlPeriod_;
      updateBandGains();
    }
    --counter_;

    float out = 0.0f;
    const float followerGain = std::sqrt(static_cast<float>(kNumBands));
    for (int b = 0; b < kNumBands; ++b) {
      float m = band(modulator_[b], b, modulator);
      float c = band(carrier_[b], b, carrier);
      // Envelope follower on the modulator band (frozen at full release).
      if (!frozen_) {
        float error = std::fabs(m * followerGain) - envelope_[b];
        envelope_[b] += (error > 0.0f ? attack_[b] : decay_[b]) * error;
      }
      if (envelope_[b] > blockPeak_[b]) blockPeak_[b] = envelope_[b];
      out += c * (carrierGain_[b] + vocoderGain_ * envelope_[b]);
    }
    return limiter_.process(out);
  }

 private:
  // Filter sections in series; low-pass, band-pass (unity peak) or
  // high-pass depending on the band.
  static float band(Svf* s, int b, float in) {
    for (int k = 0; k < kSections; ++k) {
      SvfOut o = s[k].process(in);
      in = b == 0 ? o.lp : (b == kNumBands - 1 ? o.hp : o.bp * s[k].r());
    }
    return in;
  }

  // Follower speeds: from 80 Hz (down to ~1.3 Hz at full release) for the
  // lowest band, a third of an octave faster per band (Warps).
  void update() {
    float f = 80.0f * semitonesToRatio(-72.0f * release_);
    for (int b = 0; b < kNumBands; ++b) {
      float decay = f / sampleRate_;
      attack_[b] = decay * 2.0f;
      decay_[b] = decay * 0.5f;
      f *= 1.2599f;
    }
    frozen_ = release_ > 0.995f;
  }

  // Once per millisecond: smooth each band's recent peak, and work out
  // the formant-shifted gains (Warps did this once per block).
  void updateBandGains() {
    for (int b = 0; b < kNumBands; ++b) {
      float error = blockPeak_[b] - peak_[b];
      peak_[b] += (error > 0.0f ? 0.5f : 0.1f) * error;
      blockPeak_[b] = 0.0f;
    }
    float amount = 2.0f * std::fabs(formantShift_ - 0.5f);
    amount *= 2.0f - amount;
    amount *= 2.0f - amount;
    float increment = 4.0f * semitonesToRatio(-48.0f * formantShift_);
    float position = 0.0f;
    const float lastBand = kNumBands - 1.0001f;
    for (int b = 0; b < kNumBands; ++b) {
      float source = clamp(position, 0.0f, lastBand);
      int i = static_cast<int>(source);
      float frac = source - static_cast<float>(i);
      float gain = peak_[i] + (peak_[i + 1] - peak_[i]) * frac;
      float attenuation = position - lastBand;
      if (attenuation >= 0.0f) gain *= 1.0f / (1.0f + attenuation);
      position += increment;
      carrierGain_[b] = gain * amount;
    }
    vocoderGain_ = 1.0f - amount;
  }

  float sampleRate_ = 48000.0f;
  float release_ = 0.5f, formantShift_ = 0.5f;
  bool frozen_ = false;
  Svf modulator_[kNumBands][kSections], carrier_[kNumBands][kSections];
  float attack_[kNumBands] = {}, decay_[kNumBands] = {};
  float envelope_[kNumBands] = {}, peak_[kNumBands] = {}, blockPeak_[kNumBands] = {};
  float carrierGain_[kNumBands] = {};
  float vocoderGain_ = 1.0f;
  int controlPeriod_ = 48, counter_ = 0;
  Limiter limiter_;
};

}  // namespace ml

#endif  // ML_SPECTRAL_VOCODER_H_
