// partials — mod/slope.h
//
// Slope: a function generator, the single-channel core of Tides (2018).
// An envelope, a looping LFO or an audio oscillator with a skewable,
// shapeable waveform, plus end-of-attack and end-of-release gates.
//
// How it works
// 1. A ramp (phase 0..1) runs at the set frequency. In AD mode it runs once
//    per trigger; in LOOPING mode it cycles; in AR mode it climbs while the
//    gate is high and falls when it is released.
// 2. `slope` skews the ramp into a rise and a fall of different lengths:
//    0 = instant rise, slow fall; 0.5 = symmetric; 1 = slow rise, instant
//    fall.
// 3. `shape` bends the rise and fall through a family of curves:
//    logarithmic, sine, linear, exponential... (control rates), or towards
//    rounder or spikier waveforms (audio rates).
// 4. `smoothness` below 0.5 low-passes the output (rounding corners);
//    above 0.5 it wavefolds it (adding wiggles and harmonics).
// 5. At audio rates the looping waveform is anti-aliased with PolyBLEP.
//
// Not included: Tides' four-output modes (phase-shifted, ratio-related and
// amplitude-panned copies). Run several Slopes to build them.
//
// Derived from Tides 2, Copyright 2017 Emilie Gillet. MIT licence.

#ifndef PT_MOD_SLOPE_H_
#define PT_MOD_SLOPE_H_

#include <cmath>

#include "pt/core/math.h"
#include "pt/osc/polyblep.h"

namespace pt {

class Slope {
 public:
  enum Mode { AD, LOOPING, AR };
  // CONTROL: envelopes and LFOs (sharp corners kept). AUDIO: an
  // anti-aliased oscillator (looping mode only).
  enum Range { CONTROL, AUDIO };

  void init(float sampleRate) {
    sampleTime_ = 1.0f / sampleRate;
    phase_ = 0.0f;
    frequency_ = 0.0f;
    nextSample_ = 0.0f;
    lp1_ = lp2_ = 0.0f;
    previousGate_ = false;
    previousInput_ = previousOutput_ = breakpoint_ = 0.0f;
    eoa_ = eor_ = false;
    tables();
  }

  void setMode(Mode m) { mode_ = m; }
  void setRange(Range r) { range_ = r; }
  // Cycle frequency in Hz: for AD, one attack plus decay lasts 1/hz.
  void setFrequency(float hz) { hz_ = hz; }
  // 0..1. Rise/fall balance (Tides' SLOPE knob).
  void setSlope(float x) { slope_ = clamp(x, 0.0f, 1.0f); }
  // 0..1. Curve family (Tides' SHAPE knob).
  void setShape(float x) { shape_ = clamp(x, 0.0f, 1.0f); }
  // 0..1. Below 0.5 smoother, above 0.5 folded (Tides' SMOOTHNESS knob).
  void setSmoothness(float x) { smoothness_ = clamp(x, 0.0f, 1.0f); }

  // gate: triggers AD mode, holds AR mode, resets LOOPING mode on a rising
  // edge. Returns 0..1 in AD and AR modes, ±1 in LOOPING mode.
  float process(bool gate) {
    const bool rising = gate && !previousGate_;
    previousGate_ = gate;
    const bool audio = range_ == AUDIO && mode_ == LOOPING;
    float f0 = std::fmin(hz_ * sampleTime_, 0.25f);

    // 1. Parameters, conditioned as in Tides.
    float pw = slope_;
    if (!audio && pw < 0.5f) {
      // Skew the response so the useful range of short attacks is wider.
      pw = 0.5f + 0.6f * (pw - 0.5f) / (std::fabs(pw - 0.5f) + 0.1f);
    }
    // At high frequencies, shape and fold are reduced to limit aliasing.
    const float harmonics = 3.0f + std::fabs(pw - 0.5f) * 5.0f;
    const float shapeAmount = std::fabs(shape_ - 0.5f) * 2.0f;
    const float shapeAttenuation = tame(f0, harmonics, 16.0f);
    const float shape = 0.5f + (shape_ - 0.5f) * shapeAttenuation;
    float smoothness = smoothness_;
    if (smoothness > 0.5f) {
      smoothness = 0.5f + (smoothness - 0.5f) *
          tame(f0, harmonics * (3.0f + shapeAmount * shapeAttenuation * 5.0f), 12.0f);
    }
    const float fold = std::fmax(2.0f * (smoothness - 0.5f), 0.0f);

    // 2. Advance the ramp.
    stepRamp(f0, pw, gate, rising);

    // 3. Skewed (and in audio range, band-limited) slope, 0..1.
    float raw;
    if (mode_ == AR) {
      raw = phase_;
    } else if (audio) {
      raw = bandLimitedSlope(phase_, frequency_, pw);
    } else {
      raw = skewedRamp(phase_, frequency_, pw);
    }

    // 4. Curve shape. Control-rate tables cover rise-then-fall; audio-rate
    // tables are monotonic shapers.
    float s = audio ? shape * 3.9999f : shape * 5.9999f + 5.0f;
    int si = static_cast<int>(s);
    float shaped = waveshape(raw, si, s - static_cast<float>(si));

    // 5. Fold, then the end-of-attack/release gates.
    float out = foldOutput(shaped, fold);
    if (mode_ == AR) {
      eoa_ = phase_ >= 0.5f;
      eor_ = phase_ >= 1.0f;
    } else if (mode_ == LOOPING) {
      eoa_ = phase_ >= pw;
      eor_ = phase_ < std::fmin(0.5f, 96.0f * frequency_);
    } else {
      eoa_ = phase_ >= pw;
      eor_ = phase_ >= 1.0f;
    }

    // 6. Smoothing below 0.5: a two-pole low-pass that follows the
    // frequency, so the rounding is the same at every rate.
    if (smoothness < 0.5f) {
      float ratio = smoothness * 2.0f;
      ratio *= ratio;
      ratio *= ratio;
      float f = frequency_ * 0.5f;
      f += (1.0f - f) * ratio;
      onePole(lp1_, out, f);
      onePole(lp2_, lp1_, f);
      out = lp2_;
    }
    return out;
  }

  // Gates for chaining: high once the attack is over / the cycle has ended.
  bool endOfAttack() const { return eoa_; }
  bool endOfRelease() const { return eor_; }

 private:
  void stepRamp(float f0, float pw, bool gate, bool rising) {
    frequency_ = f0;
    if (mode_ == AD) {
      if (rising) phase_ = 0.0f;
      phase_ = std::fmin(phase_ + frequency_, 1.0f);
    } else if (mode_ == AR) {
      // Rise to 0.5 while held; fall from 0.5 to 1 when released, each at
      // its own speed.
      if (phase_ < 0.5f && !gate) {
        phase_ = 0.5f;
      } else if (phase_ > 0.5f && gate) {
        phase_ = 0.0f;
      }
      float slope = phase_ < 0.5f ? 0.5f / (1.0e-6f + pw) : 0.5f / (1.0f + 1.0e-6f - pw);
      phase_ = std::fmin(phase_ + frequency_ * slope, gate ? 0.5f : 1.0f);
    } else {
      if (rising) {
        phase_ = 0.0f;
      } else {
        phase_ += frequency_;
        if (phase_ >= 1.0f) phase_ -= 1.0f;
      }
    }
  }

  // Rise over [0, pw), fall over [pw, 1); output 0..0.5 then 0.5..1 (the
  // control-rate shape tables read the first half as the rise).
  static float skewedRamp(float phase, float frequency, float pw) {
    pw = clamp(pw, std::fabs(frequency) * 2.0f, 1.0f - 2.0f * std::fabs(frequency));
    return phase < pw ? phase * (0.5f / pw) : (phase - pw) * (0.5f / (1.0f - pw)) + 0.5f;
  }

  // Triangle-like slope with PolyBLEP-corrected corners (audio range).
  float bandLimitedSlope(float phase, float frequency, float pw) {
    pw = clamp(pw, std::fabs(frequency) * 2.0f, 1.0f - 2.0f * std::fabs(frequency));
    float thisSample = nextSample_;
    float nextSample = 0.0f;
    float wrapPoint = pw;
    if (phase < pw * 0.5f) {
      wrapPoint = 0.0f;
    } else if (phase > 0.5f + pw * 0.5f) {
      wrapPoint = 1.0f;
    }
    const float slopeUp = 1.0f / pw;
    const float slopeDown = 1.0f / (1.0f - pw);
    const float d = phase - wrapPoint;
    if (d >= 0.0f && d < frequency) {
      const float t = d / frequency;
      float discontinuity = -(slopeUp + slopeDown) * frequency;
      if (wrapPoint != pw) discontinuity = -discontinuity;
      thisSample += thisIntegratedBlepSample(t) * discontinuity;
      nextSample += nextIntegratedBlepSample(t) * discontinuity;
    }
    nextSample += phase < pw ? phase * slopeUp : 1.0f - (phase - pw) * slopeDown;
    nextSample_ = nextSample;
    return thisSample;
  }

  // Blend of two neighbouring shape tables; in AR mode, rescaled around
  // the level where the gate changed, so release starts where attack
  // stopped.
  float waveshape(float input, int table, float blend) {
    const Tables& t = tables();
    float index = 1024.0f * input;
    int i = static_cast<int>(index);
    float frac = index - static_cast<float>(i);
    i &= 1023;
    const float* a = t.shape[table];
    const float* b = t.shape[table + 1 > 11 ? 11 : table + 1];
    float x = a[i] + (a[i + 1] - a[i]) * frac;
    float y = b[i] + (b[i + 1] - b[i]) * frac;
    float output = x + (y - x) * blend;
    if (mode_ != AR) return output;

    if (previousInput_ <= 0.5f && input > 0.5f) {
      breakpoint_ = previousOutput_;
    } else if (previousInput_ > 0.5f && input < 0.5f) {
      breakpoint_ = previousOutput_;
    } else if (input == 1.0f) {
      breakpoint_ = 1.0f;
    } else if (input == 0.5f) {
      breakpoint_ = 0.0f;
    }
    output = input <= 0.5f ? breakpoint_ + (1.0f - breakpoint_) * output : breakpoint_ * output;
    previousInput_ = input;
    previousOutput_ = output;
    return output;
  }

  // Wavefolding for smoothness above 0.5: bipolar for looping, unipolar
  // for envelopes.
  float foldOutput(float unipolar, float amount) {
    const Tables& t = tables();
    if (mode_ == LOOPING) {
      float bipolar = 2.0f * unipolar - 1.0f;
      if (amount <= 0.0f) return bipolar;
      float folded = interpolate(t.bipolarFold, 0.5f + bipolar * (0.03f + 0.46f * amount), 1024.0f);
      return bipolar + (folded - bipolar) * amount;
    }
    if (amount <= 0.0f) return unipolar;
    float folded = interpolate(t.unipolarFold, unipolar * amount, 1024.0f);
    return unipolar + (folded - unipolar) * amount;
  }

  // 1 at low frequencies, falling to 0 as f0 * harmonics nears Nyquist:
  // scales down settings that would alias.
  static float tame(float f0, float harmonics, float order) {
    f0 *= harmonics;
    float maxF = 0.5f / order;
    float amount = clamp(1.0f - (f0 - maxF) / (0.5f - maxF), 0.0f, 1.0f);
    return amount * amount * amount;
  }

  // Tides 2's shape and fold tables, generated as its Python script does.
  struct Tables {
    float shape[12][1025];
    float bipolarFold[1028];
    float unipolarFold[1028];
  };

  static const Tables& tables() {
    struct Make {
      static Tables build() {
        Tables t;
        const double pi = 3.14159265358979323846;
        // 1. Audio-rate shapers (monotonic, 1024 points + guard): tables 0-4.
        double tanScale = std::atan(8.0);  // max of atan(8 cos(pi x)) on [0, 1)
        for (int i = 0; i < 1024; ++i) {
          double x = i / 1024.0;
          double fadeCrop = std::fmin(1.0, 4.0 - 4.0 * x);
          double bump = (1.0 - std::cos(pi * x * 1.5)) * (1.0 - std::cos(pi * fadeCrop)) / 4.5;
          double inverseSin = std::acos(1.0 - 2.0 * x) / pi;
          double inverseTan = i == 0 ? 0.0 : std::acos(std::tan(tanScale * (1.0 - 2.0 * x)) / 8.0) / pi;
          double v[5] = {inverseTan, inverseSin, x, 0.5 - 0.5 * std::cos(x * pi), bump};
          for (int k = 0; k < 5; ++k) t.shape[k][i] = q(v[k]);
        }
        for (int k = 0; k < 5; ++k) t.shape[k][1024] = t.shape[k][1023];

        // 2. Control-rate shapes: rise over the first half, fall over the
        // second (512 + 1 + 512 points): tables 5-11.
        double expoMax = 1.0 - std::exp(-5.0 * 511.0 / 512.0);
        for (int i = 0; i < 512; ++i) {
          double x = i / 512.0;
          double expo = (1.0 - std::exp(-5.0 * x)) / expoMax;
          double log = std::log(1.0 - x * expoMax) / -5.0;
          double inverseSin = std::acos(1.0 - 2.0 * x) / pi;
          double sine = (1.0 - std::cos(pi * x)) / 2.0;
          // {curve, flip}: flip = mirror for the fall, else 1 - curve.
          const double curves[7] = {log, log, inverseSin, x, sine, expo, expo};
          const bool flips[7] = {false, true, false, false, false, true, false};
          for (int k = 0; k < 7; ++k) {
            t.shape[5 + k][i] = q(curves[k]);
            if (!flips[k]) t.shape[5 + k][513 + i] = q(1.0 - curves[k]);
          }
        }
        // Mirrored falls (flip = true) read the rising half backwards.
        for (int k = 0; k < 7; ++k) {
          t.shape[5 + k][512] = q(1.0);
          if (k == 1 || k == 5) {
            for (int i = 0; i < 512; ++i) t.shape[5 + k][513 + i] = t.shape[5 + k][511 - i];
          }
        }

        // 3. Fold curves.
        double maxB = 0.0, maxU = 0.0;
        double bip[1028], uni[1028];
        for (int i = 0; i < 1028; ++i) {
          double x = i / 512.0 - 1.0;
          if (i == 1027) x = 1026 / 512.0 - 1.0;
          double window = std::exp(-x * x * 4.0);
          window *= window;
          bip[i] = std::sin(8.0 * pi * x) * window + std::atan(3.0 * x) * (1.0 - window);
          maxB = std::fmax(maxB, std::fabs(bip[i]));

          double u = i / 1024.0;
          if (i >= 1026) u = 1025 / 1024.0;
          double w2 = std::exp(-u * u * 4.0);
          w2 *= w2;
          uni[i] = (0.38 * std::sin(16.0 * pi * u) + 4.0 * u) * w2 + std::atan(4.0 * u) * (1.0 - w2);
          maxU = std::fmax(maxU, std::fabs(uni[i]));
        }
        for (int i = 0; i < 1028; ++i) {
          t.bipolarFold[i] = static_cast<float>(bip[i] / maxB);
          t.unipolarFold[i] = static_cast<float>(uni[i] / maxU);
        }
        return t;
      }
      // Tides stored the shapes as 16-bit integers; round the same way.
      static float q(double v) { return static_cast<float>(std::round(v * 32767.0)) / 32768.0f; }
    };
    static const Tables t = Make::build();
    return t;
  }

  float sampleTime_ = 1.0f / 48000.0f;
  Mode mode_ = LOOPING;
  Range range_ = CONTROL;
  float hz_ = 1.0f, slope_ = 0.5f, shape_ = 0.5f, smoothness_ = 0.5f;

  float phase_ = 0.0f, frequency_ = 0.0f, nextSample_ = 0.0f;
  float lp1_ = 0.0f, lp2_ = 0.0f;
  float previousInput_ = 0.0f, previousOutput_ = 0.0f, breakpoint_ = 0.0f;
  bool previousGate_ = false, eoa_ = false, eor_ = false;
};

}  // namespace pt

#endif  // PT_MOD_SLOPE_H_
