// partials — fx/wavefolder.h
//
// Wavefolder: West Coast-style waveshaping that "folds" a signal back on
// itself as it is driven harder, adding bright, shifting harmonics. From
// Plaits' waveshaping engine, which has two folding curves:
// - process(): a smooth, sine-based fold (Plaits' main output). Bright and
//   bell-like at high amounts.
// - analog(): a model of an analog folder circuit (five op-amp stages and
//   a diode clipper, like the Buchla 259's timbre circuit). Rougher and
//   more vocal (Plaits' AUX output).
//
// How it works: the input, scaled by the fold amount, indexes a transfer
// curve. Near the centre the curve is almost a straight line (little
// change). Further out it bends back and forth, so peaks of the input are
// "folded" into extra wiggles, which are extra harmonics. Turning up the
// amount pushes the signal further into the wiggly region.
//
// Aliasing: folding creates harmonics far above the input's own. Feed it
// smooth, band-limited waveforms (sine, triangle, a filtered saw), and keep
// the amount modest at high pitches. Plaits reduces the amount
// automatically as the pitch rises.
//
// Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.

#ifndef PT_FX_WAVEFOLDER_H_
#define PT_FX_WAVEFOLDER_H_

#include <cmath>

#include "pt/core/math.h"

namespace pt {

class Wavefolder {
 public:
  static const int kTableSize = 512;

  // Builds the curves (once, shared by all instances).
  void init() {
    tables();
  }

  // 0..1. Fold amount: 0 = nearly clean, 1 = many folds.
  void setAmount(float x) { gain_ = 0.03f + 0.46f * clamp(x, 0.0f, 1.0f); }

  // in: about ±1. Returns the smooth (sine) fold; analog() then holds the
  // analog-circuit fold of the same sample.
  float process(float in) {
    const Tables& t = tables();
    // Map the input to a position on the curve: 0.5 is the centre.
    float index = clamp(in * gain_ + 0.5f, 0.0f, 1.0f);
    analog_ = -interpolateHermite(t.fold2 + 1, index, kTableSize);
    return interpolateHermite(t.fold + 1, index, kTableSize);
  }

  float analog() const { return analog_; }

 private:
  // 512 steps plus guard points for Hermite reads: x runs from -1 to just
  // past +1 (Plaits' layout).
  struct Tables {
    float fold[kTableSize + 4];
    float fold2[kTableSize + 4];
  };

  static const Tables& tables() {
    struct Make {
      static Tables build() {
        Tables t;
        const int n = kTableSize + 4;
        double x[kTableSize + 4], fold[kTableSize + 4], fold2[kTableSize + 4];
        for (int i = 0; i < n; ++i) x[i] = i / (kTableSize / 2.0) - 1.0;
        x[n - 1] = x[n - 2];

        // Sine fold: a fast sine near the centre, windowed and blended
        // into a gentle arctan curve further out.
        double maxAbs = 0.0;
        for (int i = 0; i < n; ++i) {
          double sine = std::sin(8.0 * 3.14159265358979323846 * x[i]);
          double window = std::exp(-x[i] * x[i] * 4.0);
          window *= window;
          fold[i] = sine * window + std::atan(3.0 * x[i]) * (1.0 - window);
          maxAbs = std::fmax(maxAbs, std::fabs(fold[i]));
        }

        // Analog fold: five op-amp stages, each clipping at a different
        // input level, summed with different gains, then a diode.
        double maxOut = -1e30;
        for (int i = 0; i < n; ++i) {
          double vIn = x[i] * 12.0;
          double s1 = deadband(vIn, 10e3, 100e3);
          double s2 = deadband(vIn, 49.9e3, 44.2e3);
          double s3 = deadband(vIn, 91e3, 18e3);
          double s4 = deadband(vIn, 30e3, 71.4e3);
          double s5 = deadband(vIn, 68e3, 33.0e3);
          double s45 = -33.0 / 71.4 * s4 - 33.0 / 33.0 * s5 - 33.0 / 240.0 * vIn;
          double v = -150.0 / 100.0 * s1 - 150.0 / 44.2 * s2 - 150.0 / 18.0 * s3 - 150.0 / 33.0 * s45;
          fold2[i] = 0.7 * v / (0.3 + std::fabs(v));  // diode
          maxOut = std::fmax(maxOut, fold2[i]);
        }
        for (int i = 0; i < n; ++i) {
          t.fold[i] = static_cast<float>(fold[i] / maxAbs);
          t.fold2[i] = static_cast<float>(fold2[i] / maxOut);
        }
        return t;
      }

      // One inverting op-amp stage with saturation, loaded by a resistor:
      // passes the input until the op-amp saturates, then only a fraction.
      static double deadband(double vIn, double rIn, double rLoad) {
        const double rFb = 150.0e3, vSat = 10.47;
        double vOut = -rFb / rIn * vIn;
        if (vOut >= vSat) vOut = vSat;
        if (vOut < -vSat) vOut = -vSat;
        return (vIn * rFb * rLoad + vOut * rIn * rLoad) / (rFb * rLoad + rIn * rLoad + rIn * rFb);
      }
    };
    static const Tables t = Make::build();
    return t;
  }

  float gain_ = 0.25f;
  float analog_ = 0.0f;
};

}  // namespace pt

#endif  // PT_FX_WAVEFOLDER_H_
