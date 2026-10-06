// partials — mod/lorenz.h
//
// Lorenz: a chaotic modulation source. Three smooth, endlessly varying
// outputs that never quite repeat. Streams' "Lorenz generator" mode.
//
// How it works: the Lorenz system is three linked equations for a point
// (x, y, z) moving through space:
//   dx/dt = sigma (y - x),  dy/dt = x (rho - z) - y,  dz/dt = x y - beta z
// With the classic constants (sigma 10, rho 28, beta 8/3) the point loops
// round two "wings", switching between them unpredictably: the famous
// butterfly. Each sample moves the point one small step (Euler's method);
// the rate sets how big a step, so how fast it moves.
//
// Original: Streams, fixed-point (8.24) at 31.089 kHz. This port is float
// and its rate is in steps per second, so it runs at any sample rate.
//
// Derived from Streams, Copyright 2014 Emilie Gillet. MIT licence.

#ifndef PT_MOD_LORENZ_H_
#define PT_MOD_LORENZ_H_

#include <cmath>

#include "pt/core/math.h"

namespace pt {

class Lorenz {
 public:
  void init(float sampleRate) {
    rateScale_ = 31089.0f / sampleRate;
    x_ = 0.1f;
    y_ = z_ = 0.0f;
    setRate(rate_);
  }

  // 0..1. Speed, exponential: from a slow drift (many seconds per loop) to
  // audio-rate buzz. Streams' rate knob.
  void setRate(float x) {
    rate_ = clamp(x, 0.0f, 1.0f);
    // Streams' curve: the step grows by a factor of 2 every 1/16.5 of the
    // knob, reaching 0.02 at the top (at 31.089 kHz).
    dt_ = 0.02f * std::exp2((rate_ - 1.0f) * 16.5f) * rateScale_;
  }

  // Advance one sample. Returns x(); y() and z() then hold the others.
  float process() {
    const float sigma = 10.0f, rho = 28.0f, beta = 8.0f / 3.0f;
    float x = x_ + dt_ * (sigma * (y_ - x_));
    float y = y_ + dt_ * (x_ * (rho - z_) - y_);
    float z = z_ + dt_ * (x_ * y_ - beta * z_);
    x_ = x;
    y_ = y;
    z_ = z;
    return this->x();
  }

  // Outputs scaled to about ±1. x and y swing between the two wings; z
  // rises and falls with each loop.
  float x() const { return x_ * (1.0f / 20.0f); }
  float y() const { return y_ * (1.0f / 27.0f); }
  float z() const { return (z_ - 25.0f) * (1.0f / 25.0f); }

 private:
  float rateScale_ = 1.0f;
  float rate_ = 0.5f;
  float dt_ = 0.0f;
  float x_ = 0.1f, y_ = 0.0f, z_ = 0.0f;
};

}  // namespace pt

#endif  // PT_MOD_LORENZ_H_
