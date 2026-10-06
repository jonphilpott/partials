// partials — mod/bouncing_ball.h
//
// BouncingBall: an envelope that falls like a dropped ball, bouncing lower
// and faster each time until it comes to rest. Peaks' bouncing-ball mode.
// Use it for accelerating ratchets, ping-pong delays of a gate, or just a
// lively decaying envelope.
//
// How it works: a tiny physics simulation, one step per sample. Gravity
// pulls the velocity down; the velocity moves the position. When the ball
// hits the floor (0) or the ceiling (1) its velocity reverses, multiplied
// by the elasticity, so each bounce loses energy.
//
// Original: Peaks, 32-bit fixed point at 48 kHz. This port is float, with
// gravity and velocity in units per second, so it runs at any sample rate.
//
// Derived from Peaks, Copyright 2013 Emilie Gillet. MIT licence.

#ifndef PT_MOD_BOUNCING_BALL_H_
#define PT_MOD_BOUNCING_BALL_H_

#include <cmath>

#include "pt/core/math.h"

namespace pt {

class BouncingBall {
 public:
  void init(float sampleRate) {
    sampleRate_ = sampleRate;
    position_ = velocity_ = 0.0f;
    previousGate_ = false;
    setGravity(gravityKnob_);
  }

  // 0..1. Gravity, i.e. how fast the ball falls: from a slow, floaty drop
  // (about 0.4 s to fall from the top) to audio-rate buzzing. Peaks' curve.
  void setGravity(float x) {
    gravityKnob_ = clamp(x, 0.0f, 1.0f);
    // Peaks' table: a fall-time curve from 1 ms to 10 s (shaped by a 0.3
    // power), scaled so the time to fall the full height is about 4.4% of
    // that.
    const double gamma = 0.3;
    double a = std::pow(0.001, gamma), b = std::pow(10.0, gamma);
    double t = std::pow(a + (b - a) * gravityKnob_, 1.0 / gamma);
    double fallTime = 0.0442 * t;
    // From height 1 at rest: h = g t^2 / 2, so g = 2 / t^2 (per second^2),
    // then per sample^2.
    gravity_ = static_cast<float>(2.0 / (fallTime * fallTime) / (double(sampleRate_) * sampleRate_));
  }
  // 0..1. Elasticity: how much speed survives each bounce. 0 = stops dead,
  // 1 = bounces almost forever.
  void setBounce(float x) {
    float b = 1.0f - clamp(x, 0.0f, 1.0f);
    elasticity_ = 1.0f - b * b;
  }
  // 0..1. Height the ball is dropped from.
  void setHeight(float x) { height_ = clamp(x, 0.0f, 1.0f); }
  // -1..1. Initial velocity: positive throws it up first.
  void setVelocity(float x) { initialVelocity_ = clamp(x, -1.0f, 1.0f); }

  // Drop the ball now.
  void trigger() {
    position_ = height_;
    // Peaks' full-scale throw: about 23 heights per second.
    velocity_ = initialVelocity_ * 23.4f / sampleRate_;
  }

  // gate: a rising edge drops the ball (or call trigger()). Returns the
  // height, 0..1.
  float process(bool gate = false) {
    if (gate && !previousGate_) trigger();
    previousGate_ = gate;
    velocity_ -= gravity_;
    position_ += velocity_;
    if (position_ < 0.0f) {
      position_ = 0.0f;
      velocity_ = -velocity_ * elasticity_;
    }
    if (position_ > 1.0f) {
      position_ = 1.0f;
      velocity_ = -velocity_ * elasticity_;
    }
    return position_;
  }

 private:
  float sampleRate_ = 48000.0f;
  float gravityKnob_ = 0.5f, gravity_ = 0.0f;
  float elasticity_ = 0.75f;
  float height_ = 1.0f, initialVelocity_ = 0.0f;
  float position_ = 0.0f, velocity_ = 0.0f;
  bool previousGate_ = false;
};

}  // namespace pt

#endif  // PT_MOD_BOUNCING_BALL_H_
