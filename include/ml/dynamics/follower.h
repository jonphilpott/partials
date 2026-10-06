// mutablelib — dynamics/follower.h
//
// Follower: an envelope follower that also tracks brightness. Feed it
// audio; it outputs how loud it is and how bright it is (its "spectral
// centroid"), as two smooth control signals. Rings uses it in its FM mode
// to make the voice react to the playing dynamics of its input.
//
// How it works
// - Two filters split the input into three bands (low / mid / high, split
//   at about 160 Hz and 1.6 kHz).
// - Each band has its own envelope detector, with faster attack and
//   release for higher bands (high frequencies change faster).
// - The envelope is the sum of the three. The brightness is a weighted
//   average of which band dominates: 0 = all low, 1 = all high. It is
//   smoothed with a fast rise and a slow fall.
//
// Sample-rate independence: crossover frequencies and detector speeds are
// set in Hz.
//
// Derived from Rings, Copyright 2015 Emilie Gillet. MIT licence.

#ifndef ML_DYNAMICS_FOLLOWER_H_
#define ML_DYNAMICS_FOLLOWER_H_

#include <cmath>

#include "ml/core/math.h"

namespace ml {

class Follower {
 public:
  // The three Hz values are Rings' defaults: the slowest detector rate, and
  // the two crossover frequencies.
  void init(float sampleRate, float lowHz = 8.0f, float lowMidHz = 160.0f, float midHighHz = 1600.0f) {
    float low = lowHz / sampleRate, lowMid = lowMidHz / sampleRate, midHigh = midHighHz / sampleRate;
    lowMid_.set(lowMid);
    midHigh_.set(midHigh);
    // Detector speeds per band: geometric means of the band edges.
    attack_[0] = lowMid;
    decay_[0] = std::sqrt(lowMid * low);
    attack_[1] = std::sqrt(lowMid * midHigh);
    decay_[1] = lowMid;
    attack_[2] = std::sqrt(midHigh * 0.5f);
    decay_[2] = std::sqrt(midHigh * lowMid);
    // Rings' centroid smoothing, per sample at 48 kHz.
    centroidRise_ = rescaleCoefficient(0.05f, 48000.0f / sampleRate);
    centroidFall_ = rescaleCoefficient(0.001f, 48000.0f / sampleRate);
    for (float& d : detector_) d = 0.0f;
    envelope_ = centroid_ = 0.0f;
  }

  // Returns the envelope (about 0..1 for a full-scale input); centroid()
  // then holds the brightness, 0..1.
  float process(float in) {
    // 1. Split into bands: the high band from the first filter, then the
    // rest split again.
    float bands[3];
    midHigh_.process(in);
    bands[2] = midHigh_.hp;
    lowMid_.process(midHigh_.lp);
    bands[1] = lowMid_.hp;
    bands[0] = lowMid_.lp;

    // 2. Envelope per band; sum, and the weighted band position.
    float weighted = 0.0f, total = 0.0f, position = 0.0f;
    for (int i = 0; i < 3; ++i) {
      slope(detector_[i], std::fabs(bands[i]), attack_[i], decay_[i]);
      weighted += detector_[i] * position;
      total += detector_[i];
      position += 0.5f;
    }

    // 3. Smooth the brightness: quick to rise, slow to fall.
    float error = weighted / (total + 0.001f) - centroid_;
    centroid_ += error * (error > 0.0f ? centroidRise_ : centroidFall_);
    envelope_ = total;
    return envelope_;
  }

  float envelope() const { return envelope_; }
  float centroid() const { return centroid_; }

 private:
  // The classic two-integrator (Chamberlin) state-variable filter, as
  // stmlib's NaiveSvf: cheap, fine at these low crossover frequencies.
  struct NaiveSvf {
    float f = 0.1f, damp = 2.0f, lp = 0.0f, bp = 0.0f, hp = 0.0f;
    void set(float frequency) {
      f = 2.0f * 3.14159265f * std::fmin(frequency, 0.158f);
      damp = 2.0f;  // q = 0.5
    }
    void process(float in) {
      float bpNormalized = bp * damp;
      float notch = in - bpNormalized;
      lp += f * bp;
      hp = notch - lp;
      bp += f * hp;
    }
  };

  NaiveSvf lowMid_, midHigh_;
  float attack_[3] = {0, 0, 0}, decay_[3] = {0, 0, 0};
  float detector_[3] = {0, 0, 0};
  float centroidRise_ = 0.05f, centroidFall_ = 0.001f;
  float envelope_ = 0.0f, centroid_ = 0.0f;
};

}  // namespace ml

#endif  // ML_DYNAMICS_FOLLOWER_H_
