// partials — filter/dc_blocker.h
//
// Removes DC offset (and sub-audio drift) from a signal: a one-pole
// high-pass filter, y[n] = x[n] - x[n-1] + pole * y[n-1].
//
// The difference x[n] - x[n-1] cancels anything constant; the feedback term
// restores everything above the cutoff. The closer `pole` is to 1, the lower
// the cutoff: pole = 1 - 2*pi*fc/sampleRate, approximately.
//
// Derived from stmlib dsp/filter.h, Copyright 2014 Emilie Gillet. MIT licence.

#ifndef PT_FILTER_DC_BLOCKER_H_
#define PT_FILTER_DC_BLOCKER_H_

namespace pt {

class DcBlocker {
 public:
  // cutoffHz around 5..20 Hz is typical.
  void init(float sampleRate, float cutoffHz = 10.0f) {
    // Gillet writes the pole as 1 - fc / sampleRate (no 2*pi): her cutoff
    // figures are nominal, so we keep the same formula to match.
    pole_ = 1.0f - cutoffHz / sampleRate;
    reset();
  }

  void reset() { x_ = y_ = 0.0f; }

  float process(float in) {
    y_ = y_ * pole_ + in - x_;
    x_ = in;
    return y_;
  }

 private:
  float pole_ = 0.999f;
  float x_ = 0.0f, y_ = 0.0f;
};

}  // namespace pt

#endif  // PT_FILTER_DC_BLOCKER_H_
