// mutablelib — fx/overdrive.h
//
// Overdrive: soft-clipping distortion with automatic level compensation.
// Plaits' post-processing overdrive. Turning up the drive adds grit and
// sustain without the output getting much louder.
//
// How it works: the input is amplified (pre-gain) into a soft clipper,
// then scaled back down (post-gain). The pre-gain curve stays gentle over
// most of the knob, then climbs steeply (up to about 24x) near the top.
// The post-gain is the inverse of how loud a full-scale signal comes out
// of the clipper, so perceived level stays roughly constant.
//
// Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.

#ifndef ML_FX_OVERDRIVE_H_
#define ML_FX_OVERDRIVE_H_

#include "ml/core/math.h"

namespace ml {

class Overdrive {
 public:
  void init(float sampleRate) {
    // Drive changes are smoothed over about 1 ms to avoid zipper noise.
    // The drive itself is smoothed, not the two gains separately: those
    // must stay matched, or a fast drive change would spike in level.
    smoothing_ = 1.0f / (0.001f * sampleRate);
    if (smoothing_ > 1.0f) smoothing_ = 1.0f;
    smoothedDrive_ = drive_;
  }

  // 0..1. 0 = gentle warmth at about unity gain, 1 = heavily clipped.
  // (Internally 0.5..1: the range Plaits uses. Below 0.5 the original curve
  // stops compensating and the output fades towards silence.)
  void setDrive(float x) { drive_ = 0.5f + 0.5f * clamp(x, 0.0f, 1.0f); }

  float process(float in) {
    onePole(smoothedDrive_, drive_, smoothing_);
    // Gain curves (Plaits): pre-gain gentle at first, steep near the top;
    // post-gain is the inverse of the clipper's output for a full-scale
    // input, to keep the level roughly constant.
    const float drive = smoothedDrive_;
    const float drive2 = drive * drive;
    const float preGainA = drive * 0.5f;
    const float preGainB = drive2 * drive2 * drive * 24.0f;
    const float preGain = preGainA + (preGainB - preGainA) * drive2;
    const float squashed = drive * (2.0f - drive);
    const float postGain = 1.0f / softClip(0.33f + squashed * (preGain - 0.33f));
    return softClip(preGain * in) * postGain;
  }

 private:
  float smoothing_ = 0.02f;
  float drive_ = 0.5f;
  float smoothedDrive_ = 0.5f;
};

}  // namespace ml

#endif  // ML_FX_OVERDRIVE_H_
