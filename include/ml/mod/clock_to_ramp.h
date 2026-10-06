// mutablelib — mod/clock_to_ramp.h
//
// ClockToRamp: turns a clock (a stream of gates or triggers) into a smooth
// phase ramp, 0..1 per clock period, multiplied or divided by a ratio.
// It is how Tides and Marbles lock their LFOs and random sequences to an
// external clock. Use the ramp to drive anything that needs to sit in time
// with a clock: Slope, Lag, a wavetable, a sequencer.
//
// How it works
// - Each clock pulse's length (and its high time) is measured.
// - The next period is predicted. The predictor tries "same as last time"
//   (smoothed) and "same as N pulses ago" for N = 1..8, and keeps whichever
//   has been most accurate. So swung or patterned clocks (long-short-long-
//   short) are followed, not just steady ones.
// - The ramp runs at the predicted speed and is nudged back on track at
//   each pulse, so it reaches the right value just as the next pulse
//   arrives.
// - If the pulse width has been steady, the falling edge is used as an
//   extra timing point mid-period.
// - Clocks faster than `maxHz` skip the prediction; after more than about
//   3 s (or 4 periods) without a pulse, the next pulse restarts.
//
// Derived from Tides 2, Copyright 2017 Emilie Gillet. MIT licence.

#ifndef ML_MOD_CLOCK_TO_RAMP_H_
#define ML_MOD_CLOCK_TO_RAMP_H_

#include <algorithm>
#include <cstdint>

#include "ml/core/math.h"

namespace ml {

class ClockToRamp {
 public:
  void init(float sampleRate, float maxHz = 40.0f) {
    sampleRate_ = sampleRate;
    minPeriod_ = sampleRate / maxHz;
    reset();
  }

  void reset() {
    trainPhase_ = 0.0f;
    targetFrequency_ = frequency_ = 0.1f / sampleRate_;
    maxTrainPhase_ = 1.0f;
    fRatio_ = ratio_;
    resetCounter_ = 1;
    resetInterval_ = static_cast<uint32_t>(sampleRate_) * 3;
    Pulse p;
    p.onDuration = static_cast<uint32_t>(sampleRate_ * 0.25f);
    p.totalDuration = static_cast<uint32_t>(sampleRate_ * 0.5f);
    p.pulseWidth = 0.5f;
    std::fill(history_, history_ + kHistorySize, p);
    current_ = 0;
    history_[0].onDuration = history_[0].totalDuration = 0;
    averagePulseWidth_ = 0.0f;
    std::fill(predictionError_, predictionError_ + kMaxPatternPeriod + 1, 50.0f);
    std::fill(predictedPeriod_, predictedPeriod_ + kMaxPatternPeriod + 1, sampleRate_ * 0.5f);
    predictionError_[0] = 0.0f;
    previousClock_ = false;
  }

  // Output rate = clock rate x multiply / divide. The ramp re-aligns every
  // `divide` clock pulses. E.g. (1, 4): one ramp per four clocks; (3, 2):
  // three ramps per two clocks.
  void setRatio(int multiply, int divide) {
    divide_ = divide < 1 ? 1 : divide;
    ratio_ = static_cast<float>(multiply < 1 ? 1 : multiply) / static_cast<float>(divide_);
  }

  // clock: the clock input as a gate (true while high). Returns the ramp.
  float process(bool clock) {
    const bool rising = clock && !previousClock_;
    const bool falling = !clock && previousClock_;
    previousClock_ = clock;

    // 1. A new clock pulse: finish measuring the previous one.
    if (rising) {
      Pulse& p = history_[current_];
      if (p.totalDuration >= resetInterval_) {
        // The clock had stopped: start again from this pulse.
        resetCounter_ = divide_;
        trainPhase_ = 0.0f;
        fRatio_ = ratio_;
        maxTrainPhase_ = static_cast<float>(divide_);
        resetInterval_ = 4 * p.totalDuration;
      } else {
        float period = static_cast<float>(p.totalDuration);
        if (period < minPeriod_) {
          // Fast clock: no prediction, follow it directly.
          frequency_ = targetFrequency_ = 1.0f / period;
        } else {
          // Pulse width: usable as a timing point if it has been steady.
          p.pulseWidth = static_cast<float>(p.onDuration) / static_cast<float>(p.totalDuration);
          averagePulseWidth_ = averagePulseWidth(0.05f);
          if (p.onDuration < 32) averagePulseWidth_ = 0.0f;
          frequency_ = targetFrequency_ = 1.0f / predictNextPeriod();
        }
        // Every `divide` pulses, realign; in between, speed up or slow down
        // so the ramp lands where it should at the next pulse.
        if (--resetCounter_ == 0) {
          trainPhase_ = 0.0f;
          resetCounter_ = divide_;
          fRatio_ = ratio_;
          maxTrainPhase_ = static_cast<float>(divide_);
        } else {
          float expected = maxTrainPhase_ - static_cast<float>(resetCounter_);
          float warp = expected - trainPhase_ + 1.0f;
          frequency_ *= std::max(warp, 0.01f);
        }
        resetInterval_ = static_cast<uint32_t>(std::max(4.0f / targetFrequency_, sampleRate_ * 3.0f));
        current_ = (current_ + 1) % kHistorySize;
      }
      history_[current_].onDuration = 0;
      history_[current_].totalDuration = 0;
    }

    // 2. Measure the current pulse.
    ++history_[current_].totalDuration;
    if (clock) ++history_[current_].onDuration;

    // 3. On the falling edge of a steady clock, retime the rest of the
    // period from the known pulse width.
    if (falling && averagePulseWidth_ > 0.0f) {
      float tOn = static_cast<float>(history_[current_].onDuration);
      float next = maxTrainPhase_ - static_cast<float>(resetCounter_) + 1.0f;
      float pw = averagePulseWidth_;
      frequency_ = std::max(next - trainPhase_, 0.0f) * pw / ((1.0f - pw) * tOn);
    }

    // 4. Run the ramp (stopping at the end of the cycle if no pulse came).
    trainPhase_ = std::min(trainPhase_ + frequency_, maxTrainPhase_);
    float phase = trainPhase_ * fRatio_;
    return phase - static_cast<float>(static_cast<int32_t>(phase));
  }

  // The output ramp's frequency in Hz.
  float frequency() const { return frequency_ * fRatio_ * sampleRate_; }

 private:
  static const int kHistorySize = 16;
  static const int kMaxPatternPeriod = 8;

  struct Pulse {
    uint32_t onDuration;
    uint32_t totalDuration;
    float pulseWidth;
  };

  // Mean pulse width, or 0 if the recent pulses' widths disagree by more
  // than `tolerance`.
  float averagePulseWidth(float tolerance) const {
    float sum = 0.0f;
    const float reference = history_[current_].pulseWidth;
    for (int i = 0; i < kHistorySize; ++i) {
      float w = history_[i].pulseWidth;
      if (w < reference * (1.0f - tolerance) || w > reference * (1.0f + tolerance)) return 0.0f;
      sum += w;
    }
    return sum / kHistorySize;
  }

  // Try "smoothed last period" (pattern 0) and "the period N pulses ago"
  // (patterns 1..8); track each one's recent error and use the best.
  float predictNextPeriod() {
    float lastPeriod = static_cast<float>(history_[current_].totalDuration);
    int best = 0;
    for (int i = 0; i <= kMaxPatternPeriod; ++i) {
      float error = predictedPeriod_[i] - lastPeriod;
      slope(predictionError_[i], error * error, 0.7f, 0.2f);
      if (i == 0) {
        onePole(predictedPeriod_[0], lastPeriod, 0.5f);
      } else {
        int t = (current_ + 1 + kHistorySize - i) % kHistorySize;
        predictedPeriod_[i] = static_cast<float>(history_[t].totalDuration);
      }
      if (predictionError_[i] < predictionError_[best]) best = i;
    }
    return predictedPeriod_[best];
  }

  float sampleRate_ = 48000.0f;
  float minPeriod_ = 1200.0f;
  float ratio_ = 1.0f;
  int divide_ = 1;

  Pulse history_[kHistorySize];
  int current_ = 0;
  float predictionError_[kMaxPatternPeriod + 1];
  float predictedPeriod_[kMaxPatternPeriod + 1];
  float averagePulseWidth_ = 0.0f;
  float trainPhase_ = 0.0f, frequency_ = 0.0f, targetFrequency_ = 0.0f;
  float maxTrainPhase_ = 1.0f, fRatio_ = 1.0f;
  int resetCounter_ = 1;
  uint32_t resetInterval_ = 0;
  bool previousClock_ = false;
};

}  // namespace ml

#endif  // ML_MOD_CLOCK_TO_RAMP_H_
