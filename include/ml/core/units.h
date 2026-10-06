// mutablelib — core/units.h
//
// Conversions between musical units (semitones, volts) and frequency.
//
// The library itself works in Hz. These helpers bridge to VCV Rack's pitch
// standard: 1 V per octave, with 0 V = C4 = 261.6256 Hz.
//
// stmlib used two lookup tables for semitonesToRatio() because a Cortex-M4
// has no fast exp(); on a desktop CPU std::exp2 is fast enough.

#ifndef ML_CORE_UNITS_H_
#define ML_CORE_UNITS_H_

#include <cmath>

namespace ml {

const float kFreqC4 = 261.6256f;

// Frequency ratio for an interval: +12 semitones = 2.0, -12 = 0.5.
inline float semitonesToRatio(float semitones) {
  return std::exp2(semitones / 12.0f);
}

// Rack 1V/oct voltage to Hz. Each volt doubles the frequency, hence exp2.
inline float voltToHz(float volts) {
  return kFreqC4 * std::exp2(volts);
}

inline float hzToVolt(float hz) {
  return std::log2(hz / kFreqC4);
}

// MIDI note number to Hz (A4 = note 69 = 440 Hz).
inline float midiToHz(float note) {
  return 440.0f * semitonesToRatio(note - 69.0f);
}

}  // namespace ml

#endif  // ML_CORE_UNITS_H_
