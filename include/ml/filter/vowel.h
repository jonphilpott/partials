// mutablelib — filter/vowel.h
//
// VowelFilter: a "talking" filter. Five resonant band-pass filters tuned
// to the formants (resonances) of a human voice, morphing between five
// vowels and five voice types. Put any bright source through it (a pulse
// train, a saw, noise) and it speaks. From Plaits' simple speech
// synthesiser.
//
// How it works: a vowel is mostly defined by where the throat and mouth
// resonate, at a handful of frequencies called formants. A table gives
// the five formants (frequency and level) for each vowel and voice type;
// `vowel` and `voice` blend between table entries, and the five filters
// are retuned to the blend.
//
// Sample-rate independence: the formants are in Hz. (Plaits used its
// cheap tan() approximation; this does too, which slightly lowers the
// highest formants at 44.1/48 kHz, as in Plaits.)
//
// Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.

#ifndef ML_FILTER_VOWEL_H_
#define ML_FILTER_VOWEL_H_

#include <cmath>
#include <cstdint>

#include "ml/core/math.h"
#include "ml/core/units.h"
#include "ml/filter/svf.h"

namespace ml {

class VowelFilter {
 public:
  static const int kNumFormants = 5;

  void init(float sampleRate) {
    sampleRate_ = sampleRate;
    for (Svf& f : filter_) f.init(sampleRate);
    update();
  }

  // 0..1. Morphs through Plaits' five vowels in order (roughly
  // "a", "e", "i", "o", "u").
  void setVowel(float x) { vowel_ = clamp(x, 0.0f, 1.0f); update(); }
  // 0..1. Voice type, from low (bass) to high (soprano/child).
  void setVoice(float x) { voice_ = clamp(x, 0.0f, 1.0f); update(); }
  // Shift all formants up or down (semitones), e.g. for a smaller or larger
  // "head".
  void setShift(float semitones) { shift_ = semitones; update(); }

  // Returns the input through the five formant filters. The resonances
  // are strong: scale the input down (Plaits drives it with a pulse train
  // at about ±1 and gets roughly ±1 out).
  float process(float in) {
    float out = 0.0f;
    for (int i = 0; i < kNumFormants; ++i) out += gain_[i] * filter_[i].process(in).bp;
    return out;
  }

 private:
  // Each formant as {pitch as a MIDI note number, level 0..255}.
  struct Formant {
    uint8_t note;
    uint8_t level;
  };

  void update() {
    static const Formant kTable[5][5][kNumFormants] = {
        {{{74, 255}, {83, 114}, {97, 90}, {98, 90}, {100, 25}},
         {{75, 255}, {84, 128}, {100, 114}, {101, 101}, {103, 20}},
         {{76, 255}, {85, 128}, {100, 18}, {102, 16}, {104, 3}},
         {{79, 255}, {85, 161}, {101, 25}, {104, 4}, {110, 0}},
         {{79, 255}, {85, 128}, {101, 6}, {106, 25}, {110, 0}}},
        {{{67, 255}, {91, 64}, {98, 90}, {101, 64}, {102, 32}},
         {{67, 255}, {92, 51}, {99, 64}, {103, 51}, {105, 25}},
         {{69, 255}, {93, 51}, {100, 32}, {102, 25}, {103, 25}},
         {{67, 255}, {91, 16}, {100, 8}, {103, 4}, {110, 0}},
         {{65, 255}, {95, 25}, {101, 45}, {105, 2}, {110, 0}}},
        {{{59, 255}, {92, 8}, {99, 40}, {102, 20}, {104, 10}},
         {{61, 255}, {94, 45}, {101, 32}, {103, 25}, {105, 8}},
         {{60, 255}, {93, 16}, {101, 16}, {104, 4}, {105, 4}},
         {{65, 255}, {92, 25}, {100, 8}, {105, 4}, {110, 0}},
         {{60, 255}, {96, 64}, {101, 12}, {106, 12}, {110, 1}}},
        {{{67, 255}, {78, 72}, {98, 22}, {99, 25}, {101, 2}},
         {{67, 255}, {79, 80}, {99, 64}, {101, 64}, {102, 12}},
         {{68, 255}, {79, 80}, {100, 12}, {102, 20}, {103, 5}},
         {{69, 255}, {79, 90}, {101, 40}, {104, 10}, {110, 0}},
         {{69, 255}, {79, 72}, {101, 20}, {106, 20}, {110, 0}}},
        {{{65, 255}, {74, 25}, {98, 6}, {100, 10}, {101, 4}},
         {{65, 255}, {74, 25}, {100, 36}, {101, 51}, {103, 12}},
         {{66, 255}, {75, 25}, {100, 18}, {102, 8}, {104, 5}},
         {{63, 255}, {77, 64}, {99, 8}, {104, 2}, {110, 0}},
         {{63, 255}, {77, 40}, {100, 4}, {106, 2}, {110, 0}}},
    };
    // 1. Position in the 5 x 5 table, blended bilinearly.
    float p = vowel_ * (5 - 1.001f);
    float r = voice_ * (5 - 1.001f);
    int pi = static_cast<int>(p), ri = static_cast<int>(r);
    float pf = p - pi, rf = r - ri;
    for (int i = 0; i < kNumFormants; ++i) {
      const Formant& a = kTable[pi][ri][i];
      const Formant& b = kTable[pi][ri + 1][i];
      const Formant& c = kTable[pi + 1][ri][i];
      const Formant& d = kTable[pi + 1][ri + 1][i];
      float f0 = a.note + (b.note - a.note) * rf;
      float f1 = c.note + (d.note - c.note) * rf;
      float note = std::fmin(f0 + (f1 - f0) * pf, 160.0f);
      float l0 = a.level + (b.level - a.level) * rf;
      float l1 = c.level + (d.level - c.level) * rf;
      gain_[i] = (l0 + (l1 - l0) * pf) / 256.0f;
      // 2. The note number is semitones above 55 Hz (MIDI 33), as in
      // Plaits.
      float hz = 55.0f * semitonesToRatio(note - 33.0f + shift_);
      filter_[i].setCoefficients(std::fmin(hz / sampleRate_, 0.49f), 20.0f, TanApprox::Dirty);
    }
  }

  float sampleRate_ = 48000.0f;
  float vowel_ = 0.0f, voice_ = 0.5f, shift_ = 0.0f;
  float gain_[kNumFormants] = {};
  Svf filter_[kNumFormants];
};

}  // namespace ml

#endif  // ML_FILTER_VOWEL_H_
