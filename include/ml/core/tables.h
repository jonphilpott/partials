// mutablelib — core/tables.h
//
// Lookup tables that can be computed rather than stored.
//
// Gillet's firmware ships tables pre-computed by Python scripts, because
// filling them at boot would cost time and RAM a Eurorack module doesn't
// have. On a desktop we fill them on first use instead.
//
// How "first use" works: each table lives in a `static` local variable
// inside an `inline` function.
// - C++11 guarantees the static is built exactly once, thread-safely, the
//   first time the function runs.
// - Because the function is `inline`, the linker keeps a single copy even
//   though every .cpp file that includes this header compiles its own.
// Components call these from init() so the one-off cost never lands on
// the audio thread.

#ifndef ML_CORE_TABLES_H_
#define ML_CORE_TABLES_H_

#include <cmath>

namespace ml {

const int kSineTableSize = 512;  // steps per cycle (a power of 2)

// One cycle of sin(), plus a quarter cycle and two guard points at the end
// (512 + 128 + 2 entries). The extra quarter means cosine can be read from
// the same table at an offset of kSineTableSize / 4 without wrapping.
// Matches Plaits' lut_sine (plus one more guard point, which a cosine read
// at exactly the end of the cycle touches).
inline const float* sineTable() {
  struct Table {
    float data[kSineTableSize + kSineTableSize / 4 + 2];
    Table() {
      for (int i = 0; i < kSineTableSize + kSineTableSize / 4 + 2; ++i) {
        data[i] = static_cast<float>(
            std::sin(2.0 * 3.14159265358979323846 * i / kSineTableSize));
      }
    }
  };
  static const Table table;
  return table.data;
}

// The tables below are shared by the physical models (modal resonator,
// string). Each has 257 entries for an index of 0..1 in 256 steps, plus one
// guard entry so interpolate() can safely read one past the end at index 1.
// They are generated exactly as Gillet's Python scripts did, so the ported
// components sound identical.
struct Lut257 {
  float data[258];
};

// 10^(4x) for x = 0..1: a 4-decade (1 to 10000) exponential curve. Maps a
// 0..1 knob onto a range wide enough for decay times or Q.
inline const float* lut4Decades() {
  struct Make {
    static Lut257 build() {
      Lut257 t;
      for (int i = 0; i < 257; ++i) {
        t.data[i] = static_cast<float>(std::pow(10.0, 4.0 * i / 256.0));
      }
      t.data[257] = t.data[256];
      return t;
    }
  };
  static const Lut257 table = Make::build();
  return table.data;
}

// "Stiffness" against the resonator's structure knob (0..1): how much each
// successive partial is stretched away from a pure harmonic series.
// - below 0.25: negative, partials squeezed together (bell-like, inharmonic)
// - 0.25..0.3: zero, a perfect harmonic series (string)
// - 0.3..0.9: rising exponentially (stiffer: bars, plates)
// - above 0.9: large, widely spread partials (metallic)
inline const float* lutStiffness() {
  struct Make {
    static Lut257 build() {
      Lut257 t;
      const double pi = 3.14159265358979323846;
      for (int i = 0; i < 257; ++i) {
        double g = i / 256.0;
        double s;
        if (g < 0.25) {
          s = -(0.25 - g) * 0.25;
        } else if (g < 0.3) {
          s = 0.0;
        } else if (g < 0.9) {
          s = 0.01 * std::pow(10.0, (g - 0.3) / 0.6 * 2.005) - 0.01;
        } else {
          double h = (g - 0.9) / 0.1;
          s = 1.5 - std::cos(h * h * pi) / 2.0;
        }
        t.data[i] = static_cast<float>(s);
      }
      t.data[255] = t.data[256] = t.data[257] = 2.0f;
      return t;
    }
  };
  static const Lut257 table = Make::build();
  return table.data;
}

// Phase delay of the string's damping low-pass filter, indexed by its
// cutoff in semitones above the string's pitch (0..256, read with size 1).
// The string shortens its delay line by this amount so the filter doesn't
// pull the note flat.
inline const float* lutSvfShift() {
  struct Make {
    static Lut257 build() {
      Lut257 t;
      const double pi = 3.14159265358979323846;
      for (int i = 0; i < 257; ++i) {
        double ratio = std::pow(2.0, i / 12.0);
        t.data[i] = static_cast<float>(2.0 * std::atan(1.0 / ratio) / (2.0 * pi));
      }
      t.data[257] = t.data[256];
      return t;
    }
  };
  static const Lut257 table = Make::build();
  return table.data;
}

}  // namespace ml

#endif  // ML_CORE_TABLES_H_
