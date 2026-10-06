// Shared helpers for the golden comparison programs.

#ifndef PT_GOLDEN_H_
#define PT_GOLDEN_H_

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "test.h"

// Largest absolute difference, relative to the original's peak level.
inline float relativeError(const std::vector<float>& a, const std::vector<float>& b,
                           size_t offset = 0) {
  float peak = 1e-9f, err = 0.0f;
  for (size_t i = 0; i + offset < b.size() && i < a.size(); ++i) {
    peak = std::fmax(peak, std::fabs(a[i]));
    err = std::fmax(err, std::fabs(a[i] - b[i + offset]));
  }
  return err / peak;
}

// Largest difference between the loudness envelopes (RMS over 10 ms
// windows) of two signals, relative to the loudest window. For comparing
// noise-like sounds whose waveforms differ only in phase.
inline float envelopeError(const std::vector<float>& a, const std::vector<float>& b, size_t window) {
  float peak = 1e-9f, err = 0.0f;
  for (size_t i = 0; i + window <= a.size() && i + window <= b.size(); i += window) {
    double ea = 0.0, eb = 0.0;
    for (size_t j = i; j < i + window; ++j) {
      ea += a[j] * a[j];
      eb += b[j] * b[j];
    }
    float ra = static_cast<float>(std::sqrt(ea / window)), rb = static_cast<float>(std::sqrt(eb / window));
    peak = std::fmax(peak, ra);
    err = std::fmax(err, std::fabs(ra - rb));
  }
  return err / peak;
}

inline void report(const char* name, float err, float tolerance) {
  std::printf("  %-28s max error %.2e of peak (tolerance %.0e)\n", name, err, tolerance);
  CHECK(err <= tolerance);
}

// A short zero-mean noise burst every `period` samples.
inline std::vector<float> bursts(size_t n, size_t period) {
  std::vector<float> x(n, 0.0f);
  uint32_t s = 1;
  for (size_t i = 0; i < n; ++i) {
    if (i % period < 32) {
      s = s * 1664525u + 1013904223u;
      x[i] = (s / 4294967296.0f - 0.5f);
    }
  }
  return x;
}

#endif  // PT_GOLDEN_H_
