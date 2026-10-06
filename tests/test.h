// mutablelib — tests/test.h
//
// The whole test "framework": a CHECK macro that reports the failing line
// and counts failures, and helpers for the checks every component gets.

#ifndef ML_TESTS_TEST_H_
#define ML_TESTS_TEST_H_

#include <cmath>
#include <cstdio>
#include <vector>

// Failure count, shared by every file of a test program (a function-local
// static in an inline function is one object program-wide).
inline int& failureCount() {
  static int n = 0;
  return n;
}

#define CHECK(cond)                                                  \
  do {                                                               \
    if (!(cond)) {                                                   \
      std::printf("%s:%d: CHECK failed: %s\n", __FILE__, __LINE__,   \
                  #cond);                                            \
      ++failureCount();                                              \
    }                                                                \
  } while (0)

#define CHECK_NEAR(a, b, tol) CHECK(std::fabs((a) - (b)) <= (tol))

// Every sample finite and within ±limit.
inline bool allBounded(const std::vector<float>& x, float limit) {
  for (float v : x) {
    if (!std::isfinite(v) || std::fabs(v) > limit) return false;
  }
  return true;
}

// Estimate the fundamental frequency by counting upward zero crossings.
// Crude, but good enough to show that pitch doesn't shift with sample rate.
inline float zeroCrossingHz(const std::vector<float>& x, float sampleRate) {
  int crossings = 0;
  for (size_t i = 1; i < x.size(); ++i) {
    if (x[i - 1] < 0.0f && x[i] >= 0.0f) ++crossings;
  }
  return crossings * sampleRate / static_cast<float>(x.size());
}

// Estimate pitch from autocorrelation: how well the signal matches a copy
// of itself shifted by `lag` samples. It matches at every whole number of
// periods, so we take the shortest lag that scores within 10% of the best,
// which avoids locking onto 2 or 3 periods. Robust to strong harmonics,
// unlike zero crossings.
inline float autocorrHz(const std::vector<float>& x, float sampleRate,
                        float minHz, float maxHz) {
  size_t minLag = static_cast<size_t>(sampleRate / maxHz);
  size_t maxLag = static_cast<size_t>(sampleRate / minHz);
  std::vector<double> score;
  double best = -1e30;
  for (size_t lag = minLag; lag <= maxLag && lag < x.size(); ++lag) {
    double sum = 0.0;
    for (size_t i = 0; i + lag < x.size(); ++i) sum += x[i] * x[i + lag];
    sum /= static_cast<double>(x.size() - lag);
    score.push_back(sum);
    if (sum > best) best = sum;
  }
  // First local maximum that is within 10% of the best score.
  for (size_t i = 1; i + 1 < score.size(); ++i) {
    if (score[i] >= 0.9 * best && score[i] >= score[i - 1] && score[i] >= score[i + 1]) {
      return sampleRate / static_cast<float>(minLag + i);
    }
  }
  return 0.0f;
}

inline int testResult(const char* name) {
  std::printf("%s: %s\n", name, failureCount() ? "FAIL" : "ok");
  return failureCount() ? 1 : 0;
}

#endif  // ML_TESTS_TEST_H_
