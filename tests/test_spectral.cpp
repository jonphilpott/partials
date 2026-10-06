// Phase 6 tests: cross-modulation, Hilbert transform, vowel filter,
// vocoder, FM operators and onset detector.

#include <cmath>
#include <cstdio>
#include <vector>

#include "pt/analysis/onset_detector.h"
#include "pt/core/random.h"
#include "pt/filter/hilbert.h"
#include "pt/filter/vowel.h"
#include "pt/osc/basic.h"
#include "pt/spectral/fm_operators.h"
#include "pt/spectral/vocoder.h"
#include "pt/spectral/xmod.h"

#include "test.h"
#include "wav.h"

static const float kRates[] = {44100.0f, 48000.0f, 96000.0f};
static const double kTwoPi = 6.283185307179586;

// Level of one frequency over the second half of x (Hann-windowed blocks).
static float level(const std::vector<float>& x, float sr, float hz) {
  const size_t n = static_cast<size_t>(sr * 0.05f);
  double total = 0.0;
  int windows = 0;
  for (size_t start = x.size() / 2; start + n <= x.size(); start += n) {
    double re = 0.0, im = 0.0;
    for (size_t i = 0; i < n; ++i) {
      double w = 0.5 - 0.5 * std::cos(kTwoPi * i / n);
      double a = kTwoPi * hz * static_cast<double>(i) / sr;
      re += w * x[start + i] * std::cos(a);
      im += w * x[start + i] * std::sin(a);
    }
    total += 4.0 * std::sqrt(re * re + im * im) / n;
    ++windows;
  }
  return static_cast<float>(total / windows);
}

static void testXmod() {
  CHECK_NEAR(pt::Xmod::xfade(1.0f, 0.0f, 0.0f), 0.0f, 1e-6f);      // all carrier
  CHECK_NEAR(pt::Xmod::xfade(0.0f, 1.0f, 0.0f), 0.7071f, 1e-3f);
  CHECK_NEAR(pt::Xmod::xfade(1.0f, 0.0f, 1.0f), 0.7071f, 1e-3f);   // all modulator
  CHECK(pt::Xmod::digitalRing(0.5f, 0.0f, 0.5f) == 0.0f);
  CHECK(pt::Xmod::digitalRing(0.5f, 0.5f, 0.0f) > 0.0f && pt::Xmod::digitalRing(0.5f, -0.5f, 0.0f) < 0.0f);
  CHECK_NEAR(pt::Xmod::bitwiseXor(0.2f, 0.3f, 0.0f), 0.35f, 1e-6f);  // param 0: plain sum
  CHECK_NEAR(pt::Xmod::comparator(0.2f, -0.4f, 0.0f), -0.4f, 1e-6f); // param 0: minimum
  CHECK_NEAR(pt::Xmod::morph(1.0f, 0.3f, 0.6f, 0.5f), pt::Xmod::comparator(0.3f, 0.6f, 0.5f) * 0.001f + 0.3f * 0.999f, 1e-3f);
  // Every algorithm, random inputs: finite and in a sane range.
  pt::Random rng;
  for (int a = 0; a <= pt::Xmod::NOP; ++a) {
    float peak = 0.0f;
    for (int i = 0; i < 20000; ++i) {
      float y = pt::Xmod::process(static_cast<pt::Xmod::Algorithm>(a), rng.bipolar(), rng.bipolar(), rng.uniform());
      CHECK(std::isfinite(y));
      peak = std::fmax(peak, std::fabs(y));
    }
    CHECK(peak < 6.0f);
  }
}

static void testHilbert() {
  // Warps' design at 96 kHz reproduces its table (sorted, negated).
  const float kWarps[17] = {9.999174437e-01f, 9.997160329e-01f, 9.993897602e-01f, 9.987952776e-01f,
                            9.976718129e-01f, 9.955280098e-01f, 9.914315323e-01f, 9.836199785e-01f,
                            9.688016569e-01f, 9.409767040e-01f, 8.897147107e-01f, 7.984785110e-01f,
                            6.454684139e-01f, 4.118108699e-01f, 9.725667152e-02f, -2.775386379e-01f,
                            -7.176356738e-01f};
  float c[17];
  pt::Hilbert::designPoles(96000.0f, c);
  float worst = 0.0f;
  for (int i = 0; i < 17; ++i) worst = std::fmax(worst, std::fabs(c[i] - (-kWarps[i])));
  std::printf("  hilbert: design vs Warps' table, worst %.2e\n", worst);
  CHECK(worst < 1e-5f);

  // I and Q: equal level and 90 degrees apart across the audio band.
  for (float sr : kRates) {
    for (float hz : {50.0f, 500.0f, 5000.0f, 15000.0f}) {
      pt::Hilbert h;
      h.init(sr);
      double ii = 0.0, qq = 0.0, iq = 0.0;
      int n = static_cast<int>(sr);
      for (int k = 0; k < n; ++k) {
        float i = h.process(static_cast<float>(std::sin(kTwoPi * hz * k / sr)));
        float q = h.q();
        if (k > n / 2) {
          ii += i * i;
          qq += q * q;
          iq += i * q;
        }
      }
      // Equal power; correlation ~0 means 90 degrees apart.
      CHECK_NEAR(ii / qq, 1.0, 0.03);
      CHECK(std::fabs(iq) / std::sqrt(ii * qq) < 0.03);
    }
  }
}

static void testVowel() {
  // An impulse train through each vowel: bounded, and the first formant
  // moves (vowel 0 has F1 near 590 Hz, vowel 0.5 near 200 Hz).
  float f1Low[2], f1High[2];
  const float vowels[2] = {0.0f, 0.5f};
  for (int v = 0; v < 2; ++v) {
    pt::BasicOscillator pulse;
    pulse.init(48000.0f);
    pulse.setShape(pt::BasicOscillator::IMPULSE_TRAIN);
    pulse.setFrequency(100.0f);
    pt::VowelFilter f;
    f.init(48000.0f);
    f.setVowel(vowels[v]);
    f.setVoice(0.0f);
    std::vector<float> out(48000);
    for (float& x : out) x = f.process(pulse.process() * 0.2f);
    CHECK(allBounded(out, 4.0f));
    f1Low[v] = level(out, 48000.0f, 200.0f);
    f1High[v] = level(out, 48000.0f, 600.0f);
    if (v == 0) writeWav("build/vowel.wav", out, 48000);
  }
  CHECK(f1High[0] > 2.0f * f1Low[0]);
  CHECK(f1Low[1] > 2.0f * f1High[1]);
}

static void testVocoder() {
  for (float sr : kRates) {
    // Modulator: a 1 kHz tone for the first half, then silence. Carrier: a
    // 110 Hz saw (harmonics every 110 Hz).
    pt::BasicOscillator saw;
    saw.init(sr);
    saw.setFrequency(110.0f);
    pt::Vocoder v;
    v.init(sr);
    v.setRelease(0.3f);
    std::vector<float> on(static_cast<size_t>(sr)), off(static_cast<size_t>(sr));
    for (size_t i = 0; i < on.size(); ++i) {
      on[i] = v.process(0.5f * static_cast<float>(std::sin(kTwoPi * 1000.0 * i / sr)), saw.process());
    }
    for (size_t i = 0; i < off.size(); ++i) off[i] = v.process(0.0f, saw.process());
    CHECK(allBounded(on, 1.0f));
    // The carrier comes through around 1 kHz (harmonics 8-10), far less
    // at 220 Hz and 4 kHz.
    float near = std::fmax(std::fmax(level(on, sr, 880.0f), level(on, sr, 990.0f)), level(on, sr, 1100.0f));
    float far = level(on, sr, 3960.0f), low = level(on, sr, 220.0f);
    std::printf("  vocoder @%.0f: around 1 kHz %.4f, 220 Hz %.4f, 4 kHz %.4f\n", sr, near, low, far);
    CHECK(near > 10.0f * far);
    CHECK(near > 10.0f * low);
    // Silence on the modulator: silence out.
    CHECK(level(off, sr, 990.0f) < 0.01f * near);
    if (sr == 48000.0f) writeWav("build/vocoder.wav", on, 48000);
  }
  // Formant shift extremes stay bounded.
  pt::Vocoder v;
  v.init(48000.0f);
  pt::Random rng;
  for (float shift : {0.0f, 1.0f}) {
    v.setFormantShift(shift);
    float peak = 0.0f;
    for (int i = 0; i < 48000; ++i) peak = std::fmax(peak, std::fabs(v.process(rng.bipolar(), rng.bipolar())));
    CHECK(peak <= 1.0f && peak > 0.01f);
  }
}

static void testFm() {
  for (float sr : kRates) {
    // DX7 algorithm 32: six carriers. Only operator 1 sounding: a pure sine.
    pt::FmOperators fm;
    fm.init(sr, 6);
    fm.setAlgorithm(32);
    fm.setFrequency(220.0f);
    fm.setLevel(1, 0.8f);
    std::vector<float> pure(static_cast<size_t>(sr / 2));
    for (float& x : pure) x = fm.process();
    CHECK(allBounded(pure, 0.81f));
    CHECK_NEAR(level(pure, sr, 220.0f), 0.8f, 0.02f);
    CHECK(level(pure, sr, 440.0f) < 0.001f);

    // Algorithm 1: operator 2 modulates operator 1 (ratio 1:1). Harmonics
    // appear as the modulator's level rises.
    fm.setAlgorithm(1);
    fm.setLevel(2, 1.0f);
    fm.setRatio(2, 1.0f);
    std::vector<float> bright(static_cast<size_t>(sr / 2));
    for (float& x : bright) x = fm.process();
    CHECK(allBounded(bright, 1.0f));
    CHECK(level(bright, sr, 440.0f) > 0.05f);
    CHECK_NEAR(autocorrHz(std::vector<float>(bright.begin() + bright.size() / 2, bright.end()), sr, 150.0f, 300.0f),
               220.0f, 3.0f);
  }
  // Every algorithm, every operator at moderate level, feedback on: bounded.
  for (int ops : {4, 6}) {
    for (int a = 1; a <= (ops == 4 ? 8 : 32); ++a) {
      pt::FmOperators fm;
      fm.init(48000.0f, ops);
      fm.setAlgorithm(a);
      fm.setFrequency(330.0f);
      fm.setFeedback(0.6f);
      for (int op = 1; op <= ops; ++op) {
        fm.setRatio(op, 0.5f * op);
        fm.setLevel(op, 0.5f);
      }
      float peak = 0.0f;
      for (int i = 0; i < 4800; ++i) peak = std::fmax(peak, std::fabs(fm.process()));
      CHECK(peak > 0.1f && peak <= 3.01f);  // at most 6 carriers x 0.5
    }
  }
}

static void testOnsets() {
  for (float sr : kRates) {
    // Five decaying 300 Hz notes, half a second apart.
    pt::OnsetDetector d;
    d.init(sr);  // default: 50 ms between onsets
    std::vector<float> times;
    int n = static_cast<int>(sr * 2.6f);
    for (int k = 0; k < n; ++k) {
      float t = k / sr;
      float local = std::fmod(t, 0.5f);
      float x = t > 0.1f ? static_cast<float>(std::sin(kTwoPi * 300.0 * k / sr)) * std::exp(-local * 10.0f) : 0.0f;
      if (d.process(0.5f * x)) times.push_back(t);
    }
    std::printf("  onsets @%.0f:", sr);
    for (float t : times) std::printf(" %.3f", t);
    std::printf("\n");
    // One onset per note, within 10 ms of it (notes at 0.5, 1.0, ... 2.5;
    // the first starts at 0.1 s mid-decay, so it may or may not register).
    int matched = 0;
    for (float t : times) {
      float nearest = std::round(t * 2.0f) / 2.0f;
      if (std::fabs(t - nearest) < 0.01f && nearest >= 0.5f) ++matched;
    }
    CHECK(matched == 5);
    CHECK(times.size() <= 6);
  }
}

int main() {
  testXmod();
  testHilbert();
  testVowel();
  testVocoder();
  testFm();
  testOnsets();
  return testResult("spectral");
}
