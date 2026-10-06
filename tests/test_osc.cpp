// Phase 5 tests: oscillators and noise. Each oscillator is checked for
// pitch and bounded output at 44.1, 48 and 96 kHz, plus its defining
// feature; each noise source for its rate and range.

#include <cmath>
#include <cstdio>
#include <vector>

#include "pt/noise/clocked_noise.h"
#include "pt/noise/dust.h"
#include "pt/noise/particle.h"
#include "pt/noise/smooth_random.h"
#include "pt/osc/formant.h"
#include "pt/osc/grainlet.h"
#include "pt/osc/harmonic.h"
#include "pt/osc/string_synth.h"
#include "pt/osc/variable_shape.h"
#include "pt/osc/vosim.h"
#include "pt/osc/wavetable.h"
#include "pt/osc/z_osc.h"

#include "test.h"
#include "wav.h"

static const float kRates[] = {44100.0f, 48000.0f, 96000.0f};
static const float kTwoPi = 6.28318530718f;

// Render `seconds` of an oscillator configured by `setup` at rate `sr`.
template <typename Osc, typename Setup>
static std::vector<float> render(float sr, float seconds, Setup setup) {
  Osc o;
  o.init(sr);
  setup(o);
  std::vector<float> out(static_cast<size_t>(sr * seconds));
  for (float& x : out) x = o.process();
  return out;
}

// Check pitch and bounds at every rate.
template <typename Osc, typename Setup>
static void checkPitch(const char* name, float expectedHz, float bound, Setup setup) {
  float hz[3];
  for (int r = 0; r < 3; ++r) {
    std::vector<float> out = render<Osc>(kRates[r], 0.5f, setup);
    CHECK(allBounded(out, bound));
    hz[r] = autocorrHz(std::vector<float>(out.begin() + out.size() / 2, out.end()), kRates[r],
                       expectedHz * 0.6f, expectedHz * 1.6f);
    if (r == 1) {
      char path[64];
      std::snprintf(path, sizeof path, "build/%s.wav", name);
      writeWav(path, out, 48000);
    }
  }
  std::printf("  %-16s %.1f/%.1f/%.1f Hz (expected %.1f)\n", name, hz[0], hz[1], hz[2], expectedHz);
  for (int r = 0; r < 3; ++r) CHECK_NEAR(hz[r], expectedHz, 0.02f * expectedHz);
}

// Level of one frequency (Hann-windowed, averaged over short windows).
static float level(const std::vector<float>& x, float sr, float hz) {
  const size_t n = static_cast<size_t>(sr * 0.05f);
  double total = 0.0;
  int windows = 0;
  for (size_t start = 0; start + n <= x.size(); start += n) {
    double re = 0.0, im = 0.0;
    for (size_t i = 0; i < n; ++i) {
      double w = 0.5 - 0.5 * std::cos(6.283185307179586 * i / n);
      double a = 6.283185307179586 * hz * static_cast<double>(i) / sr;
      re += w * x[start + i] * std::cos(a);
      im += w * x[start + i] * std::sin(a);
    }
    total += 4.0 * std::sqrt(re * re + im * im) / n;
    ++windows;
  }
  return static_cast<float>(total / windows);
}

static void testOscillators() {
  for (float shape : {0.0f, 0.5f, 1.0f}) {
    checkPitch<pt::VariableShapeOscillator>("variable_shape", 220.0f, 1.2f, [shape](pt::VariableShapeOscillator& o) {
      o.setFrequency(220.0f);
      o.setShape(shape);
      o.setPulseWidth(0.3f);
    });
  }
  // Hard sync: the pitch is the master's, whatever the slave does.
  checkPitch<pt::VariableShapeOscillator>("variable_sync", 110.0f, 1.2f, [](pt::VariableShapeOscillator& o) {
    o.setSyncFrequency(110.0f);
    o.setFrequency(317.0f);
    o.setShape(0.5f);
  });
  checkPitch<pt::FormantOscillator>("formant", 150.0f, 1.2f, [](pt::FormantOscillator& o) {
    o.setCarrierFrequency(150.0f);
    o.setFormantFrequency(1100.0f);
  });
  checkPitch<pt::ZOscillator>("z_osc", 150.0f, 1.5f, [](pt::ZOscillator& o) {
    o.setCarrierFrequency(150.0f);
    o.setFormantFrequency(900.0f);
    o.setShape(0.3f);
    o.setMode(0.5f);
  });
  checkPitch<pt::VosimOscillator>("vosim", 120.0f, 1.2f, [](pt::VosimOscillator& o) {
    o.setCarrierFrequency(120.0f);
    o.setFormantFrequencies(700.0f, 1200.0f);
    o.setShape(0.5f);
  });
  checkPitch<pt::GrainletOscillator>("grainlet", 150.0f, 1.2f, [](pt::GrainletOscillator& o) {
    o.setCarrierFrequency(150.0f);
    o.setFormantFrequency(800.0f);
    o.setShape(0.5f);
    o.setBleed(0.3f);
  });
  checkPitch<pt::StringSynthOscillator>("string_synth", 110.0f, 1.5f, [](pt::StringSynthOscillator& o) {
    o.setFrequency(110.0f);
    o.setRegistration(0.3f);
  });
  checkPitch<pt::HarmonicOscillator>("harmonic", 200.0f, 1.2f, [](pt::HarmonicOscillator& o) {
    o.setFrequency(200.0f);
    const float a[8] = {0.5f, 0.2f, 0.1f, 0.05f, 0.05f, 0.05f, 0.03f, 0.02f};
    o.setAmplitudes(a, 8);
  });

  // Harmonic oscillator: only harmonic 3 set -> a pure tone at 3x.
  {
    std::vector<float> out = render<pt::HarmonicOscillator>(48000.0f, 0.5f, [](pt::HarmonicOscillator& o) {
      o.setFrequency(200.0f);
      o.setAmplitude(1, 0.0f);
      o.setAmplitude(3, 1.0f);
    });
    CHECK(level(out, 48000.0f, 600.0f) > 0.9f);
    CHECK(level(out, 48000.0f, 200.0f) < 0.01f);
  }

  // Variable shape at a high pitch: aliasing stays low. A 5 kHz saw at
  // 48 kHz has its 10th harmonic at 50 kHz, which a naive saw folds to
  // 2 kHz.
  {
    std::vector<float> out = render<pt::VariableShapeOscillator>(48000.0f, 0.5f, [](pt::VariableShapeOscillator& o) {
      o.setFrequency(5000.0f);
      o.setShape(0.5f);
    });
    float alias = level(out, 48000.0f, 2000.0f), fundamental = level(out, 48000.0f, 5000.0f);
    std::printf("  variable_shape 5 kHz saw: fundamental %.3f, alias at 2 kHz %.5f\n", fundamental, alias);
    CHECK(alias < 0.01f * fundamental);
  }
}

static void testWavetable() {
  // Two waves: a sine and a saw, 256 samples each.
  const int n = 256;
  std::vector<float> waves(2 * n);
  for (int i = 0; i < n; ++i) {
    waves[i] = std::sin(kTwoPi * i / n);
    waves[n + i] = 2.0f * i / n - 1.0f;
  }
  for (float morph : {0.0f, 1.0f}) {
    checkPitch<pt::WavetableOscillator>("wavetable", 220.0f, 1.5f, [&](pt::WavetableOscillator& o) {
      o.loadWaves(waves.data(), n, 2);
      o.setFrequency(220.0f);
      o.setMorph(morph);
    });
  }
  // Morph 0 is the sine: almost no second harmonic. Morph 1 is the saw:
  // second harmonic at about half the fundamental.
  for (float morph : {0.0f, 1.0f}) {
    std::vector<float> out = render<pt::WavetableOscillator>(48000.0f, 0.5f, [&](pt::WavetableOscillator& o) {
      o.loadWaves(waves.data(), n, 2);
      o.setFrequency(220.0f);
      o.setMorph(morph);
    });
    float ratio = level(out, 48000.0f, 440.0f) / level(out, 48000.0f, 220.0f);
    if (morph == 0.0f) CHECK(ratio < 0.02f);
    else CHECK_NEAR(ratio, 0.5f, 0.1f);
  }
  // Anti-aliasing: a 5 kHz saw read naively from the table vs the
  // oscillator, compared at the 2 kHz alias.
  std::vector<float> naive(24000), good;
  float phase = 0.0f;
  for (float& x : naive) {
    phase += 5000.0f / 48000.0f;
    if (phase >= 1.0f) phase -= 1.0f;
    x = waves[n + static_cast<int>(phase * n)];
  }
  good = render<pt::WavetableOscillator>(48000.0f, 0.5f, [&](pt::WavetableOscillator& o) {
    o.loadWaves(waves.data(), n, 2);
    o.setFrequency(5000.0f);
    o.setMorph(1.0f);
  });
  float aliasNaive = level(naive, 48000.0f, 2000.0f), aliasGood = level(good, 48000.0f, 2000.0f);
  std::printf("  wavetable 5 kHz saw: alias at 2 kHz %.4f (naive table read %.4f)\n", aliasGood, aliasNaive);
  CHECK(aliasGood < 0.3f * aliasNaive);
}

static void testNoise() {
  for (float sr : kRates) {
    // Dust: about `density` impulses per second.
    pt::Dust d;
    d.init(sr);
    d.setDensity(500.0f);
    int count = 0;
    for (int i = 0; i < static_cast<int>(sr * 4); ++i) {
      float x = d.process();
      CHECK(x >= 0.0f && x <= 1.0f);
      if (x > 0.0f) ++count;
    }
    CHECK_NEAR(count / 4.0f, 500.0f, 30.0f);

    // Clocked noise at 100 Hz: about 100 new values per second.
    pt::ClockedNoise c;
    c.init(sr);
    c.setFrequency(100.0f);
    std::vector<float> cn(static_cast<size_t>(sr * 2));
    for (float& x : cn) x = c.process();
    CHECK(allBounded(cn, 1.5f));
    int steps = 0;
    for (size_t i = 2; i < cn.size(); ++i) {
      // A step: a big change after a run of equal samples.
      if (cn[i - 1] == cn[i - 2] && std::fabs(cn[i] - cn[i - 1]) > 1e-4f) ++steps;
    }
    CHECK_NEAR(steps / 2.0f, 100.0f, 5.0f);

    // Smooth random: within ±1, and continuous (small steps per sample).
    pt::SmoothRandom s;
    s.init(sr);
    s.setFrequency(10.0f);
    float previous = s.process(), maxStep = 0.0f;
    for (int i = 0; i < static_cast<int>(sr * 2); ++i) {
      float x = s.process();
      CHECK(x >= -1.0f && x <= 1.0f);
      maxStep = std::fmax(maxStep, std::fabs(x - previous));
      previous = x;
    }
    CHECK(maxStep < 2.0f * 1.5f * 10.0f / sr + 1e-4f);

    // Particle: bounded, makes a sound.
    pt::Particle p;
    p.init(sr);
    p.setDensity(200.0f);
    p.setFrequency(1000.0f);
    p.setSpread(12.0f);
    p.setQ(30.0f);
    std::vector<float> pn(static_cast<size_t>(sr));
    for (float& x : pn) x = p.process();
    CHECK(allBounded(pn, 4.0f));
    float energy = 0.0f;
    for (float x : pn) energy += x * x;
    CHECK(energy > 0.0f);
    if (sr == 48000.0f) writeWav("build/particle.wav", pn, 48000);
  }
}

int main() {
  testOscillators();
  testWavetable();
  testNoise();
  return testResult("osc");
}
