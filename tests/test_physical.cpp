// Phase 1 tests: plumbing (SVF, delay line, DC blocker, cosine oscillator)
// and the physical models and LPG.
//
// The key check for every component: it sounds the same at any sample
// rate. We render at 44.1, 48 and 96 kHz and compare the pitch
// (zero-crossing count) and decay time (time to fall 40 dB) in seconds.

#include <cmath>
#include <cstdio>
#include <vector>

#include "pt/core/random.h"
#include "pt/dynamics/lpg.h"
#include "pt/filter/dc_blocker.h"
#include "pt/filter/one_pole.h"
#include "pt/filter/svf.h"
#include "pt/fx/delay_line.h"
#include "pt/osc/sine.h"
#include "pt/physical/modal_resonator.h"
#include "pt/physical/plucker.h"
#include "pt/physical/string.h"
#include "pt/physical/tube.h"

#include "test.h"
#include "wav.h"

static const float kRates[] = {44100.0f, 48000.0f, 96000.0f};

// Seconds until the peak level (over 5 ms windows) falls 40 dB below the
// loudest window. Windows before `start` seconds are ignored, to skip an
// excitation that is much louder than the ringing it causes.
static float decayTime(const std::vector<float>& x, float sampleRate, float start = 0.0f) {
  size_t window = static_cast<size_t>(sampleRate * 0.005f);
  std::vector<float> peaks;
  for (size_t i = static_cast<size_t>(start * sampleRate); i + window <= x.size(); i += window) {
    float p = 0.0f;
    for (size_t j = i; j < i + window; ++j) p = std::fmax(p, std::fabs(x[j]));
    peaks.push_back(p);
  }
  float top = 0.0f;
  size_t topIndex = 0;
  for (size_t i = 0; i < peaks.size(); ++i) {
    if (peaks[i] > top) { top = peaks[i]; topIndex = i; }
  }
  for (size_t i = topIndex; i < peaks.size(); ++i) {
    if (peaks[i] < top * 0.01f) return i * 0.005f;
  }
  return peaks.size() * 0.005f;
}

// RMS level in dB of x between two times (seconds).
static float levelDb(const std::vector<float>& x, float sampleRate, float from, float to) {
  double sum = 0.0;
  size_t a = static_cast<size_t>(from * sampleRate), b = static_cast<size_t>(to * sampleRate);
  for (size_t i = a; i < b; ++i) sum += x[i] * x[i];
  return static_cast<float>(10.0 * std::log10(sum / (b - a) + 1e-30));
}

static void testPlumbing() {
  // SVF low-pass passes DC, high-pass blocks it.
  pt::Svf svf;
  svf.init(48000.0f);
  svf.setFrequency(1000.0f, 0.707f);
  pt::SvfOut o = {0, 0, 0};
  for (int i = 0; i < 48000; ++i) o = svf.process(1.0f);
  CHECK_NEAR(o.lp, 1.0f, 1e-3f);
  CHECK_NEAR(o.hp, 0.0f, 1e-3f);
  // Approximations agree with tan() at low frequencies.
  for (pt::TanApprox a : {pt::TanApprox::Accurate, pt::TanApprox::Fast, pt::TanApprox::Dirty}) {
    CHECK_NEAR(pt::tanApprox(0.01f, a), std::tan(3.14159265f * 0.01f), 1e-4f);
  }

  // One-pole: -3 dB at the cutoff, DC through the low-pass.
  pt::OnePole op;
  op.init(48000.0f);
  op.setFrequency(1000.0f);
  pt::OnePoleOut p = {0, 0};
  for (int i = 0; i < 48000; ++i) p = op.process(1.0f);
  CHECK_NEAR(p.lp, 1.0f, 1e-4f);
  CHECK_NEAR(p.hp, 0.0f, 1e-4f);
  op.reset();
  float peak = 0.0f;
  for (int i = 0; i < 48000; ++i) {
    float y = op.process(std::sin(2.0f * 3.14159265f * 1000.0f * i / 48000.0f)).lp;
    if (i > 24000) peak = std::fmax(peak, std::fabs(y));
  }
  CHECK_NEAR(peak, 0.7071f, 0.01f);

  // Delay line: read(n) returns the sample written n writes ago.
  pt::DelayLine d;
  d.init(100);
  for (int i = 1; i <= 50; ++i) d.write(static_cast<float>(i));
  CHECK(d.read(static_cast<size_t>(1)) == 50.0f);
  CHECK(d.read(static_cast<size_t>(10)) == 41.0f);
  CHECK_NEAR(d.read(10.5f), 40.5f, 1e-6f);

  // clear() zeroes a range of buffer positions and leaves the write head
  // alone; clearing 0 .. size() in chunks wipes the line. End is clamped.
  {
    pt::DelayLine c;
    c.init(100);
    for (int i = 1; i <= 50; ++i) c.write(static_cast<float>(i));
    for (size_t pos = 0; pos < c.size(); pos += 7) c.clear(pos, pos + 7);
    bool silent = true;
    for (size_t n = 1; n < c.size(); ++n) silent = silent && c.read(n) == 0.0f;
    CHECK(silent);
    c.write(1.0f);
    CHECK(c.read(static_cast<size_t>(1)) == 1.0f);  // write position kept
  }
  CHECK_NEAR(d.readHermite(10.5f), 40.5f, 1e-5f);  // exact on a ramp

  // DC blocker removes an offset.
  pt::DcBlocker dc;
  dc.init(48000.0f, 10.0f);
  float y = 0.0f;
  for (int i = 0; i < 48000; ++i) y = dc.process(1.0f);
  CHECK(std::fabs(y) < 1e-3f);

  // Cosine oscillator: 0.5 + 0.5*cos(n*w).
  pt::CosineOscillator c;
  c.init(0.1f);
  for (int n = 0; n < 20; ++n) {
    CHECK_NEAR(c.next(), 0.5f + 0.5f * std::cos(2.0f * 3.14159265f * 0.1f * n), 1e-4f);
  }
}

static void testModalResonator() {
  float hz[3], decay[3];
  for (int r = 0; r < 3; ++r) {
    float sr = kRates[r];
    pt::ModalResonator res;
    res.init(sr);
    res.setFrequency(220.0f);
    res.setStructure(0.27f);  // harmonic
    res.setBrightness(0.5f);
    res.setDamping(0.5f);
    res.setPosition(0.25f);
    res.setModes(1);  // fundamental only, so zero crossings give the pitch
    std::vector<float> out(static_cast<size_t>(sr * 3));
    for (size_t i = 0; i < out.size(); ++i) out[i] = res.process(i == 0 ? 1.0f : 0.0f);
    hz[r] = zeroCrossingHz(std::vector<float>(out.begin(), out.begin() + static_cast<size_t>(sr * 0.5f)), sr);
    decay[r] = decayTime(out, sr);
    std::printf("  resonator @%.0f: %.2f Hz, decay %.3f s\n", sr, hz[r], decay[r]);
  }
  for (int r = 0; r < 3; ++r) {
    CHECK_NEAR(hz[r], 220.0f, 3.0f);
    CHECK_NEAR(decay[r], decay[1], 0.02f * decay[1] + 0.01f);
  }

  // Full resonator with every knob swept, bowed modes on: stays bounded.
  pt::ModalResonator res;
  res.init(48000.0f);
  res.setBowedModes(true);
  pt::Random rng;
  std::vector<float> out(48000 * 10);
  for (size_t i = 0; i < out.size(); ++i) {
    if (i % 4800 == 0) {
      res.setFrequency(40.0f + rng.uniform() * 2000.0f);
      res.setStructure(rng.uniform());
      res.setBrightness(rng.uniform());
      res.setDamping(rng.uniform() * 0.95f);
      res.setPosition(rng.uniform());
    }
    float in = (i % 9600 == 0) ? 1.0f : 0.0f;
    out[i] = res.process(in, 0.5f);
  }
  CHECK(allBounded(out, 20.0f));
  writeWav("build/modal_resonator.wav", out, 48000);
}

static void testString() {
  float hz[3], decay[3];
  for (int r = 0; r < 3; ++r) {
    float sr = kRates[r];
    pt::String s;
    s.init(sr);
    s.setFrequency(110.0f);
    s.setBrightness(0.2f);
    s.setDamping(0.6f);
    s.setDispersion(0.5f);
    std::vector<float> out(static_cast<size_t>(sr * 4));
    // Zero-mean excitation: an impulse with DC in it would leave a slowly
    // decaying offset, since the loop passes DC like any other harmonic.
    for (size_t i = 0; i < out.size(); ++i) out[i] = s.process(i < 5 ? 0.5f : (i < 10 ? -0.5f : 0.0f));
    hz[r] = autocorrHz(std::vector<float>(out.begin() + static_cast<size_t>(sr * 0.2f),
                                          out.begin() + static_cast<size_t>(sr * 0.7f)), sr, 50.0f, 500.0f);
    // Decay rate: dB lost between 1 s and 3 s. (Absolute levels differ
    // between rates because a 10-sample excitation is shorter at 96 kHz.)
    decay[r] = levelDb(out, sr, 1.0f, 1.1f) - levelDb(out, sr, 3.0f, 3.1f);
    std::printf("  string @%.0f: %.2f Hz, %.1f dB lost from 1 s to 3 s\n", sr, hz[r], decay[r]);
  }
  for (int r = 0; r < 3; ++r) {
    CHECK_NEAR(hz[r], 110.0f, 2.0f);
    CHECK_NEAR(decay[r], decay[1], 1.0f);
  }

  // Sweep dispersion through both curved bridge and stiff/rattle regions.
  pt::String s;
  s.init(48000.0f);
  std::vector<float> out(48000 * 8);
  pt::Random rng;
  for (size_t i = 0; i < out.size(); ++i) {
    if (i % 4800 == 0) {
      s.setFrequency(30.0f + rng.uniform() * 1000.0f);
      s.setDispersion(rng.uniform());
      s.setBrightness(rng.uniform());
      s.setDamping(rng.uniform());
      s.setPosition(rng.uniform());
    }
    out[i] = s.process(i % 4800 < 20 ? 0.5f : 0.0f);
  }
  CHECK(allBounded(out, 20.0f));
  writeWav("build/string.wav", out, 48000);
}

static void testTube() {
  float hz[3];
  for (int r = 0; r < 3; ++r) {
    float sr = kRates[r];
    pt::Tube t;
    t.init(sr);
    t.setFrequency(220.0f);
    t.setDamping(0.5f);
    t.setTimbre(0.3f);
    // Steady breath, no noise, so the pitch is easy to measure.
    std::vector<float> out(static_cast<size_t>(sr));
    for (size_t i = 0; i < out.size(); ++i) out[i] = t.process(0.0f, 1.0f);
    CHECK(allBounded(out, 10.0f));
    hz[r] = autocorrHz(std::vector<float>(out.begin() + out.size() / 2, out.end()), sr, 50.0f, 1000.0f);
    std::printf("  tube @%.0f: %.2f Hz\n", sr, hz[r]);
  }
  for (int r = 0; r < 3; ++r) CHECK_NEAR(hz[r], 220.0f, 3.0f);

  // With breath noise: bounded, and a file to listen to.
  pt::Tube t;
  t.init(48000.0f);
  t.setFrequency(220.0f);
  pt::Random rng;
  std::vector<float> out(48000 * 2);
  for (size_t i = 0; i < out.size(); ++i) out[i] = t.process(rng.bipolar() * 0.2f, 0.8f);
  CHECK(allBounded(out, 10.0f));
  writeWav("build/tube.wav", out, 48000);
}

static void testPlucker() {
  pt::Plucker p;
  p.init(48000.0f);
  p.setFrequency(220.0f);
  p.setCutoff(4000.0f);
  p.setPosition(0.3f);
  std::vector<float> out(48000);
  for (size_t i = 0; i < out.size(); ++i) {
    if (i % 12000 == 0) p.trigger();
    out[i] = p.process();
  }
  CHECK(allBounded(out, 5.0f));
  float energy = 0.0f;
  for (float v : out) energy += v * v;
  CHECK(energy > 0.0f);
}

static void testLpg() {
  float decay[3];
  for (int r = 0; r < 3; ++r) {
    float sr = kRates[r];
    pt::LowPassGate lpg;
    lpg.init(sr);
    lpg.setDecay(0.5f);
    lpg.setColour(0.5f);
    std::vector<float> out(static_cast<size_t>(sr * 3));
    lpg.trigger();
    // A 100 Hz square wave through the gate.
    for (size_t i = 0; i < out.size(); ++i) {
      float in = std::fmod(i * 100.0f / sr, 1.0f) < 0.5f ? 1.0f : -1.0f;
      out[i] = lpg.process(in);
    }
    CHECK(allBounded(out, 2.0f));
    decay[r] = decayTime(out, sr);
  }
  CHECK(decay[1] > 0.05f);
  for (int r = 0; r < 3; ++r) CHECK_NEAR(decay[r], decay[1], 0.05f * decay[1] + 0.01f);

  // Level mode: closed at 0, open at 1.
  pt::LowPassGate lpg;
  lpg.init(48000.0f);
  float y = 0.0f;
  for (int i = 0; i < 48000; ++i) y = lpg.process(1.0f, 0.0f);
  CHECK(std::fabs(y) < 1e-3f);
  for (int i = 0; i < 48000; ++i) y = lpg.process(1.0f, 1.0f);
  CHECK(y > 0.9f);
}

int main() {
  testPlumbing();
  testModalResonator();
  testString();
  testTube();
  testPlucker();
  testLpg();
  return testResult("physical");
}
