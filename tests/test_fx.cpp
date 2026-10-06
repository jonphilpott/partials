// Phase 2 tests: effects. Each is checked for bounded output, its defining
// property (an all-pass keeps energy, a pitch shifter shifts pitch...), and
// the same behaviour at 44.1, 48 and 96 kHz.

#include <cmath>
#include <cstdio>
#include <vector>

#include "ml/core/random.h"
#include "ml/fx/chorus.h"
#include "ml/fx/decimator.h"
#include "ml/fx/diffuser.h"
#include "ml/fx/ensemble.h"
#include "ml/fx/fx_engine.h"
#include "ml/fx/limiter.h"
#include "ml/fx/overdrive.h"
#include "ml/fx/pitch_shifter.h"
#include "ml/fx/reverb.h"
#include "ml/fx/wavefolder.h"
#include "ml/osc/polyblep.h"

#include "test.h"
#include "wav.h"

static const float kRates[] = {44100.0f, 48000.0f, 96000.0f};
static const float kTwoPi = 6.28318530718f;

static float levelDb(const std::vector<float>& x, float sampleRate, float from, float to) {
  double sum = 0.0;
  size_t a = static_cast<size_t>(from * sampleRate), b = static_cast<size_t>(to * sampleRate);
  for (size_t i = a; i < b; ++i) sum += x[i] * x[i];
  return static_cast<float>(10.0 * std::log10(sum / (b - a) + 1e-30));
}

// Level of one frequency component, averaged over short (about 43 ms)
// Hann-tapered windows in the second half of x. Short windows matter for effects that
// splice audio (the pitch shifter): over a long window, the phase jumps at
// each splice would cancel out.
static float dftMagnitude(const std::vector<float>& x, float sampleRate, float hz) {
  const size_t n = static_cast<size_t>(sampleRate * 2048.0f / 48000.0f);
  double total = 0.0;
  int windows = 0;
  for (size_t start = x.size() / 2; start + n <= x.size(); start += n) {
    double re = 0.0, im = 0.0;
    for (size_t i = 0; i < n; ++i) {
      double angle = 6.283185307179586 * hz * static_cast<double>(i) / sampleRate;
      // The Hann taper stops one frequency leaking into its neighbours.
      double hann = 0.5 - 0.5 * std::cos(6.283185307179586 * i / n);
      re += hann * x[start + i] * std::cos(angle);
      im += hann * x[start + i] * std::sin(angle);
    }
    total += 4.0 * std::sqrt(re * re + im * im) / n;
    ++windows;
  }
  return static_cast<float>(total / windows);
}

static void testEngine() {
  // An impulse written into a line comes back out of its tail.
  ml::FxEngine e;
  e.reset();
  ml::FxEngine::Delay d = e.addDelay(100);
  e.allocate();
  int arrival = -1;
  for (int i = 0; i < 200; ++i) {
    e.start();
    float out;
    e.readTail(d, 1.0f);
    e.write(out, 0.0f);
    if (out > 0.5f && arrival < 0) arrival = i;
    e.read(i == 0 ? 1.0f : 0.0f);
    e.write(d, 0.0f);
  }
  CHECK(arrival == 99);

  // An all-pass passes all energy: impulse in, same total energy out.
  e.reset();
  ml::FxEngine::Delay ap = e.addDelay(37);
  e.allocate();
  double energy = 0.0;
  for (int i = 0; i < 20000; ++i) {
    e.start();
    float out;
    e.read(i == 0 ? 1.0f : 0.0f);
    e.readTail(ap, 0.6f);
    e.writeAllPass(ap, -0.6f);
    e.write(out, 0.0f);
    energy += out * out;
  }
  CHECK_NEAR(energy, 1.0, 1e-3);
}

// The saw example from docs/plumbing.html.
struct DocsSaw {
  float phase = 0.f, next = 0.f;
  float sawProcess(float hz, float sampleTime) {
    float inc = hz * sampleTime;
    float out = next;
    next = 0.f;
    phase += inc;
    if (phase >= 1.f) {
      phase -= 1.f;
      float t = phase / inc;
      out -= ml::thisBlepSample(t);
      next -= ml::nextBlepSample(t);
    }
    next += phase;
    return 2.f * out - 1.f;
  }
};

static void testPolyblep() {
  // A 3.1 kHz saw at 48 kHz: its 15th harmonic (46.5 kHz) folds down to
  // an audible, inharmonic 1.5 kHz. The BLEP version must have far less
  // energy there than a naive ramp. (PolyBLEP helps most with aliases that
  // land low; those near Nyquist are only reduced a little.)
  const float sr = 48000.0f, hz = 3100.0f;
  DocsSaw blep;
  float naivePhase = 0.0f;
  std::vector<float> a(48000), b(48000);
  for (size_t i = 0; i < a.size(); ++i) {
    a[i] = blep.sawProcess(hz, 1.0f / sr);
    naivePhase += hz / sr;
    if (naivePhase >= 1.0f) naivePhase -= 1.0f;
    b[i] = 2.0f * naivePhase - 1.0f;
  }
  CHECK(allBounded(a, 1.2f));
  float aliasBlep = dftMagnitude(a, sr, 1500.0f), aliasNaive = dftMagnitude(b, sr, 1500.0f);
  std::printf("  polyblep saw: alias at 1.5 kHz %.5f (naive %.5f)\n", aliasBlep, aliasNaive);
  CHECK(aliasBlep < 0.2f * aliasNaive);

  // The two halves of a BLEP correction sum to the step's rounding error
  // profile: at t = 0 (jump exactly now) nothing is needed on this sample.
  CHECK(ml::thisBlepSample(0.0f) == 0.0f);
  CHECK_NEAR(ml::nextBlepSample(1.0f), 0.0f, 1e-7f);
  CHECK_NEAR(ml::thisBlepSample(0.5f) - ml::nextBlepSample(0.5f), 0.25f, 1e-7f);
}

static void testReverb() {
  const ml::Reverb::Preset presets[] = {ml::Reverb::RINGS, ml::Reverb::ELEMENTS, ml::Reverb::CLOUDS};
  for (ml::Reverb::Preset preset : presets) {
    float decay[3];
    for (int r = 0; r < 3; ++r) {
      float sr = kRates[r];
      ml::Reverb rev;
      rev.init(sr, preset);
      rev.setAmount(1.0f);
      rev.setTime(0.8f);
      rev.setLp(0.7f);
      std::vector<float> out(static_cast<size_t>(sr * 3));
      for (size_t i = 0; i < out.size(); ++i) {
        float l = i == 0 ? 1.0f : 0.0f, rr = 0.0f;
        rev.process(l, rr);
        out[i] = l;
      }
      CHECK(allBounded(out, 2.0f));
      decay[r] = levelDb(out, sr, 0.3f, 0.4f) - levelDb(out, sr, 1.3f, 1.4f);
      if (preset == ml::Reverb::ELEMENTS && r == 1) writeWav("build/reverb.wav", out, 48000);
    }
    std::printf("  reverb preset %d: %.1f / %.1f / %.1f dB lost in 1 s\n", preset, decay[0], decay[1], decay[2]);
    CHECK(decay[1] > 3.0f);
    for (int r = 0; r < 3; ++r) CHECK_NEAR(decay[r], decay[1], 1.5f);
  }
}

static void testDiffuser() {
  // Fully wet, the diffuser is all-pass: total energy is preserved.
  for (float sr : kRates) {
    ml::Diffuser d;
    d.init(sr);
    d.setAmount(1.0f);
    double inEnergy = 0.0, outEnergy = 0.0;
    for (int i = 0; i < static_cast<int>(sr); ++i) {
      float l = i == 0 ? 1.0f : 0.0f, r = i == 0 ? 0.5f : 0.0f;
      inEnergy += l * l + r * r;
      d.process(l, r);
      outEnergy += l * l + r * r;
    }
    CHECK_NEAR(outEnergy, inEnergy, 1e-3 * inEnergy);
  }
}

static void testModulation() {
  // Chorus and ensemble: bounded; dry when amount = 0.
  for (float sr : kRates) {
    ml::Chorus ch;
    ml::Ensemble en;
    ch.init(sr);
    en.init(sr);
    ch.setAmount(1.0f);
    ch.setDepth(1.0f);
    en.setAmount(1.0f);
    en.setDepth(1.0f);
    std::vector<float> out(static_cast<size_t>(sr * 2));
    for (size_t i = 0; i < out.size(); ++i) {
      float x = std::sin(kTwoPi * 220.0f * i / sr);
      float l = x, r = x, l2 = x, r2 = x;
      ch.process(l, r);
      en.process(l2, r2);
      out[i] = l + r2;
    }
    CHECK(allBounded(out, 4.0f));
    ch.setAmount(0.0f);
    float l = 0.3f, r = -0.2f;
    ch.process(l, r);
    CHECK(l == 0.3f && r == -0.2f);
  }
}

static void testPitchShifter() {
  for (float sr : kRates) {
    ml::PitchShifter ps;
    ps.init(sr);
    ps.setRatio(2.0f);   // up an octave
    ps.setSize(0.6f);
    std::vector<float> out(static_cast<size_t>(sr));
    for (size_t i = 0; i < out.size(); ++i) out[i] = ps.process(std::sin(kTwoPi * 220.0f * i / sr));
    CHECK(allBounded(out, 2.0f));
    // The window crossfades add sidebands, so check the spectrum: far more
    // energy at 440 Hz than at the original 220 Hz.
    float at440 = dftMagnitude(out, sr, 440.0f), at220 = dftMagnitude(out, sr, 220.0f);
    std::printf("  pitch shifter @%.0f: 220 Hz x2 -> level at 440 Hz %.3f, at 220 Hz %.3f\n", sr, at440, at220);
    CHECK(at440 > 10.0f * at220);
  }
}

static void testOverdrive() {
  ml::Overdrive od;
  od.init(48000.0f);
  od.setDrive(0.0f);
  // Low drive: small signals pass at roughly unity gain.
  float y = 0.0f;
  for (int i = 0; i < 1000; ++i) y = od.process(0.1f);
  CHECK_NEAR(y, 0.1f, 0.02f);
  // Full drive: a full-scale sine is squashed but stays within ±1.1.
  od.setDrive(1.0f);
  float peak = 0.0f;
  for (int i = 0; i < 48000; ++i) peak = std::fmax(peak, std::fabs(od.process(std::sin(kTwoPi * 100.0f * i / 48000.0f))));
  CHECK(peak > 0.9f && peak < 1.1f);
}

static void testWavefolder() {
  ml::Wavefolder wf;
  wf.init();
  wf.setAmount(0.0f);
  // Nearly linear at low amount: output follows input's sign.
  CHECK(wf.process(0.5f) > 0.0f && wf.process(-0.5f) < 0.0f);
  // Folding adds harmonics: count zero crossings of a folded sine.
  wf.setAmount(1.0f);
  std::vector<float> out(48000), analog(48000);
  for (size_t i = 0; i < out.size(); ++i) {
    out[i] = wf.process(std::sin(kTwoPi * 100.0f * i / 48000.0f));
    analog[i] = wf.analog();
  }
  CHECK(allBounded(out, 1.01f));
  CHECK(allBounded(analog, 1.01f));
  CHECK(zeroCrossingHz(out, 48000.0f) > 300.0f);   // well above the 100 Hz input
  writeWav("build/wavefolder.wav", out, 48000);
}

static void testDecimator() {
  // Reducing to 1 kHz: the output changes value about 1000 times a second.
  for (float sr : kRates) {
    ml::Decimator d;
    d.init(sr);
    d.setRate(1000.0f);
    std::vector<float> out(static_cast<size_t>(sr));
    for (size_t i = 0; i < out.size(); ++i) out[i] = d.process(std::sin(kTwoPi * 37.0f * i / sr));
    CHECK(allBounded(out, 1.5f));
    int plateaus = 0;
    for (size_t i = 2; i < out.size(); ++i) {
      // A held value: three equal samples in a row (away from the BLEP edges).
      if (out[i] == out[i - 1] && out[i - 1] != out[i - 2]) ++plateaus;
    }
    CHECK(plateaus > 900 && plateaus < 1100);
  }
  // At the host rate, it passes the signal through.
  ml::Decimator d;
  d.init(48000.0f);
  d.setRate(48000.0f);
  CHECK(d.process(0.25f) == 0.25f);
}

static void testLimiter() {
  for (float sr : kRates) {
    ml::Limiter lim;
    lim.init(sr);
    lim.setPreGain(4.0f);   // drive a full-scale sine 12 dB over
    float peak = 0.0f;
    for (int i = 0; i < static_cast<int>(sr); ++i) {
      float l = std::sin(kTwoPi * 100.0f * i / sr), r = l;
      lim.process(l, r);
      if (i > sr / 2) peak = std::fmax(peak, std::fabs(l));
    }
    CHECK(peak <= 1.0f);
    CHECK(peak > 0.6f);
  }
}

// The FxEngine example from docs/fx.html, verbatim, to keep it honest.
struct PingPong {
    ml::FxEngine e;
    ml::FxEngine::Delay left, right, diffuse;
    float damp = 0.f;

    void init(float sr) {
        e.reset();
        left = e.addDelay(int(sr * 0.25f));     // 250 ms
        right = e.addDelay(int(sr * 0.25f));
        diffuse = e.addDelay(int(sr * 0.007f)); // 7 ms all-pass
        e.allocate();
    }

    void process(float in, float feedback, float& outL, float& outR) {
        e.start();
        // Left line: input plus feedback from the right, smeared and damped
        e.read(in);
        e.readTail(right, feedback);
        e.readTail(diffuse, 0.5f);
        e.writeAllPass(diffuse, -0.5f);
        e.lp(damp, 0.3f);
        e.write(left, 0.f);
        // Right line: fed only by the left, so echoes alternate sides
        e.readTail(left, 1.f);
        e.write(right, 0.f);
        // Outputs: the far end of each line
        e.readTail(left, 1.f);
        e.write(outL, 0.f);
        e.readTail(right, 1.f);
        e.write(outR, 0.f);
    }
};

static void testDocsPingPong() {
  // An impulse should echo left at ~250 ms, right at ~500 ms, left at ~750 ms.
  const float sr = 48000.0f;
  PingPong pp;
  pp.init(sr);
  std::vector<float> l(static_cast<size_t>(sr)), r(l.size());
  for (size_t i = 0; i < l.size(); ++i) pp.process(i == 0 ? 1.0f : 0.0f, 0.7f, l[i], r[i]);
  CHECK(allBounded(l, 2.0f) && allBounded(r, 2.0f));
  auto loudestIn = [&](const std::vector<float>& x, float from, float to) {
    size_t best = 0;
    for (size_t i = static_cast<size_t>(from * sr); i < static_cast<size_t>(to * sr); ++i) {
      if (std::fabs(x[i]) > std::fabs(x[best])) best = i;
    }
    return best / sr;
  };
  CHECK_NEAR(loudestIn(l, 0.1f, 0.4f), 0.257f, 0.02f);
  CHECK_NEAR(loudestIn(r, 0.4f, 0.6f), 0.507f, 0.02f);
  CHECK_NEAR(loudestIn(l, 0.6f, 0.9f), 0.757f, 0.02f);
}

int main() {
  testDocsPingPong();
  testEngine();
  testPolyblep();
  testReverb();
  testDiffuser();
  testModulation();
  testPitchShifter();
  testOverdrive();
  testWavefolder();
  testDecimator();
  testLimiter();
  return testResult("fx");
}
