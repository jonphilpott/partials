// Phase 4 tests: dynamics and modulation. Each component is checked for its
// defining behaviour and, where timing matters, the same timing at 44.1, 48
// and 96 kHz.

#include <cmath>
#include <cstdio>
#include <vector>

#include "pt/dynamics/compressor.h"
#include "pt/dynamics/follower.h"
#include "pt/dynamics/vactrol.h"
#include "pt/mod/bouncing_ball.h"
#include "pt/mod/clock_to_ramp.h"
#include "pt/mod/envelope.h"
#include "pt/mod/hysteresis_quantizer.h"
#include "pt/mod/lag.h"
#include "pt/mod/lorenz.h"
#include "pt/mod/quantizer.h"
#include "pt/mod/random_sequence.h"
#include "pt/mod/slope.h"

#include "test.h"
#include "wav.h"

static const float kRates[] = {44100.0f, 48000.0f, 96000.0f};
static const float kTwoPi = 6.28318530718f;

// Seconds until x first crosses `level` (upwards if rising), or -1.
static float timeTo(const std::vector<float>& x, float sr, float level, bool rising, size_t from = 0) {
  for (size_t i = from; i < x.size(); ++i) {
    if (rising ? x[i] >= level : x[i] <= level) return (i - from) / sr;
  }
  return -1.0f;
}

static void testVactrol() {
  float rise[3], fall[3], pluck[3];
  for (int r = 0; r < 3; ++r) {
    float sr = kRates[r];
    pt::Vactrol v;
    v.init(sr);
    v.setAttack(0.02f);
    v.setDecay(0.5f);
    // Light on for 1 s, off for 3 s.
    std::vector<float> g(static_cast<size_t>(sr * 4));
    for (size_t i = 0; i < g.size(); ++i) g[i] = v.process(i < sr ? 1.0f : 0.0f);
    CHECK(allBounded(g, 1.01f));
    rise[r] = timeTo(g, sr, 0.9f, true);
    fall[r] = timeTo(g, sr, 0.1f, false, static_cast<size_t>(sr));

    pt::Vactrol p;
    p.init(sr);
    p.setPlucked(true);
    p.setDecay(0.5f);
    std::vector<float> h(static_cast<size_t>(sr * 3));
    for (size_t i = 0; i < h.size(); ++i) h[i] = p.process(i < sr * 0.01f ? 1.0f : 0.0f);
    float peak = 0.0f;
    size_t peakAt = 0;
    for (size_t i = 0; i < h.size(); ++i) if (h[i] > peak) { peak = h[i]; peakAt = i; }
    pluck[r] = timeTo(h, sr, peak * 0.1f, false, peakAt);
  }
  std::printf("  vactrol rise %.3f/%.3f/%.3f s, fall %.3f/%.3f/%.3f s, pluck %.3f/%.3f/%.3f s\n",
              rise[0], rise[1], rise[2], fall[0], fall[1], fall[2], pluck[0], pluck[1], pluck[2]);
  for (int r = 0; r < 3; ++r) {
    CHECK(rise[r] > 0.0f && fall[r] > 0.0f && pluck[r] > 0.0f);
    CHECK_NEAR(rise[r], rise[1], 0.05f * rise[1] + 0.002f);
    CHECK_NEAR(fall[r], fall[1], 0.05f * fall[1] + 0.002f);
    CHECK_NEAR(pluck[r], pluck[1], 0.05f * pluck[1] + 0.002f);
  }
}

static float rmsDb(const std::vector<float>& x, size_t from) {
  double s = 0.0;
  for (size_t i = from; i < x.size(); ++i) s += x[i] * x[i];
  return static_cast<float>(10.0 * std::log10(s / (x.size() - from)));
}

static void testCompressor() {
  // A full-scale sine (RMS -3 dB), threshold -20 dB, ratio 4: 17 dB over,
  // so the output should sit 17/4 = 4.25 dB over the threshold: -15.75 dB.
  pt::Compressor c;
  c.init(48000.0f);
  c.setThreshold(-20.0f);
  c.setRatio(4.0f);
  // Equal attack and release: the detector settles on the mean power
  // (true RMS). A fast attack makes it follow the peaks instead.
  c.setAttack(0.2f);
  c.setRelease(0.2f);
  std::vector<float> out(48000);
  for (size_t i = 0; i < out.size(); ++i) out[i] = c.process(std::sin(kTwoPi * 200.0f * i / 48000.0f));
  float level = rmsDb(out, 24000);
  std::printf("  compressor: -3 dB RMS in, %.2f dB out (expected -15.75)\n", level);
  CHECK_NEAR(level, -15.75f, 0.5f);
  CHECK_NEAR(c.gainReduction(), 12.75f, 0.5f);

  // Below the threshold: untouched.
  c.setThreshold(0.0f);
  for (size_t i = 0; i < out.size(); ++i) out[i] = c.process(0.5f * std::sin(kTwoPi * 200.0f * i / 48000.0f));
  CHECK_NEAR(rmsDb(out, 24000), -9.03f, 0.1f);

  // Sidechain ducking: a loud sidechain turns down a quiet signal.
  pt::Compressor d;
  d.init(48000.0f);
  d.setThreshold(-30.0f);
  d.setRatio(10.0f);
  float y = 0.0f;
  for (int i = 0; i < 24000; ++i) y = d.process(0.1f, std::sin(kTwoPi * 50.0f * i / 48000.0f));
  CHECK(std::fabs(y) < 0.05f);
}

static void testFollower() {
  // Low tone: dark; high tone: bright. Envelope tracks level at any rate.
  for (float sr : kRates) {
    float centroid[2];
    float envelope = 0.0f;
    const float freqs[2] = {80.0f, 6000.0f};
    for (int k = 0; k < 2; ++k) {
      pt::Follower f;
      f.init(sr);
      for (int i = 0; i < static_cast<int>(sr); ++i) envelope = f.process(0.5f * std::sin(kTwoPi * freqs[k] * i / sr));
      centroid[k] = f.centroid();
    }
    CHECK(centroid[0] < 0.2f);
    CHECK(centroid[1] > 0.7f);
    CHECK(envelope > 0.1f && envelope < 1.0f);
  }
}

static void testLorenz() {
  // Bounded, and wing switches (sign changes of x) happen at the same
  // rate whatever the sample rate.
  float switches[3];
  for (int r = 0; r < 3; ++r) {
    float sr = kRates[r];
    pt::Lorenz l;
    l.init(sr);
    l.setRate(0.8f);
    std::vector<float> x(static_cast<size_t>(sr * 10)), z(x.size());
    for (size_t i = 0; i < x.size(); ++i) {
      x[i] = l.process();
      z[i] = l.z();
    }
    CHECK(allBounded(x, 1.5f));
    CHECK(allBounded(z, 1.5f));
    int s = 0;
    for (size_t i = 1; i < x.size(); ++i) if ((x[i - 1] < 0.0f) != (x[i] < 0.0f)) ++s;
    switches[r] = static_cast<float>(s);
  }
  std::printf("  lorenz wing switches in 10 s: %.0f/%.0f/%.0f\n", switches[0], switches[1], switches[2]);
  CHECK(switches[1] > 5.0f);
  // Chaos amplifies tiny differences, so only the statistics should agree.
  for (int r = 0; r < 3; ++r) CHECK_NEAR(switches[r], switches[1], 0.4f * switches[1]);
}

static void testEnvelope() {
  for (float sr : kRates) {
    pt::Envelope e;
    e.init(sr);
    e.setAdsr(0.1f, 0.2f, 0.5f, 0.3f);
    // Gate on for 1 s, then off.
    std::vector<float> out(static_cast<size_t>(sr * 2));
    for (size_t i = 0; i < out.size(); ++i) out[i] = e.process(i < sr);
    CHECK(allBounded(out, 1.0001f));
    CHECK_NEAR(timeTo(out, sr, 0.999f, true), 0.1f, 0.005f);   // attack
    CHECK_NEAR(out[static_cast<size_t>(sr * 0.9f)], 0.5f, 1e-3f);  // sustain
    CHECK(out.back() == 0.0f && !e.active());                   // released
    float releaseTime = timeTo(out, sr, 0.0005f, false, static_cast<size_t>(sr));
    CHECK_NEAR(releaseTime, 0.3f, 0.01f);

    // Looping AD: an LFO with a 0.25 s cycle.
    e.setAdLoop(0.1f, 0.15f);
    std::vector<float> lfo(static_cast<size_t>(sr * 2));
    e.process(false);
    for (size_t i = 0; i < lfo.size(); ++i) lfo[i] = e.process(true);
    int peaks = 0;
    for (size_t i = 1; i + 1 < lfo.size(); ++i) if (lfo[i] >= lfo[i - 1] && lfo[i] > lfo[i + 1]) ++peaks;
    CHECK(peaks == 8);
  }
}

static void testBouncingBall() {
  float firstBounce[3];
  for (int r = 0; r < 3; ++r) {
    float sr = kRates[r];
    pt::BouncingBall b;
    b.init(sr);
    b.setGravity(0.5f);
    b.setBounce(0.5f);
    std::vector<float> out(static_cast<size_t>(sr * 2));
    for (size_t i = 0; i < out.size(); ++i) out[i] = b.process(i == 0);
    CHECK(allBounded(out, 1.0f));
    firstBounce[r] = timeTo(out, sr, 0.0f, false);
    // Bounces get lower: the peak after the first impact is below 1.
    float laterPeak = 0.0f;
    for (size_t i = static_cast<size_t>(firstBounce[r] * sr) + 10; i < out.size(); ++i) laterPeak = std::fmax(laterPeak, out[i]);
    CHECK(laterPeak < 0.9f && laterPeak > 0.1f);
  }
  for (int r = 0; r < 3; ++r) CHECK_NEAR(firstBounce[r], firstBounce[1], 0.01f * firstBounce[1] + 0.001f);
}

static void testSlope() {
  for (float sr : kRates) {
    // AD at 2 Hz: one 0.5 s envelope, rising to 1 at the slope point.
    pt::Slope s;
    s.init(sr);
    s.setMode(pt::Slope::AD);
    s.setFrequency(2.0f);
    s.setSlope(0.5f);
    std::vector<float> ad(static_cast<size_t>(sr));
    for (size_t i = 0; i < ad.size(); ++i) ad[i] = s.process(i < 100);
    CHECK(allBounded(ad, 1.01f));
    float maxV = 0.0f;
    for (float v : ad) maxV = std::fmax(maxV, v);
    CHECK(maxV > 0.99f);
    CHECK_NEAR(timeTo(ad, sr, 0.001f, false, static_cast<size_t>(sr * 0.3f)) + 0.3f, 0.5f, 0.01f);

    // Looping at 5 Hz: ±1, 5 cycles per second.
    pt::Slope lfo;
    lfo.init(sr);
    lfo.setMode(pt::Slope::LOOPING);
    lfo.setFrequency(5.0f);
    std::vector<float> l(static_cast<size_t>(sr));
    for (size_t i = 0; i < l.size(); ++i) l[i] = lfo.process(false);
    CHECK(allBounded(l, 1.01f));
    CHECK_NEAR(zeroCrossingHz(l, sr), 5.0f, 0.5f);

    // Audio range, sweeping every knob: bounded.
    pt::Slope osc;
    osc.init(sr);
    osc.setMode(pt::Slope::LOOPING);
    osc.setRange(pt::Slope::AUDIO);
    std::vector<float> a(static_cast<size_t>(sr));
    for (size_t i = 0; i < a.size(); ++i) {
      float t = static_cast<float>(i) / a.size();
      osc.setFrequency(50.0f + 4000.0f * t);
      osc.setSlope(t);
      osc.setShape(1.0f - t);
      osc.setSmoothness(t);
      a[i] = osc.process(false);
    }
    CHECK(allBounded(a, 1.5f));

    // AR: holds at 1 while the gate is high, then falls.
    pt::Slope ar;
    ar.init(sr);
    ar.setMode(pt::Slope::AR);
    ar.setFrequency(10.0f);
    float held = 0.0f;
    for (int i = 0; i < static_cast<int>(sr * 0.5f); ++i) held = ar.process(true);
    CHECK_NEAR(held, 1.0f, 1e-3f);
    CHECK(ar.endOfAttack());
    float released = 0.0f;
    for (int i = 0; i < static_cast<int>(sr * 0.5f); ++i) released = ar.process(false);
    CHECK_NEAR(released, 0.0f, 1e-3f);
    CHECK(ar.endOfRelease());
  }
}

static void testClockToRamp() {
  const float sr = 48000.0f;
  // A steady 2 Hz clock: after a few pulses the ramp wraps on each pulse.
  for (int mult = 1; mult <= 2; ++mult) {
    pt::ClockToRamp c;
    c.init(sr);
    c.setRatio(mult, 1);
    int wraps = 0;
    float previous = 0.0f;
    for (int i = 0; i < static_cast<int>(sr * 6); ++i) {
      bool clock = (i % 24000) < 2400;
      float ramp = c.process(clock);
      if (i > sr * 2 && ramp < previous - 0.5f) ++wraps;
      previous = ramp;
    }
    CHECK_NEAR(wraps, 8 * mult, 1);
    CHECK_NEAR(c.frequency(), 2.0f * mult, 0.05f);
  }
  // A swung clock (300 ms, 200 ms, ...): once the predictor has learnt the
  // pattern, the ramp reads about 0.9 at 90% of every period, long or
  // short. (Without prediction it would run at the average speed and be
  // off by about 20% on each.)
  pt::ClockToRamp c;
  c.init(sr);
  std::vector<int> pulses;
  for (int t = 0, k = 0; t < static_cast<int>(sr * 10); t += static_cast<int>(sr * ((k++ & 1) ? 0.2f : 0.3f))) {
    pulses.push_back(t);
  }
  size_t p = 0;
  int lastPulse = -100000;
  float worst = 0.0f;
  for (int i = 0; i < static_cast<int>(sr * 10); ++i) {
    if (p < pulses.size() && i == pulses[p]) {
      lastPulse = i;
      ++p;
    }
    float ramp = c.process(i - lastPulse < static_cast<int>(sr * 0.01f));
    if (p < pulses.size() && i > sr * 5 && i == lastPulse + (pulses[p] - lastPulse) * 9 / 10) {
      worst = std::fmax(worst, std::fabs(ramp - 0.9f));
    }
  }
  std::printf("  clock to ramp: swung clock, worst error at 90%% of a period %.3f\n", worst);
  CHECK(worst < 0.05f);
}

static void testQuantizers() {
  // HysteresisQuantizer: 4 steps; hovering at a boundary doesn't flicker.
  pt::HysteresisQuantizer h;
  h.init(4);
  CHECK(h.process(0.1f) == 0);
  CHECK(h.process(0.35f) == 1);
  CHECK(h.process(0.51f) == 1);   // just over the 1/2 boundary: holds
  CHECK(h.process(0.49f) == 1);
  CHECK(h.process(0.6f) == 2);    // clearly over: moves
  CHECK(h.process(0.45f) == 2);   // just under: holds
  CHECK(h.process(0.4f) == 1);    // clearly under: moves
  CHECK(h.process(0.9f) == 3);

  pt::Quantizer q;
  q.init(pt::Quantizer::preset(pt::Quantizer::MAJOR));
  CHECK(q.process(0.37f, 0.0f) == 0.37f);                     // off
  CHECK_NEAR(q.process(0.37f, 0.2f, false), 0.3333f, 1e-4f);  // chromatic: nearest semitone (E)
  CHECK_NEAR(q.process(1.45f, 1.0f, false), 1.0f, 1e-4f);     // root only: C
  CHECK_NEAR(q.process(-0.4f, 1.0f, false), 0.0f, 1e-4f);     // below zero works too
  // At "C major" density, F# snaps to a scale note.
  float v = q.process(0.5f, 0.45f, false);
  CHECK(std::fabs(v - 0.4167f) < 1e-3f || std::fabs(v - 0.5833f) < 1e-3f);
}

static void testLag() {
  pt::Lag l;
  l.init();
  // Smoothness 0: instant.
  CHECK_NEAR(l.process(1.0f, 0.0f, 0.01f), 1.0f, 1e-3f);
  // High smoothness: halfway through a step, about halfway between values.
  l.init();
  l.process(0.0f, 1.0f, 0.0f);
  l.resetRamp();
  float y = 0.0f;
  for (int i = 1; i <= 50; ++i) y = l.process(1.0f, 1.0f, i / 100.0f);
  CHECK_NEAR(y, 0.5f, 0.05f);
}

static void testRandomSequence() {
  // Deja vu 0.5: locked; the sequence repeats every `length` steps.
  pt::RandomSequence r;
  r.init();
  r.setLength(5);
  r.setDejaVu(0.5f);
  std::vector<float> s;
  for (int i = 0; i < 20; ++i) s.push_back(r.next());
  bool repeats = true;
  for (int i = 5; i < 20; ++i) if (s[i] != s[i - 5]) repeats = false;
  CHECK(repeats);
  // Deja vu 0: always new.
  r.setDejaVu(0.0f);
  float a = r.next(), b = r.next(), c = r.next();
  CHECK(a != b && b != c);
  // External values become the loop.
  r.setDejaVu(0.0f);
  for (int i = 0; i < 5; ++i) r.next(i * 0.1f);
  r.setDejaVu(0.5f);
  for (int i = 0; i < 10; ++i) CHECK_NEAR(r.next(), (i % 5) * 0.1f, 1e-6f);
}

int main() {
  testVactrol();
  testCompressor();
  testFollower();
  testLorenz();
  testEnvelope();
  testBouncingBall();
  testSlope();
  testClockToRamp();
  testQuantizers();
  testLag();
  testRandomSequence();
  return testResult("mod");
}
