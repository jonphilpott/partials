// Golden comparison, phase 4: modulation and dynamics. Only the float
// originals (Rings, Tides 2, Marbles) and Peaks' envelope can be compared
// directly; the Streams components were rewritten for float and are tested
// for behaviour in tests/test_mod.cpp instead.

#include <cstdlib>
#include <cstring>

#include "marbles/random/lag_processor.h"
#include "marbles/random/quantizer.h"
#include "peaks/modulations/multistage_envelope.h"
#include "rings/dsp/follower.h"
#include "tides2/poly_slope_generator.h"
#include "tides2/ramp/ramp_extractor.h"

#include "ml/dynamics/follower.h"
#include "ml/mod/clock_to_ramp.h"
#include "ml/mod/envelope.h"
#include "ml/mod/lag.h"
#include "ml/mod/quantizer.h"
#include "ml/mod/slope.h"

#include "golden.h"

static void follower() {
  const float sr = 48000.0f;
  const size_t n = 48000;
  std::vector<float> in = bursts(n, 6000);
  for (size_t i = 0; i < n; ++i) in[i] += 0.3f * std::sin(6.2831853f * 3000.0f * (i / sr)) * (i > n / 2);
  rings::Follower o;
  o.Init(8.0f / sr, 160.0f / sr, 1600.0f / sr);
  ml::Follower p;
  p.init(sr);
  std::vector<float> oe(n), oc(n), pe(n), pc(n);
  for (size_t i = 0; i < n; ++i) {
    o.Process(in[i], &oe[i], &oc[i]);
    pe[i] = p.process(in[i]);
    pc[i] = p.centroid();
  }
  report("follower", std::fmax(relativeError(oe, pe), relativeError(oc, pc)), 1e-4f);
}

static void clockToRamp() {
  const float sr = 48000.0f;
  const size_t n = 48000 * 8;
  // A swung clock with 10 ms pulses.
  std::vector<bool> clock(n);
  for (size_t t = 0, k = 0; t < n; t += static_cast<size_t>(sr * ((k++ & 1) ? 0.2f : 0.3f))) {
    for (size_t j = t; j < t + 480 && j < n; ++j) clock[j] = true;
  }
  tides::RampExtractor o;
  o.Init(sr, 40.0f / sr);
  tides::Ratio r = {1.0f, 1};
  std::vector<float> oo(n), po(n);
  bool previous = false;
  for (size_t i = 0; i < n; ++i) {
    stmlib::GateFlags f = stmlib::ExtractGateFlags(previous, clock[i]);
    previous = clock[i];
    o.Process(false, false, r, &f, &oo[i], 1);
  }
  ml::ClockToRamp p;
  p.init(sr, 40.0f);
  for (size_t i = 0; i < n; ++i) po[i] = p.process(clock[i]);
  report("clock to ramp", relativeError(oo, po), 1e-4f);
}

static void quantizerAndLag() {
  // Quantiser: sweep an input over two octaves at several amounts.
  marbles::Scale scale;
  const ml::Quantizer::Scale& s = ml::Quantizer::preset(ml::Quantizer::MAJOR);
  scale.base_interval = s.baseInterval;
  scale.num_degrees = s.numDegrees;
  for (int i = 0; i < s.numDegrees; ++i) {
    scale.degree[i].voltage = s.degree[i].voltage;
    scale.degree[i].weight = s.degree[i].weight;
  }
  marbles::Quantizer o;
  o.Init(scale);
  ml::Quantizer p;
  p.init(s);
  std::vector<float> oo, po;
  for (int a = 0; a <= 10; ++a) {
    for (int i = 0; i < 2000; ++i) {
      float v = -1.0f + i / 1000.0f;
      oo.push_back(o.Process(v, a / 10.0f, true));
      po.push_back(p.process(v, a / 10.0f, true));
    }
  }
  report("quantizer", relativeError(oo, po), 1e-6f);

  // Lag: random steps on a 4 Hz ramp, several smoothness settings.
  std::vector<float> ol, pl;
  for (int k = 0; k <= 4; ++k) {
    float smooth = k / 4.0f;
    marbles::LagProcessor lo;
    lo.Init();
    ml::Lag lp;
    lp.init();
    uint32_t seed = 3;
    float value = 0.0f;
    for (int i = 0; i < 48000; ++i) {
      float phase = (i % 12000) / 12000.0f;
      if (i % 12000 == 0) {
        seed = seed * 1664525u + 1013904223u;
        value = seed / 4294967296.0f;
        lo.ResetRamp();
        lp.resetRamp();
      }
      ol.push_back(lo.Process(value, smooth, phase));
      pl.push_back(lp.process(value, smooth, phase));
    }
  }
  // Not bit-exact: the glide rate goes through stmlib's table-based
  // semitone conversion, and the S-curve through a 256-point table.
  report("lag", relativeError(ol, pl), 2e-3f);
}

static void slope() {
  const float sr = 48000.0f;
  const size_t n = 48000, block = 8;
  struct Case { const char* name; tides::RampMode mode; ml::Slope::Mode pmode; float scale; float smooth; };
  const Case cases[] = {
      {"slope (AD)", tides::RAMP_MODE_AD, ml::Slope::AD, 8.0f, 0.5f},
      {"slope (AD, smoothed)", tides::RAMP_MODE_AD, ml::Slope::AD, 8.0f, 0.2f},
      {"slope (looping, folded)", tides::RAMP_MODE_LOOPING, ml::Slope::LOOPING, 5.0f, 0.8f},
      {"slope (AR)", tides::RAMP_MODE_AR, ml::Slope::AR, 8.0f, 0.5f},
  };
  for (const Case& k : cases) {
    const float hz = 3.0f, pw = 0.3f, shape = 0.7f;
    static tides::PolySlopeGenerator o;
    o.Init();
    std::vector<float> oo(n), po(n);
    tides::PolySlopeGenerator::OutputSample out[block];
    stmlib::GateFlags flags[block];
    bool previous = false;
    for (size_t i = 0; i < n; i += block) {
      for (size_t j = 0; j < block; ++j) {
        bool gate = (i + j) % 24000 < 6000 && i + j >= block;
        flags[j] = stmlib::ExtractGateFlags(previous, gate);
        previous = gate;
      }
      o.Render(k.mode, tides::OUTPUT_MODE_SLOPE_PHASE, tides::RANGE_CONTROL,
               hz / sr, pw, shape, k.smooth, 0.5f, flags, NULL, out, block);
      for (size_t j = 0; j < block; ++j) oo[i + j] = out[j].channel[0] / k.scale;
    }
    ml::Slope p;
    p.init(sr);
    p.setMode(k.pmode);
    p.setFrequency(hz);
    p.setSlope(pw);
    p.setShape(shape);
    p.setSmoothness(k.smooth);
    for (size_t i = 0; i < n; ++i) po[i] = p.process(i % 24000 < 6000 && i >= block);
    // The original smooths its parameters from their defaults over the
    // first block, which leaves a brief transient in the smoothing filter;
    // compare from 100 ms.
    const size_t skip = 4800;
    std::vector<float> a(oo.begin() + skip, oo.end()), b(po.begin() + skip, po.end());
    if (getenv("GOLDEN_DEBUG")) {
      size_t worst = 0;
      for (size_t i = 0; i < a.size(); ++i) if (std::fabs(a[i] - b[i]) > std::fabs(a[worst] - b[worst])) worst = i;
      printf("    worst at %zu: original %f port %f\n", worst + skip, a[worst], b[worst]);
    }
    report(k.name, relativeError(a, b), 2e-3f);
  }
}

static void envelope() {
  // Peaks' ADSR, 16-bit fixed point. Its time knobs map to seconds through
  // the same curve the port uses.
  const float sr = 48000.0f;
  const size_t n = 48000 * 2;
  auto seconds = [](uint16_t knob) {
    const double gamma = 0.175;
    double a = std::pow(4294967296.0 / (0.0005 * 48000.0), -gamma);
    double b = std::pow(4294967296.0 / (8.0 * 48000.0), -gamma);
    double k = (knob >> 8) / 256.0;
    double increment = std::pow(a + (b - a) * k, -1.0 / gamma);
    return static_cast<float>(4294967296.0 / increment / 48000.0);
  };
  const uint16_t attack = 20000, decay = 30000, sustain = 16384, release = 40000;
  peaks::MultistageEnvelope o;
  o.Init();
  o.set_adsr(attack, decay, sustain, release);
  ml::Envelope p;
  p.init(sr);
  p.setAdsr(seconds(attack), seconds(decay), sustain / 32767.0f, seconds(release));
  std::vector<float> oo(n), po(n);
  bool previous = false;
  for (size_t i = 0; i < n; ++i) {
    bool gate = i > 100 && i < 48000;
    peaks::GateFlags f = (gate ? peaks::GATE_FLAG_HIGH : 0) |
                         (gate && !previous ? peaks::GATE_FLAG_RISING : 0) |
                         (!gate && previous ? peaks::GATE_FLAG_FALLING : 0);
    previous = gate;
    int16_t s;
    o.Process(&f, &s, 1);
    oo[i] = s / 32767.0f;
    po[i] = p.process(gate);
  }
  // Peaks' 16-bit levels, 8.24 fixed-point phase and 256-point curve
  // tables give about 1% differences on the steep parts of the curves.
  report("envelope (Peaks, fixed-point)", relativeError(oo, po), 2e-2f);
}

void mod() {
  follower();
  clockToRamp();
  quantizerAndLag();
  slope();
  envelope();
}
