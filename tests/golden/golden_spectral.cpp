// Golden comparison, phase 6: cross-modulation and Hilbert transform
// (Warps), FM operators (Plaits), onset detector (Rings). The vocoder and
// vowel filter were restructured and are tested for behaviour in
// tests/test_spectral.cpp.

#include <cstdlib>

// Warps keeps its Xmod functions private and defines them inline in
// modulator.cc; this test-only build opens them and includes the .cc.
#define private public
#include "warps/dsp/modulator.h"
#include "warps/dsp/modulator.cc"
#undef private
#include "warps/dsp/quadrature_transform.h"
#include "warps/resources.h"
#include "plaits/dsp/fm/algorithms.h"
#include "rings/dsp/onset_detector.h"
#include "stmlib/utils/random.h"

#include "ml/analysis/onset_detector.h"
#include "ml/filter/hilbert.h"
#include "ml/spectral/fm_operators.h"
#include "ml/spectral/xmod.h"

#include "golden.h"

static void xmod() {
  uint32_t s = 5;
  auto rnd = [&s] {
    s = s * 1664525u + 1013904223u;
    return s / 2147483648.0f - 1.0f;
  };
  std::vector<float> o, p;
  for (int i = 0; i < 20000; ++i) {
    float a = rnd() * 0.9f, b = rnd() * 0.9f, k = (rnd() + 1.0f) * 0.5f;
    o.push_back(warps::Modulator::Xmod<warps::ALGORITHM_FOLD>(a, b, k));
    p.push_back(ml::Xmod::fold(a, b, k));
    o.push_back(warps::Modulator::Xmod<warps::ALGORITHM_ANALOG_RING_MODULATION>(a, b, k));
    p.push_back(ml::Xmod::analogRing(a, b, k));
    o.push_back(warps::Modulator::Xmod<warps::ALGORITHM_DIGITAL_RING_MODULATION>(a, b, k));
    p.push_back(ml::Xmod::digitalRing(a, b, k));
    o.push_back(warps::Modulator::Xmod<warps::ALGORITHM_XOR>(a, b, k));
    p.push_back(ml::Xmod::bitwiseXor(a, b, k));
    o.push_back(warps::Modulator::Xmod<warps::ALGORITHM_COMPARATOR>(a, b, k));
    p.push_back(ml::Xmod::comparator(a, b, k));
  }
  if (getenv("GOLDEN_DEBUG")) {
    for (int k = 0; k < 5; ++k) {
      float worst = 0.0f;
      size_t at = 0;
      for (size_t i = k; i < o.size(); i += 5) if (std::fabs(o[i] - p[i]) > worst) { worst = std::fabs(o[i] - p[i]); at = i; }
      printf("    algorithm %d: worst %g (orig %g, port %g)\n", k, worst, o[at], p[at]);
    }
  }
  report("xmod (fold, rings, xor, comparator)", relativeError(o, p), 1e-5f);
  // Crossfade: the port computes the curve; Warps reads a 257-point table.
  std::vector<float> ox, px;
  for (int i = 0; i < 20000; ++i) {
    float a = rnd(), b = rnd(), k = (rnd() + 1.0f) * 0.5f;
    ox.push_back(warps::Modulator::Xmod<warps::ALGORITHM_XFADE>(a, b, k));
    px.push_back(ml::Xmod::xfade(a, b, k));
  }
  // Not exact: the port computes the sine/cosine curve; Warps reads it from
  // a 257-point table.
  report("xmod (crossfade)", relativeError(ox, px), 5e-4f);
}

static void hilbert() {
  const float sr = 96000.0f;  // Warps' rate, where its table applies
  warps::QuadratureTransform o;
  o.Init(warps::lut_ap_poles, LUT_AP_POLES_SIZE);
  ml::Hilbert p;
  p.init(sr);
  std::vector<float> oi, oq, pi, pq;
  std::vector<float> in = bursts(48000, 4000);
  for (float x : in) {
    float i, q;
    o.Process(x, &i, &q);
    oi.push_back(i);
    oq.push_back(q);
    pi.push_back(p.process(x));
    pq.push_back(p.q());
  }
  report("hilbert (96 kHz)", std::fmax(relativeError(oi, pi), relativeError(oq, pq)), 1e-5f);
}

static void fm() {
  // Plaits' operators, rendered one sample at a time through its compiled
  // algorithms, against the port's interpreter. 6 operators, a few
  // algorithms, feedback 5.
  static plaits::fm::Algorithms<6> algorithms;
  algorithms.Init();
  const float sr = 48000.0f, hz = 220.0f;
  const float ratios[6] = {1.0f, 2.0f, 3.0f, 1.0f, 0.5f, 7.0f};   // op 1..6
  const float levels[6] = {0.8f, 0.6f, 0.9f, 0.7f, 1.2f, 0.5f};
  for (int algorithm : {1, 5, 16, 22, 32}) {
    plaits::fm::Operator ops[6];
    for (auto& op : ops) op.Reset();
    float fb[2] = {0.0f, 0.0f};
    ml::FmOperators p;
    p.init(sr, 6);
    p.setAlgorithm(algorithm);
    p.setFrequency(hz);
    p.setFeedback(5.0f / 7.0f);
    for (int op = 1; op <= 6; ++op) {
      p.setRatio(op, ratios[op - 1]);
      p.setLevel(op, levels[op - 1]);
    }
    // Plaits indexes operators from op 6 down; f and a in that order.
    float f[6], a[6];
    for (int i = 0; i < 6; ++i) {
      f[i] = hz * ratios[5 - i] / sr;
      a[i] = levels[5 - i];
      ops[i].amplitude = a[i];  // no ramp from 0
    }
    std::vector<float> oo, po;
    for (int n = 0; n < 24000; ++n) {
      float buffers[4] = {0.0f, 0.0f, 0.0f, 0.0f};
      for (int i = 0; i < 6;) {
        const auto& call = algorithms.render_call(algorithm - 1, i);
        (*call.render_fn)(&ops[i], &f[i], &a[i], fb, 5, &buffers[call.input_index],
                          &buffers[call.output_index], 1);
        i += call.n;
      }
      oo.push_back(buffers[0]);
      po.push_back(p.process());
    }
    char name[48];
    std::snprintf(name, sizeof name, "FM operators, algorithm %d", algorithm);
    // The port's operator frequencies (hz * ratio * 1/sr) differ from Plaits'
    // (hz * ratio / sr) in the last bit, so phases drift very slowly.
    report(name, relativeError(oo, po), 1e-4f);
  }
}

static void onsets() {
  // Rings' detector runs on 24-sample blocks at 48 kHz; compare which
  // blocks fire.
  const float sr = 48000.0f;
  const int block = 24;
  rings::OnsetDetector o;
  o.Init(8.0f / sr, 160.0f / sr, 1600.0f / sr, sr / block, 0.01f);
  ml::OnsetDetector p;
  p.init(sr, 0.01f);
  int n = static_cast<int>(sr * 2.6f);
  std::vector<float> x(n);
  for (int k = 0; k < n; ++k) {
    float t = k / sr, local = std::fmod(t, 0.5f);
    x[k] = t > 0.1f ? 0.5f * std::sin(6.2831853f * 300.0f * k / sr) * std::exp(-local * 10.0f) : 0.0f;
  }
  std::vector<float> oo, po;
  for (int k = 0; k < n; k += block) {
    bool a = o.Process(&x[k], block);
    bool b = false;
    for (int j = 0; j < block; ++j) b = p.process(x[k + j]) || b;
    oo.push_back(a ? 1.0f : 0.0f);
    po.push_back(b ? 1.0f : 0.0f);
  }
  int count = 0;
  for (float v : oo) count += v > 0.5f;
  printf("    (Rings' detector fired %d times on 5 notes)\n", count);
  report("onset detector", relativeError(oo, po), 1e-6f);
}

void spectral() {
  xmod();
  hilbert();
  fm();
  onsets();
}
