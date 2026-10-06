// Golden comparison: original Gillet classes vs mutablelib ports, at the
// original sample rate, with the original block size matching the port's
// control period.
//
// Expected differences:
// - stmlib's SemitonesToRatio() uses lookup tables accurate to about
//   1/256 semitone; mutablelib uses exp2(). Components that convert
//   semitones (string decay, LPG rates) drift slightly over time.
// - Ports that resample internally (the tube) are one sample later.
// - The tube's tuning is corrected (see physical/tube.h).

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

// Originals. plucker.h relies on the including file's
// "using namespace stmlib", so provide one.
#include "stmlib/dsp/filter.h"
#include "stmlib/dsp/units.h"
#include "stmlib/utils/random.h"
namespace rings { using namespace stmlib; }
#include "rings/dsp/resonator.h"
#include "rings/dsp/plucker.h"
#include "elements/dsp/string.h"
#include "elements/dsp/tube.h"
#include "plaits/dsp/envelope.h"
#include "plaits/dsp/fx/low_pass_gate.h"
#include "plaits/dsp/fx/overdrive.h"
#include "plaits/dsp/fx/sample_rate_reducer.h"
#include "plaits/resources.h"
#include "rings/dsp/fx/chorus.h"
#include "rings/dsp/fx/ensemble.h"
#include "rings/dsp/fx/reverb.h"
#include "rings/dsp/limiter.h"
#include "elements/dsp/fx/reverb.h"
#include "clouds/dsp/frame.h"
#include "clouds/dsp/fx/diffuser.h"
#include "clouds/dsp/fx/pitch_shifter.h"
#include "clouds/dsp/fx/reverb.h"

// Ports.
#include "ml/dynamics/lpg.h"
#include "ml/physical/modal_resonator.h"
#include "ml/physical/plucker.h"
#include "ml/physical/string.h"
#include "ml/physical/tube.h"
#include "ml/fx/chorus.h"
#include "ml/fx/decimator.h"
#include "ml/fx/diffuser.h"
#include "ml/fx/ensemble.h"
#include "ml/fx/limiter.h"
#include "ml/fx/overdrive.h"
#include "ml/fx/pitch_shifter.h"
#include "ml/fx/reverb.h"
#include "ml/fx/wavefolder.h"


#include "golden.h"

static void resonator() {
  const float sr = 48000.0f;  // Rings
  const size_t block = 24, n = 48000 * 2;
  std::vector<float> in = bursts(n, 24000);
  const float structure[] = {0.1f, 0.27f, 0.6f, 0.95f};
  for (float st : structure) {
    rings::Resonator o;
    o.Init();
    o.set_frequency(147.0f / sr);
    o.set_structure(st);
    o.set_brightness(0.6f);
    o.set_damping(0.5f);
    o.set_position(0.3f);
    o.set_resolution(64);
    std::vector<float> oOdd(n), oEven(n);
    for (size_t i = 0; i < n; i += block) o.Process(&in[i], &oOdd[i], &oEven[i], block);

    ml::ModalResonator p;
    p.init(sr);
    p.setFrequency(147.0f);
    p.setStructure(st);
    p.setBrightness(0.6f);
    p.setDamping(0.5f);
    p.setPosition(0.3f);
    p.setModes(64);
    std::vector<float> pOdd(n), pEven(n);
    for (size_t i = 0; i < n; ++i) {
      p.process(in[i]);
      pOdd[i] = p.odd();
      pEven[i] = p.even();
    }
    char name[64];
    std::snprintf(name, sizeof name, "resonator structure %.2f", st);
    report(name, std::fmax(relativeError(oOdd, pOdd), relativeError(oEven, pEven)), 1e-4f);
  }
}

static void string() {
  const float sr = 32000.0f;  // Elements
  const size_t block = 16, n = 32000 * 2;
  std::vector<float> in = bursts(n, 16000);
  // Internal dispersion values, and the knob positions that give them in
  // each version (Elements and mutablelib map the knob differently).
  const float dispersion[] = {-0.5f, 0.0f, 0.3f, 0.9f};
  for (float d : dispersion) {
    float elementsKnob = d < 0.0f ? 0.24f + d / 4.166f : (d > 0.0f ? 0.26f + d / 1.35135f : 0.25f);
    float mlKnob = d < 0.0f ? 0.49f + d * 0.49f : (d > 0.0f ? 0.51f + d * 0.49f : 0.5f);

    stmlib::Random::Seed(0x21);
    elements::String o;
    o.Init(true);
    o.set_frequency(98.0f / sr);
    o.set_dispersion(elementsKnob);
    o.set_brightness(0.7f);
    o.set_damping(0.8f);
    o.set_position(0.3f);
    std::vector<float> oOut(n, 0.0f), oAux(n, 0.0f);
    for (size_t i = 0; i < n; i += block) o.Process(&in[i], &oOut[i], &oAux[i], block);

    ml::String p;
    p.init(sr);
    p.setFrequency(98.0f);
    p.setDispersion(mlKnob);
    p.setBrightness(0.7f);
    p.setDamping(0.8f);
    p.setPosition(0.3f);
    std::vector<float> pOut(n), pAux(n);
    for (size_t i = 0; i < n; ++i) {
      pOut[i] = p.process(in[i]);
      pAux[i] = p.aux();
    }
    char name[64];
    std::snprintf(name, sizeof name, "string dispersion %+.1f", d);
    // Loose tolerance: decay coefficients differ by ~1e-4 (see header) and
    // the rattle noise amplifies differences.
    report(name, std::fmax(relativeError(oOut, pOut), relativeError(oAux, pAux)),
           d > 0.75f ? 5e-3f : 2e-3f);
  }
}

static void tube() {
  const float sr = 32000.0f;  // Elements
  const size_t block = 16, n = 32000;
  std::vector<float> breath(n);
  uint32_t s = 7;
  for (size_t i = 0; i < n; ++i) {
    s = s * 1664525u + 1013904223u;
    breath[i] = (s / 4294967296.0f - 0.5f) * 0.4f;
  }
  const float frequency = 220.0f, envelope = 0.9f, damping = 0.4f, timbre = 0.5f;

  // Elements' Tube::Init() never clears the delay line, so give the
  // original zeroed (static) storage to start from.
  static elements::Tube o;
  o.Init();
  std::vector<float> io = breath;
  const float gain = 1.0f;
  for (size_t i = 0; i < n; i += block) {
    o.Process(frequency / sr, envelope, damping, timbre, &io[i], gain, block);
  }
  std::vector<float> oOut(n);
  for (size_t i = 0; i < n; ++i) oOut[i] = io[i] - breath[i];

  ml::Tube p;
  p.init(sr);
  // The port takes the sounding pitch (an octave below the loop) and
  // removes half a sample of delay to correct the tuning. Pick the pitch
  // that gives exactly the original's loop delay of sr / frequency.
  p.setFrequency(0.5f * sr / (sr / frequency + 0.5f));
  p.setDamping(damping);
  p.setTimbre(timbre);
  std::vector<float> pOut(n);
  for (size_t i = 0; i < n; ++i) pOut[i] = p.process(breath[i], envelope);
  // Not bit-exact: the tone filter tracks the (tuning-corrected) loop
  // frequency, which is 0.3% off the original's.
  report("tube", relativeError(oOut, pOut, 1), 2e-3f);
}

static void plucker() {
  const float sr = 48000.0f;  // Rings
  const size_t block = 24, n = 24000;
  stmlib::Random::Seed(0x21);
  rings::Plucker o;
  o.Init();
  o.Trigger(330.0f / sr, 6000.0f / sr, 0.3f);
  std::vector<float> oOut(n);
  for (size_t i = 0; i < n; i += block) o.Process(&oOut[i], block);

  ml::Plucker p;
  p.init(sr);
  p.setFrequency(330.0f);
  p.setCutoff(6000.0f);
  p.setPosition(0.3f);
  p.trigger();
  std::vector<float> pOut(n);
  for (size_t i = 0; i < n; ++i) pOut[i] = p.process();
  report("plucker", relativeError(oOut, pOut), 1e-5f);
}

static void lpg() {
  const float sr = 48000.0f;  // Plaits
  const size_t block = 12, n = 48000 * 2;
  const float decay = 0.5f, colour = 0.6f, attackPerBlock = 0.05f;
  std::vector<float> in(n);
  for (size_t i = 0; i < n; ++i) in[i] = std::sin(i * 2.0f * 3.14159265f * 200.0f / sr);

  // Plaits' voice computes these once per block from the decay and colour
  // knobs; see plaits/dsp/voice.cc.
  const float shortDecay = (200.0f * block) / sr * stmlib::SemitonesToRatio(-96.0f * decay);
  const float decayTail =
      (20.0f * block) / sr * stmlib::SemitonesToRatio(-72.0f * decay + 12.0f * colour) - shortDecay;
  plaits::LPGEnvelope env;
  plaits::LowPassGate o;
  env.Init();
  o.Init();
  std::vector<float> oOut = in;
  for (size_t i = 0; i < n; i += block) {
    if (i % 24000 == 0) env.Trigger();
    env.ProcessPing(attackPerBlock, shortDecay, decayTail, colour);
    o.Process(env.gain(), env.frequency(), env.hf_bleed(), &oOut[i], block);
  }

  ml::LowPassGate p;
  p.init(sr);
  p.setDecay(decay);
  p.setColour(colour);
  p.setAttackPitch(attackPerBlock / (2.0f * block / sr));
  std::vector<float> pOut(n);
  for (size_t i = 0; i < n; ++i) {
    if (i % 24000 == 0) p.trigger();
    pOut[i] = p.process(in[i]);
  }
  report("low-pass gate", relativeError(oOut, pOut), 1e-3f);
}

// ---------------------------------------------------------------------------
// Phase 2: effects. The originals mostly store delay memory as 12- or 16-bit
// integers; the ports use float, so expect quantisation-level differences
// (about 1/4096 of full scale for 12-bit, 1/32768 for 16-bit).

// Stereo test signal: decaying noise bursts on the left, a sine on the right.
static void stereoSignal(size_t n, float sr, std::vector<float>& l, std::vector<float>& r) {
  l = bursts(n, static_cast<size_t>(sr / 2));
  r.resize(n);
  for (size_t i = 0; i < n; ++i) r[i] = 0.3f * std::sin(6.2831853f * 330.0f * (i / sr));
}

template <typename Original>
static void runOriginalReverb(Original& o, std::vector<float>& l, std::vector<float>& r, size_t block) {
  for (size_t i = 0; i < l.size(); i += block) o.Process(&l[i], &r[i], block);
}

static void reverbs() {
  static uint16_t buffer[32768];
  struct Case { const char* name; ml::Reverb::Preset preset; float sr; float tolerance; };
  const Case cases[] = {
    {"reverb (Rings)", ml::Reverb::RINGS, 48000.0f, 2e-3f},
    {"reverb (Elements)", ml::Reverb::ELEMENTS, 32000.0f, 2e-3f},
    // 12-bit steps are 16x coarser than 16-bit, and the same topology
    // differs by ~1.4e-3 at 16-bit (Elements), so expect ~2e-2 here.
    {"reverb (Clouds, 12-bit)", ml::Reverb::CLOUDS, 32000.0f, 3e-2f},
  };
  for (const Case& k : cases) {
    const size_t n = static_cast<size_t>(k.sr * 2);
    std::vector<float> l, r;
    stereoSignal(n, k.sr, l, r);
    std::vector<float> ol = l, orr = r;
    std::memset(buffer, 0, sizeof buffer);
    if (k.preset == ml::Reverb::RINGS) {
      // Static: the original never initialises its loop filter state.
      static rings::Reverb o;
      o.Init(buffer);
      o.set_amount(0.5f); o.set_input_gain(0.2f); o.set_time(0.8f); o.set_diffusion(0.625f); o.set_lp(0.7f);
      runOriginalReverb(o, ol, orr, 24);
    } else if (k.preset == ml::Reverb::ELEMENTS) {
      // Static: the original never initialises its loop filter state.
      static elements::Reverb o;
      o.Init(buffer);
      o.set_amount(0.5f); o.set_input_gain(0.2f); o.set_time(0.8f); o.set_diffusion(0.625f); o.set_lp(0.7f);
      runOriginalReverb(o, ol, orr, 16);
    } else {
      // Static: the original never initialises its loop filter state.
      static clouds::Reverb o;
      o.Init(buffer);
      o.set_amount(0.5f); o.set_input_gain(0.2f); o.set_time(0.8f); o.set_diffusion(0.625f); o.set_lp(0.7f);
      std::vector<clouds::FloatFrame> frames(n);
      for (size_t i = 0; i < n; ++i) { frames[i].l = ol[i]; frames[i].r = orr[i]; }
      for (size_t i = 0; i < n; i += 32) o.Process(&frames[i], 32);
      for (size_t i = 0; i < n; ++i) { ol[i] = frames[i].l; orr[i] = frames[i].r; }
    }
    ml::Reverb p;
    p.init(k.sr, k.preset);
    p.setAmount(0.5f); p.setInputGain(0.2f); p.setTime(0.8f); p.setDiffusion(0.625f); p.setLp(0.7f);
    std::vector<float> pl = l, pr = r;
    for (size_t i = 0; i < n; ++i) p.process(pl[i], pr[i]);
    report(k.name, std::fmax(relativeError(ol, pl), relativeError(orr, pr)), k.tolerance);
  }
}

static void diffuser() {
  const float sr = 32000.0f;
  const size_t n = 32000;
  static float buffer[2048];
  std::vector<float> l, r;
  stereoSignal(n, sr, l, r);
  clouds::Diffuser o;
  o.Init(buffer);
  o.set_amount(0.8f);
  std::vector<clouds::FloatFrame> frames(n);
  for (size_t i = 0; i < n; ++i) { frames[i].l = l[i]; frames[i].r = r[i]; }
  for (size_t i = 0; i < n; i += 32) o.Process(&frames[i], 32);
  ml::Diffuser p;
  p.init(sr);
  p.setAmount(0.8f);
  std::vector<float> ol(n), orr(n), pl = l, pr = r;
  for (size_t i = 0; i < n; ++i) {
    ol[i] = frames[i].l; orr[i] = frames[i].r;
    p.process(pl[i], pr[i]);
  }
  report("diffuser", std::fmax(relativeError(ol, pl), relativeError(orr, pr)), 1e-5f);
}

static void chorusAndEnsemble() {
  const float sr = 48000.0f;
  const size_t n = 48000;
  static uint16_t buffer[4096];
  std::vector<float> l, r;
  stereoSignal(n, sr, l, r);
  {
    std::memset(buffer, 0, sizeof buffer);
    rings::Chorus o;
    o.Init(buffer);
    o.set_amount(0.7f);
    o.set_depth(0.6f);
    std::vector<float> ol = l, orr = r;
    for (size_t i = 0; i < n; i += 24) o.Process(&ol[i], &orr[i], 24);
    ml::Chorus p;
    p.init(sr);
    p.setAmount(0.7f);
    p.setDepth(0.6f);
    std::vector<float> pl = l, pr = r;
    for (size_t i = 0; i < n; ++i) p.process(pl[i], pr[i]);
    report("chorus (16-bit)", std::fmax(relativeError(ol, pl), relativeError(orr, pr)), 2e-4f);
  }
  {
    std::memset(buffer, 0, sizeof buffer);
    rings::Ensemble o;
    o.Init(buffer);
    o.set_amount(0.7f);
    o.set_depth(0.6f);
    std::vector<float> ol = l, orr = r;
    for (size_t i = 0; i < n; i += 24) o.Process(&ol[i], &orr[i], 24);
    ml::Ensemble p;
    p.init(sr);
    p.setAmount(0.7f);
    p.setDepth(0.6f);
    std::vector<float> pl = l, pr = r;
    for (size_t i = 0; i < n; ++i) p.process(pl[i], pr[i]);
    report("ensemble (16-bit)", std::fmax(relativeError(ol, pl), relativeError(orr, pr)), 2e-4f);
  }
}

static void pitchShifter() {
  const float sr = 32000.0f;
  const size_t n = 32000;
  static uint16_t buffer[4096];
  std::vector<float> l, r;
  stereoSignal(n, sr, l, r);
  clouds::PitchShifter o;
  o.Init(buffer);
  o.set_ratio(1.5f);
  o.set_size(1.0f);  // the initial size: no smoothing in either version
  std::vector<clouds::FloatFrame> frames(n);
  for (size_t i = 0; i < n; ++i) { frames[i].l = r[i]; frames[i].r = 0.0f; }
  o.Process(&frames[0], n);
  ml::PitchShifter p;
  p.init(sr);
  p.setRatio(1.5f);
  p.setSize(1.0f);
  std::vector<float> oo(n), po(n);
  for (size_t i = 0; i < n; ++i) {
    oo[i] = frames[i].l;
    po[i] = p.process(r[i]);
  }
  report("pitch shifter (16-bit)", relativeError(oo, po), 2e-4f);
}

static void overdriveDecimatorLimiter() {
  const float sr = 48000.0f;
  const size_t n = 48000;
  std::vector<float> x(n);
  for (size_t i = 0; i < n; ++i) x[i] = 0.9f * std::sin(6.2831853f * 110.0f * (i / sr));

  // Overdrive: the port smooths drive changes differently, so hold the
  // drive constant and compare after the first 10 ms.
  {
    const float knob = 0.6f;
    plaits::Overdrive o;
    o.Init();
    std::vector<float> oo = x;
    for (size_t i = 0; i < n; i += 12) o.Process(0.5f + 0.5f * knob, &oo[i], 12);
    ml::Overdrive p;
    p.setDrive(knob);
    p.init(sr);
    std::vector<float> po(n);
    for (size_t i = 0; i < n; ++i) po[i] = p.process(x[i]);
    std::vector<float> a(oo.begin() + 480, oo.end()), b(po.begin() + 480, po.end());
    report("overdrive", relativeError(a, b), 1e-5f);
  }
  // Decimator, general (non-optimised) path, 3.3 kHz.
  {
    plaits::SampleRateReducer o;
    o.Init();
    std::vector<float> oo = x;
    for (size_t i = 0; i < n; i += 12) o.Process<false>(3300.0f / sr, &oo[i], 12);
    ml::Decimator p;
    p.init(sr);
    p.setRate(3300.0f);
    std::vector<float> po(n);
    for (size_t i = 0; i < n; ++i) po[i] = p.process(x[i]);
    report("decimator", relativeError(oo, po), 1e-5f);
  }
  // Limiter (Rings, stereo), driven hard.
  {
    rings::Limiter o;
    o.Init();
    std::vector<float> ol = x, orr = x;
    for (size_t i = 0; i < n; ++i) orr[i] *= -0.5f;
    std::vector<float> pl = ol, pr = orr;
    for (size_t i = 0; i < n; i += 24) o.Process(&ol[i], &orr[i], 24, 3.0f);
    ml::Limiter p;
    p.init(sr);
    p.setPreGain(3.0f);
    for (size_t i = 0; i < n; ++i) p.process(pl[i], pr[i]);
    report("limiter", std::fmax(relativeError(ol, pl), relativeError(orr, pr)), 1e-6f);
  }
}

static void wavefolder() {
  // Compare the folding curves directly against Plaits' tables.
  ml::Wavefolder p;
  p.init();
  p.setAmount(1.0f);
  const float gain = 0.03f + 0.46f;
  std::vector<float> oFold, oAnalog, pFold, pAnalog;
  for (int i = 0; i <= 2000; ++i) {
    float x = -1.0f + i / 1000.0f;
    float index = x * gain + 0.5f;
    oFold.push_back(stmlib::InterpolateHermite(plaits::lut_fold + 1, index, 512.0f));
    oAnalog.push_back(-stmlib::InterpolateHermite(plaits::lut_fold_2 + 1, index, 512.0f));
    pFold.push_back(p.process(x));
    pAnalog.push_back(p.analog());
  }
  report("wavefolder", std::fmax(relativeError(oFold, pFold), relativeError(oAnalog, pAnalog)), 1e-6f);
}

void drums();  // golden_drums.cpp
void mod();    // golden_mod.cpp
void osc();    // golden_osc.cpp
void spectral();  // golden_spectral.cpp

int main() {
  resonator();
  string();
  tube();
  plucker();
  lpg();
  reverbs();
  diffuser();
  chorusAndEnsemble();
  pitchShifter();
  overdriveDecimatorLimiter();
  wavefolder();
  drums();
  mod();
  osc();
  spectral();
  return testResult("golden");
}
