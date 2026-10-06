// Golden comparison, phase 3: drums. Plaits' drums are compared at 48 kHz
// in its 12-sample blocks; Peaks' FM drum at 48 kHz in 4-sample blocks.
// Each run starts with one silent block so the originals' parameter
// smoothing has settled, then triggers.

#include <cstring>

#include "plaits/dsp/drums/analog_bass_drum.h"
#include "plaits/dsp/drums/analog_snare_drum.h"
#include "plaits/dsp/drums/hi_hat.h"
#include "plaits/dsp/drums/synthetic_bass_drum.h"
#include "plaits/dsp/drums/synthetic_snare_drum.h"
#include "peaks/drums/fm_drum.h"
#include "stmlib/utils/random.h"

#include "pt/drums/analog_kick.h"
#include "pt/drums/analog_snare.h"
#include "pt/drums/fm_drum.h"
#include "pt/drums/hihat.h"
#include "pt/drums/synth_kick.h"
#include "pt/drums/synth_snare.h"

#include "golden.h"

static const float kSr = 48000.0f;
static const size_t kBlock = 12, kN = 48000;

// Runs the original: block 0 silent, block 1 triggered, then free.
template <typename Render>
static std::vector<float> original(Render render) {
  std::vector<float> out(kN);
  for (size_t i = 0; i < kN; i += kBlock) render(i == kBlock, &out[i], kBlock);
  return out;
}

// Runs a port the same way.
template <typename Drum>
static std::vector<float> port(Drum& d) {
  std::vector<float> out(kN);
  for (size_t i = 0; i < kN; ++i) {
    if (i == kBlock) d.trigger();
    out[i] = d.process();
  }
  return out;
}

void drums() {
  const float f0 = 55.0f / kSr;

  {
    static plaits::AnalogBassDrum o;
    o.Init();
    std::vector<float> a = original([&](bool t, float* out, size_t n) {
      o.Render(false, t, 0.8f, f0, 0.4f, 0.5f, 0.5f, 0.3f, out, n);
    });
    pt::AnalogKick p;
    p.init(kSr);
    p.setAccent(0.8f); p.setFrequency(55.0f); p.setTone(0.4f); p.setDecay(0.5f);
    p.setAttackFm(0.5f); p.setSelfFm(0.3f);
    report("analog kick", relativeError(a, port(p)), 1e-3f);
  }
  {
    stmlib::Random::Seed(0x21);
    static plaits::AnalogSnareDrum o;
    o.Init();
    const float f = 180.0f / kSr;
    std::vector<float> a = original([&](bool t, float* out, size_t n) {
      o.Render(false, t, 0.8f, f, 0.8f, 0.5f, 0.5f, out, n);
    });
    pt::AnalogSnare p;
    p.init(kSr);
    p.setAccent(0.8f); p.setFrequency(180.0f); p.setTone(0.8f); p.setDecay(0.5f); p.setSnappy(0.5f);
    report("analog snare", relativeError(a, port(p)), 1e-3f);
  }
  {
    stmlib::Random::Seed(0x21);
    static plaits::SyntheticBassDrum o;
    o.Init();
    std::vector<float> a = original([&](bool t, float* out, size_t n) {
      o.Render(false, t, 0.8f, f0, 0.3f, 0.6f, 0.4f, 0.4f, 0.3f, out, n);
    });
    pt::SynthKick p;
    p.init(kSr);
    p.setAccent(0.8f); p.setFrequency(55.0f); p.setTone(0.3f); p.setDecay(0.6f);
    p.setDirtiness(0.4f); p.setFmAmount(0.4f); p.setFmDecay(0.3f);
    report("synthetic kick", relativeError(a, port(p)), 1e-3f);
  }
  {
    stmlib::Random::Seed(0x21);
    static plaits::SyntheticSnareDrum o;
    o.Init();
    const float f = 200.0f / kSr;
    std::vector<float> a = original([&](bool t, float* out, size_t n) {
      o.Render(false, t, 0.8f, f, 0.3f, 0.5f, 0.6f, out, n);
    });
    pt::SynthSnare p;
    p.init(kSr);
    p.setAccent(0.8f); p.setFrequency(200.0f); p.setFmAmount(0.3f); p.setDecay(0.5f); p.setSnappy(0.6f);
    report("synthetic snare", relativeError(a, port(p)), 1e-3f);
  }
  {
    stmlib::Random::Seed(0x21);
    static plaits::HiHat<plaits::SquareNoise, plaits::SwingVCA, true, false> o;
    o.Init();
    const float f = 400.0f / kSr;
    float t1[kBlock], t2[kBlock];
    std::vector<float> a = original([&](bool t, float* out, size_t n) {
      o.Render(false, t, 0.8f, f, 0.6f, 0.5f, 0.2f, t1, t2, out, n);
    });
    pt::HiHat p;
    p.init(kSr, pt::HiHat::SQUARE_808);
    p.setAccent(0.8f); p.setFrequency(400.0f); p.setTone(0.6f); p.setDecay(0.5f); p.setNoisiness(0.2f);
    // Not bit-exact: the envelope rate and filter cutoff go through
    // stmlib's table-based semitone conversion.
    report("hi-hat (808)", relativeError(a, port(p)), 3e-3f);
  }
  {
    stmlib::Random::Seed(0x21);
    static plaits::HiHat<plaits::RingModNoise, plaits::LinearVCA, false, true> o;
    o.Init();
    const float f = 400.0f / kSr;
    float t1[kBlock], t2[kBlock];
    std::vector<float> a = original([&](bool t, float* out, size_t n) {
      o.Render(false, t, 0.8f, f, 0.6f, 0.5f, 0.2f, t1, t2, out, n);
    });
    pt::HiHat p;
    p.init(kSr, pt::HiHat::RING_MOD);
    p.setAccent(0.8f); p.setFrequency(400.0f); p.setTone(0.6f); p.setDecay(0.5f); p.setNoisiness(0.2f);
    // Plaits ramps its oscillators' pitch up from near zero over the first
    // block, so their phases (and the exact waveform) differ from the
    // port's forever after. Compare the loudness envelope instead.
    report("hi-hat (ring mod), envelope", envelopeError(a, port(p), 480), 0.1f);
  }
  {
    // Peaks: fixed-point, so expect differences at the 16-bit level and
    // from its lookup tables.
    stmlib::Random::Seed(0x21);
    static peaks::FmDrum o;
    o.Init();
    o.set_sd_range(false);
    const float knob = 0.2f, fm = 0.3f, decay = 0.5f;
    o.set_frequency(static_cast<uint16_t>(knob * 65535));
    o.set_fm_amount(static_cast<uint16_t>((static_cast<uint16_t>(fm * 65535) >> 2) * 3));
    o.set_decay(static_cast<uint16_t>(decay * 65535));
    o.set_noise(32768);
    std::vector<float> a(kN);
    peaks::GateFlags gates[4];
    int16_t block[4];
    for (size_t i = 0; i < kN; i += 4) {
      for (int k = 0; k < 4; ++k) gates[k] = (i + k == kBlock) ? peaks::GATE_FLAG_RISING : peaks::GATE_FLAG_LOW;
      o.Process(gates, block, 4);
      for (int k = 0; k < 4; ++k) a[i + k] = block[k] / 32768.0f;
    }
    pt::FmDrum p;
    p.init(kSr);
    p.setFrequency(pt::midiToHz(24.0f + 72.0f * knob));
    p.setFmAmount(fm);
    p.setDecay(decay);
    p.setNoise(0.5f);
    report("FM drum (Peaks, fixed-point)", relativeError(a, port(p)), 5e-2f);
  }
}
