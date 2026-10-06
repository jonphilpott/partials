// Golden comparison, phase 5: oscillators and noise against Plaits. The
// originals are run one sample per call, so their parameter smoothing
// reaches each new value at once, matching the per-sample ports.

// formant_oscillator.h needs kMaxFrequency from oscillator.h first.
#include "plaits/dsp/oscillator/oscillator.h"
#include "plaits/dsp/noise/clocked_noise.h"
#include "plaits/dsp/noise/dust.h"
#include "plaits/dsp/noise/smooth_random_generator.h"
#include "plaits/dsp/oscillator/formant_oscillator.h"
#include "plaits/dsp/oscillator/grainlet_oscillator.h"
#include "plaits/dsp/oscillator/harmonic_oscillator.h"
#include "plaits/dsp/oscillator/string_synth_oscillator.h"
#include "plaits/dsp/oscillator/variable_shape_oscillator.h"
#include "plaits/dsp/oscillator/vosim_oscillator.h"
#include "plaits/dsp/oscillator/z_oscillator.h"
#include "stmlib/utils/random.h"

#include "ml/noise/clocked_noise.h"
#include "ml/noise/dust.h"
#include "ml/noise/smooth_random.h"
#include "ml/osc/formant.h"
#include "ml/osc/grainlet.h"
#include "ml/osc/harmonic.h"
#include "ml/osc/string_synth.h"
#include "ml/osc/variable_shape.h"
#include "ml/osc/vosim.h"
#include "ml/osc/z_osc.h"

#include <cstdlib>

#include "golden.h"

static const float kSr = 48000.0f;
static const size_t kN = 24000;

// Run an original (one sample per call) and a port side by side.
template <typename RenderOriginal, typename RenderPort>
static void compare(const char* name, RenderOriginal o, RenderPort p, float tolerance) {
  std::vector<float> a(kN), b(kN);
  for (size_t i = 0; i < kN; ++i) {
    a[i] = o();
    b[i] = p();
  }
  if (getenv("GOLDEN_DEBUG")) {
    size_t w = 0;
    for (size_t i = 0; i < kN; ++i) if (std::fabs(a[i] - b[i]) > std::fabs(a[w] - b[w])) w = i;
    printf("    %s worst at %zu: %g vs %g\n", name, w, a[w], b[w]);
  }
  report(name, relativeError(a, b), tolerance);
}

void osc() {
  const float f = 220.0f / kSr;
  for (float shape : {0.1f, 0.5f, 0.8f}) {
    static plaits::VariableShapeOscillator o;
    o.Init();
    ml::VariableShapeOscillator p;
    p.init(kSr);
    p.setFrequency(220.0f);
    p.setPulseWidth(0.3f);
    p.setShape(shape);
    char name[64];
    std::snprintf(name, sizeof name, "variable shape %.1f", shape);
    compare(name, [&] { float x; o.Render(f, 0.3f, shape, &x, 1); return x; },
            [&] { return p.process(); }, 1e-5f);
  }
  {
    static plaits::VariableShapeOscillator o;
    o.Init();
    ml::VariableShapeOscillator p;
    p.init(kSr);
    p.setSyncFrequency(110.0f);
    p.setFrequency(317.0f);
    p.setPulseWidth(0.4f);
    p.setShape(0.7f);
    compare("variable shape, sync", [&] { float x; o.Render(110.0f / kSr, 317.0f / kSr, 0.4f, 0.7f, &x, 1); return x; },
            [&] { return p.process(); }, 1e-5f);
  }
  {
    static plaits::FormantOscillator o;
    o.Init();
    ml::FormantOscillator p;
    p.init(kSr);
    p.setCarrierFrequency(150.0f);
    p.setFormantFrequency(1100.0f);
    p.setPhaseShift(0.2f);
    compare("formant", [&] { float x; o.Render(150.0f / kSr, 1100.0f / kSr, 0.2f, &x, 1); return x; },
            [&] { return p.process(); }, 1e-5f);
  }
  {
    static plaits::ZOscillator o;
    o.Init();
    ml::ZOscillator p;
    p.init(kSr);
    p.setCarrierFrequency(150.0f);
    p.setFormantFrequency(900.0f);
    p.setShape(0.3f);
    p.setMode(0.5f);
    compare("Z", [&] { float x; o.Render(150.0f / kSr, 900.0f / kSr, 0.3f, 0.5f, &x, 1); return x; },
            [&] { return p.process(); }, 1e-5f);
  }
  {
    static plaits::VOSIMOscillator o;
    o.Init();
    ml::VosimOscillator p;
    p.init(kSr);
    p.setCarrierFrequency(120.0f);
    p.setFormantFrequencies(700.0f, 1200.0f);
    p.setShape(0.5f);
    compare("VOSIM", [&] { float x; o.Render(120.0f / kSr, 700.0f / kSr, 1200.0f / kSr, 0.5f, &x, 1); return x; },
            [&] { return p.process(); }, 1e-5f);
  }
  {
    static plaits::GrainletOscillator o;
    o.Init();
    ml::GrainletOscillator p;
    p.init(kSr);
    p.setCarrierFrequency(150.0f);
    p.setFormantFrequency(800.0f);
    p.setShape(0.4f);
    p.setBleed(0.3f);
    compare("grainlet", [&] { float x; o.Render(150.0f / kSr, 800.0f / kSr, 0.4f, 0.3f, &x, 1); return x; },
            [&] { return p.process(); }, 1e-5f);
  }
  {
    static plaits::HarmonicOscillator<12> o;
    o.Init();
    float amps[12];
    for (int i = 0; i < 12; ++i) amps[i] = 0.5f / (i + 1);
    ml::HarmonicOscillator p;
    p.init(kSr);
    p.setFrequency(200.0f);
    p.setAmplitudes(amps, 12);
    // The port's frequency (hz * 1/sr) differs from hz / sr in the last
    // bit, so the phase drifts very slowly; the 12th harmonic magnifies it.
    compare("harmonic (additive)", [&] { float x; o.Render<1>(200.0f / kSr, amps, &x, 1); return x; },
            [&] { return p.process(); }, 1e-3f);
  }
  {
    static plaits::StringSynthOscillator o;
    o.Init();
    const float reg[7] = {0.3f, 0.0f, 0.2f, 0.3f, 0.0f, 0.2f, 0.0f};
    ml::StringSynthOscillator p;
    p.init(kSr);
    p.setFrequency(110.0f);
    p.setRegistration(reg);
    p.setGain(0.25f);
    compare("string synth", [&] { float x = 0.0f; o.Render(110.0f / kSr, reg, 0.25f, &x, 1); return x; },
            [&] { return p.process(); }, 1e-5f);
  }
  {
    stmlib::Random::Seed(0x21);
    static plaits::ClockedNoise o;
    o.Init();
    ml::ClockedNoise p;
    p.init(kSr);
    p.setFrequency(f * kSr * 3.0f);
    compare("clocked noise", [&] { float x; o.Render(false, 3.0f * f, &x, 1); return x; },
            [&] { return p.process(); }, 1e-6f);
  }
  {
    stmlib::Random::Seed(0x21);
    static plaits::SmoothRandomGenerator o;
    o.Init();
    ml::SmoothRandom p;
    p.init(kSr);
    p.setFrequency(20.0f);
    compare("smooth random", [&] { return o.Render(20.0f / kSr); },
            [&] { return p.process(); }, 1e-6f);
  }
  {
    stmlib::Random::Seed(0x21);
    ml::Dust p;
    p.init(kSr);
    p.setDensity(500.0f);
    compare("dust", [&] { return plaits::Dust(500.0f / kSr); },
            [&] { return p.process(); }, 1e-6f);
  }
}
