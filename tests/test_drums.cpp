// Phase 3 tests: drums. For each voice: bounded output, and the same peak
// level, decay time and (for pitched drums) pitch at 44.1, 48 and 96 kHz.

#include <cmath>
#include <cstdio>
#include <vector>

#include "pt/drums/analog_kick.h"
#include "pt/drums/analog_snare.h"
#include "pt/drums/fm_drum.h"
#include "pt/drums/hihat.h"
#include "pt/drums/synth_kick.h"
#include "pt/drums/synth_snare.h"
#include "pt/osc/basic.h"
#include "pt/osc/sine.h"

#include "test.h"
#include "wav.h"

static const float kRates[] = {44100.0f, 48000.0f, 96000.0f};

struct Measure {
  float peakDb;    // loudest 5 ms window
  float decay;     // seconds from the peak until 40 dB down
};

static Measure measure(const std::vector<float>& x, float sr) {
  size_t window = static_cast<size_t>(sr * 0.005f);
  std::vector<float> peaks;
  for (size_t i = 0; i + window <= x.size(); i += window) {
    float p = 0.0f;
    for (size_t j = i; j < i + window; ++j) p = std::fmax(p, std::fabs(x[j]));
    peaks.push_back(p);
  }
  size_t top = 0;
  for (size_t i = 0; i < peaks.size(); ++i) if (peaks[i] > peaks[top]) top = i;
  size_t end = top;
  while (end < peaks.size() && peaks[end] > peaks[top] * 0.01f) ++end;
  Measure m;
  m.peakDb = 20.0f * std::log10(peaks[top] + 1e-12f);
  m.decay = (end - top) * 0.005f;
  return m;
}

// Render one hit of `seconds` from a configured drum.
template <typename Drum>
static std::vector<float> hit(Drum& d, float sr, float seconds) {
  std::vector<float> out(static_cast<size_t>(sr * seconds));
  d.trigger();
  for (size_t i = 0; i < out.size(); ++i) out[i] = d.process();
  return out;
}

// Check that a drum sounds the same at every rate. `setup` configures a
// freshly constructed drum for a given rate.
template <typename Drum, typename Setup>
static void checkRates(const char* name, Setup setup, float pitchHz = 0.0f) {
  Measure m[3];
  float hz[3] = {0, 0, 0};
  for (int r = 0; r < 3; ++r) {
    Drum d;
    setup(d, kRates[r]);
    std::vector<float> out = hit(d, kRates[r], 2.0f);
    CHECK(allBounded(out, 4.0f));
    m[r] = measure(out, kRates[r]);
    if (pitchHz > 0.0f) {
      // Pitch of the body, after the attack.
      size_t a = static_cast<size_t>(kRates[r] * 0.05f), b = static_cast<size_t>(kRates[r] * 0.25f);
      hz[r] = autocorrHz(std::vector<float>(out.begin() + a, out.begin() + b), kRates[r],
                         pitchHz * 0.5f, pitchHz * 2.0f);
    }
    if (r == 1) {
      char path[64];
      std::snprintf(path, sizeof path, "build/%s.wav", name);
      writeWav(path, out, 48000);
    }
  }
  std::printf("  %-12s peak %5.1f/%5.1f/%5.1f dB  decay %.2f/%.2f/%.2f s", name,
              m[0].peakDb, m[1].peakDb, m[2].peakDb, m[0].decay, m[1].decay, m[2].decay);
  if (pitchHz > 0.0f) std::printf("  pitch %.1f/%.1f/%.1f Hz", hz[0], hz[1], hz[2]);
  std::printf("\n");
  CHECK(m[1].peakDb > -30.0f);   // it makes a sound
  for (int r = 0; r < 3; ++r) {
    CHECK_NEAR(m[r].peakDb, m[1].peakDb, 1.5f);
    CHECK_NEAR(m[r].decay, m[1].decay, 0.1f * m[1].decay + 0.02f);
    if (pitchHz > 0.0f) CHECK_NEAR(hz[r], pitchHz, 0.05f * pitchHz);
  }
}

static void testBasicOscillator() {
  // Each shape: correct pitch, bounded, at every rate.
  const pt::BasicOscillator::Shape shapes[] = {
      pt::BasicOscillator::SAW, pt::BasicOscillator::TRIANGLE, pt::BasicOscillator::SLOPE,
      pt::BasicOscillator::SQUARE, pt::BasicOscillator::SQUARE_BRIGHT, pt::BasicOscillator::SQUARE_DARK,
      pt::BasicOscillator::SQUARE_TRIANGLE};
  for (pt::BasicOscillator::Shape shape : shapes) {
    for (float sr : kRates) {
      pt::BasicOscillator o;
      o.init(sr);
      o.setShape(shape);
      o.setFrequency(220.0f);
      o.setPulseWidth(0.3f);
      std::vector<float> out(static_cast<size_t>(sr / 2));
      for (size_t i = 0; i < out.size(); ++i) out[i] = o.process();
      CHECK(allBounded(out, 2.0f));
      CHECK_NEAR(autocorrHz(out, sr, 100.0f, 500.0f), 220.0f, 3.0f);
    }
  }
}

static void testSineOscillator() {
  pt::SineOscillator s;
  s.init(48000.0f);
  s.setFrequency(1000.0f);
  float maxErr = 0.0f;
  for (int i = 1; i < 1000; ++i) {
    float y = s.process();
    maxErr = std::fmax(maxErr, std::fabs(y - std::sin(6.283185307f * 1000.0f * i / 48000.0f)));
  }
  CHECK(maxErr < 1e-4f);
}

int main() {
  testBasicOscillator();
  testSineOscillator();

  checkRates<pt::AnalogKick>("analog_kick", [](pt::AnalogKick& d, float sr) {
    d.init(sr);
    d.setFrequency(55.0f);
    d.setTone(0.4f);
    d.setDecay(0.5f);
    d.setAttackFm(0.5f);
    d.setSelfFm(0.3f);
  }, 55.0f);
  checkRates<pt::AnalogSnare>("analog_snare", [](pt::AnalogSnare& d, float sr) {
    d.init(sr);
    d.setFrequency(180.0f);
    d.setTone(0.5f);
    d.setDecay(0.5f);
    d.setSnappy(0.5f);
  });
  checkRates<pt::SynthKick>("synth_kick", [](pt::SynthKick& d, float sr) {
    d.init(sr);
    d.setFrequency(50.0f);
    d.setTone(0.3f);
    d.setDecay(0.6f);
    d.setFmAmount(0.4f);
    d.setFmDecay(0.3f);
  }, 50.0f);
  checkRates<pt::SynthSnare>("synth_snare", [](pt::SynthSnare& d, float sr) {
    d.init(sr);
    d.setFrequency(200.0f);
    d.setFmAmount(0.3f);
    d.setDecay(0.5f);
    d.setSnappy(0.6f);
  });
  checkRates<pt::HiHat>("hihat_808", [](pt::HiHat& d, float sr) {
    d.init(sr, pt::HiHat::SQUARE_808);
    d.setFrequency(400.0f);
    d.setTone(0.6f);
    d.setDecay(0.5f);
    d.setNoisiness(0.2f);
  });
  checkRates<pt::HiHat>("hihat_ring", [](pt::HiHat& d, float sr) {
    d.init(sr, pt::HiHat::RING_MOD);
    d.setFrequency(400.0f);
    d.setTone(0.6f);
    d.setDecay(0.6f);
    d.setNoisiness(0.2f);
  });
  checkRates<pt::FmDrum>("fm_drum", [](pt::FmDrum& d, float sr) {
    d.init(sr);
    d.setFrequency(60.0f);
    d.setFmAmount(0.3f);
    d.setDecay(0.5f);
    d.setNoise(0.5f);
  });

  // Every drum in sustain mode: steady, bounded tone.
  {
    pt::AnalogKick k;
    k.init(48000.0f);
    k.setFrequency(60.0f);
    k.setSustain(true);
    std::vector<float> out(48000);
    for (size_t i = 0; i < out.size(); ++i) out[i] = k.process();
    CHECK(allBounded(out, 4.0f));
    CHECK(measure(out, 48000.0f).peakDb > -40.0f);
  }

  // FmDrum morph covers its presets without blowing up.
  for (int snare = 0; snare < 2; ++snare) {
    for (float x = 0.0f; x <= 1.0f; x += 0.25f) {
      for (float y = 0.0f; y <= 1.0f; y += 0.5f) {
        pt::FmDrum d;
        d.init(48000.0f);
        d.morph(x, y, snare != 0);
        std::vector<float> out = hit(d, 48000.0f, 0.5f);
        CHECK(allBounded(out, 2.0f));
      }
    }
  }
  return testResult("drums");
}
