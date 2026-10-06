// Tests for the core/ headers.

#include "pt/core/math.h"
#include "pt/core/random.h"
#include "pt/core/tables.h"
#include "pt/core/units.h"

#include "test.h"

int main() {
  // clamp
  CHECK(pt::clamp(2.0f, -1.0f, 1.0f) == 1.0f);
  CHECK(pt::clamp(-2.0f, -1.0f, 1.0f) == -1.0f);
  CHECK(pt::clamp(0.5f, -1.0f, 1.0f) == 0.5f);

  // interpolate: table {0, 10, 20}, 2 steps; halfway between entries 0 and 1.
  const float t[] = {0.0f, 10.0f, 20.0f};
  CHECK_NEAR(pt::interpolate(t, 0.25f, 2.0f), 5.0f, 1e-6f);
  CHECK_NEAR(pt::interpolateWrap(t, 1.25f, 2.0f), 5.0f, 1e-6f);

  // Hermite reproduces a straight line exactly.
  const float line[] = {-1.0f, 0.0f, 1.0f, 2.0f, 3.0f, 4.0f};
  CHECK_NEAR(pt::interpolateHermite(line + 1, 0.5f, 2.0f), 1.0f, 1e-6f);
  CHECK_NEAR(pt::interpolateHermite(line + 1, 0.3f, 2.0f), 0.6f, 1e-6f);

  // softLimit hits ±1 at ±3; softClip holds there.
  CHECK_NEAR(pt::softLimit(3.0f), 1.0f, 1e-6f);
  CHECK(pt::softClip(10.0f) == 1.0f);
  CHECK(pt::softClip(-10.0f) == -1.0f);

  // onePole converges; slew moves by at most maxDelta.
  float s = 0.0f;
  for (int i = 0; i < 1000; ++i) pt::onePole(s, 1.0f, 0.01f);
  CHECK_NEAR(s, 1.0f, 1e-4f);
  s = 0.0f;
  pt::slew(s, 1.0f, 0.1f);
  CHECK_NEAR(s, 0.1f, 1e-6f);

  // Units: C4 = 0 V, octave = 1 V, A4 = MIDI 69 = 440 Hz.
  CHECK_NEAR(pt::voltToHz(0.0f), 261.6256f, 1e-3f);
  CHECK_NEAR(pt::voltToHz(1.0f), 523.2512f, 1e-3f);
  CHECK_NEAR(pt::hzToVolt(523.2512f), 1.0f, 1e-5f);
  CHECK_NEAR(pt::midiToHz(69.0f), 440.0f, 1e-3f);
  CHECK_NEAR(pt::semitonesToRatio(-12.0f), 0.5f, 1e-6f);

  // Random: stays in range; same seed gives the same sequence (matches
  // stmlib's LCG constants).
  pt::Random a, b;
  a.seed(42);
  b.seed(42);
  bool same = true, inRange = true;
  for (int i = 0; i < 10000; ++i) {
    float x = a.uniform();
    if (x != b.uniform()) same = false;
    if (x < 0.0f || x >= 1.0f) inRange = false;
  }
  CHECK(same);
  CHECK(inRange);
  pt::Random c;
  c.seed(0);
  CHECK(c.word() == 1013904223u);

  // Sine table: sin at 0, quarter, half; cosine readable at +quarter.
  const float* sine = pt::sineTable();
  CHECK_NEAR(sine[0], 0.0f, 1e-6f);
  CHECK_NEAR(sine[128], 1.0f, 1e-6f);
  CHECK_NEAR(sine[256], 0.0f, 1e-6f);
  CHECK_NEAR(sine[640], 1.0f, 1e-6f);  // last guard = sin(5π/2)
  CHECK_NEAR(pt::interpolateWrap(sine, 1.125f, pt::kSineTableSize),
             0.70710678f, 1e-4f);

  return testResult("core");
}
