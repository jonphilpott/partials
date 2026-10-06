// partials — filter/svf.h
//
// State-variable filter (SVF): one filter giving low-pass, band-pass and
// high-pass outputs at once.
//
// This is the "topology-preserving transform" (TPT, or zero-delay feedback)
// SVF described by Andrew Simper and Vadim Zavalishin. Unlike the classic
// Chamberlin SVF it stays stable and in tune right up to Nyquist, and its
// coefficients can change every sample without blowing up, which is why
// Gillet uses it everywhere: filters, resonator modes, formants, drums.
//
// Derived from stmlib dsp/filter.h, Copyright 2014 Emilie Gillet. MIT licence.

#ifndef PT_FILTER_SVF_H_
#define PT_FILTER_SVF_H_

#include <cmath>

namespace pt {

// The SVF needs g = tan(pi * f), where f is the cutoff divided by the sample
// rate. tan() was too slow for the firmware's Cortex-M4, so stmlib offers
// polynomial approximations. They differ audibly only near Nyquist, but
// components ported from Gillet's code use the same approximation as the
// original so they sound identical at the original sample rate.
enum class TanApprox {
  Exact,     // std::tan
  Accurate,  // 11th-order polynomial; good across the whole audio range
  Fast,      // 5th-order; tuned for 16 Hz..16 kHz at 48 kHz
  Dirty      // 3rd-order; fine below about 8 kHz at 48 kHz
};

// tan(pi * f) for normalised frequency f (0..0.5).
inline float tanApprox(float f, TanApprox approx) {
  // Each polynomial is a truncated Taylor series of tan(pi*f) with
  // coefficients tweaked to minimise error over its intended range. The
  // pi^n factors fold the "pi *" inside tan() into the coefficients. They
  // are computed in double, as stmlib does, so results match bit for bit.
  const double pi = 3.14159265358979323846;
  const double pi3 = pi * pi * pi;
  const double pi5 = pi3 * pi * pi;
  const float piF = static_cast<float>(pi);
  float f2 = f * f;
  switch (approx) {
    case TanApprox::Exact:
      f = f < 0.497f ? f : 0.497f;  // tan explodes at f = 0.5
      return std::tan(piF * f);
    case TanApprox::Dirty: {
      const float a = 3.736e-01f * pi3;
      return f * (piF + a * f * f);
    }
    case TanApprox::Fast: {
      const float a = 3.260e-01f * pi3;
      const float b = 1.823e-01f * pi5;
      return f * (piF + f2 * (a + b * f2));
    }
    case TanApprox::Accurate:
    default: {
      const double pi7 = pi5 * pi * pi;
      const double pi9 = pi7 * pi * pi;
      const double pi11 = pi9 * pi * pi;
      const float a = 3.333314036e-01f * pi3;
      const float b = 1.333923995e-01f * pi5;
      const float c = 5.33740603e-02f * pi7;
      const float d = 2.900525e-03f * pi9;
      const float e = 9.5168091e-03f * pi11;
      return f * (piF + f2 * (a + f2 * (b + f2 * (c + f2 * (d + f2 * e)))));
    }
  }
}

struct SvfOut {
  float lp, bp, hp;
};

class Svf {
 public:
  // Stores the sample rate (needed only by setFrequency) and clears state.
  void init(float sampleRate) {
    sampleTime_ = 1.0f / sampleRate;
    setCoefficients(0.01f, 100.0f, TanApprox::Dirty);
    reset();
  }

  void reset() { state1_ = state2_ = 0.0f; }

  // Cutoff in Hz; q = resonance (0.5 = no peak, 0.707 = Butterworth, 10+ =
  // ringing).
  void setFrequency(float hz, float q) {
    setCoefficients(hz * sampleTime_, q, TanApprox::Exact);
  }

  // Lower-level form used by components: f is cutoff / sampleRate.
  void setCoefficients(float f, float q, TanApprox approx) {
    setGQ(tanApprox(f, approx), q);
  }

  // Set the pre-warped frequency g directly (e.g. copied from another
  // filter's g()) with resonance q.
  void setGQ(float g, float q) {
    g_ = g;
    r_ = 1.0f / q;
    h_ = 1.0f / (1.0f + r_ * g_ + g_ * g_);
  }

  // One sample in, all three responses out. Unused outputs cost nothing:
  // the compiler drops them.
  //
  // How it works: the filter is two integrators in a loop. Solving that
  // loop for the high-pass output first (the line computing hp) lets each
  // integrator use the current sample instead of the previous one, which is
  // what keeps tuning exact at high frequencies.
  SvfOut process(float in) {
    SvfOut o;
    o.hp = (in - r_ * state1_ - g_ * state1_ - state2_) * h_;
    o.bp = g_ * o.hp + state1_;
    state1_ = g_ * o.hp + o.bp;
    o.lp = g_ * o.bp + state2_;
    state2_ = g_ * o.bp + o.lp;
    return o;
  }

  float g() const { return g_; }
  // 1 / q. Multiply bp by this for a band-pass with unity gain at the peak
  // whatever the resonance ("normalised band-pass").
  float r() const { return r_; }

 private:
  float sampleTime_ = 1.0f / 48000.0f;
  float g_ = 0.0f, r_ = 1.0f, h_ = 1.0f;
  float state1_ = 0.0f, state2_ = 0.0f;
};

}  // namespace pt

#endif  // PT_FILTER_SVF_H_
