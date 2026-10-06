// Bridge: two inputs, three ways of combining them, built from
// partials's spectral parts (all from Warps).
//
//   XMOD:    Xmod::morph sweeps crossfade, fold, ring mods, XOR, comparator
//   SHIFT:   the carrier frequency-shifted (Hilbert + quadrature sine)
//   VOCODER: the modulator's spectrum imposed on the carrier
//
// The same pattern as every other part: plain members, init() in
// onSampleRateChange, setters, one process() call per sample.

#include "plugin.hpp"

#include "pt/core/math.h"
#include "pt/filter/hilbert.h"
#include "pt/spectral/vocoder.h"
#include "pt/spectral/xmod.h"

struct Bridge : Module {
  enum ParamId { ALGORITHM_PARAM, TIMBRE_PARAM, SHIFT_PARAM, RELEASE_PARAM, PARAMS_LEN };
  enum InputId { CARRIER_INPUT, MODULATOR_INPUT, ALGORITHM_INPUT, SHIFT_INPUT, INPUTS_LEN };
  enum OutputId { XMOD_OUTPUT, SHIFT_OUTPUT, VOCODER_OUTPUT, OUTPUTS_LEN };

  pt::Hilbert hilbert;
  pt::Vocoder vocoder;
  float shiftPhase = 0.f;

  Bridge() {
    config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, 0);
    configParam(ALGORITHM_PARAM, 0.f, 1.f, 0.f, "Algorithm");
    configParam(TIMBRE_PARAM, 0.f, 1.f, 0.5f, "Timbre");
    configParam(SHIFT_PARAM, -1.f, 1.f, 0.f, "Frequency shift", " Hz", 0.f, 500.f);
    configParam(RELEASE_PARAM, 0.f, 1.f, 0.3f, "Vocoder release");
    configInput(CARRIER_INPUT, "Carrier");
    configInput(MODULATOR_INPUT, "Modulator");
    configInput(ALGORITHM_INPUT, "Algorithm CV");
    configInput(SHIFT_INPUT, "Shift CV (100 Hz/V)");
    configOutput(XMOD_OUTPUT, "Cross-modulation");
    configOutput(SHIFT_OUTPUT, "Frequency-shifted carrier");
    configOutput(VOCODER_OUTPUT, "Vocoder");
  }

  void onSampleRateChange(const SampleRateChangeEvent& e) override {
    hilbert.init(e.sampleRate);
    vocoder.init(e.sampleRate);
  }

  void process(const ProcessArgs& args) override {
    float carrier = inputs[CARRIER_INPUT].getVoltage() / 5.f;
    float modulator = inputs[MODULATOR_INPUT].getVoltage() / 5.f;

    // 1. Cross-modulation, one knob through all of Warps' algorithms.
    float algorithm = pt::clamp(params[ALGORITHM_PARAM].getValue() + inputs[ALGORITHM_INPUT].getVoltage() / 10.f, 0.f, 1.f);
    float xmod = pt::Xmod::morph(algorithm, modulator, carrier, params[TIMBRE_PARAM].getValue());
    outputs[XMOD_OUTPUT].setVoltage(5.f * pt::clamp(xmod, -2.f, 2.f));

    // 2. Frequency shifter: rotate the carrier's analytic signal (I, Q)
    // by a sine/cosine at the shift frequency.
    float shiftHz = 500.f * params[SHIFT_PARAM].getValue() + 100.f * inputs[SHIFT_INPUT].getVoltage();
    shiftPhase += shiftHz * args.sampleTime;
    shiftPhase -= std::floor(shiftPhase);
    float i = hilbert.process(carrier);
    float q = hilbert.q();
    float c = std::cos(2.f * M_PI * shiftPhase), s = std::sin(2.f * M_PI * shiftPhase);
    outputs[SHIFT_OUTPUT].setVoltage(5.f * (i * c - q * s));

    // 3. Vocoder. Only runs when its output is patched: it is the
    // expensive part (about 160 filter sections per sample).
    if (outputs[VOCODER_OUTPUT].isConnected()) {
      vocoder.setRelease(params[RELEASE_PARAM].getValue());
      outputs[VOCODER_OUTPUT].setVoltage(5.f * vocoder.process(modulator, carrier));
    }
  }
};

struct BridgeWidget : ModuleWidget {
  BridgeWidget(Bridge* module) {
    setModule(module);
    setPanel(createPanel(asset::plugin(pluginInstance, "res/Bridge.svg")));
    addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(20.32, 24)), module, Bridge::ALGORITHM_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(10.16, 46)), module, Bridge::TIMBRE_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(30.48, 46)), module, Bridge::SHIFT_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(20.32, 64)), module, Bridge::RELEASE_PARAM));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.16, 82)), module, Bridge::CARRIER_INPUT));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(30.48, 82)), module, Bridge::MODULATOR_INPUT));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.16, 96)), module, Bridge::ALGORITHM_INPUT));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(30.48, 96)), module, Bridge::SHIFT_INPUT));
    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(8.0, 114)), module, Bridge::XMOD_OUTPUT));
    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(20.32, 114)), module, Bridge::SHIFT_OUTPUT));
    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(32.64, 114)), module, Bridge::VOCODER_OUTPUT));
  }
};

Model* modelBridge = createModel<Bridge, BridgeWidget>("Bridge");
