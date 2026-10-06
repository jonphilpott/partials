// Drift: clocked random voltages, Marbles-style, built from mutablelib's
// modulation parts.
//
//   clock -> ClockToRamp -> new value each step from RandomSequence
//                        -> Lag glides between steps in time with the clock
//                        -> Quantizer snaps the stepped value to a scale
//   clock -> Slope (AD) gives an envelope on every step
//
// The pattern for every part: a plain member, init() in onSampleRateChange
// (or the constructor, for parts with no sample rate), setters, and one
// process() call per sample.

#include "plugin.hpp"

#include "ml/mod/clock_to_ramp.h"
#include "ml/mod/lag.h"
#include "ml/mod/quantizer.h"
#include "ml/mod/random_sequence.h"
#include "ml/mod/slope.h"

struct Drift : Module {
  enum ParamId { DEJA_VU_PARAM, LENGTH_PARAM, STEPS_PARAM, QUANTIZE_PARAM, DECAY_PARAM, PARAMS_LEN };
  enum InputId { CLOCK_INPUT, INPUTS_LEN };
  enum OutputId { PITCH_OUTPUT, SMOOTH_OUTPUT, ENV_OUTPUT, OUTPUTS_LEN };

  ml::ClockToRamp clock;
  ml::RandomSequence sequence;
  ml::Lag lag;
  ml::Quantizer quantizer;
  ml::Slope envelope;
  float value = 0.5f;
  float previousRamp = 0.f;

  Drift() {
    config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, 0);
    configParam(DEJA_VU_PARAM, 0.f, 1.f, 0.f, "Deja vu");
    configParam(LENGTH_PARAM, 1.f, 16.f, 8.f, "Loop length");
    paramQuantities[LENGTH_PARAM]->snapEnabled = true;
    configParam(STEPS_PARAM, 0.f, 1.f, 0.f, "Steps (smoothness)");
    configParam(QUANTIZE_PARAM, 0.f, 1.f, 0.5f, "Quantize amount");
    configParam(DECAY_PARAM, 0.f, 1.f, 0.5f, "Envelope decay");
    configInput(CLOCK_INPUT, "Clock");
    configOutput(PITCH_OUTPUT, "Quantized pitch (V/oct, 2 octaves)");
    configOutput(SMOOTH_OUTPUT, "Smoothed random (0-10 V)");
    configOutput(ENV_OUTPUT, "Envelope per step (0-8 V)");
    sequence.init();
    lag.init();
    quantizer.init(ml::Quantizer::preset(ml::Quantizer::MAJOR));
  }

  void onSampleRateChange(const SampleRateChangeEvent& e) override {
    clock.init(e.sampleRate);
    envelope.init(e.sampleRate);
    envelope.setMode(ml::Slope::AD);
    envelope.setSlope(0.05f);  // fast attack
  }

  void process(const ProcessArgs& args) override {
    bool gate = inputs[CLOCK_INPUT].getVoltage() >= 1.f;

    // 1. A smooth phase ramp locked to the clock; a wrap means a new step.
    float ramp = clock.process(gate);
    bool newStep = ramp < previousRamp - 0.5f;
    previousRamp = ramp;

    // 2. New random value on each step (with deja vu looping).
    sequence.setDejaVu(params[DEJA_VU_PARAM].getValue());
    sequence.setLength(static_cast<int>(params[LENGTH_PARAM].getValue()));
    if (newStep) {
      value = sequence.next();
      lag.resetRamp();
    }

    // 3. Outputs: quantized pitch, glide in time with the clock, envelope.
    float pitch = quantizer.process(2.f * value, params[QUANTIZE_PARAM].getValue());
    outputs[PITCH_OUTPUT].setVoltage(pitch);
    outputs[SMOOTH_OUTPUT].setVoltage(10.f * lag.process(value, params[STEPS_PARAM].getValue(), ramp));

    // The envelope's length follows the clock: one AD cycle per step at
    // full decay, shorter below.
    float stepHz = std::max(clock.frequency(), 0.1f);
    envelope.setFrequency(stepHz / (0.05f + 0.95f * params[DECAY_PARAM].getValue()));
    outputs[ENV_OUTPUT].setVoltage(8.f * envelope.process(newStep));
  }
};

struct DriftWidget : ModuleWidget {
  DriftWidget(Drift* module) {
    setModule(module);
    setPanel(createPanel(asset::plugin(pluginInstance, "res/Drift.svg")));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(10.16, 24)), module, Drift::DEJA_VU_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(30.48, 24)), module, Drift::LENGTH_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(10.16, 44)), module, Drift::STEPS_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(30.48, 44)), module, Drift::QUANTIZE_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(20.32, 64)), module, Drift::DECAY_PARAM));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(20.32, 84)), module, Drift::CLOCK_INPUT));
    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(8.0, 108)), module, Drift::PITCH_OUTPUT));
    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(20.32, 108)), module, Drift::SMOOTH_OUTPUT));
    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(32.64, 108)), module, Drift::ENV_OUTPUT));
  }
};

Model* modelDrift = createModel<Drift, DriftWidget>("Drift");
