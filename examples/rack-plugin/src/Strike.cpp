// Strike: a small voice built from three mutablelib parts.
//
//   Plucker  ->  ModalResonator  ->  LowPassGate
//   (burst)      (the object)        (vactrol VCA + filter)
//
// A trigger plucks the resonator and pings the gate at the same time. The
// pattern for each part is the same: a plain member, init() in
// onSampleRateChange, setters, then one process() call per sample.

#include "plugin.hpp"

#include "ml/core/units.h"
#include "ml/dynamics/lpg.h"
#include "ml/filter/dc_blocker.h"
#include "ml/physical/modal_resonator.h"
#include "ml/physical/plucker.h"

struct Strike : Module {
  enum ParamId {
    PITCH_PARAM,
    STRUCTURE_PARAM,
    BRIGHTNESS_PARAM,
    DAMPING_PARAM,
    POSITION_PARAM,
    DECAY_PARAM,
    PARAMS_LEN
  };
  enum InputId { VOCT_INPUT, TRIG_INPUT, IN_INPUT, INPUTS_LEN };
  enum OutputId { OUT_OUTPUT, OUTPUTS_LEN };

  ml::Plucker plucker;
  ml::ModalResonator resonator;
  ml::LowPassGate lpg;
  ml::DcBlocker dcBlocker;
  dsp::SchmittTrigger trigger;

  Strike() {
    config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, 0);
    configParam(PITCH_PARAM, -3.f, 3.f, 0.f, "Pitch", " Hz", 2.f, ml::kFreqC4);
    configParam(STRUCTURE_PARAM, 0.f, 1.f, 0.27f, "Structure");
    configParam(BRIGHTNESS_PARAM, 0.f, 1.f, 0.6f, "Brightness");
    configParam(DAMPING_PARAM, 0.f, 1.f, 0.6f, "Damping");
    configParam(POSITION_PARAM, 0.f, 1.f, 0.3f, "Position");
    configParam(DECAY_PARAM, 0.f, 1.f, 0.6f, "Gate decay");
    configInput(VOCT_INPUT, "1V/oct");
    configInput(TRIG_INPUT, "Trigger");
    configInput(IN_INPUT, "Audio (excites the resonator)");
    configOutput(OUT_OUTPUT, "Audio");
  }

  // Also called when the module is added, so this is the only init needed.
  void onSampleRateChange(const SampleRateChangeEvent& e) override {
    plucker.init(e.sampleRate);
    resonator.init(e.sampleRate);
    lpg.init(e.sampleRate);
    dcBlocker.init(e.sampleRate);
  }

  void process(const ProcessArgs& args) override {
    float hz = ml::voltToHz(params[PITCH_PARAM].getValue() + inputs[VOCT_INPUT].getVoltage());
    float brightness = params[BRIGHTNESS_PARAM].getValue();
    float position = params[POSITION_PARAM].getValue();

    // 1. Exciter. Brighter settings get a harder pick.
    plucker.setFrequency(hz);
    plucker.setCutoff(hz * (2.f + 30.f * brightness));
    plucker.setPosition(position);
    if (trigger.process(inputs[TRIG_INPUT].getVoltage(), 0.1f, 1.f)) {
      plucker.trigger();
      lpg.trigger();
    }
    float excitation = plucker.process() + inputs[IN_INPUT].getVoltage() / 5.f;

    // 2. Resonator.
    resonator.setFrequency(hz);
    resonator.setStructure(params[STRUCTURE_PARAM].getValue());
    resonator.setBrightness(brightness);
    resonator.setDamping(params[DAMPING_PARAM].getValue());
    resonator.setPosition(position);
    float s = dcBlocker.process(resonator.process(excitation));

    // 3. Gate. Without a trigger cable the gate stays open, so audio
    // input alone still sounds.
    lpg.setDecay(params[DECAY_PARAM].getValue());
    lpg.setColour(brightness);
    lpg.setAttackPitch(hz);
    float out = inputs[TRIG_INPUT].isConnected() ? lpg.process(s) : s;

    outputs[OUT_OUTPUT].setVoltage(5.f * out);
  }
};

struct StrikeWidget : ModuleWidget {
  StrikeWidget(Strike* module) {
    setModule(module);
    setPanel(createPanel(asset::plugin(pluginInstance, "res/Strike.svg")));
    addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(20.32, 24)), module, Strike::PITCH_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(10.16, 44)), module, Strike::STRUCTURE_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(30.48, 44)), module, Strike::BRIGHTNESS_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(10.16, 62)), module, Strike::DAMPING_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(30.48, 62)), module, Strike::POSITION_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(20.32, 80)), module, Strike::DECAY_PARAM));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.16, 98)), module, Strike::VOCT_INPUT));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(30.48, 98)), module, Strike::TRIG_INPUT));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.16, 114)), module, Strike::IN_INPUT));
    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(30.48, 114)), module, Strike::OUT_OUTPUT));
  }
};

Model* modelStrike = createModel<Strike, StrikeWidget>("Strike");
