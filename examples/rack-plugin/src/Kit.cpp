// Kit: three 808-style drum voices built from mutablelib's drums.
//
// Each voice has a trigger input, tune and decay knobs and its own output;
// a mix output sums all three. The pattern for every drum: a plain member,
// init() in onSampleRateChange, setters, trigger() on a rising edge, and
// process() once per sample.

#include "plugin.hpp"

#include "ml/core/units.h"
#include "ml/drums/analog_kick.h"
#include "ml/drums/analog_snare.h"
#include "ml/drums/hihat.h"

struct Kit : Module {
  enum ParamId {
    KICK_TUNE_PARAM, KICK_DECAY_PARAM,
    SNARE_TUNE_PARAM, SNARE_DECAY_PARAM,
    HAT_TUNE_PARAM, HAT_DECAY_PARAM,
    PARAMS_LEN
  };
  enum InputId { KICK_TRIG_INPUT, SNARE_TRIG_INPUT, HAT_TRIG_INPUT, INPUTS_LEN };
  enum OutputId { KICK_OUTPUT, SNARE_OUTPUT, HAT_OUTPUT, MIX_OUTPUT, OUTPUTS_LEN };

  ml::AnalogKick kick;
  ml::AnalogSnare snare;
  ml::HiHat hat;
  dsp::SchmittTrigger kickTrig, snareTrig, hatTrig;

  Kit() {
    config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, 0);
    // Tune knobs are in octaves around each drum's natural pitch.
    configParam(KICK_TUNE_PARAM, -1.f, 1.f, 0.f, "Kick tune", " Hz", 2.f, 50.f);
    configParam(KICK_DECAY_PARAM, 0.f, 1.f, 0.5f, "Kick decay");
    configParam(SNARE_TUNE_PARAM, -1.f, 1.f, 0.f, "Snare tune", " Hz", 2.f, 180.f);
    configParam(SNARE_DECAY_PARAM, 0.f, 1.f, 0.4f, "Snare decay");
    configParam(HAT_TUNE_PARAM, -1.f, 1.f, 0.f, "Hat tune", " Hz", 2.f, 400.f);
    configParam(HAT_DECAY_PARAM, 0.f, 1.f, 0.3f, "Hat decay");
    configInput(KICK_TRIG_INPUT, "Kick trigger");
    configInput(SNARE_TRIG_INPUT, "Snare trigger");
    configInput(HAT_TRIG_INPUT, "Hat trigger");
    configOutput(KICK_OUTPUT, "Kick");
    configOutput(SNARE_OUTPUT, "Snare");
    configOutput(HAT_OUTPUT, "Hat");
    configOutput(MIX_OUTPUT, "Mix");
  }

  void onSampleRateChange(const SampleRateChangeEvent& e) override {
    kick.init(e.sampleRate);
    snare.init(e.sampleRate);
    hat.init(e.sampleRate, ml::HiHat::SQUARE_808);
  }

  void process(const ProcessArgs& args) override {
    // 1. Settings. The tune knobs are octaves, so 2^knob is the ratio.
    kick.setFrequency(50.f * std::pow(2.f, params[KICK_TUNE_PARAM].getValue()));
    kick.setDecay(params[KICK_DECAY_PARAM].getValue());
    kick.setTone(0.4f);
    kick.setAttackFm(0.5f);
    kick.setSelfFm(0.3f);
    snare.setFrequency(180.f * std::pow(2.f, params[SNARE_TUNE_PARAM].getValue()));
    snare.setDecay(params[SNARE_DECAY_PARAM].getValue());
    snare.setTone(0.5f);
    snare.setSnappy(0.6f);
    hat.setFrequency(400.f * std::pow(2.f, params[HAT_TUNE_PARAM].getValue()));
    hat.setDecay(params[HAT_DECAY_PARAM].getValue());
    hat.setTone(0.6f);

    // 2. Triggers.
    if (kickTrig.process(inputs[KICK_TRIG_INPUT].getVoltage(), 0.1f, 1.f)) kick.trigger();
    if (snareTrig.process(inputs[SNARE_TRIG_INPUT].getVoltage(), 0.1f, 1.f)) snare.trigger();
    if (hatTrig.process(inputs[HAT_TRIG_INPUT].getVoltage(), 0.1f, 1.f)) hat.trigger();

    // 3. Render and mix.
    float k = kick.process(), s = snare.process(), h = hat.process();
    outputs[KICK_OUTPUT].setVoltage(5.f * k);
    outputs[SNARE_OUTPUT].setVoltage(5.f * s);
    outputs[HAT_OUTPUT].setVoltage(5.f * h);
    outputs[MIX_OUTPUT].setVoltage(5.f * (k + s + 0.5f * h) / 2.f);
  }
};

struct KitWidget : ModuleWidget {
  KitWidget(Kit* module) {
    setModule(module);
    setPanel(createPanel(asset::plugin(pluginInstance, "res/Kit.svg")));
    const float rows[3] = {24.f, 54.f, 84.f};
    const int tune[3] = {Kit::KICK_TUNE_PARAM, Kit::SNARE_TUNE_PARAM, Kit::HAT_TUNE_PARAM};
    const int decay[3] = {Kit::KICK_DECAY_PARAM, Kit::SNARE_DECAY_PARAM, Kit::HAT_DECAY_PARAM};
    const int trig[3] = {Kit::KICK_TRIG_INPUT, Kit::SNARE_TRIG_INPUT, Kit::HAT_TRIG_INPUT};
    const int out[3] = {Kit::KICK_OUTPUT, Kit::SNARE_OUTPUT, Kit::HAT_OUTPUT};
    for (int i = 0; i < 3; ++i) {
      addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(10.16, rows[i])), module, tune[i]));
      addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(30.48, rows[i])), module, decay[i]));
      addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.16, rows[i] + 14.f)), module, trig[i]));
      addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(30.48, rows[i] + 14.f)), module, out[i]));
    }
    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(20.32, 116)), module, Kit::MIX_OUTPUT));
  }
};

Model* modelKit = createModel<Kit, KitWidget>("Kit");
