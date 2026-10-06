// Haze: a stereo effects chain built from partials's effects.
//
//   Overdrive -> Ensemble -> Diffuser -> Reverb -> Limiter
//
// Every part follows the same pattern: a plain member, init() in
// onSampleRateChange (the delay-based effects allocate memory there),
// setters, then one process() call per sample.

#include "plugin.hpp"

#include "pt/fx/diffuser.h"
#include "pt/fx/ensemble.h"
#include "pt/fx/limiter.h"
#include "pt/fx/overdrive.h"
#include "pt/fx/reverb.h"

struct Haze : Module {
  enum ParamId { DRIVE_PARAM, ENSEMBLE_PARAM, TEXTURE_PARAM, REVERB_PARAM, SIZE_PARAM, TONE_PARAM, PARAMS_LEN };
  enum InputId { LEFT_INPUT, RIGHT_INPUT, INPUTS_LEN };
  enum OutputId { LEFT_OUTPUT, RIGHT_OUTPUT, OUTPUTS_LEN };

  pt::Overdrive driveL, driveR;
  pt::Ensemble ensemble;
  pt::Diffuser diffuser;
  pt::Reverb reverb;
  pt::Limiter limiter;

  Haze() {
    config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, 0);
    configParam(DRIVE_PARAM, 0.f, 1.f, 0.f, "Drive");
    configParam(ENSEMBLE_PARAM, 0.f, 1.f, 0.3f, "Ensemble");
    configParam(TEXTURE_PARAM, 0.f, 1.f, 0.3f, "Texture (diffusion)");
    configParam(REVERB_PARAM, 0.f, 1.f, 0.4f, "Reverb mix");
    configParam(SIZE_PARAM, 0.f, 1.f, 0.6f, "Reverb size");
    configParam(TONE_PARAM, 0.f, 1.f, 0.6f, "Reverb tone");
    configInput(LEFT_INPUT, "Left");
    configInput(RIGHT_INPUT, "Right (normalled to left)");
    configOutput(LEFT_OUTPUT, "Left");
    configOutput(RIGHT_OUTPUT, "Right");
  }

  void onSampleRateChange(const SampleRateChangeEvent& e) override {
    driveL.init(e.sampleRate);
    driveR.init(e.sampleRate);
    ensemble.init(e.sampleRate);
    diffuser.init(e.sampleRate);
    reverb.init(e.sampleRate, pt::Reverb::ELEMENTS);
    limiter.init(e.sampleRate);
  }

  void process(const ProcessArgs& args) override {
    float drive = params[DRIVE_PARAM].getValue();
    float ensembleAmount = params[ENSEMBLE_PARAM].getValue();
    float size = params[SIZE_PARAM].getValue();

    // 1. Inputs (Rack ±5 V to ±1); right is normalled to left.
    float l = inputs[LEFT_INPUT].getVoltage() / 5.f;
    float r = inputs[RIGHT_INPUT].getNormalVoltage(inputs[LEFT_INPUT].getVoltage()) / 5.f;

    // 2. Overdrive, bypassed at zero so the chain can stay clean.
    if (drive > 0.f) {
      driveL.setDrive(drive);
      driveR.setDrive(drive);
      l = driveL.process(l);
      r = driveR.process(r);
    }

    // 3. Ensemble: depth and mix follow one knob.
    ensemble.setAmount(ensembleAmount);
    ensemble.setDepth(0.3f + 0.7f * ensembleAmount);
    ensemble.process(l, r);

    // 4. Diffuser, then reverb (settings ranges from Clouds and Rings).
    diffuser.setAmount(params[TEXTURE_PARAM].getValue());
    diffuser.process(l, r);
    reverb.setAmount(params[REVERB_PARAM].getValue());
    reverb.setTime(0.35f + 0.63f * size);
    reverb.setLp(0.3f + 0.65f * params[TONE_PARAM].getValue());
    reverb.process(l, r);

    // 5. Keep the output within ±5 V.
    limiter.process(l, r);
    outputs[LEFT_OUTPUT].setVoltage(5.f * l);
    outputs[RIGHT_OUTPUT].setVoltage(5.f * r);
  }
};

struct HazeWidget : ModuleWidget {
  HazeWidget(Haze* module) {
    setModule(module);
    setPanel(createPanel(asset::plugin(pluginInstance, "res/Haze.svg")));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(10.16, 26)), module, Haze::DRIVE_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(30.48, 26)), module, Haze::ENSEMBLE_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(10.16, 46)), module, Haze::TEXTURE_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(30.48, 46)), module, Haze::REVERB_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(10.16, 66)), module, Haze::SIZE_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(30.48, 66)), module, Haze::TONE_PARAM));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.16, 96)), module, Haze::LEFT_INPUT));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(30.48, 96)), module, Haze::RIGHT_INPUT));
    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(10.16, 114)), module, Haze::LEFT_OUTPUT));
    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(30.48, 114)), module, Haze::RIGHT_OUTPUT));
  }
};

Model* modelHaze = createModel<Haze, HazeWidget>("Haze");
