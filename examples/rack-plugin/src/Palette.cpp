// Palette: six of mutablelib's oscillators behind one set of controls.
//
// MODEL picks the oscillator; TIMBRE and COLOR are mapped to each one's
// most musical pair of controls. Only the selected oscillator runs, which
// saves CPU; switching model may click, since the new one starts from
// wherever it last stopped.

#include "plugin.hpp"

#include "ml/core/units.h"
#include "ml/mod/hysteresis_quantizer.h"
#include "ml/osc/formant.h"
#include "ml/osc/grainlet.h"
#include "ml/osc/string_synth.h"
#include "ml/osc/variable_shape.h"
#include "ml/osc/vosim.h"
#include "ml/osc/z_osc.h"

struct Palette : Module {
  enum ParamId { MODEL_PARAM, PITCH_PARAM, TIMBRE_PARAM, COLOR_PARAM, PARAMS_LEN };
  enum InputId { VOCT_INPUT, TIMBRE_INPUT, COLOR_INPUT, MODEL_INPUT, INPUTS_LEN };
  enum OutputId { OUT_OUTPUT, OUTPUTS_LEN };

  static const int kNumModels = 6;
  ml::HysteresisQuantizer modelSelect;
  ml::VariableShapeOscillator va;
  ml::FormantOscillator formant;
  ml::ZOscillator z;
  ml::VosimOscillator vosim;
  ml::GrainletOscillator grainlet;
  ml::StringSynthOscillator strings;

  Palette() {
    config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, 0);
    configSwitch(MODEL_PARAM, 0.f, kNumModels - 1, 0.f, "Model",
                 {"Variable shape", "Formant", "Z (CZ resonance)", "VOSIM", "Grainlet", "String machine"});
    configParam(PITCH_PARAM, -4.f, 4.f, 0.f, "Pitch", " Hz", 2.f, ml::kFreqC4);
    configParam(TIMBRE_PARAM, 0.f, 1.f, 0.5f, "Timbre");
    configParam(COLOR_PARAM, 0.f, 1.f, 0.5f, "Color");
    configInput(VOCT_INPUT, "1V/oct");
    configInput(TIMBRE_INPUT, "Timbre CV");
    configInput(COLOR_INPUT, "Color CV");
    configInput(MODEL_INPUT, "Model CV (0-10 V)");
    configOutput(OUT_OUTPUT, "Audio");
    modelSelect.init(kNumModels);
  }

  void onSampleRateChange(const SampleRateChangeEvent& e) override {
    va.init(e.sampleRate);
    formant.init(e.sampleRate);
    z.init(e.sampleRate);
    vosim.init(e.sampleRate);
    grainlet.init(e.sampleRate);
    strings.init(e.sampleRate);
  }

  void process(const ProcessArgs& args) override {
    // 1. Controls (knob + CV, 0..1).
    float hz = ml::voltToHz(params[PITCH_PARAM].getValue() + inputs[VOCT_INPUT].getVoltage());
    float timbre = ml::clamp(params[TIMBRE_PARAM].getValue() + inputs[TIMBRE_INPUT].getVoltage() / 10.f, 0.f, 1.f);
    float color = ml::clamp(params[COLOR_PARAM].getValue() + inputs[COLOR_INPUT].getVoltage() / 10.f, 0.f, 1.f);
    int model = inputs[MODEL_INPUT].isConnected()
        ? modelSelect.process(ml::clamp(inputs[MODEL_INPUT].getVoltage() / 10.f, 0.f, 1.f))
        : static_cast<int>(params[MODEL_PARAM].getValue());

    // 2. Formant frequencies follow the pitch, so timbre is the same on
    // every note.
    float formantHz = hz * std::pow(2.f, 1.f + 4.f * timbre);

    float out = 0.f;
    switch (model) {
      case 0:  // TIMBRE: shape (triangle-saw-square), COLOR: pulse width
        va.setFrequency(hz);
        va.setShape(timbre);
        va.setPulseWidth(0.5f + 0.45f * (color - 0.5f) * 2.f);
        out = va.process();
        break;
      case 1:  // TIMBRE: formant, COLOR: phase shift
        formant.setCarrierFrequency(hz);
        formant.setFormantFrequency(formantHz);
        formant.setPhaseShift(color);
        out = formant.process();
        break;
      case 2:  // TIMBRE: resonance, COLOR: window shape
        z.setCarrierFrequency(hz);
        z.setFormantFrequency(formantHz);
        z.setShape(color);
        z.setMode(0.2f);
        out = z.process();
        break;
      case 3:  // TIMBRE: first formant, COLOR: second formant
        vosim.setCarrierFrequency(hz);
        vosim.setFormantFrequencies(formantHz, hz * std::pow(2.f, 2.f + 4.f * color));
        vosim.setShape(0.3f);
        out = vosim.process();
        break;
      case 4:  // TIMBRE: grain frequency, COLOR: window shape
        grainlet.setCarrierFrequency(hz);
        grainlet.setFormantFrequency(formantHz);
        grainlet.setShape(color);
        grainlet.setBleed(0.2f);
        out = grainlet.process();
        break;
      default:  // TIMBRE: registration, COLOR: unused
        strings.setFrequency(hz * 0.5f);
        strings.setRegistration(timbre);
        out = strings.process();
        break;
    }
    outputs[OUT_OUTPUT].setVoltage(5.f * out);
  }
};

struct PaletteWidget : ModuleWidget {
  PaletteWidget(Palette* module) {
    setModule(module);
    setPanel(createPanel(asset::plugin(pluginInstance, "res/Palette.svg")));
    addParam(createParamCentered<RoundBlackSnapKnob>(mm2px(Vec(20.32, 22)), module, Palette::MODEL_PARAM));
    addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(20.32, 42)), module, Palette::PITCH_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(10.16, 64)), module, Palette::TIMBRE_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(30.48, 64)), module, Palette::COLOR_PARAM));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.16, 84)), module, Palette::TIMBRE_INPUT));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(30.48, 84)), module, Palette::COLOR_INPUT));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.16, 100)), module, Palette::VOCT_INPUT));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(30.48, 100)), module, Palette::MODEL_INPUT));
    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(20.32, 116)), module, Palette::OUT_OUTPUT));
  }
};

Model* modelPalette = createModel<Palette, PaletteWidget>("Palette");
