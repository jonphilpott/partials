#include "plugin.hpp"

#include "pt/core/units.h"
#include "pt/filter/svf.h"
#include "pt/osc/basic.h"


struct SawFilter : Module {
	enum ParamId {
		FREQ_PARAM,
		CUTOFF_PARAM,
		PARAMS_LEN
	};
	enum InputId {
		INPUTS_LEN
	};
	enum OutputId {
		OUT_OUTPUT,
		OUTPUTS_LEN
	};
	enum LightId {
		LIGHTS_LEN
	};

	// The two partials components: plain member variables.
	pt::BasicOscillator oscillator;
	pt::Svf filter;

	SawFilter() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		// Frequency knob: -4..+4 octaves around C4. Rack shows it in Hz:
		// 261.63 * 2^value, from 16 Hz to 4186 Hz.
		configParam(FREQ_PARAM, -4.f, 4.f, 0.f, "Frequency", " Hz", 2.f, pt::kFreqC4);
		// Cutoff knob: 0..1 mapped exponentially to 20 Hz..20 kHz. Rack
		// shows it in Hz: 20 * 1000^value.
		configParam(CUTOFF_PARAM, 0.f, 1.f, 0.5f, "Cutoff", " Hz", 1000.f, 20.f);
		configOutput(OUT_OUTPUT, "Audio");

		oscillator.setShape(pt::BasicOscillator::SAW);
	}

	// Called when the module is added and whenever the sample rate
	// changes: the place to (re)initialise DSP components.
	void onSampleRateChange(const SampleRateChangeEvent& e) override {
		oscillator.init(e.sampleRate);
		filter.init(e.sampleRate);
	}

	// Called once per sample.
	void process(const ProcessArgs& args) override {
		// 1. Read the knobs and convert them to Hz.
		float frequency = pt::voltToHz(params[FREQ_PARAM].getValue());
		float cutoff = 20.f * std::pow(1000.f, params[CUTOFF_PARAM].getValue());

		// 2. Oscillator: one sample of an alias-free sawtooth, about +-1.
		oscillator.setFrequency(frequency);
		float saw = oscillator.process();

		// 3. Filter: the saw through a 12 dB/octave low-pass. Q 1.0 adds a
		// little resonance at the cutoff.
		filter.setFrequency(cutoff, 1.f);
		float filtered = filter.process(saw).lp;

		// 4. Rack's audio level is +-5 V.
		outputs[OUT_OUTPUT].setVoltage(5.f * filtered);
	}
};


struct SawFilterWidget : ModuleWidget {
	SawFilterWidget(SawFilter* module) {
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/SawFilter.svg")));

		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(15.24, 38.0)), module, SawFilter::FREQ_PARAM));
		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(15.24, 68.0)), module, SawFilter::CUTOFF_PARAM));

		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(15.24, 108.0)), module, SawFilter::OUT_OUTPUT));
	}
};


Model* modelSawFilter = createModel<SawFilter, SawFilterWidget>("SawFilter");
