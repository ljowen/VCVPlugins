#include "plugin.hpp"
#include <dsp/fft.hpp>
#include <iterator>
#include <algorithm>

struct MyModule : Module
{
	enum ParamIds
	{
		PITCH_PARAM,
		NUM_PARAMS
	};
	enum InputIds
	{
		SIGNAL_INPUT,
		NUM_INPUTS
	};
	enum OutputIds
	{
		V_OCT_OUTPUT,
		NUM_OUTPUTS
	};
	enum LightIds
	{
		BLINK_LIGHT,
		NUM_LIGHTS
	};

	MyModule()
	{
		config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
		configParam(PITCH_PARAM, 0.f, 1.f, 0.f, "");
	}

	float phase = 0.f;
	float blinkPhase = 0.f;
	static const int BUFFER_LEN = 2048;
	float buffer[BUFFER_LEN] = {0};
	float bufferWindowed[BUFFER_LEN] = {0};

	rack::dsp::RealFFT fft = rack::dsp::RealFFT(BUFFER_LEN);
	float fftOutput[BUFFER_LEN] = {0};	
	long buffIdx = 0;
	float frequencySmoothed = 0;

	float outputVoltage = 0;

	int getFFTMaxIdx()
	{
		int idx = std::distance(fftOutput, std::max_element(std::begin(fftOutput), std::end(fftOutput)));
		return idx;
	}

	void process(const ProcessArgs &args) override
	{
		// Compute the frequency from the pitch parameter and input
		float sample = inputs[SIGNAL_INPUT].getVoltage();

		// Add this sample to the buffer
		if (buffIdx == BUFFER_LEN)
		{
			buffIdx = 0;
		}
		buffer[buffIdx] = sample;
		buffIdx += 1;

		if (buffIdx % 100 == 0)
		{
			std::copy(std::begin(buffer), std::end(buffer), std::begin(bufferWindowed));
			rack::dsp::hannWindow(bufferWindowed, BUFFER_LEN);
			fft.rfft(bufferWindowed, fftOutput);
			fftOutput[0] = 0; // discard DC component

			int fftPeak = getFFTMaxIdx();
			float frequency = (args.sampleRate * fftPeak / BUFFER_LEN) / 2;

			// frequencySmoothed = (alpha * new_value) + (1.0 - alpha) * accumulator

			// DEBUG("I think the f is %f ", frequency / 2);
			// update output voltages
			outputVoltage = log2( (frequency / 2) / rack::dsp::FREQ_C4);
		}

		outputs[V_OCT_OUTPUT].setVoltage(outputVoltage);
		// Blink light at 1Hz
		blinkPhase += args.sampleTime;
		if (blinkPhase >= 1.f)
			blinkPhase -= 1.f;
		lights[BLINK_LIGHT].setBrightness(blinkPhase < 0.5f ? 1.f : 0.f);
	}
};

struct MyModuleWidget : ModuleWidget
{
	MyModuleWidget(MyModule *module)
	{
		setModule(module);
		setPanel(APP->window->loadSvg(asset::plugin(pluginInstance, "res/MyModule.svg")));

		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(15.24, 46.063)), module, MyModule::PITCH_PARAM));

		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(15.24, 77.478)), module, MyModule::SIGNAL_INPUT));

		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(15.24, 108.713)), module, MyModule::V_OCT_OUTPUT));

		addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(15.24, 25.81)), module, MyModule::BLINK_LIGHT));
	}
};

Model *modelMyModule = createModel<MyModule, MyModuleWidget>("MyModule");