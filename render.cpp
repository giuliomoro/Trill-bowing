#include <Bela.h>
#include <cmath>
#include <libraries/Trill/Trill.h>
#include <libraries/OnePole/OnePole.h>
#include <libraries/Oscillator/Oscillator.h>
#include <libraries/Scope/Scope.h>
#include <libraries/math_neon/math_neon.h>

#define NUM_TOUCH 5 // Number of touches on Trill sensor

// Trill object declaration
Trill touchSensor;

// Location of touches on Trill Bar
float gTouchLocation[NUM_TOUCH] = { 0.0, 0.0, 0.0, 0.0, 0.0 };
// Size of touches on Trill Bar
float gTouchSize[NUM_TOUCH] = { 0.0, 0.0, 0.0, 0.0, 0.0 };

float gLocationVelocity[NUM_TOUCH];
float gSizeVelocity[NUM_TOUCH];

// Number of active touches
int gNumActiveTouches = 0;

// Sleep time for auxiliary task in microseconds
unsigned int gTaskSleepTime = 5000; // microseconds

OnePole freqFilt[NUM_TOUCH], ampFilt[NUM_TOUCH];
float gCutOffFreq = 5, gCutOffAmp = 1;
// Oscillators objects declaration
Oscillator osc[NUM_TOUCH];
// Range for oscillator frequency mapping
float gFreqRange[2] = { 200.0, 1500.0 };
// Range for oscillator amplitude mapping
float gAmplitudeRange[2] = { 0.0, 1.0 } ;

Scope scope;
void loop(void*)
{
	// number of frames to use for velocity computation.
	// Larger numFrames means smoother velocity, smaller numFrames means more responsive velocity.
	unsigned int numFrames = 4;
	std::vector<float[NUM_TOUCH]> locations(numFrames);
	std::vector<float[NUM_TOUCH]> sizes(numFrames);
	unsigned int currentFrame = 0;
	
	size_t sinceLastChange = 0;
	const size_t maxNoChange = 3; // after this many without a frame, scope.log a frame anyhow so the time on the scope keeps progressing
	while(!Bela_stopRequested())
	{
		// Read locations from Trill sensor
		touchSensor.readI2C();
		gNumActiveTouches = touchSensor.getNumTouches();
		for(unsigned int i = 0; i <  gNumActiveTouches; i++) {
			gTouchLocation[i] = touchSensor.touchLocation(i);
			gTouchSize[i] = touchSensor.touchSize(i);
		}
		// For all inactive touches, set location and size to 0
		for(unsigned int i = gNumActiveTouches; i < NUM_TOUCH; i++) {
			gTouchLocation[i] = 0.0;
			gTouchSize[i] = 0.0;
		}
		// see if this is a new reading
		bool changed = false;
		for(unsigned int n = 0; n < NUM_TOUCH; ++n)
		{
			if(sizes[currentFrame][n] != gTouchSize[n] || locations[currentFrame][n] != gTouchLocation[n])
			{
				changed = true;
				sinceLastChange = 0;
				break;
			}
		}
		if(!changed)
		{
			if(maxNoChange == sinceLastChange)
			{
				sinceLastChange = 0;
				changed = true;
			}
			sinceLastChange++;
		}
		if(changed)
		{
			currentFrame = (currentFrame + 1) % sizes.size();
			// store the current frame
			for(unsigned int n = 0; n < NUM_TOUCH; ++n)
			{
				// copy always all of them: we want as much information as possible to be able to detect a change
				sizes[currentFrame][n] = gTouchSize[n];
				locations[currentFrame][n] = gTouchLocation[n];
			}
			size_t oldestFrame = (currentFrame + 1) % sizes.size();
			
			// calculate velocity across frames
			for(unsigned int n = 0; n < NUM_TOUCH; ++n)
			{
				// this loop could be limited to n = 1 if you only want velocity from one touch

				// check if we are on an onset or offset for current touch
				bool onset = sizes[currentFrame][n] && !sizes[oldestFrame][n];
				bool offset = !sizes[currentFrame][n] && sizes[oldestFrame][n];
				if(onset || offset)
				{
					// remove artefacts due to onset/offset: set all frames to the current value
					// this way velocity goes to 0 for this frame.
					// For onset, successive frames will use this as a reference for computing velocity,
					// giving the physical equivalent of starting from velocity 0, which is reasonable
					for(unsigned int i = 0; i < sizes.size(); ++i)
					{
						sizes[i][n] = sizes[currentFrame][n];
						locations[i][n] = locations[currentFrame][n];
					}
				}
				gSizeVelocity[n] = sizes[currentFrame][n] - sizes[oldestFrame][n];
				gLocationVelocity[n] = locations[currentFrame][n] - locations[oldestFrame][n];
			}
			// printf("%6.3f %6.3f\n", gLocationVelocity[0], gSizeVelocity[0]);
		}
		usleep(gTaskSleepTime);
	}
}

bool setup(BelaContext *context, void *userData)
{
	scope.setup(4, context->audioSampleRate); // assuming approx 10ms sampling period
	// Setup a Trill Bar sensor on i2c bus 1, using the default mode and address
	if(touchSensor.setup(1, Trill::BAR, 0x20) != 0) {
		fprintf(stderr, "Unable to initialise Trill Bar\n");
		return false;
	}
	touchSensor.printDetails();
	// Set and schedule auxiliary task for reading sensor data from the I2C bus
	Bela_runAuxiliaryTask(loop);

	// For each possible touch...
	for(unsigned int i = 0; i < NUM_TOUCH; i++) {
		// Setup corresponding oscillator
		osc[i].setup(context->audioSampleRate, Oscillator::sine);
		// Setup low pass filters for smoothing frequency and amplitude
		freqFilt[i].setup(gCutOffFreq, context->audioSampleRate);
		ampFilt[i].setup(gCutOffAmp, context->audioSampleRate);
	}

	return true;
}

void render(BelaContext *context, void *userData)
{
	for(unsigned int n = 0; n < context->audioFrames; n++) {
		float out = 0.0;
		/* For each touch:
		*
		* 	- Map touch location to frequency of the oscillator
		* 	and smooth value changes using a single pole LP filter
		* 	- Map touch size toa amplitude of the oscillator and
		* 	smooth value changes using a single pole LP filter
		* 	- Compute oscillator value and add to output.
		* 	- The overall output will be scaled by the number of touches.
		*/
		static int count = 0;
		count++;
		float logs[4];
		for(unsigned int i = 0; i < NUM_TOUCH; i++) {
			float frequency, amplitude;
			frequency = 500 * (1 + i);
			amplitude = map(fabs(gLocationVelocity[i]), 0, 0.1, gAmplitudeRange[0], gAmplitudeRange[1]);
			amplitude = ampFilt[i].process(amplitude);
			float sine = tanhf_neon(amplitude * osc[i].process(frequency));
			if(i < 2)
			{
				logs[i * 2] = amplitude;
				logs[i * 2 + 1] = sine;
			}
			out += sine / 3.f;
		}
		scope.log(logs);

		// Write computed output to audio channels
		for(unsigned int channel = 0; channel < context->audioOutChannels; channel++) {
			audioWrite(context, n, channel, out);
		}
	}
}

void cleanup(BelaContext *context, void *userData)
{}