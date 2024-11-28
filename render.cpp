#include <Bela.h>
#include <cmath>
#include <libraries/Trill/Trill.h>
#include <libraries/OnePole/OnePole.h>
#include <libraries/Oscillator/Oscillator.h>
#include <libraries/Scope/Scope.h>
#include <libraries/math_neon/math_neon.h>

#define NUM_TOUCH 5 // Number of touches on Trill sensor

// Trill object declaration
std::array<Trill,3> touchSensors;

std::vector<std::array<float,NUM_TOUCH>> gLocationVelocities(touchSensors.size());

// Sleep time for auxiliary task in microseconds
unsigned int gTaskSleepTime = 2000; // microseconds

std::vector<std::array<OnePole,NUM_TOUCH>> ampFilts(touchSensors.size());
float gCutOffAmp = 1;
// Oscillators objects declaration
std::vector<std::array<Oscillator,NUM_TOUCH>> oscs(touchSensors.size());

// Range for oscillator amplitude mapping
float gAmplitudeRange[2] = { 0.0, 1.0 } ;

Scope scope;
void loop(void*)
{
	// number of frames to use for velocity computation.
	// Larger numFrames means smoother velocity, smaller numFrames means more responsive velocity.
	enum { numFrames = 4};
	std::vector<std::array<std::array<float,NUM_TOUCH>,numFrames>> locationss(touchSensors.size());
	std::vector<std::array<std::array<float,NUM_TOUCH>,numFrames>> sizess(touchSensors.size());
	for(auto& a : locationss)
		for(auto& b : a)
			for(auto& c : b)
				c = 0;
	for(auto& a : sizess)
		for(auto& b : a)
			for(auto& c : b)
				c = 0;

	unsigned int currentFrame = 0;
	
	size_t sinceLastChange = 0;
	const size_t maxNoChange = 3; // after this many without a frame, scope.log a frame anyhow so the time on the scope keeps progressing
	while(!Bela_stopRequested())
	{
		for(size_t t = 0; t < touchSensors.size(); ++t)
		{
			auto& touchSensor = touchSensors[t];
			auto& locations = locationss[t];
			auto& sizes = sizess[t];
			auto& gLocationVelocity = gLocationVelocities[t];
			if(Trill::NONE == touchSensor.deviceType())
				continue;

			// Read locations from Trill sensor
			touchSensor.readI2C();
			// see if this is a new reading
			bool changed = false;
			for(unsigned int n = 0; n < NUM_TOUCH && !changed; ++n)
			{
				if(Trill::CENTROID == touchSensor.getMode())
				{
					if(sizes[currentFrame][n] != touchSensor.touchSize(n) || locations[currentFrame][n] != touchSensor.touchLocation(n))
					{
						changed = true;
						sinceLastChange = 0;
					}
				} else {
					if(locations[currentFrame][n] != touchSensor.rawData[n])
					{
						changed = true;
						sinceLastChange = 0;
					}
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
					if(Trill::CENTROID == touchSensor.getMode())
					{
						sizes[currentFrame][n] = touchSensor.touchSize(n);
						locations[currentFrame][n] = touchSensor.touchLocation(n);
					} else {
						sizes[currentFrame][n] = touchSensor.rawData[n];
						locations[currentFrame][n] = touchSensor.rawData[n];
					}
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
					//gSizeVelocity[n] = sizes[currentFrame][n] - sizes[oldestFrame][n];
					size_t howmany = Trill::CENTROID == touchSensor.getMode() ? touchSensor.getNumTouches() : std::min(touchSensor.rawData.size(), size_t(NUM_TOUCH));
					if(n < howmany)
					{
						float newL = locations[currentFrame][n];
						float oldL = locations[oldestFrame][n];
						if(Trill::RING == touchSensor.deviceType())
						{
							// wrap around
							if(newL > 0.9 && oldL < 0.1)
								oldL += 1;
							else if(newL < 0.1 && oldL > 0.9)
								newL += 1;
						}
						gLocationVelocity[n] = newL - oldL;
					}
					else
						gLocationVelocity[n] = 0;
				}
				// printf("%6.3f %6.3f\n", gLocationVelocity[0], gSizeVelocity[0]);
			}
		}
		usleep(gTaskSleepTime);
	}
}

bool setup(BelaContext *context, void *userData)
{
	scope.setup(4, context->audioSampleRate); // assuming approx 10ms sampling period
	// Setup a Trill Bar sensor on i2c bus 1, using the default mode and address
#if 0
	if(touchSensors[0].setup(1, Trill::CRAFT, 0x37) != 0)
		fprintf(stderr, "Unable to initialise Trill Craft\n");
	usleep(10000);
	touchSensors[0].setPrescaler(4);
	usleep(10000);
	touchSensors[0].updateBaseline();
#endif
	if(touchSensors[0].setup(1, Trill::BAR, 0x20) != 0)
		fprintf(stderr, "Unable to initialise Trill Bar\n");
	if(touchSensors[1].setup(1, Trill::RING, 0x3b) != 0)
		fprintf(stderr, "Unable to initialise Trill Ring\n");
	if(touchSensors[2].setup(1, Trill::SQUARE) != 0)
		fprintf(stderr, "Unable to initialise Trill Square\n");
	for(auto& touchSensor : touchSensors)
		touchSensor.printDetails();

	// For each possible touch...
	for(unsigned int t = 0; t < touchSensors.size(); ++t)
	{
		auto& osc = oscs[t];
		auto& ampFilt = ampFilts[t];
		for(auto& v : gLocationVelocities[t])
			v = 0;
		for(unsigned int i = 0; i < NUM_TOUCH; i++) {
			// Setup corresponding oscillator
			osc[i].setup(context->audioSampleRate, Oscillator::sine);
			// Setup low pass filters for smoothing frequency and amplitude
			ampFilt[i].setup(gCutOffAmp, context->audioSampleRate);
		}
	}

	// Set and schedule auxiliary task for reading sensor data from the I2C bus
	Bela_runAuxiliaryTask(loop);

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
		float logs[4] = { 0.f };
		int num = 0;
		for(unsigned int t = 0; t < touchSensors.size(); ++t)
		{
			auto& touchSensor = touchSensors[t];
			auto& gLocationVelocity = gLocationVelocities[t];
			auto& ampFilt = ampFilts[t];
			auto& osc = oscs[t];
			if(Trill::NONE == touchSensor.deviceType())
				continue;

			for(unsigned int i = 0; i < NUM_TOUCH; i++) {
				float frequency = (200 + (200 * (t + 1))) * (1 + i);
				float amplitude = map(fabs(gLocationVelocity[i]), 0, 0.2, gAmplitudeRange[0], gAmplitudeRange[1]);
				amplitude = ampFilt[i].process(amplitude);
				if(amplitude > 0.001)
				{
					num++;
					float sine = tanhf_neon(amplitude * osc[i].process(frequency));
					if(i < 2)
					{
						logs[i * 2] = amplitude;
						logs[i * 2 + 1] = sine;
					}
					out += sine / 3.f;
				}
			}
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
