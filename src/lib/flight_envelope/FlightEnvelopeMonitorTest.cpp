/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <gtest/gtest.h>
#include <cmath>

#include "FlightEnvelopeMonitor.hpp"

using namespace flight_envelope;

static constexpr uint64_t T0 = 1000000000ULL;
static constexpr uint64_t STEP_US = 10000; // 100 Hz
static constexpr uint64_t DEBOUNCE_US = 100000; // 10 samples

class EnvelopeChannelTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		_upper.setLimits(3.0f, 3.8f);
		_upper.setHysteresis(0.2f);
		_lower.setLimits(-0.5f, -1.5f);
		_lower.setHysteresis(0.2f);
	}

	// feed the same value for n samples, return the number of state changes
	int feed(EnvelopeChannel &channel, float value, int n)
	{
		int changes = 0;

		for (int i = 0; i < n; i++) {
			_time += STEP_US;

			if (channel.update(value, _time)) {
				changes++;
			}
		}

		return changes;
	}

	EnvelopeChannel _upper{true};
	EnvelopeChannel _lower{false};
	uint64_t _time{T0};
};

TEST_F(EnvelopeChannelTest, initialState)
{
	EXPECT_EQ(_upper.state(), State::NOMINAL);
	EXPECT_FALSE(_upper.valid());
	EXPECT_EQ(_upper.timeInExceedance(), 0u);
	EXPECT_TRUE(_upper.enabled());
}

TEST_F(EnvelopeChannelTest, thresholdsUpperLimit)
{
	// no debounce: transitions latch on the first sample
	EXPECT_EQ(feed(_upper, 1.0f, 1), 0);
	EXPECT_EQ(_upper.state(), State::NOMINAL);
	EXPECT_TRUE(_upper.valid());

	// exactly at the limit is not an exceedance
	EXPECT_EQ(feed(_upper, 3.0f, 1), 0);
	EXPECT_EQ(_upper.state(), State::NOMINAL);

	EXPECT_EQ(feed(_upper, 3.1f, 1), 1);
	EXPECT_EQ(_upper.state(), State::WARNING);
	EXPECT_FLOAT_EQ(_upper.activeLimit(), 3.0f);

	EXPECT_EQ(feed(_upper, 3.9f, 1), 1);
	EXPECT_EQ(_upper.state(), State::CRITICAL);
	EXPECT_FLOAT_EQ(_upper.activeLimit(), 3.8f);

	// straight from nominal to critical
	EXPECT_EQ(feed(_upper, 1.0f, 1), 1);
	EXPECT_EQ(_upper.state(), State::NOMINAL);
	EXPECT_EQ(feed(_upper, 5.0f, 1), 1);
	EXPECT_EQ(_upper.state(), State::CRITICAL);
}

TEST_F(EnvelopeChannelTest, thresholdsLowerLimit)
{
	EXPECT_EQ(feed(_lower, 1.0f, 1), 0);
	EXPECT_EQ(_lower.state(), State::NOMINAL);

	EXPECT_EQ(feed(_lower, -0.6f, 1), 1);
	EXPECT_EQ(_lower.state(), State::WARNING);

	EXPECT_EQ(feed(_lower, -1.6f, 1), 1);
	EXPECT_EQ(_lower.state(), State::CRITICAL);

	EXPECT_EQ(feed(_lower, 1.0f, 1), 1);
	EXPECT_EQ(_lower.state(), State::NOMINAL);
}

TEST_F(EnvelopeChannelTest, hysteresisUpperLimit)
{
	feed(_upper, 3.1f, 1);
	ASSERT_EQ(_upper.state(), State::WARNING);

	// dropping just below the warning limit is inside the band: stays in warning
	EXPECT_EQ(feed(_upper, 2.9f, 5), 0);
	EXPECT_EQ(_upper.state(), State::WARNING);

	// at the lower edge of the band the channel recovers
	EXPECT_EQ(feed(_upper, 2.8f, 1), 1);
	EXPECT_EQ(_upper.state(), State::NOMINAL);

	// critical -> warning also needs to clear the band
	feed(_upper, 4.0f, 1);
	ASSERT_EQ(_upper.state(), State::CRITICAL);
	EXPECT_EQ(feed(_upper, 3.7f, 5), 0);
	EXPECT_EQ(_upper.state(), State::CRITICAL);
	EXPECT_EQ(feed(_upper, 3.5f, 1), 1);
	EXPECT_EQ(_upper.state(), State::WARNING);

	// re-entering critical uses the plain limit
	EXPECT_EQ(feed(_upper, 3.81f, 1), 1);
	EXPECT_EQ(_upper.state(), State::CRITICAL);
}

TEST_F(EnvelopeChannelTest, hysteresisLowerLimit)
{
	feed(_lower, -0.6f, 1);
	ASSERT_EQ(_lower.state(), State::WARNING);

	EXPECT_EQ(feed(_lower, -0.4f, 5), 0);
	EXPECT_EQ(_lower.state(), State::WARNING);

	EXPECT_EQ(feed(_lower, -0.3f, 1), 1);
	EXPECT_EQ(_lower.state(), State::NOMINAL);
}

TEST_F(EnvelopeChannelTest, debounceRejectsSingleSample)
{
	_upper.setDebounceTime(DEBOUNCE_US);

	feed(_upper, 1.0f, 5);

	// one noisy sample above critical does not change anything
	EXPECT_EQ(feed(_upper, 5.0f, 1), 0);
	EXPECT_EQ(_upper.state(), State::NOMINAL);
	EXPECT_EQ(feed(_upper, 1.0f, 5), 0);
	EXPECT_EQ(_upper.state(), State::NOMINAL);

	// a persistent exceedance latches once the debounce time has elapsed
	EXPECT_EQ(feed(_upper, 3.2f, 10), 0);
	EXPECT_EQ(_upper.state(), State::NOMINAL);
	EXPECT_EQ(feed(_upper, 3.2f, 1), 1);
	EXPECT_EQ(_upper.state(), State::WARNING);

	// recovery is debounced too
	EXPECT_EQ(feed(_upper, 1.0f, 10), 0);
	EXPECT_EQ(_upper.state(), State::WARNING);
	EXPECT_EQ(feed(_upper, 1.0f, 1), 1);
	EXPECT_EQ(_upper.state(), State::NOMINAL);
}

TEST_F(EnvelopeChannelTest, debounceRestartsWhenCandidateChanges)
{
	_upper.setDebounceTime(DEBOUNCE_US);

	feed(_upper, 1.0f, 1);

	// warning candidate for half of the debounce time, then critical: timer restarts
	feed(_upper, 3.2f, 5);
	EXPECT_EQ(feed(_upper, 4.0f, 6), 0);
	EXPECT_EQ(_upper.state(), State::NOMINAL);
	EXPECT_EQ(feed(_upper, 4.0f, 5), 1);
	EXPECT_EQ(_upper.state(), State::CRITICAL);
}

TEST_F(EnvelopeChannelTest, exceedanceTimeAndPeak)
{
	feed(_upper, 1.0f, 3);
	EXPECT_EQ(_upper.timeInExceedance(), 0u);

	feed(_upper, 3.2f, 1);
	ASSERT_EQ(_upper.state(), State::WARNING);
	EXPECT_EQ(_upper.timeInExceedance(), 0u);
	EXPECT_FLOAT_EQ(_upper.peakValue(), 3.2f);

	feed(_upper, 3.5f, 10);
	EXPECT_EQ(_upper.timeInExceedance(), 10 * STEP_US);
	EXPECT_FLOAT_EQ(_upper.peakValue(), 3.5f);

	// a lower sample keeps the peak
	feed(_upper, 3.1f, 10);
	EXPECT_EQ(_upper.timeInExceedance(), 20 * STEP_US);
	EXPECT_FLOAT_EQ(_upper.peakValue(), 3.5f);

	// escalation keeps accumulating
	feed(_upper, 4.2f, 10);
	EXPECT_EQ(_upper.state(), State::CRITICAL);
	EXPECT_EQ(_upper.timeInExceedance(), 30 * STEP_US);
	EXPECT_FLOAT_EQ(_upper.peakValue(), 4.2f);

	// recovery clears the current exceedance
	feed(_upper, 1.0f, 1);
	EXPECT_EQ(_upper.state(), State::NOMINAL);
	EXPECT_EQ(_upper.timeInExceedance(), 0u);
	EXPECT_FLOAT_EQ(_upper.peakValue(), 1.0f);
}

TEST_F(EnvelopeChannelTest, peakIncludesDebounceSamples)
{
	_upper.setDebounceTime(DEBOUNCE_US);

	feed(_upper, 1.0f, 1);
	feed(_upper, 4.5f, 3);   // spike while still debouncing
	feed(_upper, 3.3f, 20);
	ASSERT_EQ(_upper.state(), State::WARNING);
	EXPECT_FLOAT_EQ(_upper.peakValue(), 4.5f);
}

TEST_F(EnvelopeChannelTest, peakLowerLimit)
{
	feed(_lower, 1.0f, 1);
	feed(_lower, -0.8f, 2);
	feed(_lower, -1.2f, 2);
	feed(_lower, -0.9f, 2);
	EXPECT_EQ(_lower.state(), State::WARNING);
	EXPECT_FLOAT_EQ(_lower.peakValue(), -1.2f);
}

TEST_F(EnvelopeChannelTest, invalidSamplesHoldState)
{
	_upper.setDebounceTime(DEBOUNCE_US);

	feed(_upper, 3.3f, 11);
	ASSERT_EQ(_upper.state(), State::WARNING);
	const uint64_t time_before = _upper.timeInExceedance();

	// NaN holds the state, marks the channel invalid and does not count towards exceedance time
	EXPECT_EQ(feed(_upper, NAN, 20), 0);
	EXPECT_EQ(_upper.state(), State::WARNING);
	EXPECT_FALSE(_upper.valid());
	EXPECT_EQ(_upper.timeInExceedance(), time_before);

	// a NaN in the middle of a pending transition restarts the debounce timer
	feed(_upper, 1.0f, 8);
	feed(_upper, NAN, 1);
	EXPECT_EQ(feed(_upper, 1.0f, 10), 0);
	EXPECT_EQ(_upper.state(), State::WARNING);
	EXPECT_EQ(feed(_upper, 1.0f, 1), 1);
	EXPECT_EQ(_upper.state(), State::NOMINAL);
	EXPECT_TRUE(_upper.valid());

	EXPECT_EQ(feed(_upper, INFINITY, 5), 0);
	EXPECT_EQ(_upper.state(), State::NOMINAL);
	EXPECT_FALSE(_upper.valid());
}

TEST_F(EnvelopeChannelTest, repeatedSamplesDoNotCountTowardsDebounce)
{
	_upper.setDebounceTime(DEBOUNCE_US);
	feed(_upper, 1.0f, 2);

	// one fresh outlier re-evaluated as a stale value for longer than the debounce time does not latch
	_time += STEP_US;
	EXPECT_FALSE(_upper.update(3.3f, _time, true));

	for (int i = 0; i < 30; i++) {
		_time += STEP_US;
		EXPECT_FALSE(_upper.update(3.3f, _time, false));
	}

	EXPECT_EQ(_upper.state(), State::NOMINAL);
	EXPECT_TRUE(_upper.valid());

	// a second fresh outlier after the debounce time completes the transition
	_time += STEP_US;
	EXPECT_TRUE(_upper.update(3.3f, _time, true));
	EXPECT_EQ(_upper.state(), State::WARNING);

	// a fresh nominal sample cancels the pending exceedance even if the value is later repeated
	_time += STEP_US;
	EXPECT_FALSE(_upper.update(1.0f, _time, true));

	for (int i = 0; i < 30; i++) {
		_time += STEP_US;
		EXPECT_FALSE(_upper.update(1.0f, _time, false));
	}

	EXPECT_EQ(_upper.state(), State::WARNING);

	// stale samples keep the exceedance clock running while latched (1 fresh + 30 stale samples)
	EXPECT_EQ(_upper.timeInExceedance(), 31 * STEP_US);

	_time += STEP_US;
	EXPECT_TRUE(_upper.update(1.0f, _time, true));
	EXPECT_EQ(_upper.state(), State::NOMINAL);
}

TEST_F(EnvelopeChannelTest, invalidLimitsDisableChannel)
{
	EnvelopeChannel channel(true);
	EXPECT_FALSE(channel.enabled());
	EXPECT_FALSE(channel.update(100.f, T0));
	EXPECT_EQ(channel.state(), State::NOMINAL);

	channel.setLimits(NAN, 3.8f);
	EXPECT_FALSE(channel.enabled());

	// critical tighter than warning is inconsistent for an upper limit
	channel.setLimits(3.8f, 3.0f);
	EXPECT_FALSE(channel.enabled());

	channel.setLimits(3.0f, 3.8f);
	EXPECT_TRUE(channel.enabled());
	EXPECT_TRUE(channel.update(5.f, T0));
	EXPECT_EQ(channel.state(), State::CRITICAL);

	// tightening the limits to something invalid mid-exceedance resets the channel
	channel.setLimits(3.0f, NAN);
	EXPECT_EQ(channel.state(), State::NOMINAL);
	EXPECT_EQ(channel.timeInExceedance(), 0u);

	// equal warning and critical is allowed
	EnvelopeChannel lower(false);
	lower.setLimits(-1.f, -1.f);
	EXPECT_TRUE(lower.enabled());
	lower.setLimits(-1.5f, -0.5f);
	EXPECT_FALSE(lower.enabled());
}

TEST_F(EnvelopeChannelTest, disableResets)
{
	feed(_upper, 4.0f, 1);
	ASSERT_EQ(_upper.state(), State::CRITICAL);

	_upper.setEnabled(false);
	EXPECT_FALSE(_upper.enabled());
	EXPECT_EQ(_upper.state(), State::NOMINAL);
	EXPECT_EQ(feed(_upper, 4.0f, 5), 0);
	EXPECT_EQ(_upper.state(), State::NOMINAL);

	_upper.setEnabled(true);
	EXPECT_EQ(feed(_upper, 4.0f, 1), 1);
	EXPECT_EQ(_upper.state(), State::CRITICAL);
}

TEST_F(EnvelopeChannelTest, nonMonotonicTimeDoesNotAccumulate)
{
	_upper.update(3.5f, T0);
	ASSERT_EQ(_upper.state(), State::WARNING);
	_upper.update(3.5f, T0 + STEP_US);
	EXPECT_EQ(_upper.timeInExceedance(), STEP_US);

	// time going backwards contributes nothing
	_upper.update(3.5f, T0);
	EXPECT_EQ(_upper.timeInExceedance(), STEP_US);
}

class FlightEnvelopeMonitorTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		_monitor.channel(Channel::LOAD_FACTOR_HIGH).setLimits(3.0f, 3.8f);
		_monitor.channel(Channel::LOAD_FACTOR_LOW).setLimits(-0.5f, -1.5f);
		_monitor.channel(Channel::AIRSPEED).setLimits(35.f, 40.f);
		_monitor.channel(Channel::ALTITUDE_HIGH).setLimits(1000.f, 1100.f);
		_monitor.channel(Channel::ALTITUDE_LOW).setLimits(100.f, 50.f);
		_monitor.channel(Channel::SINK_RATE).setLimits(5.f, 8.f);
		// angle of attack left without limits: no source available

		_inputs.timestamp = T0;
		_inputs.load_factor = 1.f;
		_inputs.airspeed = 20.f;
		_inputs.altitude = 500.f;
		_inputs.vertical_speed = 0.f;
		_inputs.angle_of_attack = NAN;
	}

	uint8_t step(int n = 1)
	{
		uint8_t changed = 0;

		for (int i = 0; i < n; i++) {
			_inputs.timestamp += STEP_US;
			changed |= _monitor.update(_inputs);
		}

		return changed;
	}

	FlightEnvelopeMonitor _monitor;
	Inputs _inputs{};
};

TEST_F(FlightEnvelopeMonitorTest, nominalFlight)
{
	EXPECT_EQ(step(10), 0);
	EXPECT_EQ(_monitor.worstState(), State::NOMINAL);
	EXPECT_EQ(_monitor.exceedanceMask(), 0);
	EXPECT_EQ(_monitor.timeInWarning(), 0u);
	EXPECT_EQ(_monitor.timeInCritical(), 0u);
	EXPECT_FLOAT_EQ(_monitor.peakLoadFactorMax(), 1.f);
	EXPECT_FLOAT_EQ(_monitor.peakLoadFactorMin(), 1.f);
	EXPECT_FALSE(_monitor.channel(Channel::ANGLE_OF_ATTACK).enabled());
}

TEST_F(FlightEnvelopeMonitorTest, channelMapping)
{
	step();

	_inputs.load_factor = 3.5f;
	EXPECT_EQ(step(), channelBit(Channel::LOAD_FACTOR_HIGH));
	EXPECT_EQ(_monitor.channelState(Channel::LOAD_FACTOR_HIGH), State::WARNING);
	EXPECT_EQ(_monitor.channelState(Channel::LOAD_FACTOR_LOW), State::NOMINAL);
	EXPECT_EQ(_monitor.worstState(), State::WARNING);
	EXPECT_EQ(_monitor.exceedanceMask(), channelBit(Channel::LOAD_FACTOR_HIGH));

	_inputs.load_factor = -2.f;
	EXPECT_EQ(step(), channelBit(Channel::LOAD_FACTOR_HIGH) | channelBit(Channel::LOAD_FACTOR_LOW));
	EXPECT_EQ(_monitor.channelState(Channel::LOAD_FACTOR_HIGH), State::NOMINAL);
	EXPECT_EQ(_monitor.channelState(Channel::LOAD_FACTOR_LOW), State::CRITICAL);
	EXPECT_EQ(_monitor.worstState(), State::CRITICAL);
	EXPECT_EQ(_monitor.exceedanceMask(), channelBit(Channel::LOAD_FACTOR_LOW));

	_inputs.load_factor = 1.f;
	_inputs.airspeed = 36.f;
	EXPECT_EQ(step(), channelBit(Channel::LOAD_FACTOR_LOW) | channelBit(Channel::AIRSPEED));
	EXPECT_EQ(_monitor.channelState(Channel::AIRSPEED), State::WARNING);

	_inputs.airspeed = 20.f;
	_inputs.altitude = 1150.f;
	EXPECT_EQ(step(), channelBit(Channel::AIRSPEED) | channelBit(Channel::ALTITUDE_HIGH));
	EXPECT_EQ(_monitor.channelState(Channel::ALTITUDE_HIGH), State::CRITICAL);
	EXPECT_EQ(_monitor.channelState(Channel::ALTITUDE_LOW), State::NOMINAL);

	_inputs.altitude = 40.f;
	EXPECT_EQ(step(), channelBit(Channel::ALTITUDE_HIGH) | channelBit(Channel::ALTITUDE_LOW));
	EXPECT_EQ(_monitor.channelState(Channel::ALTITUDE_LOW), State::CRITICAL);

	// sink rate is positive down: descending at 6 m/s is a warning, climbing at 10 m/s is not
	_inputs.altitude = 500.f;
	_inputs.vertical_speed = -6.f;
	EXPECT_EQ(step(), channelBit(Channel::ALTITUDE_LOW) | channelBit(Channel::SINK_RATE));
	EXPECT_EQ(_monitor.channelState(Channel::SINK_RATE), State::WARNING);
	EXPECT_FLOAT_EQ(_monitor.channel(Channel::SINK_RATE).peakValue(), 6.f);

	_inputs.vertical_speed = 10.f;
	EXPECT_EQ(step(), channelBit(Channel::SINK_RATE));
	EXPECT_EQ(_monitor.channelState(Channel::SINK_RATE), State::NOMINAL);
	EXPECT_EQ(_monitor.exceedanceMask(), 0);
	EXPECT_EQ(_monitor.worstState(), State::NOMINAL);
}

TEST_F(FlightEnvelopeMonitorTest, cumulativeTimeAndPeakLoadFactor)
{
	step();

	_inputs.load_factor = 3.2f;
	step(1);   // transition sample
	step(10);  // 10 samples in warning
	EXPECT_EQ(_monitor.timeInWarning(), 10 * STEP_US);
	EXPECT_EQ(_monitor.timeInCritical(), 0u);

	_inputs.load_factor = 4.1f;
	step(1);
	step(5);
	EXPECT_EQ(_monitor.timeInWarning(), 11 * STEP_US);
	EXPECT_EQ(_monitor.timeInCritical(), 5 * STEP_US);

	_inputs.load_factor = 1.f;
	step(5);
	EXPECT_EQ(_monitor.timeInWarning(), 11 * STEP_US);
	EXPECT_EQ(_monitor.timeInCritical(), 6 * STEP_US);

	// a second warning excursion adds to the same counter
	_inputs.load_factor = -0.7f;
	step(1);
	step(4);
	EXPECT_EQ(_monitor.timeInWarning(), 15 * STEP_US);

	EXPECT_FLOAT_EQ(_monitor.peakLoadFactorMax(), 4.1f);
	EXPECT_FLOAT_EQ(_monitor.peakLoadFactorMin(), -0.7f);
}

TEST_F(FlightEnvelopeMonitorTest, invalidInputs)
{
	step();

	_inputs.airspeed = NAN;
	_inputs.altitude = NAN;
	_inputs.vertical_speed = NAN;
	_inputs.load_factor = NAN;
	EXPECT_EQ(step(5), 0);
	EXPECT_EQ(_monitor.worstState(), State::NOMINAL);
	EXPECT_FALSE(_monitor.channel(Channel::AIRSPEED).valid());
	EXPECT_FALSE(_monitor.channel(Channel::SINK_RATE).valid());
	EXPECT_FALSE(_monitor.channel(Channel::LOAD_FACTOR_HIGH).valid());
	EXPECT_FLOAT_EQ(_monitor.peakLoadFactorMax(), 1.f);

	// a valid channel still trips while the others are invalid
	_inputs.load_factor = 5.f;
	EXPECT_EQ(step(), channelBit(Channel::LOAD_FACTOR_HIGH));
	EXPECT_EQ(_monitor.worstState(), State::CRITICAL);
	EXPECT_FLOAT_EQ(_monitor.peakLoadFactorMax(), 5.f);
}

TEST_F(FlightEnvelopeMonitorTest, sensorOutageStopsCumulativeClock)
{
	step();

	_inputs.load_factor = 4.1f;
	step(1);
	step(5);
	EXPECT_EQ(_monitor.worstState(), State::CRITICAL);
	EXPECT_EQ(_monitor.timeInCritical(), 5 * STEP_US);

	// the source drops out: the channel holds CRITICAL but neither its own clock nor the aggregate one advances
	// (the aggregate clock still covers the last interval that ended with valid data)
	_inputs.load_factor = NAN;
	step(20);
	EXPECT_EQ(_monitor.worstState(), State::CRITICAL);
	EXPECT_EQ(_monitor.channel(Channel::LOAD_FACTOR_HIGH).timeInExceedance(), 5 * STEP_US);
	EXPECT_EQ(_monitor.timeInCritical(), 6 * STEP_US);

	// a second channel with valid data in the same state keeps the aggregate clock running
	_inputs.airspeed = 45.f;
	step(1);
	step(4);
	EXPECT_EQ(_monitor.timeInCritical(), 10 * STEP_US);

	_inputs.airspeed = 20.f;
	step(1);
	step(3);
	EXPECT_EQ(_monitor.worstState(), State::CRITICAL);
	EXPECT_EQ(_monitor.timeInCritical(), 11 * STEP_US);

	// the source comes back: the clock resumes from the first interval with valid data
	_inputs.load_factor = 4.1f;
	step(1);
	step(2);
	EXPECT_EQ(_monitor.timeInCritical(), 13 * STEP_US);
}

TEST_F(FlightEnvelopeMonitorTest, staleInputsDoNotLatch)
{
	_monitor.setDebounceTime(DEBOUNCE_US);
	step();

	// a single fresh airspeed outlier replayed at the load factor rate for longer than the debounce
	_inputs.airspeed = 36.f;
	step();
	_inputs.airspeed_fresh = false;
	EXPECT_EQ(step(30), 0);
	EXPECT_EQ(_monitor.channelState(Channel::AIRSPEED), State::NOMINAL);

	_inputs.airspeed_fresh = true;
	EXPECT_EQ(step(), channelBit(Channel::AIRSPEED));
	EXPECT_EQ(_monitor.channelState(Channel::AIRSPEED), State::WARNING);
}

TEST_F(FlightEnvelopeMonitorTest, clearExceedancesKeepsFlightStatistics)
{
	step();
	_inputs.load_factor = 4.5f;
	step(11);
	ASSERT_EQ(_monitor.worstState(), State::CRITICAL);

	_monitor.clearExceedances();
	EXPECT_EQ(_monitor.worstState(), State::NOMINAL);
	EXPECT_EQ(_monitor.exceedanceMask(), 0);
	EXPECT_EQ(_monitor.channelState(Channel::LOAD_FACTOR_HIGH), State::NOMINAL);
	EXPECT_EQ(_monitor.timeInCritical(), 10 * STEP_US);
	EXPECT_FLOAT_EQ(_monitor.peakLoadFactorMax(), 4.5f);

	_monitor.reset();
	EXPECT_EQ(_monitor.timeInCritical(), 0u);
	EXPECT_FLOAT_EQ(_monitor.peakLoadFactorMax(), 0.f);

	// limits survive a reset
	_inputs.load_factor = 4.5f;
	EXPECT_EQ(step(), channelBit(Channel::LOAD_FACTOR_HIGH));
}

TEST_F(FlightEnvelopeMonitorTest, sharedDebounce)
{
	_monitor.setDebounceTime(DEBOUNCE_US);
	step();

	_inputs.load_factor = 5.f;
	_inputs.airspeed = 50.f;
	EXPECT_EQ(step(10), 0);
	EXPECT_EQ(_monitor.worstState(), State::NOMINAL);
	EXPECT_EQ(step(), channelBit(Channel::LOAD_FACTOR_HIGH) | channelBit(Channel::AIRSPEED));
	EXPECT_EQ(_monitor.worstState(), State::CRITICAL);
}
