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

/**
 * @file FlightEnvelopeMonitor.hpp
 *
 * Advisory flight-test envelope monitor.
 *
 * Compares load factor, airspeed, altitude, sink rate and (optionally)
 * angle of attack against configurable warning and critical limits. Each
 * channel has hysteresis and a debounce time so that a single noisy sample
 * does not change its state, and tracks how long it has been in exceedance
 * and the peak value seen during that exceedance.
 *
 * The library is platform independent: all times are absolute microseconds
 * supplied by the caller.
 */

#pragma once

#include <stdint.h>

namespace flight_envelope
{

enum class State : uint8_t {
	NOMINAL = 0,
	WARNING = 1,
	CRITICAL = 2,
};

enum class Channel : uint8_t {
	LOAD_FACTOR_HIGH = 0,	///< Nz above upper limit [g]
	LOAD_FACTOR_LOW = 1,	///< Nz below lower limit [g]
	AIRSPEED = 2,		///< calibrated airspeed above VNE [m/s]
	ALTITUDE_HIGH = 3,	///< altitude AMSL above ceiling [m]
	ALTITUDE_LOW = 4,	///< altitude AMSL below floor [m]
	SINK_RATE = 5,		///< sink rate (-vertical speed) above limit [m/s]
	ANGLE_OF_ATTACK = 6,	///< angle of attack above limit [deg]

	COUNT
};

static constexpr uint8_t CHANNEL_COUNT = static_cast<uint8_t>(Channel::COUNT);

static constexpr uint8_t channelBit(Channel id) { return static_cast<uint8_t>(1u << static_cast<uint8_t>(id)); }

struct Inputs {
	uint64_t timestamp{0};		///< [us] sample time
	float load_factor{0.f};		///< [g] normal load factor Nz, +1 in steady level flight
	float airspeed{0.f};		///< [m/s] calibrated airspeed, NaN if unavailable
	float altitude{0.f};		///< [m] altitude AMSL, NaN if unavailable
	float vertical_speed{0.f};	///< [m/s] positive up, NaN if unavailable
	float angle_of_attack{0.f};	///< [deg] NaN if no source is available
};

/**
 * Single monitored quantity with a warning and a critical limit.
 *
 * The channel is either an upper limit (exceeded when the value rises above
 * the limit) or a lower limit (exceeded when the value falls below it).
 */
class EnvelopeChannel
{
public:
	explicit EnvelopeChannel(bool upper_limit = true) : _sign(upper_limit ? 1.f : -1.f) {}

	/**
	 * Set warning and critical limits in engineering units. For an upper limit
	 * channel critical must be >= warning; for a lower limit channel critical
	 * must be <= warning. Non-finite or inconsistent limits disable the channel.
	 */
	void setLimits(float warning, float critical);

	/** Hysteresis band [engineering units, >= 0] that must be cleared before de-escalating. */
	void setHysteresis(float band);

	/** Time [us] a new state must persist before it is latched. */
	void setDebounceTime(uint64_t debounce_us) { _debounce_us = debounce_us; }

	void setEnabled(bool enabled);
	bool enabled() const { return _enabled && _limits_valid; }

	/**
	 * Feed a new sample.
	 * @param value sample in engineering units, NaN marks the sample invalid
	 * @param now_us sample time
	 * @return true if the latched state changed on this sample
	 */
	bool update(float value, uint64_t now_us);

	/** Clear state, timers and peak; limits are kept. */
	void reset();

	State state() const { return _state; }
	bool valid() const { return _valid; }
	uint64_t timeInExceedance() const { return _time_in_exceedance_us; }
	float peakValue() const { return _peak_value; }
	float warningLimit() const { return _warning; }
	float criticalLimit() const { return _critical; }
	float activeLimit() const;

private:
	State evaluate(float value) const;

	float _sign;			///< +1 for upper limit, -1 for lower limit
	float _warning{0.f};
	float _critical{0.f};
	float _hysteresis{0.f};
	uint64_t _debounce_us{0};
	bool _enabled{true};
	bool _limits_valid{false};

	State _state{State::NOMINAL};
	State _pending_state{State::NOMINAL};
	uint64_t _pending_since_us{0};
	uint64_t _last_update_us{0};
	uint64_t _time_in_exceedance_us{0};
	float _peak_value{0.f};
	bool _valid{false};
};

class FlightEnvelopeMonitor
{
public:
	FlightEnvelopeMonitor();

	EnvelopeChannel &channel(Channel id) { return _channels[static_cast<uint8_t>(id)]; }
	const EnvelopeChannel &channel(Channel id) const { return _channels[static_cast<uint8_t>(id)]; }

	/** Apply the same debounce time to every channel. */
	void setDebounceTime(uint64_t debounce_us);

	/**
	 * Feed one set of samples.
	 * @return bitmask of channels whose latched state changed on this update
	 */
	uint8_t update(const Inputs &inputs);

	/** Return all channels to NOMINAL and clear their exceedance timers; flight statistics are kept. */
	void clearExceedances();

	/** Reset everything including flight statistics (call at the start of a flight). */
	void reset();

	State worstState() const { return _worst_state; }
	State channelState(Channel id) const { return channel(id).state(); }

	/** Bitmask (channelBit) of channels currently in WARNING or CRITICAL. */
	uint8_t exceedanceMask() const { return _exceedance_mask; }

	float peakLoadFactorMax() const { return _load_factor_max; }
	float peakLoadFactorMin() const { return _load_factor_min; }
	uint64_t timeInWarning() const { return _time_in_warning_us; }
	uint64_t timeInCritical() const { return _time_in_critical_us; }

private:
	EnvelopeChannel _channels[CHANNEL_COUNT] {
		EnvelopeChannel(true),  // LOAD_FACTOR_HIGH
		EnvelopeChannel(false), // LOAD_FACTOR_LOW
		EnvelopeChannel(true),  // AIRSPEED
		EnvelopeChannel(true),  // ALTITUDE_HIGH
		EnvelopeChannel(false), // ALTITUDE_LOW
		EnvelopeChannel(true),  // SINK_RATE
		EnvelopeChannel(true),  // ANGLE_OF_ATTACK
	};

	State _worst_state{State::NOMINAL};
	uint8_t _exceedance_mask{0};
	uint64_t _last_update_us{0};
	uint64_t _time_in_warning_us{0};
	uint64_t _time_in_critical_us{0};
	float _load_factor_max{0.f};
	float _load_factor_min{0.f};
	bool _load_factor_peak_valid{false};
};

} // namespace flight_envelope
