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

#include "FlightEnvelopeMonitor.hpp"

#include <px4_platform_common/defines.h>

namespace flight_envelope
{

void EnvelopeChannel::setLimits(float warning, float critical)
{
	_warning = warning;
	_critical = critical;

	// in normalised (upper limit) space the critical limit must not be tighter than the warning limit
	_limits_valid = PX4_ISFINITE(warning) && PX4_ISFINITE(critical) && (_sign * critical >= _sign * warning);

	if (!_limits_valid) {
		reset();
	}
}

void EnvelopeChannel::setHysteresis(float band)
{
	_hysteresis = (PX4_ISFINITE(band) && (band > 0.f)) ? band : 0.f;
}

void EnvelopeChannel::setEnabled(bool enabled)
{
	_enabled = enabled;

	if (!_enabled) {
		reset();
	}
}

void EnvelopeChannel::reset()
{
	_state = State::NOMINAL;
	_pending_state = State::NOMINAL;
	_pending_since_us = 0;
	_last_update_us = 0;
	_time_in_exceedance_us = 0;
	_peak_value = 0.f;
	_valid = false;
}

float EnvelopeChannel::activeLimit() const
{
	return (_state == State::CRITICAL) ? _critical : _warning;
}

State EnvelopeChannel::evaluate(float value) const
{
	// normalise so that every channel behaves as an upper limit
	const float x = _sign * value;
	const float warning = _sign * _warning;
	const float critical = _sign * _critical;

	switch (_state) {
	case State::NOMINAL:
		if (x > critical) { return State::CRITICAL; }

		if (x > warning) { return State::WARNING; }

		return State::NOMINAL;

	case State::WARNING:
		if (x > critical) { return State::CRITICAL; }

		if (x <= warning - _hysteresis) { return State::NOMINAL; }

		return State::WARNING;

	case State::CRITICAL:
		if (x <= warning - _hysteresis) { return State::NOMINAL; }

		if (x <= critical - _hysteresis) { return State::WARNING; }

		return State::CRITICAL;
	}

	return State::NOMINAL;
}

bool EnvelopeChannel::update(float value, uint64_t now_us)
{
	if (!enabled()) {
		_valid = false;
		return false;
	}

	uint64_t dt_us = 0;

	if ((_last_update_us != 0) && (now_us > _last_update_us)) {
		dt_us = now_us - _last_update_us;
	}

	_last_update_us = now_us;

	if (!PX4_ISFINITE(value)) {
		// hold the latched state, restart the debounce timer
		_valid = false;
		_pending_state = _state;
		return false;
	}

	_valid = true;

	if (_state != State::NOMINAL) {
		_time_in_exceedance_us += dt_us;
	}

	const State candidate = evaluate(value);

	if ((_state == State::NOMINAL) && (candidate == State::NOMINAL)) {
		// no exceedance in progress or building up
		_peak_value = value;
		_time_in_exceedance_us = 0;

	} else if (_sign * value > _sign * _peak_value) {
		_peak_value = value;
	}

	bool changed = false;

	if (candidate == _state) {
		_pending_state = _state;

	} else {
		if (candidate != _pending_state) {
			_pending_state = candidate;
			_pending_since_us = now_us;
		}

		if (now_us - _pending_since_us >= _debounce_us) {
			_state = candidate;
			changed = true;

			if (_state == State::NOMINAL) {
				_time_in_exceedance_us = 0;
				_peak_value = value;
			}
		}
	}

	return changed;
}

FlightEnvelopeMonitor::FlightEnvelopeMonitor()
{
	reset();
}

void FlightEnvelopeMonitor::setDebounceTime(uint64_t debounce_us)
{
	for (EnvelopeChannel &channel : _channels) {
		channel.setDebounceTime(debounce_us);
	}
}

void FlightEnvelopeMonitor::clearExceedances()
{
	for (EnvelopeChannel &channel : _channels) {
		channel.reset();
	}

	_worst_state = State::NOMINAL;
	_exceedance_mask = 0;
	_last_update_us = 0;
}

void FlightEnvelopeMonitor::reset()
{
	clearExceedances();
	_time_in_warning_us = 0;
	_time_in_critical_us = 0;
	_load_factor_max = 0.f;
	_load_factor_min = 0.f;
	_load_factor_peak_valid = false;
}

uint8_t FlightEnvelopeMonitor::update(const Inputs &inputs)
{
	const uint64_t now_us = inputs.timestamp;

	if ((_last_update_us != 0) && (now_us > _last_update_us)) {
		const uint64_t dt_us = now_us - _last_update_us;

		if (_worst_state == State::WARNING) {
			_time_in_warning_us += dt_us;

		} else if (_worst_state == State::CRITICAL) {
			_time_in_critical_us += dt_us;
		}
	}

	_last_update_us = now_us;

	// sink rate is positive down, the channel is an upper limit on it
	const float sink_rate = -inputs.vertical_speed;

	uint8_t changed = 0;

	if (channel(Channel::LOAD_FACTOR_HIGH).update(inputs.load_factor, now_us)) { changed |= channelBit(Channel::LOAD_FACTOR_HIGH); }

	if (channel(Channel::LOAD_FACTOR_LOW).update(inputs.load_factor, now_us)) { changed |= channelBit(Channel::LOAD_FACTOR_LOW); }

	if (channel(Channel::AIRSPEED).update(inputs.airspeed, now_us)) { changed |= channelBit(Channel::AIRSPEED); }

	if (channel(Channel::ALTITUDE_HIGH).update(inputs.altitude, now_us)) { changed |= channelBit(Channel::ALTITUDE_HIGH); }

	if (channel(Channel::ALTITUDE_LOW).update(inputs.altitude, now_us)) { changed |= channelBit(Channel::ALTITUDE_LOW); }

	if (channel(Channel::SINK_RATE).update(sink_rate, now_us)) { changed |= channelBit(Channel::SINK_RATE); }

	if (channel(Channel::ANGLE_OF_ATTACK).update(inputs.angle_of_attack, now_us)) { changed |= channelBit(Channel::ANGLE_OF_ATTACK); }

	if (PX4_ISFINITE(inputs.load_factor)) {
		if (!_load_factor_peak_valid) {
			_load_factor_max = inputs.load_factor;
			_load_factor_min = inputs.load_factor;
			_load_factor_peak_valid = true;

		} else {
			if (inputs.load_factor > _load_factor_max) { _load_factor_max = inputs.load_factor; }

			if (inputs.load_factor < _load_factor_min) { _load_factor_min = inputs.load_factor; }
		}
	}

	_worst_state = State::NOMINAL;
	_exceedance_mask = 0;

	for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
		const State state = _channels[i].state();

		if (state != State::NOMINAL) {
			_exceedance_mask |= channelBit(static_cast<Channel>(i));
		}

		if (static_cast<uint8_t>(state) > static_cast<uint8_t>(_worst_state)) {
			_worst_state = state;
		}
	}

	return changed;
}

} // namespace flight_envelope
