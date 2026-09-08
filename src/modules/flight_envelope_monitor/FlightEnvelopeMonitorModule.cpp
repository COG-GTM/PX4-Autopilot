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

#include "FlightEnvelopeMonitorModule.hpp"

#include <lib/geo/geo.h>
#include <px4_platform_common/events.h>
#include <px4_platform_common/log.h>

using namespace time_literals;
using flight_envelope::Channel;
using flight_envelope::State;
using flight_envelope::channelBit;

namespace flight_envelope_monitor
{

ModuleBase::Descriptor FlightEnvelopeMonitorModule::desc{task_spawn, custom_command, print_usage};

static constexpr uint8_t toMsgState(State state)
{
	switch (state) {
	case State::WARNING: return flight_envelope_status_s::STATE_WARNING;

	case State::CRITICAL: return flight_envelope_status_s::STATE_CRITICAL;

	default: return flight_envelope_status_s::STATE_NOMINAL;
	}
}

FlightEnvelopeMonitorModule::FlightEnvelopeMonitorModule() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	_inputs.load_factor = NAN;
	_inputs.airspeed = NAN;
	_inputs.altitude = NAN;
	_inputs.vertical_speed = NAN;
	_inputs.angle_of_attack = NAN;
}

FlightEnvelopeMonitorModule::~FlightEnvelopeMonitorModule()
{
	perf_free(_cycle_perf);
}

bool FlightEnvelopeMonitorModule::init()
{
	if (!_vehicle_acceleration_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	updateParams();
	_flight_envelope_status_pub.advertise();

	ScheduleDelayed(BACKUP_SCHEDULE_INTERVAL);

	return true;
}

int FlightEnvelopeMonitorModule::task_spawn(int argc, char *argv[])
{
	FlightEnvelopeMonitorModule *instance = new FlightEnvelopeMonitorModule();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	desc.object.store(instance);
	desc.task_id = task_id_is_work_queue;

	if (instance->init()) {
		return PX4_OK;
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

void FlightEnvelopeMonitorModule::applyChannelLimits(Channel id, bool enabled, float warning, float critical,
		float hysteresis)
{
	flight_envelope::EnvelopeChannel &channel = _monitor.channel(id);
	channel.setLimits(warning, critical);
	channel.setHysteresis(hysteresis);
	channel.setEnabled(enabled);
}

void FlightEnvelopeMonitorModule::updateParams()
{
	ModuleParams::updateParams();

	const int32_t enabled_mask = _param_fem_chan_en.get();
	const auto channel_enabled = [enabled_mask](Channel id) {
		return (enabled_mask & channelBit(id)) != 0;
	};

	applyChannelLimits(Channel::LOAD_FACTOR_HIGH, channel_enabled(Channel::LOAD_FACTOR_HIGH),
			   _param_fem_nz_max_warn.get(), _param_fem_nz_max_crit.get(), _param_fem_nz_hyst.get());
	applyChannelLimits(Channel::LOAD_FACTOR_LOW, channel_enabled(Channel::LOAD_FACTOR_LOW),
			   _param_fem_nz_min_warn.get(), _param_fem_nz_min_crit.get(), _param_fem_nz_hyst.get());
	applyChannelLimits(Channel::AIRSPEED, channel_enabled(Channel::AIRSPEED),
			   _param_fem_vne_warn.get(), _param_fem_vne_crit.get(), _param_fem_vne_hyst.get());
	applyChannelLimits(Channel::ALTITUDE_HIGH, channel_enabled(Channel::ALTITUDE_HIGH),
			   _param_fem_alt_max_warn.get(), _param_fem_alt_max_crit.get(), _param_fem_alt_hyst.get());
	applyChannelLimits(Channel::ALTITUDE_LOW, channel_enabled(Channel::ALTITUDE_LOW),
			   _param_fem_alt_min_warn.get(), _param_fem_alt_min_crit.get(), _param_fem_alt_hyst.get());
	applyChannelLimits(Channel::SINK_RATE, channel_enabled(Channel::SINK_RATE),
			   _param_fem_sink_warn.get(), _param_fem_sink_crit.get(), _param_fem_sink_hyst.get());

	// no angle of attack source is wired up; the channel stays disabled until one is available
	_monitor.channel(Channel::ANGLE_OF_ATTACK).setEnabled(false);

	const float debounce_s = fmaxf(_param_fem_debounce.get(), 0.f);
	_monitor.setDebounceTime(static_cast<uint64_t>(debounce_s * 1e6f));

	for (uint8_t i = 0; i < flight_envelope::CHANNEL_COUNT; i++) {
		const Channel id = static_cast<Channel>(i);

		if (!_monitor.channel(id).enabled() && (enabled_mask & channelBit(id)) && id != Channel::ANGLE_OF_ATTACK) {
			PX4_WARN("channel %u disabled: inconsistent or non-finite limits", i);
		}
	}
}

void FlightEnvelopeMonitorModule::updateVehicleState(const hrt_abstime &now)
{
	vehicle_status_s vehicle_status;

	if (_vehicle_status_sub.update(&vehicle_status)) {
		_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	}

	vehicle_land_detected_s land_detected;

	if (_vehicle_land_detected_sub.update(&land_detected)) {
		_landed = land_detected.landed;
	}

	const bool active = _armed && !_landed;

	if (active != _monitoring_active) {
		_monitoring_active = active;

		if (active) {
			// new flight: start statistics from scratch
			_monitor.reset();

		} else {
			// keep the flight statistics for the post-flight log, drop any active exceedance
			_monitor.clearExceedances();
		}

		for (uint8_t i = 0; i < flight_envelope::CHANNEL_COUNT; i++) {
			_last_event_state[i] = State::NOMINAL;
			_last_event_time[i] = 0;
		}

		publishStatus(now);
		_last_publish = now;
	}
}

void FlightEnvelopeMonitorModule::updateInputs(const hrt_abstime &now)
{
	vehicle_acceleration_s acceleration;

	if (_vehicle_acceleration_sub.update(&acceleration)) {
		// specific force along body Z (FRD) is -1 g in steady level flight
		_inputs.load_factor = -acceleration.xyz[2] / CONSTANTS_ONE_G;
		_load_factor_time = acceleration.timestamp_sample;
		_timestamp_sample = acceleration.timestamp_sample;
	}

	airspeed_validated_s airspeed;

	if (_airspeed_validated_sub.update(&airspeed)) {
		_inputs.airspeed = airspeed.calibrated_airspeed_m_s;
		_airspeed_time = airspeed.timestamp;
	}

	if (_param_fem_alt_src.get() == 1) {
		vehicle_air_data_s air_data;

		if (_vehicle_air_data_sub.update(&air_data)) {
			_inputs.altitude = air_data.baro_alt_meter;
			_altitude_time = air_data.timestamp_sample;
		}

		vehicle_local_position_s local_position;

		if (_vehicle_local_position_sub.update(&local_position)) {
			_inputs.vertical_speed = local_position.v_z_valid ? -local_position.vz : NAN;
			_vertical_speed_time = local_position.timestamp_sample;
		}

	} else {
		vehicle_local_position_s local_position;

		if (_vehicle_local_position_sub.update(&local_position)) {
			_inputs.altitude = (local_position.z_valid && local_position.z_global)
					   ? local_position.ref_alt - local_position.z : NAN;
			_inputs.vertical_speed = local_position.v_z_valid ? -local_position.vz : NAN;
			_altitude_time = local_position.timestamp_sample;
			_vertical_speed_time = local_position.timestamp_sample;
		}
	}

	if (_load_factor_time == 0 || now - _load_factor_time > INPUT_TIMEOUT) {
		_inputs.load_factor = NAN;
	}

	if (_airspeed_time == 0 || now - _airspeed_time > INPUT_TIMEOUT) {
		_inputs.airspeed = NAN;
	}

	if (_altitude_time == 0 || now - _altitude_time > INPUT_TIMEOUT) {
		_inputs.altitude = NAN;
	}

	if (_vertical_speed_time == 0 || now - _vertical_speed_time > INPUT_TIMEOUT) {
		_inputs.vertical_speed = NAN;
	}

	_inputs.timestamp = now;
}

void FlightEnvelopeMonitorModule::reportTransitions(const hrt_abstime &now)
{
	using events::px4::enums::flight_envelope_channel_t;
	using events::px4::enums::flight_envelope_state_t;

	for (uint8_t i = 0; i < flight_envelope::CHANNEL_COUNT; i++) {
		const flight_envelope::EnvelopeChannel &channel = _monitor.channel(static_cast<Channel>(i));
		const State state = channel.state();

		if (state == _last_event_state[i]) {
			continue;
		}

		// rate limit per channel, but always report an escalation over the last announced state;
		// a suppressed de-escalation is announced once the interval has elapsed
		const bool escalation = state > _last_event_state[i];
		const bool interval_elapsed = (_last_event_time[i] == 0) || (now - _last_event_time[i] >= EVENT_MIN_INTERVAL);

		if (!escalation && !interval_elapsed) {
			continue;
		}

		events::LogLevels log_levels{events::Log::Info, events::LogInternal::Info};

		if (state == State::WARNING) {
			log_levels = {events::Log::Warning, events::LogInternal::Warning};

		} else if (state == State::CRITICAL) {
			log_levels = {events::Log::Critical, events::LogInternal::Warning};
		}

		// peak of the current exceedance, or the current value once recovered
		const float value = channel.peakValue();
		const float limit = (state == State::NOMINAL) ? channel.warningLimit() : channel.activeLimit();

		/* EVENT
		 * @description Advisory flight-test envelope monitor: a monitored channel changed state.
		 */
		events::send<flight_envelope_channel_t, flight_envelope_state_t, float, float>(
			events::ID("flight_envelope_channel_state"), log_levels,
			"Envelope {1}: {2} (value {3:.2}, limit {4:.2})",
			static_cast<flight_envelope_channel_t>(i), static_cast<flight_envelope_state_t>(toMsgState(state)),
			value, limit);

		_last_event_state[i] = state;
		_last_event_time[i] = now;
	}
}

void FlightEnvelopeMonitorModule::publishStatus(const hrt_abstime &now)
{
	flight_envelope_status_s status{};

	status.timestamp_sample = _timestamp_sample;
	status.monitoring_active = _monitoring_active;

	status.load_factor = _inputs.load_factor;
	status.airspeed = _inputs.airspeed;
	status.altitude = _inputs.altitude;
	status.vertical_speed = _inputs.vertical_speed;
	status.angle_of_attack = _inputs.angle_of_attack;

	static_assert(flight_envelope_status_s::CHANNEL_COUNT == flight_envelope::CHANNEL_COUNT,
		      "flight_envelope_status channel count mismatch");

	for (uint8_t i = 0; i < flight_envelope::CHANNEL_COUNT; i++) {
		status.channel_state[i] = toMsgState(_monitor.channelState(static_cast<Channel>(i)));
	}

	status.exceedance_mask = _monitor.exceedanceMask();
	status.worst_state = toMsgState(_monitor.worstState());

	status.peak_load_factor_max = _monitor.peakLoadFactorMax();
	status.peak_load_factor_min = _monitor.peakLoadFactorMin();
	status.time_in_warning_s = static_cast<float>(_monitor.timeInWarning()) * 1e-6f;
	status.time_in_critical_s = static_cast<float>(_monitor.timeInCritical()) * 1e-6f;

	status.timestamp = hrt_absolute_time();
	_flight_envelope_status_pub.publish(status);
}

void FlightEnvelopeMonitorModule::Run()
{
	if (should_exit()) {
		_vehicle_acceleration_sub.unregisterCallback();
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	// backup schedule in case the acceleration topic stops updating
	ScheduleDelayed(BACKUP_SCHEDULE_INTERVAL);

	perf_begin(_cycle_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	const hrt_abstime now = hrt_absolute_time();

	updateVehicleState(now);
	updateInputs(now);

	if (_monitoring_active) {
		const uint8_t changed = _monitor.update(_inputs);

		reportTransitions(now);

		if (changed != 0) {
			publishStatus(now);
			_last_publish = now;
		}
	}

	if (now - _last_publish >= PUBLISH_INTERVAL) {
		publishStatus(now);
		_last_publish = now;
	}

	perf_end(_cycle_perf);
}

int FlightEnvelopeMonitorModule::print_status()
{
	static const char *state_names[] = {"NOMINAL", "WARNING", "CRITICAL"};
	static const char *channel_names[] = {"Nz high", "Nz low", "airspeed", "alt high", "alt low", "sink rate", "AoA"};

	PX4_INFO("monitoring %s (armed: %s, landed: %s)", _monitoring_active ? "active" : "inactive",
		 _armed ? "yes" : "no", _landed ? "yes" : "no");
	PX4_INFO("Nz: %.2f g, airspeed: %.1f m/s, altitude: %.1f m, vertical speed: %.1f m/s",
		 (double)_inputs.load_factor, (double)_inputs.airspeed, (double)_inputs.altitude, (double)_inputs.vertical_speed);
	PX4_INFO("worst state: %s, exceedance mask: 0x%02x", state_names[toMsgState(_monitor.worstState())],
		 _monitor.exceedanceMask());
	PX4_INFO("peak Nz: %.2f / %.2f g, time in warning: %.1f s, critical: %.1f s",
		 (double)_monitor.peakLoadFactorMax(), (double)_monitor.peakLoadFactorMin(),
		 (double)_monitor.timeInWarning() * 1e-6, (double)_monitor.timeInCritical() * 1e-6);

	for (uint8_t i = 0; i < flight_envelope::CHANNEL_COUNT; i++) {
		const flight_envelope::EnvelopeChannel &channel = _monitor.channel(static_cast<Channel>(i));

		if (!channel.enabled()) {
			PX4_INFO("  %-9s disabled", channel_names[i]);
			continue;
		}

		PX4_INFO("  %-9s %-8s warn %.2f crit %.2f peak %.2f (%.1f s)", channel_names[i],
			 state_names[toMsgState(channel.state())], (double)channel.warningLimit(), (double)channel.criticalLimit(),
			 (double)channel.peakValue(), (double)channel.timeInExceedance() * 1e-6);
	}

	perf_print_counter(_cycle_perf);

	return 0;
}

int FlightEnvelopeMonitorModule::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FlightEnvelopeMonitorModule::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Advisory flight-test envelope monitor. It compares the current normal load factor (Nz),
calibrated airspeed, altitude AMSL and sink rate against configurable warning and critical
limits (`FEM_*` parameters) while the vehicle is armed and airborne.

Each channel has a hysteresis band and a debounce time so that a single noisy sample does not
trip a flag. Per-channel state, an exceedance bitmask, the overall worst state, the peak load
factor of the flight and the cumulative time spent in warning and critical are published in
`flight_envelope_status` (logged by default) and every state transition is reported as a
rate-limited event so that it is visible on the ground station.

This module is an advisory aid for the test team only. It does not take any action on the vehicle
and does not replace the flight-test safety process.

### Implementation
The module runs on the low priority work queue and is driven by `vehicle_acceleration`
callbacks with a periodic fallback. Nz is derived from the body Z (FRD) specific force, airspeed
from `airspeed_validated`, altitude and vertical speed from `vehicle_local_position` or
`vehicle_air_data` (`FEM_ALT_SRC`).
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("flight_envelope_monitor", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

} // namespace flight_envelope_monitor

extern "C" __EXPORT int flight_envelope_monitor_main(int argc, char *argv[])
{
	return ModuleBase::main(flight_envelope_monitor::FlightEnvelopeMonitorModule::desc, argc, argv);
}
