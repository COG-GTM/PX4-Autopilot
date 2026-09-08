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

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/flight_envelope/FlightEnvelopeMonitor.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/flight_envelope_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

namespace flight_envelope_monitor
{

/**
 * Advisory flight-test envelope monitor.
 *
 * Feeds the pure flight_envelope library from uORB, publishes flight_envelope_status
 * and reports channel state transitions through the events interface.
 */
class FlightEnvelopeMonitorModule : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	FlightEnvelopeMonitorModule();
	~FlightEnvelopeMonitorModule() override;

	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	bool init();

private:
	static constexpr hrt_abstime BACKUP_SCHEDULE_INTERVAL{100000}; // 10 Hz fallback when no acceleration data arrives
	static constexpr hrt_abstime PUBLISH_INTERVAL{100000};         // status publication is bounded to 10 Hz
	static constexpr hrt_abstime INPUT_TIMEOUT{1000000};           // inputs older than this are treated as invalid
	static constexpr hrt_abstime EVENT_MIN_INTERVAL{1000000};      // per-channel event rate limit

	void Run() override;

	void updateParams() override;
	void applyChannelLimits(flight_envelope::Channel id, bool enabled, float warning, float critical, float hysteresis);

	void updateVehicleState(const hrt_abstime &now);
	void updateInputs(const hrt_abstime &now);
	void reportTransitions(const hrt_abstime &now);
	void publishStatus(const hrt_abstime &now);

	flight_envelope::FlightEnvelopeMonitor _monitor{};
	flight_envelope::Inputs _inputs{};

	// uORB inputs
	uORB::SubscriptionCallbackWorkItem _vehicle_acceleration_sub{this, ORB_ID(vehicle_acceleration)};
	uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)};
	uORB::Subscription _vehicle_air_data_sub{ORB_ID(vehicle_air_data)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1000000};

	uORB::Publication<flight_envelope_status_s> _flight_envelope_status_pub{ORB_ID(flight_envelope_status)};

	// latest input sample timestamps, used for staleness checks
	hrt_abstime _timestamp_sample{0};
	hrt_abstime _load_factor_time{0};
	hrt_abstime _airspeed_time{0};
	hrt_abstime _altitude_time{0};
	hrt_abstime _vertical_speed_time{0};

	hrt_abstime _last_publish{0};
	hrt_abstime _last_event_time[flight_envelope::CHANNEL_COUNT] {};
	flight_envelope::State _last_event_state[flight_envelope::CHANNEL_COUNT] {};

	bool _armed{false};
	bool _landed{true};
	bool _monitoring_active{false};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::FEM_CHAN_EN>) _param_fem_chan_en,
		(ParamInt<px4::params::FEM_ALT_SRC>) _param_fem_alt_src,
		(ParamFloat<px4::params::FEM_DEBOUNCE>) _param_fem_debounce,
		(ParamFloat<px4::params::FEM_NZ_MAX_WARN>) _param_fem_nz_max_warn,
		(ParamFloat<px4::params::FEM_NZ_MAX_CRIT>) _param_fem_nz_max_crit,
		(ParamFloat<px4::params::FEM_NZ_MIN_WARN>) _param_fem_nz_min_warn,
		(ParamFloat<px4::params::FEM_NZ_MIN_CRIT>) _param_fem_nz_min_crit,
		(ParamFloat<px4::params::FEM_NZ_HYST>) _param_fem_nz_hyst,
		(ParamFloat<px4::params::FEM_VNE_WARN>) _param_fem_vne_warn,
		(ParamFloat<px4::params::FEM_VNE_CRIT>) _param_fem_vne_crit,
		(ParamFloat<px4::params::FEM_VNE_HYST>) _param_fem_vne_hyst,
		(ParamFloat<px4::params::FEM_ALT_MAX_WARN>) _param_fem_alt_max_warn,
		(ParamFloat<px4::params::FEM_ALT_MAX_CRIT>) _param_fem_alt_max_crit,
		(ParamFloat<px4::params::FEM_ALT_MIN_WARN>) _param_fem_alt_min_warn,
		(ParamFloat<px4::params::FEM_ALT_MIN_CRIT>) _param_fem_alt_min_crit,
		(ParamFloat<px4::params::FEM_ALT_HYST>) _param_fem_alt_hyst,
		(ParamFloat<px4::params::FEM_SINK_WARN>) _param_fem_sink_warn,
		(ParamFloat<px4::params::FEM_SINK_CRIT>) _param_fem_sink_crit,
		(ParamFloat<px4::params::FEM_SINK_HYST>) _param_fem_sink_hyst
	)
};

} // namespace flight_envelope_monitor
