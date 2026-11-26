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

#include "Solano.hpp"
#include <px4_platform_common/px4_work_queue/WorkItemSingleShot.hpp>

using namespace time_literals;

static float decodeWord(const uint8_t *bytes)
{
	int32_t value = (static_cast<int32_t>(bytes[0]) << 8) | bytes[1];

	if (value >= 0x8000) {
		value -= 0x10000;
	}

	return value * 0.01f;
}

Solano::Solano(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config)
{
}

Solano::~Solano()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int Solano::probe()
{
	_retries = 1;
	uint8_t data[ReadSize] {};
	return transfer(nullptr, 0, data, sizeof(data));
}

int Solano::init()
{
	const int ret = I2C::init();

	if (ret == PX4_OK) {
		ScheduleNow();
	}

	return ret;
}

void Solano::RunImpl()
{
	perf_begin(_sample_perf);
	solano_raw_s report{};
	static_assert(sizeof(report.data) == ReadSize, "raw topic must match I2C read size");
	report.timestamp_sample = hrt_absolute_time();

	if (transfer(nullptr, 0, report.data, sizeof(report.data)) == PX4_OK) {
		report.device_id = get_device_id();
		report.error_count = perf_event_count(_comms_errors);
		report.timestamp = hrt_absolute_time();
		_raw_pub.publish(report);

		solano_s decoded{};
		decoded.timestamp = report.timestamp;
		decoded.timestamp_sample = report.timestamp_sample;
		decoded.device_id = report.device_id;
		decoded.windspeed = decodeWord(&report.data[0]);
		decoded.windspeed_f = decodeWord(&report.data[2]);
		decoded.angle_ns = decodeWord(&report.data[4]);
		decoded.angle_ns_f = decodeWord(&report.data[6]);
		decoded.angle_ew = decodeWord(&report.data[8]);
		decoded.angle_ew_f = decodeWord(&report.data[10]);
		_solano_pub.publish(decoded);

	} else {
		perf_count(_comms_errors);
	}

	perf_end(_sample_perf);
	ScheduleDelayed(10_ms);
}

void Solano::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

int Solano::dump(BusInstanceIterator &iterator)
{
	bool found = false;
	int result = PX4_OK;

	while (iterator.next()) {
		if (!iterator.instance()) {
			continue;
		}

		found = true;
		auto *driver = static_cast<Solano *>(static_cast<I2CSPIDriverBase *>(iterator.instance()));
		struct Read {
			Solano *driver;
			uint8_t data[ReadSize];
			int result;
		} read{driver, {}, PX4_ERROR};

		px4::WorkItemSingleShot runner(*driver, [](void *argument) {
			auto &request = *static_cast<Read *>(argument);
			request.result = request.driver->transfer(nullptr, 0, request.data, sizeof(request.data));

			if (request.result != PX4_OK) {
				perf_count(request.driver->_comms_errors);
			}
		}, &read);
		runner.ScheduleNow();
		runner.wait();
		driver->I2CSPIDriverBase::print_status();

		if (read.result != PX4_OK) {
			PX4_ERR("read failed: %d", read.result);
			result = PX4_ERROR;
			continue;
		}

		PX4_INFO("raw: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
			 read.data[0], read.data[1], read.data[2], read.data[3], read.data[4], read.data[5],
			 read.data[6], read.data[7], read.data[8], read.data[9], read.data[10], read.data[11]);
	}

	if (!found) {
		PX4_ERR("not running on the selected bus/address; start the driver first");
		return PX4_ERROR;
	}

	return result;
}
