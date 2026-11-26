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
#include <lib/drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/solano.h>
#include <uORB/topics/solano_raw.h>
#include <uORB/topics/sensor_airflow.h>

class Solano : public device::I2C, public I2CSPIDriver<Solano>
{
public:
	static constexpr uint8_t DefaultAddress = 0x7a;
	static constexpr uint32_t BusFrequency = 400000;
	static constexpr unsigned ReadSize = 12;

	Solano(const I2CSPIDriverConfig &config);
	~Solano() override;

	static void print_usage();
	static int dump(BusInstanceIterator &iterator);
	int init() override;
	void RunImpl();
	void print_status() override;

private:
	int probe() override;

	uORB::PublicationMulti<solano_raw_s> _raw_pub{ORB_ID(solano_raw)};
	uORB::PublicationMulti<solano_s> _solano_pub{ORB_ID(solano)};
	uORB::Publication<sensor_airflow_s> _airflow_pub{ORB_ID(sensor_airflow)};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": read")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME ": communication errors")};
};
