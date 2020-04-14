/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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


#include "PX4Winglet.hpp"

#include <lib/drivers/device/Device.hpp>

PX4Winglet::PX4Winglet(uint32_t device_id, uint8_t priority, enum Rotation rotation) :
	CDev(nullptr),
	_sensor_winglet_pub{ORB_ID(sensor_winglet), priority},
	_rotation{rotation}
{
	_class_device_instance = register_class_devname(WINGLET_BASE_DEVICE_PATH);

	_sensor_winglet_pub.get().device_id = device_id;
	_sensor_winglet_pub.get().scaling = 1.0f;
}

PX4Winglet::~PX4Winglet()
{
	if (_class_device_instance != -1) {
		unregister_class_devname(WINGLET_BASE_DEVICE_PATH, _class_device_instance);
	}
}

int PX4Winglet::ioctl(cdev::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case WINGLETIOCSSCALE: {
			// Copy offsets and scale factors in
			winglet_calibration_s cal{};
			memcpy(&cal, (winglet_calibration_s *) arg, sizeof(cal));

			_calibration_orientation = matrix::Vector4f{cal.W_AXIS, cal.X_AXIS, cal.Y_AXIS, cal.Z_AXIS};

		}

		return PX4_OK;

	case WINGLETIOCGSCALE: {
			// copy out scale factors
			mag_calibration_s cal{};
			cal.W_AXIS = _calibration_orientation(0);
			cal.W_AXIS = _calibration_orientation(1);
			cal.W_AXIS = _calibration_orientation(2);
			cal.W_AXIS = _calibration_orientation(3);
			memcpy((winglet_calibration_s *)arg, &cal, sizeof(cal));
		}

		return 0;

	case DEVIOCGDEVICEID:
		return _sensor_winglet_pub.get().device_id;

	default:
		return -ENOTTY;
	}
}

void PX4Winglet::set_device_type(uint8_t devtype)
{
	// current DeviceStructure
	union device::Device::DeviceId device_id;
	device_id.devid = _sensor_winglet_pub.get().device_id;

	// update to new device type
	device_id.devid_s.devtype = devtype;

	// copy back to report
	_sensor_winglet_pub.get().device_id = device_id.devid;
}

void PX4Winglet::update(hrt_abstime timestamp_sample, uint8_t quatString[])
{
	sensor_winglet_s &report = _sensor_winglet_pub.get();
	report.timestamp = timestamp_sample;
	
	const matrix::Quatf quat{w, x, y, z};

	for(int i = 0; i < 3; i++){
		int j = 0;
		std::string convert = "";
		while(quatString[j] != ","){
			convert += std::to_string(quatString[j]); 
			
		}
		quat(i) = stof(convert);
	}
	int j = 0;
	while(quatString[j] != "\n"){
		convert += std::to_string(quatString[j]); 
		
	}
	quat(i) = stof(convert);

	// Apply rotation (before scaling)
	rotate_4f(_rotation, w, x, y, z);

	// Apply range scale and the calibrating offset/scale
	const matrix::Quatf val_calibrated{(quat.emult(_sensitivity) * report.scaling)};

	report.w = val_calibrated(0);
	report.x = val_calibrated(1);
	report.y = val_calibrated(2);
	report.z = val_calibrated(3);
	

	_sensor_winglet_pub.update();
}

void PX4Winglet::print_status()
{
	PX4_INFO(WINGLET_BASE_DEVICE_PATH " device instance: %d", _class_device_instance);

	PX4_INFO("calibration scale: %.5f %.5f %.5f %.5f", (double)_calibration_scale(0), (double)_calibration_scale(1),
		 (double)_calibration_scale(2), (double)_calibration_scale(3));
	PX4_INFO("calibration offset: %.5f %.5f %.5f %.5f", (double)_calibration_offset(0), (double)_calibration_offset(1),
		 (double)_calibration_offset(2), (double)_calibration_offset(3));
}
