#ifndef _DRV_WINGLET_H
#define _DRV_WINGLET_H

#include <stdint.h>
#include <sys/ioctl.h>

#include "drv_sensor.h"
#include "drv_orb_dev.h"

#define WINGLET_BASE_DEVICE_PATH	"/dev/winglet"
#define WINGLET0_DEVICE_PATH		"/dev/winglet0"
#define WINGLET1_DEVICE_PATH		"/dev/winglet1"
#define WINGLET2_DEVICE_PATH		"/dev/winglet2"

#include <uORB/topics/sensor_winglet.h>


struct winglet_calibration_s {
	float	W_AXIS;
	float	X_AXIS;
	float	Y_AXIS;
	float	Z_AXIS;
};

/*
 * ioctl() definitions
 */

#define _WINGLETIOCBASE		(0x2800)
#define _WINGLETIOC(_n)		(_PX4_IOC(_WINGLETIOCBASE, _n))

/** determine if WINGLET is external or onboard */
#define WINGLETIOCGEXTERNAL	_WINGLETIOC(11)

#endif /* _DRV_WINGLET_H */
