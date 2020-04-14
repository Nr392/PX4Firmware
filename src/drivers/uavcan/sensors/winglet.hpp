
#pragma once

#include "sensor_bridge.hpp"

#include <drivers/drv_winglet.h>

#include <uavcan/protocol/debug/KeyValue.hpp>

class UavcanWingletBridge : public UavcanCDevSensorBridgeBase
{
public:
	static const char *const NAME;

	UavcanWingletBridge(uavcan::INode &node);

	const char *get_name() const override { return NAME; }

	int init() override;

private:

	int ioctl(struct file *filp, int cmd, unsigned long arg) override;

	int init_driver(uavcan_bridge::Channel *channel) override;

	void winglet_sub_cb(const uavcan::ReceivedDataStructure<uavcan::protocol::debug::KeyValue> &msg);

	typedef uavcan::MethodBinder < UavcanWingletBridge *,
		void (UavcanWingletBridge::*)
		(const uavcan::ReceivedDataStructure<uavcan::protocol::debug::KeyValue> &) >
		WingletCbBinder;

	uavcan::Subscriber<uavcan::protocol::debug::KeyValue, WingletCbBinder> _sub_winglet;


	winglet_calibration_s _scale{};
	sensor_winglet_s _report{};
};
