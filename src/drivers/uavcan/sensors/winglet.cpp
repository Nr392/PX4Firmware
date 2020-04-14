
#include "winglet.hpp"

#include <drivers/drv_hrt.h>
#include <systemlib/err.h>

#include <lib/drivers/winglet/PX4Winglet.hpp>

const char *const UavcanWingletBridge::NAME = "winglet";

#define UAVCAN_WINGLET_BASE_DEVICE_PATH "/dev/uavcan/winglet"

UavcanWingletBridge::UavcanWingletBridge(uavcan::INode &node) :
	UavcanCDevSensorBridgeBase("uavcan_winglet", UAVCAN_WINGLET_BASE_DEVICE_PATH, WINGLET_BASE_DEVICE_PATH, ORB_ID(sensor_winglet)),
	_sub_winglet(node)
{

}

int
UavcanWingletBridge::init()
{
	int res = device::CDev::init();

	if (res < 0) {
		return res;
	}

	res = _sub_winglet.start(WingletCbBinder(this, &UavcanWingletBridge::winglet_sub_cb));

	if (res < 0) {
		PX4_ERR("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;

}

int
UavcanWingletBridge::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case WINGLETIOCGEXTERNAL: {
			return 1;           // declare it external rise it's priority and to allow for correct orientation compensation
		}
	default: {
			return CDev::ioctl(filp, cmd, arg);
		}
	}
}

void
UavcanWingletBridge::winglet_sub_cb(const uavcan::ReceivedDataStructure<uavcan::protocol::debug::KeyValue>
				     &msg)
{
	uavcan_bridge::Channel *channel = get_channel_for_node(msg.getSrcNodeID().get());

	if(channel == nullptr) {
		return;
	}
	
	PX4Winglet *winglet = (PX4Winglet *)channel->h_driver;

	if(winglet == nullptr){
		return;
	}	

	uint * key = new uint[1];
	
	key[0] = msg.key[0];

	float value = msg.value;

	winglet->update(hrt_absolute_time(), key, value);
}

int UavcanWingletBridge::init_driver(uavcan_bridge::Channel *channel)
{
	// update device id as we now know our device node_id
	DeviceId device_id{_device_id};

	device_id.devid_s.devtype = DRV_WINGLET_DEVTYPE_UAVCAN;
	device_id.devid_s.address = static_cast<uint8_t>(channel->node_id);

	channel->h_driver = new PX4Winglet(device_id.devid, ORB_PRIO_HIGH, ROTATION_NONE);

	if (channel->h_driver == nullptr) {
		return PX4_ERROR;
	}

	PX4Winglet *winglet = (PX4Winglet *)channel->h_driver;

	channel->class_instance = winglet->get_class_instance();

	if (channel->class_instance < 0) {
		PX4_ERR("UavcanWinglet: Unable to get a class instance");
		delete winglet;
		channel->h_driver = nullptr;
		return PX4_ERROR;
	}

	winglet->set_external(true);

	return PX4_OK;
}

