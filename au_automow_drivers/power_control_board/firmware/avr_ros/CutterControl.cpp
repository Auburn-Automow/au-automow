#include "avr_ros/CutterControl.h"
using namespace avr_bridge;
ros::MsgSz CutterControl::serialize(uint8_t *in_data)
{
	ros::MsgSz offset = 0;
	union {
		bool real;
		uint8_t base;
	} u_LeftControl;
	u_LeftControl.real = this->LeftControl;
	*(in_data + offset + 0) = (u_LeftControl.base >> (8 * 0)) & 0xFF;
	offset += sizeof(this->LeftControl);
	union {
		bool real;
		uint8_t base;
	} u_RightControl;
	u_RightControl.real = this->RightControl;
	*(in_data + offset + 0) = (u_RightControl.base >> (8 * 0)) & 0xFF;
	offset += sizeof(this->RightControl);
	return offset;
}
ros::MsgSz CutterControl::deserialize(uint8_t *out_data)
{
	ros::MsgSz offset = 0;
	union {
		bool real;
		uint8_t base;
	} u_LeftControl;
	u_LeftControl.base = 0;
	u_LeftControl.base |= ((typeof(u_LeftControl.base)) (*(out_data + offset + 0))) << (8 * 0);
	this->LeftControl = u_LeftControl.real;
	offset += sizeof(this->LeftControl);
	union {
		bool real;
		uint8_t base;
	} u_RightControl;
	u_RightControl.base = 0;
	u_RightControl.base |= ((typeof(u_RightControl.base)) (*(out_data + offset + 0))) << (8 * 0);
	this->RightControl = u_RightControl.real;
	offset += sizeof(this->RightControl);
	return offset;
}
ros::MsgSz CutterControl::bytes()
{
	ros::MsgSz msgSize = 0;
	msgSize += sizeof(bool);
	msgSize += sizeof(bool);
	return msgSize;
}
