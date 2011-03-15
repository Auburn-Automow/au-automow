#include "avr_ros/PowerControl.h"
using namespace avr_bridge;
ros::MsgSz PowerControl::serialize(uint8_t *in_data)
{
	ros::MsgSz offset = 0;
	union {
		float real;
		uint32_t base;
	} u_Voltage;
	u_Voltage.real = this->Voltage;
	*(in_data + offset + 0) = (u_Voltage.base >> (8 * 0)) & 0xFF;
	*(in_data + offset + 1) = (u_Voltage.base >> (8 * 1)) & 0xFF;
	*(in_data + offset + 2) = (u_Voltage.base >> (8 * 2)) & 0xFF;
	*(in_data + offset + 3) = (u_Voltage.base >> (8 * 3)) & 0xFF;
	offset += sizeof(this->Voltage);
	union {
		float real;
		uint32_t base;
	} u_Current;
	u_Current.real = this->Current;
	*(in_data + offset + 0) = (u_Current.base >> (8 * 0)) & 0xFF;
	*(in_data + offset + 1) = (u_Current.base >> (8 * 1)) & 0xFF;
	*(in_data + offset + 2) = (u_Current.base >> (8 * 2)) & 0xFF;
	*(in_data + offset + 3) = (u_Current.base >> (8 * 3)) & 0xFF;
	offset += sizeof(this->Current);
	union {
		float real;
		uint32_t base;
	} u_StateofCharge;
	u_StateofCharge.real = this->StateofCharge;
	*(in_data + offset + 0) = (u_StateofCharge.base >> (8 * 0)) & 0xFF;
	*(in_data + offset + 1) = (u_StateofCharge.base >> (8 * 1)) & 0xFF;
	*(in_data + offset + 2) = (u_StateofCharge.base >> (8 * 2)) & 0xFF;
	*(in_data + offset + 3) = (u_StateofCharge.base >> (8 * 3)) & 0xFF;
	offset += sizeof(this->StateofCharge);
	union {
		float real;
		uint32_t base;
	} u_Temperature1;
	u_Temperature1.real = this->Temperature1;
	*(in_data + offset + 0) = (u_Temperature1.base >> (8 * 0)) & 0xFF;
	*(in_data + offset + 1) = (u_Temperature1.base >> (8 * 1)) & 0xFF;
	*(in_data + offset + 2) = (u_Temperature1.base >> (8 * 2)) & 0xFF;
	*(in_data + offset + 3) = (u_Temperature1.base >> (8 * 3)) & 0xFF;
	offset += sizeof(this->Temperature1);
	union {
		float real;
		uint32_t base;
	} u_Temperature2;
	u_Temperature2.real = this->Temperature2;
	*(in_data + offset + 0) = (u_Temperature2.base >> (8 * 0)) & 0xFF;
	*(in_data + offset + 1) = (u_Temperature2.base >> (8 * 1)) & 0xFF;
	*(in_data + offset + 2) = (u_Temperature2.base >> (8 * 2)) & 0xFF;
	*(in_data + offset + 3) = (u_Temperature2.base >> (8 * 3)) & 0xFF;
	offset += sizeof(this->Temperature2);
	union {
		bool real;
		uint8_t base;
	} u_LeftCutterStatus;
	u_LeftCutterStatus.real = this->LeftCutterStatus;
	*(in_data + offset + 0) = (u_LeftCutterStatus.base >> (8 * 0)) & 0xFF;
	offset += sizeof(this->LeftCutterStatus);
	union {
		bool real;
		uint8_t base;
	} u_RightCutterStatus;
	u_RightCutterStatus.real = this->RightCutterStatus;
	*(in_data + offset + 0) = (u_RightCutterStatus.base >> (8 * 0)) & 0xFF;
	offset += sizeof(this->RightCutterStatus);
	return offset;
}
ros::MsgSz PowerControl::deserialize(uint8_t *out_data)
{
	ros::MsgSz offset = 0;
	union {
		float real;
		uint32_t base;
	} u_Voltage;
	u_Voltage.base = 0;
	u_Voltage.base |= ((typeof(u_Voltage.base)) (*(out_data + offset + 0))) << (8 * 0);
	u_Voltage.base |= ((typeof(u_Voltage.base)) (*(out_data + offset + 1))) << (8 * 1);
	u_Voltage.base |= ((typeof(u_Voltage.base)) (*(out_data + offset + 2))) << (8 * 2);
	u_Voltage.base |= ((typeof(u_Voltage.base)) (*(out_data + offset + 3))) << (8 * 3);
	this->Voltage = u_Voltage.real;
	offset += sizeof(this->Voltage);
	union {
		float real;
		uint32_t base;
	} u_Current;
	u_Current.base = 0;
	u_Current.base |= ((typeof(u_Current.base)) (*(out_data + offset + 0))) << (8 * 0);
	u_Current.base |= ((typeof(u_Current.base)) (*(out_data + offset + 1))) << (8 * 1);
	u_Current.base |= ((typeof(u_Current.base)) (*(out_data + offset + 2))) << (8 * 2);
	u_Current.base |= ((typeof(u_Current.base)) (*(out_data + offset + 3))) << (8 * 3);
	this->Current = u_Current.real;
	offset += sizeof(this->Current);
	union {
		float real;
		uint32_t base;
	} u_StateofCharge;
	u_StateofCharge.base = 0;
	u_StateofCharge.base |= ((typeof(u_StateofCharge.base)) (*(out_data + offset + 0))) << (8 * 0);
	u_StateofCharge.base |= ((typeof(u_StateofCharge.base)) (*(out_data + offset + 1))) << (8 * 1);
	u_StateofCharge.base |= ((typeof(u_StateofCharge.base)) (*(out_data + offset + 2))) << (8 * 2);
	u_StateofCharge.base |= ((typeof(u_StateofCharge.base)) (*(out_data + offset + 3))) << (8 * 3);
	this->StateofCharge = u_StateofCharge.real;
	offset += sizeof(this->StateofCharge);
	union {
		float real;
		uint32_t base;
	} u_Temperature1;
	u_Temperature1.base = 0;
	u_Temperature1.base |= ((typeof(u_Temperature1.base)) (*(out_data + offset + 0))) << (8 * 0);
	u_Temperature1.base |= ((typeof(u_Temperature1.base)) (*(out_data + offset + 1))) << (8 * 1);
	u_Temperature1.base |= ((typeof(u_Temperature1.base)) (*(out_data + offset + 2))) << (8 * 2);
	u_Temperature1.base |= ((typeof(u_Temperature1.base)) (*(out_data + offset + 3))) << (8 * 3);
	this->Temperature1 = u_Temperature1.real;
	offset += sizeof(this->Temperature1);
	union {
		float real;
		uint32_t base;
	} u_Temperature2;
	u_Temperature2.base = 0;
	u_Temperature2.base |= ((typeof(u_Temperature2.base)) (*(out_data + offset + 0))) << (8 * 0);
	u_Temperature2.base |= ((typeof(u_Temperature2.base)) (*(out_data + offset + 1))) << (8 * 1);
	u_Temperature2.base |= ((typeof(u_Temperature2.base)) (*(out_data + offset + 2))) << (8 * 2);
	u_Temperature2.base |= ((typeof(u_Temperature2.base)) (*(out_data + offset + 3))) << (8 * 3);
	this->Temperature2 = u_Temperature2.real;
	offset += sizeof(this->Temperature2);
	union {
		bool real;
		uint8_t base;
	} u_LeftCutterStatus;
	u_LeftCutterStatus.base = 0;
	u_LeftCutterStatus.base |= ((typeof(u_LeftCutterStatus.base)) (*(out_data + offset + 0))) << (8 * 0);
	this->LeftCutterStatus = u_LeftCutterStatus.real;
	offset += sizeof(this->LeftCutterStatus);
	union {
		bool real;
		uint8_t base;
	} u_RightCutterStatus;
	u_RightCutterStatus.base = 0;
	u_RightCutterStatus.base |= ((typeof(u_RightCutterStatus.base)) (*(out_data + offset + 0))) << (8 * 0);
	this->RightCutterStatus = u_RightCutterStatus.real;
	offset += sizeof(this->RightCutterStatus);
	return offset;
}
ros::MsgSz PowerControl::bytes()
{
	ros::MsgSz msgSize = 0;
	msgSize += sizeof(float);
	msgSize += sizeof(float);
	msgSize += sizeof(float);
	msgSize += sizeof(float);
	msgSize += sizeof(float);
	msgSize += sizeof(bool);
	msgSize += sizeof(bool);
	return msgSize;
}
