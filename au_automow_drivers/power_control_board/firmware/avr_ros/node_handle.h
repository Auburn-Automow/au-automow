/*
 * Ros.h
 *
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Adam Stambler
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Adam Stambler, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef NODE_HANDLE_H_
#define NODE_HANDLE_H_

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avr_ros/ros_types.h"
#include "avr_ros/ros_string.h"
#include "avr_ros/Msg.h"

#define __deprecated __attribute__((deprecated))

#ifndef UINT8_MAX
#define UINT8_MAX 0xff
#endif

namespace ros {

typedef void (RosCb)(Msg const *msg);

/* XXX: there are 3 ways to go about giving class Ros the ability to send
 * data over the wire:
 *  1) use hard coded function names which (for some inexplicable
 *     reason, use the stdio style even though it is uneeded)
 *  2) pass a (FILE *) to the Ros constructor.
 *  3) pass a class implimenting a send_packet method of some sort.
 *
 * Lets try to get #3 in here
 */
int fputc(char c, FILE *stream);
static FILE *ros_io = fdevopen(fputc, NULL);

enum PktType {
	PT_TOPIC = 0,
	PT_SERVICE = 1,
	PT_GETID = 0xff
};

struct PktHeader {
	uint8_t packet_type;
	uint8_t topic_tag;
	uint16_t msg_length;
};

typedef uint8_t Publisher;

template <size_t MSG_CT, size_t BUFFER_SZ>
struct InputCtx {
	InputCtx()
		: buffer_index(0)
	{}

	bool append(char c)
	{
		/* the last call to append completed the packet, start over */
		if (buffer_index == sizeof(this->header)
				+ this->header.msg_length) {
			this->reset();
		}

		if (buffer_index == (BUFFER_SZ - 1)) {
			this->reset();
		}

		this->buffer[this->buffer_index] = c;
		this->buffer_index++;

		bool header_completed = this->buffer_index ==
			sizeof(this->header);
		bool packet_completed = this->buffer_index ==
			this->header.msg_length + sizeof(this->header);
		if (header_completed) {
			/* is the packet type something we know about? */
			if ((this->header.packet_type != PT_TOPIC) &&
					(this->header.packet_type != PT_GETID)) {
				this->reset();
				return false;
			}

			/* does the topic_tag make sense? */
			if (this->header.topic_tag >= MSG_CT) {
				this->reset();
				return false;
			}

			/* does the msg_length make sense? */
			if (this->header.msg_length >= BUFFER_SZ) {
				this->reset();
				return false;
			}

			return false;
		} else if (packet_completed) {
			return true;
		}
		return false;
	}

	void reset(void) {
		this->buffer_index = 0;
	}

	/* buffer incomming chars. */
	union {
		uint8_t buffer[BUFFER_SZ];
		/* convenient access to the buffer */
		PktHeader header;
	};

	uint8_t buffer_index;
};


template <size_t MSG_CT, size_t BUFFER_SZ>
class NodeHandle {
public:
	NodeHandle(char const *node_name)
		: name(node_name)
	{
		this->io = ros_io;
	}

	NodeHandle(char const *node_name, FILE *_io)
		: io(_io)
		, name(node_name)
	{}

	//Get the publisher for a topic
	//You cannot advertise a topic that was not in the configuration
	//file
	Publisher advertise(char const *topic)
	{
		return getTopicTag(topic);
	}

	void publish(Publisher pub, Msg *msg)
	{
		MsgSz bytes = msg->serialize(this->outBuffer);
		this->send_pkt(PT_TOPIC, pub, outBuffer, bytes);
	}

	void subscribe(char const *topic, RosCb *funct, Msg *msg)
	{
		uint8_t tag = getTopicTag(topic);
		this->cb_list[tag] = funct;
		this->msg_list[tag] = msg;
	}

	void spin(char c)
	{
		if (this->in_ctx.append(c)) {
			this->process_pkt();
		}
	}

private:
	FILE *io;

	string name;

	RosCb *cb_list[MSG_CT];
	Msg *msg_list[MSG_CT];
	uint8_t outBuffer[BUFFER_SZ];

	void send_id()
	{
		MsgSz size = this->name.serialize(this->outBuffer);
		this->send_pkt(PT_GETID, 0, outBuffer, size);
	}

	void process_pkt()
	{
		switch(this->in_ctx.header.packet_type) {
		case PT_GETID:
			this->send_id();
			break;
		case PT_TOPIC:
			this->msg_list[this->in_ctx.header.topic_tag]->
				deserialize(this->in_ctx.buffer +
						sizeof(PktHeader));
			this->cb_list[this->in_ctx.header.topic_tag](this->
					msg_list[this->in_ctx.header.topic_tag]);
			break;
		case PT_SERVICE:
			break;
		}
	}

	/* XXX: use an enum for pkt_type and topic to prevent swapping? */
	void send_pkt(enum PktType pkt_type, uint8_t topic,
			uint8_t const *data, MsgSz data_len)
	{
		PktHeader head = {
			pkt_type,
			topic,
			data_len
		};

		fwrite(&head, sizeof(head), 1, this->io);

		fwrite(data, data_len, 1, this->io);
	}

	/* given the character string of a topic, determines the numeric tag to
	 * place in a packet */
	/* char getTopicTag(char const *topic); */
#include "ros_get_topic_tag.h"

	InputCtx <MSG_CT, BUFFER_SZ> in_ctx;
};

} /* namespace ros */

#endif /* NODE_HANDLE_H_ */
