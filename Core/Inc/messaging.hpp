/*
 * messaging.hpp
 *
 *  Created on: July 19, 2020
 *      Author: Karthik
 */

#ifndef INC_MESSAGING_HPP_
#define INC_MESSAGING_HPP_

#include "nu2pi.h"
#include "pi2nu.h"

#include <stdint.h>
#include <stdlib.h>

class Messaging {
	public:
		// for message distinction
		enum messageTypeIdentifier : uint8_t {
			NULLMSG = 0x00,
			NU2PI = 0x01,
			PI2NU = 0x02
		};

		enum messagingError_t : uint8_t {
			//
		};

		typedef struct {
			messageTypeIdentifier msgType;
			uint8_t               msgLenBytes;
			uint8_t               msgData[60];
			uint16_t              msgHash;
		} Message;

		Messaging();

		Messaging(void (*sendMessage_fcn)(Message* msg_buf),
		          void (*messageReaction_fcn)(Message &msg));

		bool rxMessageSequence(uint8_t* serial_data);

		// will return if the msg is valid and store it in rxLastMessage_
		bool parseMessage(uint8_t* serial_data);

		// react to lastMessage_
		bool reactMessage();

		bool checkMessageHash(Message* msg);

		// to be used if you did make the struct yourself
		// needs overloaded protos for each msg
		bool generateMessage(Message* msg_buf, nu2pi &nu2pi_msg);
		bool generateMessage(Message* msg_buf, pi2nu &pi2nu_msg);

		bool sendMessage(Message* msg_buf);
	private:
		Message rxLastMessage_;
		Message txLastMessage_;
		// function pointer to send message with
		void (*sendMessage_)(Message* msg_buf);

		// function poiner to message reaction
		void (*messageReaction_)(Message &msg);
		// hash function for error checking
		uint16_t DJBHash(const uint8_t* msg_data, uint8_t length);

};

#endif /* INC_MESSAGING_HPP_ */
