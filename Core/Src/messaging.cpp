/*
 * messaging.cpp
 *
 *  Created on: July 19, 2020
 *      Author: Karthik
 */

#include "messaging.hpp"

Messaging::Messaging() {
	sendMessage_ = nullptr;
	messageReaction_ = nullptr;
}

Messaging::Messaging(void (*sendMessage_fcn)(Message* msg_buf),
                     void (*messageReaction_fcn)(Message &msg)) :
										 sendMessage_(sendMessage_fcn),
										 messageReaction_(messageReaction_fcn) { 
	rxLastMessage_.msgType = NULLMSG;
	txLastMessage_.msgType = NULLMSG;
}


bool Messaging::rxMessageSequence(uint8_t* const serial_data) {
	if(!parseMessage(serial_data)) return false;
	if(!reactMessage()) return false;
	return true;
}

// will return if the msg is valid and store it in rxLastMessage_
bool Messaging::parseMessage(uint8_t* serial_data) {
	if (serial_data[0] != NU2PI && serial_data[0] != PI2NU) return false;
	Message* temp = (Message*)(serial_data);
	if(!checkMessageHash(temp)) return false;
	rxLastMessage_ = *temp;
	return true;
}

// react to lastMessage_
bool Messaging::reactMessage() {
	messageReaction_(rxLastMessage_);
	return true;
}

bool Messaging::checkMessageHash(Message* msg) {
	uint16_t computed_hash = DJBHash((msg->msgData), msg->msgLenBytes);
	return (computed_hash == msg->msgHash);
}

//  the struct yourself first
// needs overloaded protos for each msg
bool Messaging::generateMessage(Message* msg_buf, nu2pi &nu2pi_msg) {
	msg_buf->msgType = NU2PI;
	msg_buf->msgLenBytes = sizeof(nu2pi_msg);
	uint8_t* data = (uint8_t*)(&nu2pi_msg);
	for (int i = 0; i < msg_buf->msgLenBytes; ++i) {
		msg_buf->msgData[i] = data[i];
	}
	msg_buf->msgHash = DJBHash((msg_buf->msgData), msg_buf->msgLenBytes);
	return true;
}

// TODO: Code duplication!!
bool Messaging::generateMessage(Message* msg_buf, pi2nu &pi2nu_msg) {
	msg_buf->msgType = PI2NU;
	msg_buf->msgLenBytes = sizeof(pi2nu_msg);
	uint8_t* data = (uint8_t*)(&pi2nu_msg);
	for (int i = 0; i < msg_buf->msgLenBytes; ++i) {
		msg_buf->msgData[i] = data[i];
	}
	msg_buf->msgHash = DJBHash((msg_buf->msgData), msg_buf->msgLenBytes);
	return true;
}

bool Messaging::sendMessage(Message* msg_buf) {
	sendMessage_(msg_buf);
	return true;
}

//http://www.partow.net/programming/hashfunctions/index.html
uint16_t Messaging::DJBHash(const uint8_t* msg_data, uint8_t length) {
	uint16_t hash = 5381;
	uint16_t i    = 0;

	for (i = 0; i < length; ++msg_data, ++i) {
		hash = ((hash << 5) + hash) + (*msg_data);
	}

	return hash;
}
