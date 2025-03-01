/*
 * Copyright (c) 2015, Thomas Keh
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
 
#include <iostream>
#include <chrono>
#include <future>
#include <string>
#include <cassert>

#include "logger.h"
#include "core.h"

namespace kaco {

//-------------------------------------------//
// Linkage to the CAN driver.                //
// Needs to be in lobal scope!               //
// TODO: Maybe encapsulate in a driver class //
//-------------------------------------------//

/// This struct contains C-strings for
/// busname and baudrate and is passed
/// to a CAN driver
struct CANBoard {

	/// Bus name
	const char * busname;

	/// Baudrate
	const char * baudrate;
	
};

#define MAX_CHANNEL_NUM 2
#define MAX_BUFFER_SIZE 100
    typedef struct{
        unsigned int nDeviceType;
        int nDeviceInd;
        int nRunning;
        int nMaxChannel;
        int nMaxWaitMs;
        int nChannel; // active channel used in send and recv
        int nChannelStatus[MAX_CHANNEL_NUM]; //0 : init value, 1 : ok, -1 : error
    } CAN_Dev;
#undef MAX_CHANNEL_NUM
#undef MAX_BUFFER_SIZE


/// This type is returned by the CAN driver
/// to identify the driver instance.
typedef void* CANHandle;

extern "C" uint8_t canReceive_driver(CANHandle, Message *);
extern "C" uint8_t canReceiveBatch_driver(CANHandle, Message *, int* );
extern "C" uint8_t canSend_driver(CANHandle, Message const *);
extern "C" uint8_t canSendBatch_driver(CANHandle, Message const *, int);
extern "C" CANHandle canOpen_driver(CANBoard *);
extern "C" int32_t canClose_driver(CANHandle);
extern "C" uint8_t canChangeBaudRate_driver(CANHandle, char *);

Core::Core()
	: nmt(*this),
		sdo(*this),
		pdo(*this)
	{ }
	
Core::~Core() {
	if (m_running) {
		stop();
	}
    close();
}

bool Core::start(const std::string& busname, const std::string& baudrate) {

    if(m_handle != 0){
        std::cerr << "opening device, but can diver is opened"<< std::endl;
        return false;
    }
	assert(!m_running);

	CANBoard board = {busname.c_str(), baudrate.c_str()};
	m_handle = canOpen_driver(&board);

	if(!m_handle) {
		ERROR("Cannot open the CANOpen device.");
		return false;
	}

	return true;

}

bool Core::start(const std::string& busname, const unsigned baudrate) {
	if (baudrate>=1000000 && baudrate%1000000==0) {
		return start(busname, std::to_string(baudrate/1000000)+"M");
	} else if (baudrate>=1000 && baudrate%1000==0) {
		return start(busname, std::to_string(baudrate/1000)+"K");
	} else {
		return start(busname, std::to_string(baudrate));
	}
}

void Core::stop() {
    m_running = false;
    if(m_loop_thread.joinable()){
        m_loop_thread.join();
    }
}

void Core::close() {
    if(m_handle == 0){
        std::cerr << "closing device, but can diver is not opened"<< std::endl;
        return;
    }
    DEBUG_LOG("Calling canClose.");
    canClose_driver(m_handle);
    m_handle = 0;
}

bool Core::start_loop(){
    if(m_handle == 0){
        std::cerr << "can diver is not opened"<< std::endl;
        return false;
    }
    m_running = true;
    m_loop_thread = std::thread(&Core::receive_loop, this, std::ref(m_running));

    return true;
}

void Core::receive_loop(std::atomic<bool>& running) {

    if(m_handle == 0){
        std::cerr << "can diver is not opened"<< std::endl;
        return;
    }
	Message message;

	while (running) {

		if(canReceive_driver(m_handle, &message) ==0){
            received_message(message);
        }

	}

}

void Core::register_receive_callback(const MessageReceivedCallback& callback) {
	std::lock_guard<std::mutex> scoped_lock(m_receive_callbacks_mutex);
	m_receive_callbacks.push_back(callback);
}
void Core::recv_message(std::vector<Message>& msg_buffer, int channel, int max_num){
    if(m_handle == 0){
        std::cerr << "can diver is not opened"<< std::endl;
        return;
    }
    CAN_Dev * can_dev =static_cast<CAN_Dev *>(m_handle) ;

    if(channel < 0 || channel >= can_dev->nMaxChannel){
        std::cerr << "recv message, but can diver channel is wrong " << channel  << ", nMaxChannel : " << can_dev->nMaxChannel << std::endl;
        return;
    }


    can_dev->nChannel = channel;
    canReceiveBatch_driver(m_handle,msg_buffer.data(),&max_num);
//    std::cout << "get " << max_num << "msg" << std::endl;
    for(int i = 0 ; i < max_num; i++ ){
        received_message(msg_buffer[i]);
    }
}

void Core::received_message(const Message& message) {

	DEBUG_LOG(" ");
	DEBUG_LOG("Received message:");

	// cleaning up old futures
//	if (m_cleanup_futures) {
//		std::lock_guard<std::mutex> scoped_lock(m_callback_futures_mutex);
//		m_callback_futures.remove_if([](const std::future<void>& f) {
//			// return true if callback has finished it's computation.
//			return (f.wait_for(std::chrono::steady_clock::duration::zero())==std::future_status::ready);
//		});
//	}

	// first call registered callbacks
	{
		std::lock_guard<std::mutex> scoped_lock(m_receive_callbacks_mutex);
		for (const MessageReceivedCallback& callback : m_receive_callbacks) {
			// The future returned by std::async has to be stored,
			// otherwise the immediately called future destructor
			// blocks until callback has finished.
//			std::lock_guard<std::mutex> scoped_lock(m_callback_futures_mutex);
//			m_callback_futures.push_front( std::async(std::launch::async, callback, message) );
            callback(message);
		}
	}

    if(message.ext){
        return;
    }
	// sencondly process known message types
	switch (message.get_function_code()) {
		
		case 0: {
			DEBUG_LOG("NMT Module Control");
			DEBUG(message.print();)
			break;
		}

		case 1: {
			DEBUG_LOG("Sync or Emergency");
            nmt.process_incoming_emcy_message(message);
			DEBUG(message.print();)
			break;
		}

		case 2: {
			DEBUG_LOG("Time stamp");
			DEBUG(message.print();)
			break;
		}

		case 3:
		case 5:
		case 7:
		case 9: {
			// TODO: This will be process_incoming_tpdo()
			pdo.process_incoming_message(message);
			break;
		}
		
		case 4:
		case 6:
		case 8:
		case 10: {
			// TODO: Implement this for slave functionality
			// 	-> delegate to pdo.process_incoming_rpdo()
            pdo.process_incoming_rpdo(message);
			DEBUG_LOG("PDO receive");
			DEBUG(message.print();)
			break;
		}
		
		case 11: {
			// TODO: This will be process_incoming_server_sdo()
			sdo.process_incoming_message(message);
			break;
		}
		
		case 12: {
			// TODO: Implement this for slave functionality
			// 	-> delegate to sdo.process_incoming_client_sdo()
			DEBUG_LOG("SDO (receive/client)");
			DEBUG(message.print();)
			break;
		}
		
		case 14: {
			// HeartBeat, aka NMT Error Control
			nmt.process_incoming_message(message);
			break;
		}

		default: {
			DEBUG_LOG("Unknown message:");
			DEBUG(message.print();)
			break;
		}

	}

	DEBUG_LOG(" ");

}

bool Core::send(const Message& message) {

    if(m_handle == 0){
        std::cerr << "can diver is not opened"<< std::endl;
        return false;
    }
	if (m_lock_send) {
		m_send_mutex.lock();
	}
	
	DEBUG_LOG_EXHAUSTIVE("Sending message:");
	DEBUG_EXHAUSTIVE(message.print();)
	uint8_t retval = canSend_driver(m_handle, &message);
	
	if (m_lock_send) {
		m_send_mutex.unlock();
	}
	
	if (retval != 0) {
		return false;
	}
	return true;
}

bool Core::send(const Message& message,int  channel) {
    return send(&message,1,channel);
}

bool Core::send(const std::vector<Message>& message,int channel){
    return send(message.data(),message.size(),channel);
}

bool Core::send(const Message * message,int len, int channel){
    if(m_handle == 0){
        std::cerr << "can diver is not opened"<< std::endl;
        return false;
    }
    CAN_Dev * can_dev =static_cast<CAN_Dev *>(m_handle) ;
    if(channel < 0 || channel >= can_dev->nMaxChannel){
        std::cerr << "sending message, but can diver channel is wrong " << channel  << ", nMaxChannel : " << can_dev->nMaxChannel << std::endl;
        return false;
    }

    if (m_lock_send) {
        m_send_mutex.lock();
    }
//    std::cerr << "sending message to channel " << channel  << " message:\n";
//    message->print();

    can_dev->nChannel = channel;
    DEBUG_LOG_EXHAUSTIVE("Sending message:");
    DEBUG_EXHAUSTIVE(message.print();)
//    uint8_t retval = canSend_driver(m_handle, &message);
    uint8_t retval = canSendBatch_driver(m_handle, message,len );
    if (m_lock_send) {
        m_send_mutex.unlock();
    }

    if (retval != 0) {
        return false;
    }
    return true;
}

} // namespace co
