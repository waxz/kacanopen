//
// Created by waxz on 5/6/23.
//

#ifndef CMAKE_SUPER_BUILD_MESSAGEMANAGER_H
#define CMAKE_SUPER_BUILD_MESSAGEMANAGER_H

#include <functional>

namespace message{


    class MessageManager{

    private:

    public:
        /// \param args
        /// \param argc
        /// \param argv
        /// \return : status: can be status or pointer address

        virtual long open(char*arg, int argc, const char** argv)= 0;


        virtual long start(char*arg)= 0;

        ///
        /// \param arg : define channel configuration in char array, contains channel type: CHANNEL, PUB/SUB, TOPIC , INDEX, SERVER,  CACHE_SIZE
        /// \return : status: can be status or pointer address
        /// \details: all data store in map<string, wild_ptr<>>, use topic string as key, contains device handler and data buffer queue, and other data
//    virtual long add_channel(char* args) = 0;

        virtual long recv_message(const char* channel, size_t max_num, double timeout, const std::function<void(void*)>& callback) = 0;

        virtual long send_message(const char* channel, void* data, size_t max_num, double timeout) = 0;

        virtual bool is_running() = 0;

        virtual void stop() = 0;

//    virtual long read_param(char* channel, void* data) = 0;

//    virtual long write_param(char* channel, void* data) = 0;

//    virtual long reset(char*) = 0;

//    virtual long close() = 0;


    };
}


#endif //CMAKE_SUPER_BUILD_MESSAGEMANAGER_H
