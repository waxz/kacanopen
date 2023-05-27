//
// Created by waxz on 5/13/23.
//

#ifndef CMAKE_SUPER_BUILD_MQTTMESSAGEMANAGER_H
#define CMAKE_SUPER_BUILD_MQTTMESSAGEMANAGER_H

#include <map>
#include <vector>

#include "message/common_message.h"
#include "message/MessageManager.h"
#include "common/smart_pointer.h"
#include "common/functions.h"

#include "MqttClient.h"
#include <mutex>


namespace message{

    class MqttMessageManager: message::MessageManager{

    private:
        std::map<std::string, common::wild_ptr> channel_config;
        common::wild_ptr node_handler;

        std::string server_ip;
        int server_port = 0;
        int keep_alive = 0;

    public:

        long open(char *arg, int argc, const char **argv) override;
        long start(char *arg) override;
        long recv_message(const char *channel, size_t max_num, double timeout, const std::function<void (void *)> &callback) override;
        long send_message(const char *channel, void *data, size_t max_num, double timeout) override;

        bool is_running() override;

        void stop() override;





        template<typename T>
        long add_channel(const char *arg);

    };
}



#endif //CMAKE_SUPER_BUILD_MQTTMESSAGEMANAGER_H
