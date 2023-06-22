//
// Created by waxz on 5/11/23.
//

#ifndef CMAKE_SUPER_BUILD_ROSMESSAGEMANAGER_H
#define CMAKE_SUPER_BUILD_ROSMESSAGEMANAGER_H
#include <map>
#include <vector>

#include "message/common_message.h"
#include "message/MessageManager.h"
#include "common/smart_pointer.h"
#include "common/functions.h"


namespace message{
    class RosMessageManager:message::MessageManager{

    private:
        std::map<std::string, common::wild_ptr> channel_config;
        common::wild_ptr node_handler;

    public:
        long open(char*arg, int argc,const char** argv) override;
        long start(char *arg) override;
        long send_message(const char *channel, void *data, size_t max_num, double timeout) override;
        long recv_message(const char *channel, size_t max_num, double timeout, const std::function<void (void *)> &callback) override;

        bool is_running() override;
        void stop() override;


        template<typename T>
        long create_sub(char* topic, int queue_size);


        template<typename T>
        long add_channel( const char *arg);

        long recv_tf(common_message::TransformStamped & data, float timeout );
        long send_tf(const common_message::TransformStamped & data);

    };

}


#endif //CMAKE_SUPER_BUILD_ROSMESSAGEMANAGER_H
