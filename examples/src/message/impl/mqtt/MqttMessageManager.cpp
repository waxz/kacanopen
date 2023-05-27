//
// Created by waxz on 5/13/23.
//

#include "MqttMessageManager.h"
#include "common/string_func.h"

#include "MqttMessageConvert.h"

namespace message{

    struct MQTT_SUB_CONTAINER{

        common::wild_ptr buffer;
        std::function<int(const char*, const char*)> on_message;
        std::function<int(int, double, const std::function<void(void*)>)> func;
        std::mutex buffer_mtx;
    };

    struct MQTT_PUB_CONTAINER{
        std::string topic;
        int qos;
        std::function<int(void*, int, double)> func;
    };

    long MqttMessageManager::open(char *arg, int argc, const char **argv) {

        node_handler.set<message::MqttClient>(arg, true);
        auto& client = node_handler.ref<message::MqttClient>(); // = std::make_shared<message::MqttClient>(arg, true);
        mosqpp::lib_init();

        int type = 0;
        auto my_handler = common::fnptr<int(int,char*)>([&](int n, char* ch){

            std::cout << "get n = " <<  n << ", ch = " << ch << std::endl;

            if(n==0 && std::strcmp(ch , "server") == 0 ){
                type = 1;
            }
            if(n==0 && std::strcmp(ch , "port") == 0 ){
                type = 2;
            }
            if(n==0 && std::strcmp(ch , "keep_alive") == 0 ){
                type = 3;
            }

            if(n == 1 && type == 1){
                server_ip.assign(ch);
                type = 0;
            }

            if(n == 1 && type == 2){
                if(str2int(&server_port, ch, 10) == STR2INT_SUCCESS){
                    return 0;
                }else{
                    return -1;
                }
                type = 0;
            }
            if(n == 1 && type == 3){
                if(str2int(&keep_alive, ch, 10) == STR2INT_SUCCESS){
                    return 0;
                }else{
                    return -1;
                }
                type = 0;
            }


            return 0;
        });
        for(int i = 0 ; i < argc;i++){
            type = 0;

            split_str(argv[i],":", my_handler);

        }


        std::cout << "get mqtt server " << server_ip << ", " << server_port << ", " << keep_alive << std::endl;
#if 1
        client.message_callback_func = [&](const char* topic, const char* playload){

//            std::cout  << "message_callback_func : " << topic << ", " << playload << std::endl;

            char key_buffer[100];
            sprintf(key_buffer,"%s:%s","SUB",topic);
//            std::cout <<__FILE__ << ":" << __LINE__ << key_buffer<<std::endl;
            auto it = channel_config.find(key_buffer);
            if(it == channel_config.end()){
                std::cout   << key_buffer << " is not exist" << std::endl;
                return 0;
            }

            it->second.ref<MQTT_SUB_CONTAINER>().on_message(topic, playload);
            return 0;

            return 1;
        };
#endif


        dynamic_assert(!server_ip.empty());
        dynamic_assert(server_port >0);
        dynamic_assert(keep_alive >0);
        return 0;

    }


    template<typename T>
    long MqttMessageManager::add_channel(const char *arg) {

        auto& client = node_handler.ref<message::MqttClient>(); // = std::make_shared<message::MqttClient>(arg, true);

        char * channel_str[] = {"SUB","PUB"};

        struct ChannelInfo{
            int type = 0;// 1: SUB, 2: PUB, 3: TF Listen , 4 TF Broadcast, 5 Param Reader, 6 Param Write

            std::string topic;
            int qos = 1;
        };

        ChannelInfo channelInfo ;
        auto my_handler = common::fnptr<int(int,char*)>([&channel_str,&channelInfo](int n, char* ch){

            std::cout << "get n = " <<  n << ", ch = " << ch << std::endl;

            if(n == 0  ){
                if (std::strcmp(ch,"SUB")== 0){
                    channelInfo.type = 0;
                }else if(std::strcmp(ch,"PUB")== 0){
                    channelInfo.type=1;
                }
                else{
                    return -1;
                }

                return 0;
            }


            if(n == 1){
                channelInfo.topic.assign(ch);
                return 0;
            }

            if(n == 2){

                if(str2int(&channelInfo.qos, ch, 10) == STR2INT_SUCCESS){
                    return 0;
                }else{
                    return -1;
                }
                return 0;
            }
            return 0;
        });

        split_str(arg,":", my_handler);

        if(channelInfo.type == 0){
            char key_buffer[100];
            sprintf(key_buffer,"%s:%s",channel_str[channelInfo.type],channelInfo.topic.c_str());
//            std::cout <<__FILE__ << ":" << __LINE__ << key_buffer<<std::endl;
            auto it = channel_config.find(key_buffer);
            if(it != channel_config.end()){
                std::cout   << key_buffer << " is already exist" << std::endl;

                return 0;

            }

            std::cout << "creat sub " << key_buffer << std::endl;

            common::wild_ptr  channel_ptr;

            channel_ptr.set<MQTT_SUB_CONTAINER>();

            std::tie(it,std::ignore )= channel_config.emplace(key_buffer,channel_ptr);

            MQTT_SUB_CONTAINER& sub_container = it->second.ref<MQTT_SUB_CONTAINER>();
            common::wild_ptr& buffer = sub_container.buffer;


            buffer.set(std::vector<T>{});
            {

                auto& buffer_ref =  buffer.ref<std::vector<T>>();

            }

            sub_container.on_message = [&](const char* topic, const char* playload){



                auto& buffer_ref =  sub_container.buffer.ref<std::vector<T>>();

                T data = common_message::to_common<T>(playload);

                {
                    std::lock_guard<std::mutex> locker(sub_container.buffer_mtx);
                    buffer_ref.emplace_back(data);

                }
                return 0;
            };

            sub_container.func = [&](int max_num, double s, const std::function<void(void*)>& func){

//                client->loop_misc();

                {
                    std::lock_guard<std::mutex> locker(sub_container.buffer_mtx);
                    auto& buffer_ref =  buffer.ref<std::vector<T>>();
                    int recv_len = buffer_ref.size();

                    int start_index = std::max(  static_cast<int>(0),recv_len- max_num);
                    for(size_t i = start_index; i < recv_len;i++){
                        func(&buffer_ref[i]);
                    }
                    buffer_ref.clear();
                }


                return 0;
            };

            client.addSubOpt(channelInfo.topic.c_str(), channelInfo.qos);


        }else if(channelInfo.type == 1){

            char key_buffer[100];
            sprintf(key_buffer,"%s:%s",channel_str[channelInfo.type],channelInfo.topic.c_str());
            auto it = channel_config.find(key_buffer);
            if(it != channel_config.end()){
                std::cout   << key_buffer << " is already exist" << std::endl;

                return 0;

            }

            std::cout << "creat pub " << key_buffer << std::endl;
            common::wild_ptr  channel_ptr;

            channel_ptr.set<MQTT_PUB_CONTAINER>();
            std::tie(it,std::ignore )= channel_config.emplace(key_buffer,channel_ptr);

            MQTT_PUB_CONTAINER& pub_container = it->second.ref<MQTT_PUB_CONTAINER>();

            pub_container.qos = channelInfo.qos;
            pub_container.topic.assign(channelInfo.topic);

            pub_container.func = [&](void* data, int max_size, double s){
                T* data_ptr = static_cast<T*>(data);
                std::string msg;

                for(int i = 0 ; i < max_size; i++){

                    msg = common_message::from_common<T>((*(data_ptr + i)));

                    client.publish(nullptr,pub_container.topic.c_str(),msg.size(),msg.c_str(),pub_container.qos );

                }

                return 0;
            };

        }



        return 0;
    }

    long MqttMessageManager::start(char *arg) {
        auto& client = node_handler.ref<message::MqttClient>(); // = std::make_shared<message::MqttClient>(arg, true);

        client.connect_async(server_ip.c_str(),server_port,keep_alive);
        client.loop_start();
        return 0;
    }

    long MqttMessageManager::recv_message(const char *channel, size_t max_num, double timeout,
                                          const std::function<void(void *)> &callback) {
        auto& client = node_handler.ref<message::MqttClient>(); // = std::make_shared<message::MqttClient>(arg, true);

        client.loop();

        char * channel_str[] = {"SUB","PUB","TFL","TFB","PAR","PAW"};
        char key_buffer[100];

        sprintf(key_buffer,"%s:%s",channel_str[0],channel);

        auto it = channel_config.find(key_buffer);
        if(it == channel_config.end()){
            std::cout   << key_buffer << " is not exist" << std::endl;
            return 0;
        }
        MQTT_SUB_CONTAINER& sub_container = it->second.ref<MQTT_SUB_CONTAINER>();

        sub_container.func(max_num, timeout,callback);


        return 0;
    }

    long MqttMessageManager::send_message(const char *channel, void *data, size_t max_num, double timeout) {
        auto& client = node_handler.ref<message::MqttClient>(); // = std::make_shared<message::MqttClient>(arg, true);

        char * channel_str[] = {"SUB","PUB","TFL","TFB","PAR","PAW"};
        char key_buffer[100];

        sprintf(key_buffer,"%s:%s",channel_str[1],channel);


        auto it = channel_config.find(key_buffer);
        if(it == channel_config.end()){
            std::cout   << key_buffer << " is not exist" << std::endl;
            return 0;
        }

        MQTT_PUB_CONTAINER& pub_container = it->second.ref<MQTT_PUB_CONTAINER>();
        pub_container.func(data, max_num, timeout);
//        client->publish(nullptr,"hello2",msg.size(),msg.c_str(),mqtt_topic_qos );


//        client->loop_misc();
        client.loop();

        return 0;
    }

    bool MqttMessageManager::is_running() {
        return true;
    }

    void MqttMessageManager::stop() {
        auto& client = node_handler.ref<message::MqttClient>(); // = std::make_shared<message::MqttClient>(arg, true);

        client.loop_stop(true);
    }


    template long MqttMessageManager::add_channel<std::string>(const char *);
    template long MqttMessageManager::add_channel<common_message::Twist>(const char *);

}