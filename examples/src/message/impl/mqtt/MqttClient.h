//
// Created by waxz on 9/8/22.
//

#ifndef MAP_REPUBLISHER_MQTTCLIENT_H
#define MAP_REPUBLISHER_MQTTCLIENT_H
#include <mosquittopp.h>
#include <string>
#include <map>
#include <iostream>
#include <functional>

namespace message{
    class MqttClient: public mosqpp::mosquittopp{

    public:
        MqttClient(const char *id=NULL, bool clean_session=true):mosqpp::mosquittopp(id,clean_session){

        }
    public:

        struct ConnOpt{
            std::string server_ip;
            int server_port;
            int keepalive = 60;
            std::string will_topic;
            std::string will_massage;
        };
        ConnOpt m_conn_opt;
        bool is_loop_started = false;






        struct SubOption{
            std::string topic;
            int qos;
        };
        std::map<std::string, int> m_subs_topic_qos;

        void addSubOpt(const char* topic,int qos){
            m_subs_topic_qos.emplace(topic, qos);
        }
        void listSubOpts(){
            std::cout << "\n  listSubOpts\n" <<std::endl;
            for(auto it = m_subs_topic_qos.begin();it != m_subs_topic_qos.end();it++){
                std::cout << "-- k: " << it->first << ", v: " << it->second << std::endl;
            }
        }



        void on_connect(int reason_code);

        /* Callback called when the broker sends a SUBACK in response to a SUBSCRIBE. */
        void on_subscribe( int mid, int qos_count, const int *granted_qos);

        std::function<int(const char*, const char*)> message_callback_func;


        /* Callback called when the client receives a message. */
        void on_message(const struct mosquitto_message *msg);
        void on_disconnect(int rc)
        {
            std::cerr << "mosq disconnect try to reconnect" << std::endl;
            reconnect_async();
        }
        ~MqttClient() {
            std::cout << "~MqttClient"<<std::endl;
            mosqpp::lib_cleanup();

        }

        int loop();

    };

}


#endif //MAP_REPUBLISHER_MQTTCLIENT_H
