//
// Created by waxz on 9/8/22.
//
#include "MqttClient.h"

namespace message{

    void MqttClient::on_connect(int reason_code){

        int rc;
        /* Print out the connection result. mosquitto_connack_string() produces an
         * appropriate string for MQTT v3.x clients, the equivalent for MQTT v5.0
         * clients is mosquitto_reason_string().
         */
        printf("on_connect: %s\n", mosquitto_connack_string(reason_code));
        if(reason_code != 0){
            /* If the connection fails for any reason, we don't want to keep on
             * retrying in this example, so disconnect. Without this, the client
             * will attempt to reconnect. */
            disconnect();
        }

        for(auto it = m_subs_topic_qos.begin();it != m_subs_topic_qos.end();it++){
            std::cout << "-- k: " << it->first << ", v: " << it->second << std::endl;
            /* Making subscriptions in the on_connect() callback means that if the
             * connection drops and is automatically resumed by the client, then the
             * subscriptions will be recreated when the client reconnects. */
            rc = subscribe( NULL, it->first.c_str(), it->second);
            if(rc != MOSQ_ERR_SUCCESS){
                fprintf(stderr, "Error subscribing: %s\n", mosquitto_strerror(rc));
                /* We might as well disconnect if we were unable to subscribe */
                disconnect();

            }
        }

    }


    void MqttClient::on_subscribe( int mid, int qos_count, const int *granted_qos)
    {
        int i;
        bool have_subscription = false;

        /* In this example we only subscribe to a single topic at once, but a
         * SUBSCRIBE can contain many topics at once, so this is one way to check
         * them all. */
        for(i=0; i<qos_count; i++){
            printf("on_subscribe: %d:granted qos = %d\n", i, granted_qos[i]);
            if(granted_qos[i] <= 2){
                have_subscription = true;
            }
        }
        if(!have_subscription){
            /* The broker rejected all of our subscriptions, we know we only sent
             * the one SUBSCRIBE, so there is no point remaining connected. */
            fprintf(stderr, "Error: All subscriptions rejected.\n");
            disconnect();
        }
    }

    /* Callback called when the client receives a message. */
    void MqttClient::on_message(const struct mosquitto_message *msg)
    {
        /* This blindly prints the payload, but the payload can be anything so take care. */
//        printf("sub receive: %s %d %s\n", msg->topic, msg->qos, (char *)msg->payload);
        if (message_callback_func){
            message_callback_func(msg->topic, (char *)msg->payload);

        }else{
            std::cout << "message_callback_func is no valid" << std::endl;

        }
    }

    int MqttClient::loop() {

        int rc = loop_misc();
        if(rc != MOSQ_ERR_SUCCESS){
            fprintf(stderr, "Error mosquitto_loop_misc: %s\n", mosquitto_strerror(rc));
        }
        if ( rc == MOSQ_ERR_NO_CONN){
            std::cout << " MOSQ_ERR_NO_CONN, reconnect_async "  << std::endl;
            reconnect_async();
        }
        return rc;
    }

}
