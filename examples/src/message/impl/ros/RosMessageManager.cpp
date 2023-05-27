//
// Created by waxz on 5/11/23.
//

#include "RosMessageManager.h"
#include "RosMessageConvert.h"
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "common/string_func.h"


namespace message{
    struct NODE_CONTAINER {ros::NodeHandle nh;ros::NodeHandle nh_private;};

    struct ROS_SUB_CONTAINER{
        std::string signature;
        std::shared_ptr<ros::CallbackQueue> callback_queue;
        bool rt;
        std::shared_ptr<ros::NodeHandle> nh;
        std::shared_ptr<ros::Subscriber> sub;
        common::wild_ptr buffer;
        std::function<int(int, double, const std::function<void(void*)>)> func;
    };
    struct ROS_PUB_CONTAINER{
        std::string signature;
        std::shared_ptr<ros::NodeHandle> nh;
        std::shared_ptr<ros::Publisher> pub;
        common::wild_ptr buffer;
        std::function<int(void*, int, double)> func;
    };
    struct TFL_CONTAINER{
        tf::TransformListener tfl;
        tf::StampedTransform transform;
    };
    struct TFB_CONTAINER{
        tf::TransformBroadcaster tfb;
        tf::StampedTransform transform;

    };
    struct PAR_CONTAINER{};
    struct PAW_CONTAINER{};

    long RosMessageManager::open(char *arg, int argc,const char **argv)  {
        ros::init(argc, (char **)argv,arg);
        node_handler.set<NODE_CONTAINER>({ros::NodeHandle(),ros::NodeHandle("~")});
        return 0;
    }

    long RosMessageManager::start(char *arg) {
        return 0;
    }

    long RosMessageManager::send_message(const char *channel, void *data, size_t max_num, double timeout) {
        char * channel_str[] = {"SUB","PUB","TFL","TFB","PAR","PAW"};
        char key_buffer[100];

        sprintf(key_buffer,"%s:%s",channel_str[1],channel);


        auto it = channel_config.find(key_buffer);
        if(it == channel_config.end()){
            std::cout   << key_buffer << " is not exist" << std::endl;
            return 0;
        }
        ROS_PUB_CONTAINER& pub_container = it->second.ref<ROS_PUB_CONTAINER>();
        pub_container.func(data, max_num, timeout);
        return 0;
    }

    long RosMessageManager::recv_message(const char *channel, size_t max_num, double timeout,
                                         const std::function<void(void *)> &callback) {
        char * channel_str[] = {"SUB","PUB","TFL","TFB","PAR","PAW"};
        char key_buffer[100];

        sprintf(key_buffer,"%s:%s",channel_str[0],channel);

        auto it = channel_config.find(key_buffer);
        if(it == channel_config.end()){
            std::cout   << key_buffer << " is not exist" << std::endl;
            return 0;
        }
        ROS_SUB_CONTAINER& sub_container = it->second.ref<ROS_SUB_CONTAINER>();
        sub_container.func(max_num, timeout,callback);


        return 0;
    }

    template<typename T>
    long RosMessageManager::create_sub(char *topic, int queue_size) {

        return 0;
    }

    bool RosMessageManager::is_running() {

        return ros::ok();
    }

    void RosMessageManager::stop() {
        ros::shutdown();
    }


    template<typename T>
    long RosMessageManager::add_channel(const char *arg) {
        char * channel_str[] = {"SUB","PUB","TFL","TFB","PAR","PAW"};

        struct ChannelInfo{
            int type = 0;// 1: SUB, 2: PUB, 3: TF Listen , 4 TF Broadcast, 5 Param Reader, 6 Param Write
            std::string topic;
            int queue_size = 1;
        };
        ChannelInfo channelInfo;
        auto my_handler = common::fnptr<int(int,char*)>([&channel_str,&channelInfo](int n, char* ch){

            std::cout << "get n = " <<  n << ", ch = " << ch << std::endl;

            if(n == 0  ){
                if (std::strcmp(ch,"SUB")== 0){
                    channelInfo.type = 0;
                }else if(std::strcmp(ch,"PUB")== 0){
                    channelInfo.type=1;
                }
                else if(std::strcmp(ch,"TFL")== 0){
                    channelInfo.type=2;
                }

                else if(std::strcmp(ch,"TFB")== 0){
                    channelInfo.type =3;
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
                if(str2int(&channelInfo.queue_size, ch, 10) == STR2INT_SUCCESS){
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
            std::cout <<__FILE__ << ":" << __LINE__ << key_buffer<<std::endl;
            auto it = channel_config.find(key_buffer);
            if(it != channel_config.end()){
                std::cout   << key_buffer << " is already exist" << std::endl;

                return 0;

            }

            std::cout << "creat sub " << key_buffer << " TypeName<T>::Get() = " << common::TypeName<T>::Get()  << std::endl;

            std::shared_ptr<ros::CallbackQueue> q = std::make_shared<ros::CallbackQueue>();

            std::shared_ptr<ros::NodeHandle> n = std::make_shared<ros::NodeHandle>();
            std::shared_ptr<ros::Subscriber> sub = std::make_shared<ros::Subscriber>();


            using ROS_MSG_TYPE = decltype(common_message::from_common(std::declval<const T&>()));
            std::cout << " TypeName<T>::Get() = " << common::TypeName<T>::Get()  << std::endl;
            std::cout << " TypeName<ROS_MSG_TYPE>::Get() = " << common::TypeName<ROS_MSG_TYPE>::Get()  << std::endl;

            common::wild_ptr  channel_ptr;

            channel_ptr.set<ROS_SUB_CONTAINER>({common::TypeName<T>().Get(), q, false, n, sub, common::wild_ptr{}, {}});
            std::tie(it,std::ignore )= channel_config.emplace(key_buffer,channel_ptr);


            n->setCallbackQueue(q.get());
            ros::SubscribeOptions ops;
            ROS_SUB_CONTAINER& sub_container = it->second.ref<ROS_SUB_CONTAINER>();
            bool & rt = sub_container.rt;
            common::wild_ptr& buffer = sub_container.buffer;
            buffer.set(std::vector<T>{});
            {

                auto& buffer_ref =  buffer.ref<std::vector<T>>();

            }

            sub_container.func = [q,&buffer](int max_num, double s, const std::function<void(void*)>& func){

                auto& buffer_ref =  buffer.ref<std::vector<T>>();
                buffer_ref.clear();

                q->callAvailable(ros::WallDuration(s));

                int recv_len = buffer_ref.size();

                int start_index = std::max(  static_cast<int>(0),recv_len- max_num);
                for(size_t i = start_index; i < recv_len;i++){
                    func(&buffer_ref[i]);
                }

                return 1;
            };

            // pointer should be copied by value
            // reference should be copied by reference
            auto cb = [&rt,&buffer] (typename ROS_MSG_TYPE::ConstPtr msg) mutable {
                rt = true;
                buffer.ref<std::vector<T>>().push_back(common_message::to_common(*msg));
            };

            ops.template init<ROS_MSG_TYPE>(channelInfo.topic, channelInfo.queue_size, cb);
            ops.allow_concurrent_callbacks = true;
            *sub = n->subscribe(ops);
        }else if(channelInfo.type == 1){
            char key_buffer[100];
            sprintf(key_buffer,"%s:%s",channel_str[channelInfo.type],channelInfo.topic.c_str());

            std::cout << "creat pub " << key_buffer << std::endl;
            auto it = channel_config.find(key_buffer);
            if(it != channel_config.end()){
                std::cout   << key_buffer << " is already exist" << std::endl;

                return 0;

            }

            std::shared_ptr<ros::NodeHandle> n = std::make_shared<ros::NodeHandle>();
            std::shared_ptr<ros::Publisher> pub = std::make_shared<ros::Publisher>();

            using ROS_MSG_TYPE = decltype(common_message::from_common(std::declval<const T&>()));
            std::cout << " TypeName<T>::Get() = " << common::TypeName<T>::Get()  << std::endl;
            std::cout << " TypeName<ROS_MSG_TYPE>::Get() = " << common::TypeName<ROS_MSG_TYPE>::Get()  << std::endl;


            common::wild_ptr  channel_ptr;
            std::function<int(void*, int, double)> func;


            channel_ptr.set<ROS_PUB_CONTAINER>({common::TypeName<T>().Get(),n, pub, common::wild_ptr{},func});
            std::tie(it,std::ignore )= channel_config.emplace(key_buffer,channel_ptr);

            ROS_PUB_CONTAINER& pub_container = it->second.ref<ROS_PUB_CONTAINER>();

            common::wild_ptr& buffer = pub_container.buffer;
            buffer.set(std::vector<T>{});


            *pub = n->advertise<ROS_MSG_TYPE>(channelInfo.topic, channelInfo.queue_size);
            pub_container.func = [pub](void* data, int max_size, double s){
                T* data_ptr = static_cast<T*>(data);
                for(int i = 0 ; i < max_size; i++){
                    pub->publish(common_message::from_common(*(data_ptr + i)));
                }
                return 1;
            };
        }
        return 0;

    }

    long RosMessageManager::recv_tf(common_message::TransformStamped &data, float timeout) {

        char * channel_str[] = {"SUB","PUB","TFL","TFB","PAR","PAW"};

        char key_buffer[100];
        sprintf(key_buffer,"%s",channel_str[2]);
        auto it = channel_config.find(key_buffer);
        if(it != channel_config.end()){
            std::cout   << key_buffer << " is already exist" << std::endl;
        }else{

            common::wild_ptr  channel_ptr;
            channel_ptr.set<TFL_CONTAINER>();

            std::tie(it,std::ignore )= channel_config.emplace(key_buffer,channel_ptr);

        }
        ros::Time t(0);

        try{

            auto& tfl = it->second.ref<TFL_CONTAINER>();
            if(timeout > 0.0){

                common::ToRos(data.time,t);
                tfl.tfl.waitForTransform(data.base_frame, data.target_frame,t, ros::Duration(timeout));

            }
            tfl.tfl.lookupTransform(data.base_frame, data.target_frame,t, tfl.transform);

            common_message::to_common(tfl.transform,data);
            return 0;

        }catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            return -1;
        }
        return 0;
    }

    long RosMessageManager::send_tf(common_message::TransformStamped &data) {

        char * channel_str[] = {"SUB","PUB","TFL","TFB","PAR","PAW"};

        char key_buffer[100];
        sprintf(key_buffer,"%s",channel_str[3]);

        auto it = channel_config.find(key_buffer);
        if(it != channel_config.end()){
            std::cout   << key_buffer << " is already exist" << std::endl;
        }else{

            common::wild_ptr  channel_ptr;
            channel_ptr.set<TFB_CONTAINER>();

            std::tie(it,std::ignore )= channel_config.emplace(key_buffer,channel_ptr);

        }
        auto& tfb = it->second.ref<TFB_CONTAINER>();
        common_message::from_common(data, tfb.transform);
        tfb.tfb.sendTransform( tfb.transform);

        return 0;
    }
    template long RosMessageManager::add_channel<common_message::Odometry>(const char *);
    template long RosMessageManager::add_channel<common_message::Twist>(const char *);
    template long RosMessageManager::create_sub<common_message::Twist>(char * , int );
}
