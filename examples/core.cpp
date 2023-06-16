/*
 * Copyright (c) 2015-2016, Thomas Keh
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
 
#include <chrono>
#include <vector>
#include <iostream>
#include <string>
#include <iostream>
#include <bitset>
#include <fstream>
#include <deque>

#include "lyra/lyra.hpp"

#include "math/geometry.h"
//#include "master.h"
#include "global_config.h"
#include "core.h"

#include "common/task.h"
#include "common/functions.h"
#include "common/signals.h"
#include "nlohmann/json.hpp"

#include "message/crc.h"

#include "message/impl/ros/RosMessageManager.h"
#include "message/impl/mqtt/MqttMessageManager.h"

#include "message/MessageManager.h"


#include "control/MobileRobotController.h"

#include <plog/Log.h> // Step1: include the headers
#include <plog/Initializers/RollingFileInitializer.h>
#include <plog/Appenders/ColorConsoleAppender.h>
#include "absl/strings/str_format.h"
#include "absl/strings/escaping.h"

#include "common/random.h"

#define SOL_CHECK_ARGUMENTS 1
#include <sol.hpp>

struct IntInt{
    int num_1 = 0;
    int num_2 = 0;
};

struct BitArray{
    std::bitset<64> data;
};

IntInt to_common(const kaco::Message& data){
    IntInt target;
    target.num_1 = * (int*)(&data.data[0]);
    target.num_2 = * (int*)(&data.data[4]);
    return target;
}
void from_common(const IntInt& data, kaco::Message& target){

#if 0
    *(int*)(&target.data[0]) = data.num_1;
    *(int*)(&target.data[4]) = data.num_2;

#endif
#if 1
    int* p1 = (int*)(&target.data[0]);
    int* p2 = (int*)(&target.data[4]);
    *p1 =  data.num_1;
    *p2 =  data.num_2;
#endif
#if 0
    for(size_t i = 0 ; i < 4;i++){
        target.data[0] =* ((char*)(&data.num_1) + i);
        target.data[4 + i] =* ((char*)(&data.num_2) + i);
    }
#endif
}


void to_common(const kaco::Message& data,BitArray& target){

    target.data = * (u_int64_t *)(&data.data[0]);
}


struct AngleStamped{
    common::Time time = common::FromUnixNow();
    float angle = 0;
};

template<typename T>
struct ValueStamped{

    common::Time time = common::FromUnixNow();
    T value ;
};


template<typename T>
struct ValueStampedBuffer{
    std::deque<ValueStamped<float>> buffer;
    void add(float value){

        buffer.push_back(ValueStamped<float>{common::FromUnixNow(), value});

        if(buffer.size() > 2000){
            buffer.erase(buffer.begin(),buffer.begin() + 1000);
        }
    }
    bool empty(){
        return buffer.empty();
    }

    size_t size(){
        return buffer.size();
    }
    bool query(const common::Time& time, T & value){


        if(time == buffer.back().time){

            value = buffer.back().value;
            return true;
        }

        // first v > t
        auto low_it = std::lower_bound(buffer.begin(),buffer.end(), time, [](auto & v, auto& t){
            return v.time < t;
        });

        // first v < t
        auto up_it = std::lower_bound(buffer.begin(),buffer.end(), time, [](auto & v, auto& t){
            return v.time > t;
        });


        if(low_it != buffer.end() && up_it != buffer.end()){

           auto s1 =  low_it->time - up_it->time;
            auto s2 =  time - up_it->time;

            value = up_it->value + (up_it->value - low_it->value)*s2/s1;

            return true;
        }else{
            return false;
        }

        return false;
    }
    ValueStamped<float>& back(){
        return buffer.back();
    }

};
struct ControllerConfig{

    bool use_rot_angle_abs = false;
    bool is_rot_angle_calib = false;
    // angle1 = feedback1 * rot_angle_k + rot_angle_b
    float rot_angle_k = 1.0;
    float rot_angle_b = 0.0;
    // angle2 = feedback2 * rot_angle_abs_k + rot_angle_abs_b
    float rot_angle_abs_k = 1.0;
    float rot_angle_abs_b = 1.0;
    // real_angle = angle2 = angle1 + rot_angle_abs_offset
    // rot_angle_abs_offset = angle2 - angle1
    float rot_angle_abs_offset = 0.0;

    float forward_speed_k = 1.0;
    float mount_x = 0.0;
    float mount_y = 0.0;
    float forward_acc = 0.8;
    float rotate_vel = 0.5;

    float actual_rot_angle = 0.0;
    float actual_forward_vel = 0.0;
    float actual_rot_abs_angle = 0.0;

    ValueStampedBuffer<float> angle_buffer;
    ValueStampedBuffer<float> angle_abs_buffer;

    common::Time forward_time;
    common::Time rotate_time;

    
    

    void addAngle(float angle){

        actual_rot_angle = angle;
        if(use_rot_angle_abs){
            angle_buffer.add( angle );
        }else{
            rot_angle_abs_offset = 0.0;
            is_rot_angle_calib = true;
            actual_rot_abs_angle = actual_rot_angle;
        }


    }
    void addAngleAbs(float angle){

        if(use_rot_angle_abs){
            actual_rot_abs_angle = angle;
            angle_abs_buffer.add( angle );
        }else{
//            is_rot_angle_calib = true;
        }

    }
    bool isAngleCalib(){

        return is_rot_angle_calib;
    }

    float getActualRotateAngle(){
        if(use_rot_angle_abs){
            angleCalib();
            return actual_rot_angle + rot_angle_abs_offset;
        }else{
            return actual_rot_angle;
        }
    }
    bool angleCalib(){

        if(!use_rot_angle_abs){
            is_rot_angle_calib = true;
            return true;
        }
        is_rot_angle_calib = !use_rot_angle_abs;
        if(angle_abs_buffer.size() < 10 || angle_buffer.size() < 10 ){
            return false;
        }

        common::Time check_point = angle_abs_buffer.back().time > angle_buffer.back().time ? angle_buffer.back().time : angle_abs_buffer.back().time;

        float angle, angle_abs;
        bool ok1 = angle_buffer.query(check_point,angle);
        bool ok2 = angle_abs_buffer.query(check_point,angle_abs);

        is_rot_angle_calib = ok1 && ok2;
        if(is_rot_angle_calib){

            rot_angle_abs_offset = angle_abs - angle;
        }

        return is_rot_angle_calib;
    }

};
struct ControllerState{
    ControllerConfig config;
    IntInt state;
    IntInt command;
    BitArray signal;
    bool left_limit = false;
    bool right_limit = false;
    bool find_home = false;
    bool ready = false;

    // fault
    bool is_OverHeat = false;
    bool is_OverVolt = false;
    bool is_UnderVolt = false;
    bool is_Short = false;
    bool is_EStop = false;
    bool is_Motor_Sensor = false;
    bool is_MOSFail = false;
    bool is_DefConfig = false;
    bool is_STOFault = false;

    std::bitset<16> fault_flag = 0;
    float rot_angle = 0.0;
    float forward_vel = 0.0;
    kaco::Message command_message;
    kaco::Message recv_message;
    kaco::Message recv_state_message;
    size_t msg_num = 0;
    common::Time last_update_time;

    bool updated = false;
    void reset(){
        msg_num = 0;
        last_update_time = common::FromUnixNow();
    }


    void createCommand(float command_forward_vel, float command_rot_angle){

        command.num_1 = command_forward_vel/config.forward_speed_k;
        command.num_2 = (command_rot_angle - config.rot_angle_b)/config.rot_angle_k;
//        PLOGD << "command_forward_vel: " << command_forward_vel << ", command_rot_angle: " << command_rot_angle ;
//        PLOGD << "command.num_1: " << command.num_1 << ", command.num_2: " << command.num_2 ;

        from_common(command, command_message);

    }

};




constexpr auto ControllerConfig_properties = std::make_tuple(
        common::property(&ControllerConfig::rot_angle_k, "rot_angle_k"),
        common::property(&ControllerConfig::rot_angle_b, "rot_angle_b"),
        common::property(&ControllerConfig::rot_angle_abs_k, "rot_angle_abs_k"),
        common::property(&ControllerConfig::rot_angle_abs_b, "rot_angle_abs_b"),
        common::property(&ControllerConfig::use_rot_angle_abs, "use_rot_angle_abs"),
        common::property(&ControllerConfig::forward_speed_k, "forward_speed_k"),
        common::property(&ControllerConfig::mount_x, "mount_x"),
        common::property(&ControllerConfig::mount_y, "mount_y")
);
void to_json(nlohmann::json& j, const ControllerConfig& object)
{

    constexpr auto nbProperties = std::tuple_size<decltype(ControllerConfig_properties)>::value;
    common::for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
        // get the property
        auto& property = std::get<i>(ControllerConfig_properties);
        // set the value to the member
        j[property.name] = object.*(property.member);
    });
}

void from_json(const nlohmann::json& j, ControllerConfig& object) {

    constexpr auto nbProperties = std::tuple_size<decltype(ControllerConfig_properties)>::value;
    common::for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
        // get the property
        auto& property = std::get<i>(ControllerConfig_properties);
        // set the value to the member
        if(j.contains(property.name))
        j.at(property.name).get_to(object.*(property.member));
        else{
            std::cerr << "from_json: key["<<property.name<<"] not exist"<< std::endl;
        }
    });
}


struct RawCommand{
    float forward_vel_1 = 0.0;
    float forward_vel_2 = 0.0;

    float rotate_angle_1 = 0.0;
    float rotate_angle_2 = 0.0;


};

constexpr auto RawCommand_properties = std::make_tuple(
        common::property(&RawCommand::forward_vel_1, "forward_vel_1"),
        common::property(&RawCommand::forward_vel_2, "forward_vel_2"),
        common::property(&RawCommand::rotate_angle_1, "rotate_angle_1"),
        common::property(&RawCommand::rotate_angle_2, "rotate_angle_2")
);

void to_json(nlohmann::json& j, const RawCommand& object)
{

    constexpr auto nbProperties = std::tuple_size<decltype(RawCommand_properties)>::value;
    common::for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
        // get the property
        auto& property = std::get<i>(RawCommand_properties);
        // set the value to the member
        j[property.name] = object.*(property.member);
    });
}

void from_json(const nlohmann::json& j, RawCommand& object) {

    constexpr auto nbProperties = std::tuple_size<decltype(RawCommand_properties)>::value;
    common::for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
        // get the property
        auto& property = std::get<i>(RawCommand_properties);
        // set the value to the member
        if(j.contains(property.name))
            j.at(property.name).get_to(object.*(property.member));
        else{
            std::cerr << "from_json: key["<<property.name<<"] not exist"<< std::endl;
        }
    });
}

struct Controller{
    ControllerState state;
    ControllerConfig config;
};


struct CanMsgForward{
    size_t channel_id = 0;
    std::vector<uint16_t> recv_cob_id;
    std::string sub_topic;
    std::string pub_topic;
};
constexpr auto CanMsgForward_properties = std::make_tuple(
        common::property(&CanMsgForward::channel_id, "channel_id"),
        common::property(&CanMsgForward::recv_cob_id, "recv_cob_id") ,
        common::property(&CanMsgForward::sub_topic, "sub_topic"),
        common::property(&CanMsgForward::pub_topic, "pub_topic")
);

void to_json(nlohmann::json& j, const CanMsgForward& object)
{

    constexpr auto nbProperties = std::tuple_size<decltype(CanMsgForward_properties)>::value;
    common::for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
        // get the property
        auto& property = std::get<i>(CanMsgForward_properties);
        // set the value to the member
        j[property.name] = object.*(property.member);
    });
}

void from_json(const nlohmann::json& j, CanMsgForward& object) {

    constexpr auto nbProperties = std::tuple_size<decltype(CanMsgForward_properties)>::value;
    common::for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
        // get the property
        auto& property = std::get<i>(CanMsgForward_properties);
        // set the value to the member
        if(j.contains(property.name))
//            object.*(property.member) = j[property.name];
           j.at(property.name).get_to(object.*(property.member));
        else{
            std::cerr << "from_json: key["<<property.name<<"] not exist"<< std::endl;
        }
    });
}

struct CanForward{

};
int test_tec(int argc, char** argv) {


    std::atomic_bool program_run(true);
    auto my_handler = common::fnptr<void(int)>([&](int sig){ std::cout << "get sig " << sig;program_run = false;});
    common::set_signal_handler(my_handler);

    plog::RollingFileAppender<plog::CsvFormatter> fileAppender("tec.csv", 80000000, 20); // Create the 1st appender.
    plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender; // Create the 2nd appender.
    plog::init(plog::debug, &fileAppender).addAppender(&consoleAppender); // Initialize the logger with the both appenders.



    std::string exe_name;
    bool get_help = false;
    bool use_sim = false;
    bool smooth_stop = true;
    bool enable_control = false;


    bool enable_fix_command = false;
    std::vector<float> command_array;

    int cmd_vel_timeout_ms = 100;
    int feedback_timeout_ms = 100;
    int initialise_wait_s = 1;

    std::vector<std::string> command;
    std::vector<std::string> extra_command;


    std::string mqtt_server = "192.168.1.101";
    std::string mqtt_pub_topic = "hello_robot";
    std::string mqtt_sub_topic = "hello_lua";
    std::string config_file;

    std::string can_forward_file;


    float forward_vel_acc = 0.8;
    float rotate_angle_vel = 0.8;

    bool use_rot_angle_abs = false;


    auto cli
            =  lyra::exe_name(exe_name)
               | lyra::help(get_help)
               | lyra::opt( config_file, "config_file" )["-c"]["--config_file"]("config_file")
               | lyra::opt( enable_control)["-e"]["--enable_control"]("enable_control")
               | lyra::opt(smooth_stop,"smooth_stop")["-s"]["--smooth_stop"]("smooth_stop")
//               | lyra::opt(use_rot_angle_abs,"use_rot_angle_abs")["-a"]["--abs_rot"]("use_rot_angle_abs")
               | lyra::opt(cmd_vel_timeout_ms,"cmd_vel_timeout_ms")["-t"]["--cmdvel_timeout"]("cmd_vel_timeout_ms")
               | lyra::opt(feedback_timeout_ms,"feedback_timeout_ms")["-f"]["--feedback_timeout"]("feedback_timeout_ms")
               | lyra::opt(initialise_wait_s,"initialise_wait_s")["-i"]["--initialise_wait_s"]("initialise_wait_s")
                 | lyra::opt(mqtt_server,"mqtt_server")["-M"]["--mqtt_server"]("mqtt_server")
                   | lyra::opt(can_forward_file,"can_forward_file")["-T"]["--can_forward_file"]("can_forward_file")

                 //---
                 | lyra::group().sequential()
                 | lyra::opt(enable_fix_command, "ON|OFF")["--command"]
                 | lyra::arg(command_array, "command_array")
                 //---
               | lyra::arg(command, "command")("The command, and arguments, to attempt to run.")
               ;


    auto result = cli.parse( { argc, argv } );
    if(get_help){
        std::cout << cli << std::endl;
        return 0;

    }


    if ( !result )
    {
        std::cerr << "Error in command line: " << result.message() << std::endl;
        return 0;
    }

    if(config_file.empty()){
        std::cout << "config_file not set" << std::endl;
        return 0;

    }



    PLOGD << "enable_fix_command: " << enable_fix_command;


    if(enable_fix_command &&command_array.size() !=4 ){
        std::cout << " enable_fix_command , command_array.size() = " << command_array.size() << std::endl;
        return 0;

    }

    const int CONTROLLER_NUM = 2;

    std::vector<ControllerConfig> SteerConfigArray;
    std::vector<ControllerState >SteerStateArray;
    std::vector<CanMsgForward> CanForwardConfigArray;


    if(!config_file.empty()){

        std::ifstream ifs(config_file.c_str());
        if(ifs.is_open()){

            try{
                nlohmann::json jf = nlohmann::json::parse(ifs);

                std::cout << "config_file: " << config_file<< "\n jf :\n" << jf << std::endl;

                SteerConfigArray = jf;
            }catch(nlohmann::detail::parse_error & e) {

                std::cout << e.what() << std::endl;
                std::cout << "json format error in config_file: " << config_file<<  std::endl;

                return 0;

            }


        }else{
            std::cout << "config_file not exist" << std::endl;
            return 0;

        }
        if(SteerConfigArray.size() != CONTROLLER_NUM){
            std::cout << "config_file error: SteerConfigArray.size() = " << SteerConfigArray.size() << std::endl;
            return 0;
        }

        SteerStateArray.resize(SteerConfigArray.size());

        for(int i = 0 ; i < CONTROLLER_NUM;i++){
//            SteerConfigArray[i].use_rot_angle_abs = use_rot_angle_abs;
            SteerStateArray[i].config = SteerConfigArray[i];
        }
    }


    if(!can_forward_file.empty()){
        std::ifstream ifs(can_forward_file.c_str());
        if(ifs.is_open()){

            try{
                nlohmann::json jf = nlohmann::json::parse(ifs);

                std::cout << "can_forward_file: " << can_forward_file<< "\n jf :\n" << jf << std::endl;

                CanForwardConfigArray = jf;
            }catch(nlohmann::detail::parse_error & e) {

                std::cout << e.what() << std::endl;
                std::cout << "json format error in can_forward_file: " << can_forward_file<<  std::endl;

                return 0;

            }


        }else{
            std::cout << "can_forward_file not exist" << std::endl;
            return 0;

        }
    }

    extra_command.emplace_back(exe_name);
    std::copy(command.begin(), command.end(), std::back_inserter(extra_command));


    int ARGC = extra_command.size();
    const char *  ARGS[50];
    for(int i = 0 ; i <ARGC;i++ ){
        ARGS[i] = extra_command[i].data();
    }

    const char** ARGS1 = const_cast<const char **>(ARGS);


    //mqtt
    std::cout << "run lua" << std::endl;
    lua_State* L = luaL_newstate();
    sol::state_view lua(L);
    lua.open_libraries(sol::lib::base, sol::lib::package,sol::lib::os, sol::lib::table, sol::lib::jit,sol::lib::coroutine);



    auto simple_handler_lambda =
            [](lua_State*, const sol::protected_function_result& result) {
                // You can just pass it through to let the
                // call-site handle it
                std::cout << "\n******LUA INFO:\nAn exception occurred in a function, here's what it says ";
                sol::error err = result;
                std::cout << "call failed, sol::error::what() is " <<  err.what() << std::endl;
//                MLOGW("%s", what.c_str());
                return result;
            };
    lua.set_function("move_command",[&](float forward_vel_1, float  rotate_angle_1, float forward_vel_2, float  rotate_angle_2){

        command_array.resize(4);
        command_array[0] = forward_vel_1;
        command_array[1] = rotate_angle_1;
        command_array[2] = forward_vel_2;
        command_array[3] = rotate_angle_2;

        command_array[1] = std::max(std::min(1.918888889f, command_array[1]),-1.918888889f);
        command_array[3] = std::max(std::min(1.918888889f, command_array[3]),-1.918888889f);

    });


    std::string mqtt_server_str = absl::StrFormat("server:%s", mqtt_server.c_str());

    PLOGD << "mqtt_server_str: " << mqtt_server_str;

//    char* mqtt_config_str[] = {"server:broker-cn.emqx.io", "port:1883","keep_alive:30","clean_session:1"};
//    char* mqtt_config_str[] = {"server:172.30.254.199", "port:1883","keep_alive:30","clean_session:1"};
    const char* mqtt_config_str[] = {mqtt_server_str.c_str(), "port:1883","keep_alive:30","clean_session:1"};

    message::MqttMessageManager mqttMessageManager;
    char* mqtt_client_id = "cpp_test";
    mqttMessageManager.open(mqtt_client_id,4, mqtt_config_str);
//    char topic_arg[100];
    std::string mqtt_pub_topic_arg = absl::StrFormat("PUB:%s:0", mqtt_pub_topic.c_str());
    std::string mqtt_sub_topic_arg = absl::StrFormat("SUB:%s:0", mqtt_sub_topic.c_str());

    mqttMessageManager.add_channel<std::string>(mqtt_pub_topic_arg.c_str());

    mqttMessageManager.add_channel<std::string>(mqtt_sub_topic_arg.c_str());



    std::string decode_str;
    auto lua_cb =[&](void* data){
        std::string* data_ptr = static_cast<std::string*>(data);
        std::cout << "test123 recv msg: " << *data_ptr << std::endl;
        bool decode_ok =  absl::Base64Unescape(*data_ptr,&decode_str);
        std::cout << "test123 recv decode_str: " << decode_str << std::endl;

        if(decode_ok){

            auto result = lua.script(
                    decode_str,
                    simple_handler_lambda);
            if (result.valid()) {
                std::cout << "run lua script ok"
                          << std::endl;
            }
            else {
                std::cout << "run lua script fail"
                          << std::endl;
            }



        }
    };

    mqttMessageManager.start("");

    std::cout << "mqtt start done" << std::endl;

    const int STATUS_NUM = 11;
    const int FRAME_NUM = 60;

    common::Time record_start_time = common::FromUnixNow();
    int mqtt_msg_status_cnt = 0;
    std::vector<float> mqtt_msg_status(STATUS_NUM*FRAME_NUM);
    std::vector<int16_t> mqtt_msg_status_int(STATUS_NUM*FRAME_NUM);

    std::string mqtt_binary_str;
    std::string mqtt_base64_str;


    //mqtt

    //ros


    message::RosMessageManager rosMessageManager;

    rosMessageManager.open("driver_comm", ARGC, ARGS1 );


    rosMessageManager.add_channel<common_message::Twist>("SUB:control_cmd_vel:10");
    rosMessageManager.add_channel<common_message::Odometry>("PUB:odom:200");

    std::vector<common_message::Twist> cmd_vel_msgs(10);
    common_message::Odometry odom;
    odom.twist.twist.linear.z = 0.0f;

    odom.twist.twist.angular.x = 0.0f;
    odom.twist.twist.angular.y = 0.0f;

    odom.header.frame_id.assign("odom");
    odom.child_frame_id.assign("base_link");

    odom.pose.covariance[0] = 0.1;
    odom.pose.covariance[7] = 0.1;
    odom.pose.covariance[14] = 0.1;
    odom.pose.covariance[21] = 0.1;
    odom.pose.covariance[28] = 0.1;

    odom.pose.covariance[35] = 0.1;
    odom.twist.covariance[0] = 0.1;
    odom.twist.covariance[7] = 0.1;

    odom.twist.covariance[14] = 0.1;
    odom.twist.covariance[21] = 0.1;
    odom.twist.covariance[28] = 0.1;

    odom.twist.covariance[35] = 0.1;


    // todo: callback may fail if topic and datetye not matched
    bool recv_new_cmd_vel = false;
    common_message::Twist recv_cmd_vel_msg;
    common::Time  recv_cmd_vel_time = common::FromUnixNow();
    auto cmd_vel_sub_cb =[&recv_new_cmd_vel,&recv_cmd_vel_msg,&recv_cmd_vel_time](void* data){
        common_message::Twist * data_ptr = static_cast<common_message::Twist*>(data);
        recv_cmd_vel_msg = * data_ptr;
//        std::cout << "odom_sub_cb recv msg: " << data_ptr->linear.x << ", " << data_ptr->angular.z << std::endl;
        recv_cmd_vel_time = common::FromUnixNow();
        recv_new_cmd_vel = true;
    };


    //ros








    /*

     device list
     1. forward_1
     2. rotate_1
     3. forward_2
     4. rotate_2



    */

    // Set the name of your CAN bus. "slcan0" is a common bus name
    // for the first SocketCAN device on a Linux system.
    const std::string busname = "USBCAN-II";

    // Set the baudrate of your CAN bus. Most drivers support the values
    // "1M", "500K", "125K", "100K", "50K", "20K", "10K" and "5K".
    const std::string baudrate = "500K:500K";

    // -------------- //
    // Initialization //
    // -------------- //

    common::TaskManager taskManager;

    std::cout << "This is an example which shows the usage of the Core library." << std::endl;

    // Create core.
    kaco::Core device;


    const int MAX_MESSAGE_SIZE = 1000;
    std::vector<kaco::Message> message_buffer(MAX_MESSAGE_SIZE);


    auto send_to_channel_1 = [&](const auto& msg){
        device.send(msg,0);
        return 1;
    };
    auto send_to_channel_2 = [&](const auto& msg){
        device.send(msg,1);
        return 1;
    };

    // nmt
    // 1,2,3,4,5,6
    const size_t DEVICE_NUM = 6;
    std::vector<kaco::NMT::NodeState> device_node_state_array(DEVICE_NUM + 1);
    std::vector<std::bitset<16>> device_node_emcy_array(DEVICE_NUM + 1);

    std::vector<kaco::Message> forward_driver_initializer_msg(4);
    std::vector<kaco::Message> rotate_driver_initializer_msg(4);

    std::vector<kaco::Message> forward_driver_disable_msg(1);
    std::vector<kaco::Message> rotate_driver_disable_msg(1);

    std::vector<kaco::Message> driver_command_msg_channel_1(2);
    std::vector<kaco::Message> driver_command_msg_channel_2(2);

    forward_driver_disable_msg[0] = kaco::Message{0x200,false,false,8,{0x06,0x00,0x03,0,0,0,0,0}};
    rotate_driver_disable_msg[0] = kaco::Message{0x200,false,false,8,{0x06,0x00,0x08,0,0,0,0,0}};

    forward_driver_initializer_msg[0] = kaco::Message{0x00,false,false,2,{0x01,0x00,0x00,0,0,0,0,0}};
    forward_driver_initializer_msg[1] = kaco::Message{0x200,false,false,8,{0x06,0x00,0x03,0,0,0,0,0}};
    forward_driver_initializer_msg[2] = kaco::Message{0x200,false,false,8,{0x07,0x00,0x03,0,0,0,0,0}};
    forward_driver_initializer_msg[3] = kaco::Message{0x200,false,false,8,{0x0f,0x00,0x03,0,0,0,0,0}};

    rotate_driver_initializer_msg[0] = kaco::Message{0x00,false,false,2,{0x01,0x00,0x00,0,0,0,0,0}};
    rotate_driver_initializer_msg[1] = kaco::Message{0x200,false,false,8,{0x06,0x00,0x08,0,0,0,0,0}};
    rotate_driver_initializer_msg[2] = kaco::Message{0x200,false,false,8,{0x07,0x00,0x08,0,0,0,0,0}};
    rotate_driver_initializer_msg[3] = kaco::Message{0x200,false,false,8,{0x0f,0x00,0x08,0,0,0,0,0}};

    driver_command_msg_channel_1[0] =kaco::Message{0x301,false,false,8,{0x00,0x00,0x00,0,0,0,0,0}};
    driver_command_msg_channel_1[1] =kaco::Message{0x302,false,false,8,{0x00,0x00,0x00,0,0,0,0,0}};
    driver_command_msg_channel_2[0] =kaco::Message{0x304,false,false,8,{0x00,0x00,0x00,0,0,0,0,0}};
    driver_command_msg_channel_2[1] =kaco::Message{0x305,false,false,8,{0x00,0x00,0x00,0,0,0,0,0}};

    auto initialise_forward_driver = [&](size_t driver_id){
        PLOGD << "initialise_forward_driver: " << driver_id;

        forward_driver_initializer_msg[0].data[1] = driver_id;

        for(size_t i = 1; i < forward_driver_initializer_msg.size();i++){

            forward_driver_initializer_msg[i].cob_id = 0x200 + driver_id;
        }
        int channel = (driver_id <= 2) ? 0 : 1;
        device.send(forward_driver_initializer_msg.data(),forward_driver_initializer_msg.size(),channel);
    };

    auto initialise_rotate_driver = [&](size_t driver_id){

        PLOGD << "initialise_rotate_driver: " << driver_id;
        rotate_driver_initializer_msg[0].data[1] = driver_id;

        for(size_t i = 1; i < rotate_driver_initializer_msg.size();i++){

            rotate_driver_initializer_msg[i].cob_id = 0x200 + driver_id;
        }
        int channel = (driver_id <= 2) ? 0 : 1;
        device.send(rotate_driver_initializer_msg.data(),rotate_driver_initializer_msg.size(),channel);
    };

    auto disable_forward_driver = [&](size_t driver_id){
        for(auto& m:forward_driver_disable_msg){
            m.cob_id = 0x200 + driver_id;
        }
        int channel = (driver_id <= 2) ? 0 : 1;
        device.send(forward_driver_disable_msg.data(),forward_driver_disable_msg.size(),channel);

    };

    auto disable_rotate_driver = [&](size_t driver_id){
        for(auto& m:rotate_driver_disable_msg){
            m.cob_id = 0x200 + driver_id;
        }
        int channel = (driver_id <= 2) ? 0 : 1;
        device.send(rotate_driver_disable_msg.data(),rotate_driver_disable_msg.size(),channel);

    };

    bool is_all_alive = false;
    bool is_all_initialised = false;
    bool is_all_initialise_triggered = false;
    bool is_rot_sensor_ready = false;

    bool is_all_operational = false;
    bool is_any_fault = false;

    auto node_heartbeat_cb = [&](const kaco::Message& message, const kaco::NMT::NodeState & state){

        PLOGD << "node_heartbeat : " << int(message.get_node_id()) << ", state: " << int(state);
        if(message.get_node_id() > 0 && message.get_node_id() <= DEVICE_NUM){
            device_node_state_array[message.get_node_id()] = state;
        }
    };

    auto node_emcy_cb = [&](const kaco::Message& message){
        if(message.get_node_id() > 0 && message.get_node_id() <= DEVICE_NUM){
            device_node_emcy_array[message.get_node_id()] =  * (u_int64_t *)(&message.data[3]);
        }
    };

    auto check_all_state = [&](){

        is_all_alive = true;
        std::array<size_t, 4> driver_id_vec{1,2,4,5};
        for(auto i : driver_id_vec){
            is_all_alive = is_all_alive &&  (device_node_state_array[i] == kaco::NMT::NodeState::Pre_operational || device_node_state_array[i] == kaco::NMT::NodeState::Operational );
        }
        is_any_fault = false;
        for(auto i : driver_id_vec)   {
            is_any_fault = is_any_fault || (device_node_emcy_array[i] != 0 );
        }
    };
    device.nmt.register_device_alive_callback(node_heartbeat_cb);

    device.nmt.register_device_emcy_callback(node_emcy_cb);

    std::vector<kaco::Message> send_message_array(6);

    auto control_wheel = [&](bool enable, float forward_vel_1, float rotate_angle_1, float forward_vel_2, float rotate_angle_2){


        if(!enable_control){
            PLOGD << "enable_control is not set";
            return ;
        }
        if(!is_all_alive || !is_all_initialised){
            PLOGD << "not is_all_alive or not is_all_initialised";
            return;
        }

        PLOGD <<"control_wheel :[enable]: " <<enable;

        forward_vel_1 *= enable;
        rotate_angle_1 *= enable;
        forward_vel_2 *= enable;
        rotate_angle_2 *= enable;
        PLOGD <<"control_wheel 1:[forward_vel, rotate_angle]: " << forward_vel_1 << ", " << rotate_angle_1;
        PLOGD <<"control_wheel 2:[forward_vel, rotate_angle]: " << forward_vel_2 << ", " << rotate_angle_2;



        *(int*)(&driver_command_msg_channel_1[0].data[0]) = forward_vel_1 / SteerConfigArray[0].forward_speed_k;
        *(int*)(&driver_command_msg_channel_1[1].data[0]) = (rotate_angle_1 - SteerConfigArray[0].rot_angle_b + SteerConfigArray[0].rot_angle_abs_offset)/SteerConfigArray[0].rot_angle_k + SteerConfigArray[0].rot_angle_abs_offset;

        *(int*)(&driver_command_msg_channel_2[0].data[0]) = forward_vel_2 / SteerConfigArray[1].forward_speed_k;
        *(int*)(&driver_command_msg_channel_2[1].data[0]) = (rotate_angle_2 - SteerConfigArray[1].rot_angle_b + SteerConfigArray[1].rot_angle_abs_offset)/SteerConfigArray[1].rot_angle_k + SteerConfigArray[1].rot_angle_abs_offset;

        PLOGD << "send can data to channel 1";
        driver_command_msg_channel_1[0].print();
        driver_command_msg_channel_1[1].print();


        PLOGD << "send can data to channel 2";
        driver_command_msg_channel_2[0].print();
        driver_command_msg_channel_2[1].print();

        device.send(driver_command_msg_channel_1.data(),driver_command_msg_channel_1.size(),0);
        device.send(driver_command_msg_channel_2.data(),driver_command_msg_channel_2.size(),1);


        if(!enable) {

            disable_forward_driver(1);
            disable_rotate_driver(2);

            disable_forward_driver(4);
            disable_rotate_driver(5);

        }

    };

    bool state_update = false;
    auto update_wheel_forward = [&](size_t driver_id, const kaco::Message& message){

        PLOGD << "receive [" << driver_id << "]";

        int num = * (int*)(&message.data[0]);

        std::bitset<16> fault_code = * (int*)(&message.data[4]);

        SteerConfigArray[driver_id].actual_forward_vel = num * SteerConfigArray[driver_id].forward_speed_k;
        state_update = true;

        PLOGD << "actual_forward_vel: " << SteerConfigArray[driver_id].actual_forward_vel;
    };

    auto update_wheel_rotate = [&](size_t driver_id, const kaco::Message& message){
        PLOGD << "receive [" << driver_id << "]";
        int num = * (int*)(&message.data[0]);

        std::bitset<16> fault_code = * (int*)(&message.data[4]);

        float rot_angle = num * SteerConfigArray[driver_id].rot_angle_k +  SteerConfigArray[driver_id].rot_angle_b;

        SteerConfigArray[driver_id].addAngle(rot_angle);
        state_update = true;
        PLOGD << "actual_rot_angle: " << rot_angle;

    };
    auto update_wheel_rotate_abs = [&](size_t driver_id, const kaco::Message& message){
        PLOGD << "receive [" << driver_id << "]";

        int num = * (int*)(&message.data[0]);

        float rot_abs_angle = num * SteerConfigArray[driver_id].rot_angle_abs_k +  SteerConfigArray[driver_id].rot_angle_abs_b;

        SteerConfigArray[driver_id].addAngleAbs(rot_abs_angle);
        PLOGD <<  "actual_rot_abs_angle: " << rot_abs_angle;


    };

    auto update_wheel_rotate_abs_calib = [&](size_t driver_id){

        SteerConfigArray[driver_id].angleCalib();
    };



    // motor
    // wheel 1: forward 1, rotate 2
    // wheel 2: forward 4, rotate 5

    // absolute encoder
    //
    // node id 3, 6




    for(u_int8_t i = 0 ; i < 2; i++){
        uint8_t forward_driver_id = 3*i + 1;
        uint8_t rotate_driver_id = 3*i + 2;
        uint8_t encoder_id = 3*i + 3;


        device.pdo.add_pdo_received_callback(0x180 + forward_driver_id, [i,&update_wheel_forward](const kaco::Message& message){

            std::cout << "forward : " << int(message.get_node_id())  <<std::endl;
            message.print();
            update_wheel_forward(i,message);

        });

        device.pdo.add_pdo_received_callback(0x180 + rotate_driver_id, [i,&update_wheel_rotate](const kaco::Message& message){
            std::cout << "rotate : " << int(message.get_node_id())  <<std::endl;
            message.print();
            update_wheel_rotate(i,message);
        });

        device.pdo.add_pdo_received_callback(0x180 + encoder_id, [i,&update_wheel_rotate_abs](const kaco::Message& message){

            std::cout << "encoder : " << int(message.get_node_id())  <<std::endl;
            message.print();
            update_wheel_rotate_abs(i,message);
        });

    }

    // can forward
    std::vector<std::vector<common_message::CanMessage>> can_channel_forward_msg(2);
    std::vector<std::vector<kaco::Message>> can_channel_send_msg(2);



    for(size_t i = 0 ; i < CanForwardConfigArray.size();i++){
        auto& c = CanForwardConfigArray[i];

        if(c.channel_id <0 || c.channel_id>1 || c.recv_cob_id.empty() || c.pub_topic.empty() || c.sub_topic.empty() ){
            continue;
        }

        // ros

//        rosMessageManager.add_channel<common_message::Odometry>("PUB:odom:200");
        std::string sub_topic = absl::StrFormat("SUB:%s:100", c.sub_topic.c_str());
        rosMessageManager.add_channel<std::vector<common_message::CanMessage>>(sub_topic.c_str());

        std::string pub_topic = absl::StrFormat("PUB:%s:100", c.pub_topic.c_str());
        rosMessageManager.add_channel<std::vector<common_message::CanMessage>>(pub_topic.c_str());



        //can
#if 0

        for(size_t  j = 0 ; j < c.recv_cob_id.size();j++){
            device.pdo.add_pdo_received_callback(c.recv_cob_id[j], [&c,&can_channel_forward_msg](const kaco::Message& message){

                std::cout << "can forward : " << int(message.get_node_id())  <<std::endl;
                message.print();
                common_message::CanMessage msg;
                msg.id = message.cob_id;
                msg.is_rtr = message.rtr;
                msg.is_extended = message.ext;
                msg.is_error = false;
                msg.dlc = message.len;
                std::copy(std::begin(message.data), std::end(message.data), std::begin(msg.data));
                can_channel_forward_msg[c.channel_id].push_back(msg);


            });


        }
#endif

        device.register_receive_callback([&c,&can_channel_forward_msg](const kaco::Message& message){

            auto it = std::find(c.recv_cob_id.begin(), c.recv_cob_id.end(),message.cob_id);
            if(it ==c.recv_cob_id.end() ){

                return ;
            }else{
                PLOGD << "recv data" << std::endl;

                common_message::CanMessage msg;
                msg.id = message.cob_id;
                msg.is_rtr = message.rtr;
                msg.is_extended = message.ext;
                msg.is_error = false;
                msg.dlc = message.len;
                std::copy(std::begin(message.data), std::end(message.data), std::begin(msg.data));
                can_channel_forward_msg[c.channel_id].push_back(msg);
            }

        });
        auto can_message_sub_cb =[&c,&can_channel_send_msg](void* data){
            std::vector<common_message::CanMessage> * data_ptr = static_cast<std::vector<common_message::CanMessage>*>(data);

            kaco::Message msg;
            PLOGD << "recv data" << std::endl;

            for(size_t i = 0 ; i < data_ptr->size();i++){
                msg.cob_id = data_ptr->at(i).id;
                msg.rtr = data_ptr->at(i).is_rtr;
                msg.ext = data_ptr->at(i).is_extended;
                msg.len = data_ptr->at(i).dlc;
                std::copy(std::begin(data_ptr->at(i).data), std::end(data_ptr->at(i).data), std::begin(msg.data));
                can_channel_send_msg[c.channel_id].push_back(msg);
            }

        };
        taskManager.addTask([&rosMessageManager,&c,&device,&can_channel_send_msg,can_message_sub_cb]{


            rosMessageManager.recv_message(c.sub_topic.c_str(),10,0.001, can_message_sub_cb);

            if(can_channel_send_msg[c.channel_id].empty()){
                return true;
            }

            device.send( can_channel_send_msg[c.channel_id].data(), can_channel_send_msg[c.channel_id].size(),c.channel_id);
            can_channel_send_msg[c.channel_id].clear();

            return true;
        },100*1000,2);
        taskManager.addTask([&rosMessageManager,&c,&can_channel_forward_msg]{
            if(can_channel_forward_msg[c.channel_id].empty()){
                return true;
            }

            rosMessageManager.send_message(c.pub_topic.c_str(),&can_channel_forward_msg[c.channel_id], 1, 0.1 );
            can_channel_forward_msg[c.channel_id].clear();
            return true;
        },100*1000,2);

    }
    // controller
    control::DoubleSteerController controller;


    //
    control::SteerWheelBase wheel_1;
    wheel_1.enable_rot = true;
    wheel_1.mount_position_x = SteerConfigArray[0].mount_x;
    wheel_1.mount_position_y = SteerConfigArray[0].mount_y;

    wheel_1.max_forward_acc = forward_vel_acc;
    wheel_1.max_rot_vel = rotate_angle_vel;


    control::SteerWheelBase wheel_2;
    wheel_2.enable_rot = true;
    wheel_2.mount_position_x = SteerConfigArray[1].mount_x;
    wheel_2.mount_position_y = SteerConfigArray[1].mount_y;

    wheel_2.max_forward_acc = forward_vel_acc;
    wheel_2.max_rot_vel = rotate_angle_vel;

    controller.set_wheel(wheel_1,wheel_2);

    controller.smooth_stop = smooth_stop;




    if(!use_sim){
        if (!device.start(busname, baudrate)) {
            std::cout << "Starting device failed." << std::endl;
            return EXIT_FAILURE;
        }

        taskManager.addTask([&]{

            check_all_state();
            // if not operational, send nmt start
//            if(!is_all_alive){
//                device.nmt.broadcast_nmt_message(kaco::NMT::Command::start_node,send_to_channel_1);
//                device.nmt.broadcast_nmt_message(kaco::NMT::Command::start_node,send_to_channel_2);
//            }
            device.nmt.broadcast_nmt_message(kaco::NMT::Command::start_node,send_to_channel_1);
            device.nmt.broadcast_nmt_message(kaco::NMT::Command::start_node,send_to_channel_2);

            if(is_any_fault){

            }

            return true;
        },1000*1000,0);

        taskManager.addTask([&]{
            device.nmt.send_sync_message(send_to_channel_1);
            device.nmt.send_sync_message(send_to_channel_2);
            device.recv_message(message_buffer,0, message_buffer.size());
            device.recv_message(message_buffer,1, message_buffer.size());
            return true;
        },5*1000,0);


        // initializer
        taskManager.addTask([&]{

            if(is_all_alive){


                if(!is_all_initialise_triggered){

                    is_all_initialise_triggered = true;
                    taskManager.addTask([&]{


                        if(!is_all_initialised){
                            initialise_forward_driver(1);
                            initialise_rotate_driver(2);

                            initialise_forward_driver(4);
                            initialise_rotate_driver(5);

                            taskManager.addTask([&]{
                                is_all_initialised = true;
                                return false;
                            }, initialise_wait_s*1000*1000, 0);
                        }
                        
                        


                        return false;
                        },100*1000, 0);


                }


            }else{
                return true;
            }


            return true;
        }, 10*1000,1);


    }else{

        is_all_alive = true;
        is_all_initialised = true;
        is_any_fault = false;

    }

    // rot
    taskManager.addTask([&]{
        is_rot_sensor_ready = true;
        for(size_t i = 0 ; i < SteerConfigArray.size();i++){
            is_rot_sensor_ready = SteerConfigArray[i].isAngleCalib();
        }
        return true;
    });


    // odom
    taskManager.addTask([&]{

        PLOGD << "is_all_alive: " << is_all_alive;
        PLOGD << "is_rot_sensor_ready: " << is_rot_sensor_ready;
        PLOGD << "is_all_initialised: " << is_all_initialised;


        if(is_all_alive && is_rot_sensor_ready && is_all_initialised){


            PLOGD << "update driver0 feedback[forward_vel, rotate_angle], " << SteerConfigArray[0].actual_forward_vel << ", " << SteerConfigArray[0].actual_rot_abs_angle;
            PLOGD << "update driver1 feedback[forward_vel, rotate_angle], " << SteerConfigArray[1].actual_forward_vel << ", " << SteerConfigArray[1].actual_rot_abs_angle;

            {
                controller.updateState(SteerConfigArray[0].actual_forward_vel,SteerConfigArray[0].actual_rot_abs_angle, SteerConfigArray[1].actual_forward_vel,SteerConfigArray[1].actual_rot_abs_angle);
                state_update = false;
            }



            odom.header.stamp = common::FromUnixNow();


            const transform::Transform2d& robot_pose = controller.getPosition();
            odom.pose.pose.position.x = robot_pose.x();
            odom.pose.pose.position.y = robot_pose.y();
            odom.pose.pose.position.z = 0.0;

            math::yaw_to_quaternion(robot_pose.yaw(), odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w );

            odom.twist.twist.linear.x = controller.getActualForwardVel() * std::cos(controller.getActualForwardAngle());
            odom.twist.twist.linear.y = controller.getActualForwardVel() * std::sin(controller.getActualForwardAngle());

            odom.twist.twist.angular.z = controller.getActualRotateVel();



            rosMessageManager.send_message("odom",&odom, 1, 0.1 );


            {
                auto now = common::FromUnixNow();
                // mqtt
                if(mqtt_msg_status_cnt < FRAME_NUM ){
                    float ts = common::ToMicroSeconds(now - record_start_time) * 1e-6;
                    mqtt_msg_status[mqtt_msg_status_cnt*STATUS_NUM + 0 ] = ts;

                    mqtt_msg_status[mqtt_msg_status_cnt*STATUS_NUM + 1 ] = controller.m_steer_wheel[0].getCommandForwardVel();
                    mqtt_msg_status[mqtt_msg_status_cnt*STATUS_NUM + 2 ] = controller.m_steer_wheel[0].actual_forward_vel;
                    mqtt_msg_status[mqtt_msg_status_cnt*STATUS_NUM + 3 ] = controller.m_steer_wheel[0].getCommandRotateAngle();
                    mqtt_msg_status[mqtt_msg_status_cnt*STATUS_NUM + 4 ] = controller.m_steer_wheel[0].actual_rot_angle;
                    mqtt_msg_status[mqtt_msg_status_cnt*STATUS_NUM + 5 ] = controller.m_steer_wheel[0].steer_constrain_vel;

                    mqtt_msg_status[mqtt_msg_status_cnt*STATUS_NUM + 6 ] = controller.m_steer_wheel[1].getCommandForwardVel();
                    mqtt_msg_status[mqtt_msg_status_cnt*STATUS_NUM + 7 ] = controller.m_steer_wheel[1].actual_forward_vel;
                    mqtt_msg_status[mqtt_msg_status_cnt*STATUS_NUM + 8 ] = controller.m_steer_wheel[1].getCommandRotateAngle();
                    mqtt_msg_status[mqtt_msg_status_cnt*STATUS_NUM + 9 ] = controller.m_steer_wheel[1].actual_rot_angle;
                    mqtt_msg_status[mqtt_msg_status_cnt*STATUS_NUM + 10 ] = controller.m_steer_wheel[1].steer_constrain_vel;

                }
                mqtt_msg_status_cnt++;


                if(mqtt_msg_status_cnt == FRAME_NUM){
                    record_start_time = now;

                    for(int i = 0; i <FRAME_NUM*STATUS_NUM; i++ ){
                        mqtt_msg_status_int[i] =  int16_t (mqtt_msg_status[i]*1e4) ;
                    }

                    mqtt_binary_str.assign((char*)(&mqtt_msg_status_int[0])  , FRAME_NUM*STATUS_NUM*2 );

                    mqtt_base64_str= absl::Base64Escape(mqtt_binary_str);

                    mqttMessageManager.send_message(mqtt_pub_topic.c_str(), &mqtt_base64_str,1,0.1);


                    mqtt_msg_status_cnt = 0;
                }
            }


        }else{

        }


        return true;
    }, 15*1000,1);

    // cmd_vel
    int cmd_vel_timeout_counter = 0;
    if(!enable_fix_command){
        PLOGD << "add cmd_vel command";
        taskManager.addTask([&]{


            if((is_all_alive && is_all_initialised && is_rot_sensor_ready && ! is_any_fault)){
                rosMessageManager.recv_message("control_cmd_vel",10,0.001, cmd_vel_sub_cb);

                auto now = common::FromUnixNow();
                if( common::ToMillSeconds(now - recv_cmd_vel_time) > cmd_vel_timeout_ms){

                    PLOGD << "cmd_vel timeout, cmd_vel reset";
                    recv_cmd_vel_msg.linear.x = 0.0;
                    recv_cmd_vel_msg.linear.y = 0.0;
                    recv_cmd_vel_msg.angular.z = 0.0;
                    recv_cmd_vel_time = now;

                }
                if(recv_new_cmd_vel){
                    recv_new_cmd_vel = false;
                    PLOGD<< "recv cmd_vel:[rotate_vel,forward_vel]: " << recv_cmd_vel_msg.angular.z << ", " << recv_cmd_vel_msg.linear.x;
                }






                // controller compute
                {
                    float vel = std::sqrt(recv_cmd_vel_msg.linear.x*recv_cmd_vel_msg.linear.x + recv_cmd_vel_msg.linear.y*recv_cmd_vel_msg.linear.y);

                    if (std::abs(vel) < 0.001){
                        controller.cmd_vel(0.0,recv_cmd_vel_msg.angular.z);
                    }else{
                        float vel_angle = std::atan2(recv_cmd_vel_msg.linear.y, recv_cmd_vel_msg.linear.x);
                        controller.cmd_vel(vel,recv_cmd_vel_msg.angular.z,vel_angle);
                    }
                }

                controller.interpolate();

                PLOGD<< "get target   :[rotate_angle,forward_vel]\n" << controller.m_steer_wheel[0].target_rot_angle << ", "
                     << controller.m_steer_wheel[0].target_forward_vel << ", "
                     << controller.m_steer_wheel[1].target_rot_angle << ", "
                     << controller.m_steer_wheel[1].target_forward_vel;

                PLOGD << "get command:[rotate_angle,forward_vel]\n" << controller.m_steer_wheel[0].command_rotate_angle << ", "
                      << controller.m_steer_wheel[0].command_forward_vel << ", "
                      << controller.m_steer_wheel[1].command_rotate_angle << ", "
                      << controller.m_steer_wheel[1].command_forward_vel;

                PLOGD << "get interpolate command:[rotate_angle,forward_vel]\n" << controller.m_steer_wheel[0].getCommandRotateAngle() << ", "
                      << controller.m_steer_wheel[0].getCommandForwardVel() << ", "
                      << controller.m_steer_wheel[1].getCommandRotateAngle() << ", "
                      << controller.m_steer_wheel[1].getCommandForwardVel();




                // can send

                control_wheel(true, controller.m_steer_wheel[0].getCommandForwardVel(),controller.m_steer_wheel[0].getCommandRotateAngle() ,controller.m_steer_wheel[1].getCommandForwardVel(),controller.m_steer_wheel[1].getCommandRotateAngle() );

                return true;
            }else{
                PLOGD << "driver not ready, cmd_vel reset";

                recv_cmd_vel_msg.linear.x = 0.0;
                recv_cmd_vel_msg.linear.y = 0.0;
                recv_cmd_vel_msg.angular.z = 0.0;


                return true;

            }



            return true;
        }, 10*1000,1);

    }else{
        PLOGD << "add fix command";

        taskManager.addTask([&control_wheel,&command_array,&mqttMessageManager,&mqtt_sub_topic,&lua_cb]{

            mqttMessageManager.recv_message(mqtt_sub_topic.c_str(),10,0.01, lua_cb);

            command_array[1] = std::max(std::min(1.918888889f, command_array[1]),-1.918888889f);
            command_array[3] = std::max(std::min(1.918888889f, command_array[3]),-1.918888889f);

            PLOGD << "send fix command: " << command_array[0] << ", " << command_array[1] << ", " <<command_array[2] << ", " << command_array[3];

            control_wheel(true, command_array[0],command_array[1],command_array[2],command_array[3] );

            return true;
        } ,100*1000,0);

    }




    // add timeout or fault callback
    // add odom computation
    // add cmd_vel computation

    // driver initializer

    PLOGD << "start loop, " << program_run;
    while (program_run && rosMessageManager.is_running()){

        taskManager.call();



    }
    PLOGD << "end loop, " << program_run;

    control_wheel(false, 0.0, 0.0 , 0.0, 0.0);



    mqttMessageManager.stop();

    rosMessageManager.stop();
    device.stop();


    return 0;

}
int test_roboteq(int argc, char** argv){

    plog::RollingFileAppender<plog::CsvFormatter> fileAppender("roboteq.csv", 8000000, 10); // Create the 1st appender.
    plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender; // Create the 2nd appender.
    plog::init(plog::debug, &fileAppender).addAppender(&consoleAppender); // Initialize the logger with the both appenders.


    std::cout << "run lua" << std::endl;
    lua_State* L = luaL_newstate();
    sol::state_view lua(L);
    lua.open_libraries(sol::lib::base, sol::lib::package,sol::lib::os, sol::lib::table, sol::lib::jit,sol::lib::coroutine);

    lua.script(R"(
print("start lua state")


)");

    //


    const int CONTROLLER_NUM = 2;
    const int MAX_ROT_COMMAND_RAW = 2.2e6;

    //
    PLOGD <<  "start roboteq controller";

    std::string mqtt_server = "192.168.1.101";


    std::atomic_bool program_run(true);
    auto my_handler = common::fnptr<void(int)>([&](int sig){ std::cout << "get sig " << sig;program_run = false;});
    common::set_signal_handler(my_handler);



    bool get_help = false;


    bool calib_done = false;
    bool do_calib = false;
    int calib_id = -1;
    float left_limit_angle = 0.0;
    float right_limit_angle = 0.0;
    int left_limit_feedback = 0;
    int right_limit_feedback = 0;

    int cmd_vel_timeout_ms = 100;
    int feedback_timeout_ms = 100;

    std::string mqtt_pub_topic = "hello_robot";

    std::string mqtt_sub_topic = "hello_lua";

//    bool left_limit_reach = false;
//    bool right_limit_reach = false;


    /*

     angle = k * feedback + b

     k = angle_change/feed_back_change
     b = angle - k*feedback


     vel = k* feedback
     k = vel/feedback

     减速比21
     直径 0.167m
     额定转速3000RPM
     3000RPM * k = 3000RPM *  PI * 0.167m /(60s * 21)
     k = 3.14*0.167/(60*21)
     */



    std::vector<std::string> command;
    std::string exe_name;

    std::vector<std::string> extra_command;

    std::string config_file;

    bool enable_control = false;
    std::vector<int> used_controller_id;

    {
        enable_control = true;
        used_controller_id = std::vector<int>{0,1};
    }

    bool use_sim = false;

    // controller
    bool smooth_stop = true;

    bool enable_fix_command = false;
    std::vector<float> command_array;


    float forward_vel_acc = 0.8;
    float rotate_angle_vel = 0.8;

    auto cli
            =  lyra::exe_name(exe_name)
               | lyra::help(get_help)
               | lyra::opt( do_calib )
              ["-d"]["--do_calib"]
                      ("Do the calibration" )
                      | lyra::opt(use_sim)["-s"]["--use_sim"]("use simulation")
               | lyra::opt(smooth_stop,"smooth_stop")["-S"]["--smooth_stop"]("smooth_stop")

               | lyra::opt(forward_vel_acc,"forward_vel_acc")["-F"]["--forward_vel_acc"]("forward_vel_acc")
                 | lyra::opt(rotate_angle_vel,"rotate_angle_vel")["-R"]["--rotate_angle_vel"]("rotate_angle_vel")
                 | lyra::opt( config_file, "config_file" )["-c"]["--config_file"]("config_file")
                | lyra::opt( calib_id, "calib_id" )
                               ["-i"]["--calib_id"]
                                       ("calib_id, use 0 or 1 ")

                  | lyra::opt( left_limit_angle, "left_limit_angle" )
                                                 ["-l"]["--left_limit_angle"]
                                                         ("left_limit_angle")
                    | lyra::opt( right_limit_angle, "right_limit_angle" )
                                                                     ["-r"]["--right_limit_angle"]
                                                                             ("right_limit_angle")
                      | lyra::opt(mqtt_server,"mqtt_server")["-M"]["--mqtt_server"]("mqtt_server")
                    | lyra::opt(mqtt_pub_topic,"mqtt_pub_topic")["-m"]["--mqtt"]("mqtt topic")
                    | lyra::opt(cmd_vel_timeout_ms,"cmd_vel_timeout_ms")
                    ["-t"]["--cmdvel_timeout"]("cmd_vel_timeout_ms")
                      | lyra::opt(feedback_timeout_ms,"feedback_timeout_ms")
                      ["-f"]["--feedback_timeout"]("feedback_timeout_ms")
                      //---
                      | lyra::group().sequential()
                      | lyra::opt(enable_control, "ON|OFF")["--control"]
                      | lyra::arg(used_controller_id, "ids")

                      //---
                      //---
                        | lyra::group().sequential()
                          | lyra::opt(enable_fix_command, "ON|OFF")["--command"]
                            | lyra::arg(command_array, "command_array")
                      //---


                      | lyra::arg(command, "command")("The command, and arguments, to attempt to run.")
    ;

    auto result = cli.parse( { argc, argv } );
    if(get_help){
        std::cout << cli << std::endl;
        return 0;

    }


    if ( !result )
    {
        std::cerr << "Error in command line: " << result.message() << std::endl;
        return 0;
    }

    if(!enable_control || used_controller_id.empty()){
        std::cout << "not in enable_control mode" << std::endl;
        return 0;

    }
    if(enable_fix_command &&command_array.size() !=4 ){
        std::cout << " enable_fix_command , command_array.size() = " << command_array.size() << std::endl;
        return 0;

    }

    if(enable_fix_command){
        command_array[1] = std::max(std::min(1.918888889f, command_array[1]),-1.918888889f);
        command_array[3] = std::max(std::min(1.918888889f, command_array[3]),-1.918888889f);

        for(auto& c: command_array){
            PLOGD << "fix command: " << c;

        }

    }
    for(auto i : used_controller_id){
        if(i <0 || i >= CONTROLLER_NUM){
            std::cout << i <<  " is no in range = [0 , " << CONTROLLER_NUM-1 << "]" << std::endl;
            return 0;
        }

    }

    if(do_calib && !(calib_id == 0 || calib_id == 1 )){

        std::cout << "calib_id not set in calib mode" << std::endl;
        return 0;
    }
    if(config_file.empty()){
        std::cout << "config_file not set" << std::endl;
        return 0;

    }

    std::cout << "do_calib " <<  do_calib << std::endl;
    std::cout << "config_file " <<  config_file << std::endl;
    std::cout << "calib_id " <<  calib_id << std::endl;
    std::cout << "left_limit_angle " <<  left_limit_angle << std::endl;
    std::cout << "right_limit_angle " <<  right_limit_angle << std::endl;

    std::cout << "command " <<  command.size() << std::endl;

    for(auto& c : command){
        std::cout << "command " <<  c << std::endl;

    }

    extra_command.emplace_back(exe_name);
    std::copy(command.begin(), command.end(), std::back_inserter(extra_command));


    int ARGC = extra_command.size();
    const char *  ARGS[50];
    for(int i = 0 ; i <ARGC;i++ ){
        ARGS[i] = extra_command[i].data();
    }

    const char** ARGS1 = const_cast<const char **>(ARGS);


    //"server:broker-cn.emqx.io"

    std::string mqtt_server_str = absl::StrFormat("server:%s", mqtt_server.c_str());

    PLOGD << "mqtt_server_str: " << mqtt_server_str;

//    char* mqtt_config_str[] = {"server:broker-cn.emqx.io", "port:1883","keep_alive:30","clean_session:1"};
//    char* mqtt_config_str[] = {"server:172.30.254.199", "port:1883","keep_alive:30","clean_session:1"};
    const char* mqtt_config_str[] = {mqtt_server_str.c_str(), "port:1883","keep_alive:30","clean_session:1"};

    message::MqttMessageManager mqttMessageManager;
    char* mqtt_client_id = "cpp_test";
    mqttMessageManager.open(mqtt_client_id,4, mqtt_config_str);
//    char topic_arg[100];
    std::string mqtt_pub_topic_arg = absl::StrFormat("PUB:%s:0", mqtt_pub_topic.c_str());
    std::string mqtt_sub_topic_arg = absl::StrFormat("SUB:%s:0", mqtt_sub_topic.c_str());

    mqttMessageManager.add_channel<std::string>(mqtt_pub_topic_arg.c_str());

    mqttMessageManager.add_channel<std::string>(mqtt_sub_topic_arg.c_str());



    auto simple_handler_lambda =
            [](lua_State*, const sol::protected_function_result& result) {
                // You can just pass it through to let the
                // call-site handle it
                std::cout << "\n******LUA INFO:\nAn exception occurred in a function, here's what it says ";
                sol::error err = result;
                std::cout << "call failed, sol::error::what() is " <<  err.what() << std::endl;
//                MLOGW("%s", what.c_str());
                return result;
            };


    // lua
    lua.set_function("move_command",[&](float forward_vel_1, float  rotate_angle_1, float forward_vel_2, float  rotate_angle_2){

        command_array.resize(4);
        command_array[0] = forward_vel_1;
        command_array[1] = rotate_angle_1;
        command_array[2] = forward_vel_2;
        command_array[3] = rotate_angle_2;

        command_array[1] = std::max(std::min(1.918888889f, command_array[1]),-1.918888889f);
        command_array[3] = std::max(std::min(1.918888889f, command_array[3]),-1.918888889f);

    });

    lua.set_function("hello",[&](int v){

        std::cout << " hello " << v << " from lua" << std::endl;

    });




    std::string decode_str;

    auto lua_cb =[&](void* data){
        std::string* data_ptr = static_cast<std::string*>(data);
        std::cout << "test123 recv msg: " << *data_ptr << std::endl;
        bool decode_ok =  absl::Base64Unescape(*data_ptr,&decode_str);
        std::cout << "test123 recv decode_str: " << decode_str << std::endl;

        if(decode_ok){

            auto result = lua.script(
                    decode_str,
                    simple_handler_lambda);
            if (result.valid()) {
                std::cout << "run lua script ok"
                          << std::endl;
            }
            else {
                std::cout << "run lua script fail"
                          << std::endl;
            }



        }
    };
    mqttMessageManager.start("");

    std::cout << "mqtt start done" << std::endl;


    message::RosMessageManager rosMessageManager;

    rosMessageManager.open("driver_comm", ARGC, ARGS1 );


    rosMessageManager.add_channel<common_message::Twist>("SUB:control_cmd_vel:10");
    rosMessageManager.add_channel<common_message::Odometry>("PUB:odom:200");

    std::vector<common_message::Twist> cmd_vel_msgs(10);
    common_message::Odometry odom;
    odom.twist.twist.linear.z = 0.0f;

    odom.twist.twist.angular.x = 0.0f;
    odom.twist.twist.angular.y = 0.0f;

    odom.header.frame_id.assign("odom");
    odom.child_frame_id.assign("base_link");

    odom.pose.covariance[0] = 0.1;
    odom.pose.covariance[7] = 0.1;
    odom.pose.covariance[14] = 0.1;
    odom.pose.covariance[21] = 0.1;
    odom.pose.covariance[28] = 0.1;

    odom.pose.covariance[35] = 0.1;
    odom.twist.covariance[0] = 0.1;
    odom.twist.covariance[7] = 0.1;

    odom.twist.covariance[14] = 0.1;
    odom.twist.covariance[21] = 0.1;
    odom.twist.covariance[28] = 0.1;

    odom.twist.covariance[35] = 0.1;


    // todo: callback may fail if topic and datetye not matched
    bool recv_new_cmd_vel = false;
    common_message::Twist recv_cmd_vel_msg;
    common::Time  recv_cmd_vel_time = common::FromUnixNow();
    auto cmd_vel_sub_cb =[&recv_new_cmd_vel,&recv_cmd_vel_msg,&recv_cmd_vel_time](void* data){
        common_message::Twist * data_ptr = static_cast<common_message::Twist*>(data);
        recv_cmd_vel_msg = * data_ptr;
//        std::cout << "odom_sub_cb recv msg: " << data_ptr->linear.x << ", " << data_ptr->angular.z << std::endl;
        recv_cmd_vel_time = common::FromUnixNow();
        recv_new_cmd_vel = true;
    };



    std::cout << "ros start done" << std::endl;




    std::vector<ControllerConfig> SteerConfigArray;
    std::vector<ControllerState >SteerStateArray;
    std::vector<kaco::Message> SteerCommand(CONTROLLER_NUM);

#if 0
    {
        Steer1_Config.rot_angle_k = 10.2;
        Steer1_Config.rot_angle_b = 120;
        Steer1_Config.forward_speed_k = 5.6;

        nlohmann::json j1 = Steer1_Config;

        std::cout << "j1:\n" << j1.dump()<< std::endl;

        Steer2_Config = j1;
        Steer2_Config.forward_speed_k = 4.5;
        SteerConfigArray.push_back(Steer1_Config);
        SteerConfigArray.push_back(Steer2_Config);

        nlohmann::json j2 = SteerConfigArray;
        std::cout << "j2:\n" << j2.dump()<< std::endl;
        SteerConfigArray.clear();


    }
#endif



    if(!config_file.empty()){

        std::ifstream ifs(config_file.c_str());
        if(ifs.is_open()){

            try{
                nlohmann::json jf = nlohmann::json::parse(ifs);

                std::cout << "config_file: " << config_file<< "\n jf :\n" << jf << std::endl;

                SteerConfigArray = jf;
            }catch(nlohmann::detail::parse_error & e) {

                std::cout << e.what() << std::endl;
                std::cout << "json format error in config_file: " << config_file<<  std::endl;

                return 0;

            }


        }else{
            std::cout << "config_file not exist" << std::endl;
            return 0;

        }
        if(SteerConfigArray.size() != CONTROLLER_NUM){
            std::cout << "config_file error: SteerConfigArray.size() = " << SteerConfigArray.size() << std::endl;
            return 0;
        }

        SteerStateArray.resize(SteerConfigArray.size());

        for(int i = 0 ; i < CONTROLLER_NUM;i++){
            SteerStateArray[i].config = SteerConfigArray[i];
        }
    }


//    IntInt Steer1_Forward_Rot_State_common ;
//    IntInt Steer1_Forward_Rot_Command_common ;
//    BitArray Steer1_Signal;
//
//    IntInt Steer2_Forward_Rot_State_common ;
//    IntInt Steer2_Forward_Rot_Command_common ;
//    BitArray Steer2_Signal;

#if 0
    {

        BitArray Steer1_Signal;

        kaco::Message Steer1_BitSignal_recv = {0x201,0,0,8,{0x03,0x01,0x00,0x00,0x00,0,0,01}};

        std::cout << "Steer1_BitSignal_recv\n: ";
        Steer1_BitSignal_recv.print();

        to_common(Steer1_BitSignal_recv, Steer1_Signal);


        std::cout << "get bits:\n"<< Steer1_Signal.data.to_string()<< std::endl;

        for(int i = 0 ; i < 64;i++){
            std::cout << int(i) << ": " << Steer1_Signal.data[i] << "\n";
        }
        std::cout << std::endl;

    }
#endif


    /*
     calibration process

                             can node 1 --> channel1: forward motor
                                                      send: 1. speed feedback
                                                      recv: 1. speed command
                                       |
                                       |
                                       |--> channel2: rotate motor, need calibration
                                                     send: 1. position feedback
                                                           2. find_home flag
                                                           3. left right limit sensor
                                                     recv: 1. position command

     computer


     while loop:

         1.get can message

         2. check node heartbeat state

         3. check node channel2 feedback ,

         4.



     end

     */





    // Set the name of your CAN bus. "slcan0" is a common bus name
    // for the first SocketCAN device on a Linux system.
    const std::string busname = "USBCAN-II";

    // Set the baudrate of your CAN bus. Most drivers support the values
    // "1M", "500K", "125K", "100K", "50K", "20K", "10K" and "5K".
    const std::string baudrate = "500K";

    // -------------- //
    // Initialization //
    // -------------- //

    common::TaskManager taskManager;

    std::cout << "This is an example which shows the usage of the Core library." << std::endl;

    // Create core.
    kaco::Core device;



#if 0
    {
        IntInt Steer1_Forward_Rot_State_common ;
        kaco::Message Steer1_Forward_Rot_recv = {0x201,0,0,8,{0x64,0,0,0,0xc8,0,0,0}};

        Steer1_Forward_Rot_State_common = to_common(Steer1_Forward_Rot_recv);

        std::cout << "Steer1_Forward_Rot_State_common: " << Steer1_Forward_Rot_State_common.num_1 << ", " << Steer1_Forward_Rot_State_common.num_2 << std::endl;

        Steer1_Forward_Rot_State_common.num_1 = 101;
        Steer1_Forward_Rot_State_common.num_2 = 201;

        from_common(Steer1_Forward_Rot_State_common, Steer1_Forward_Rot_recv);
        std::cout << "Steer1_Forward_Rot_State_common: " << Steer1_Forward_Rot_State_common.num_1 << ", " << Steer1_Forward_Rot_State_common.num_2 << std::endl;
        std::cout << "Steer1_Forward_Rot_recv\n: ";
        Steer1_Forward_Rot_recv.print();
    }
#endif
    device.register_receive_callback([](const  kaco::Message& v){
//        std::cout  << "device get data" << std::endl;
//        v.print();
    });

    for(int i = 0; i < CONTROLLER_NUM;i++){
        {
            // add command pdo
            uint16_t cob_id = 0x201+i;
            SteerStateArray[i].command_message =  kaco::Message{cob_id,0,0,8,{0,0,0,0,0,0,0,0}};

        }
        {
            // add receive feedback pdo
            uint16_t cob_id = 0x181+i;
            device.pdo.add_pdo_received_callback(cob_id,[&SteerStateArray,i,cob_id,&SteerConfigArray](const  kaco::Message& v){
//                std::cout << "device get pdo data from " << cob_id << std::endl;
                SteerStateArray[i].state = to_common(v);
                SteerStateArray[i].recv_message = v ;

                SteerStateArray[i].forward_vel = SteerConfigArray[i].forward_speed_k *SteerStateArray[i].state.num_1;
                SteerStateArray[i].rot_angle = SteerConfigArray[i].rot_angle_k *SteerStateArray[i].state.num_2 + SteerConfigArray[i].rot_angle_b ;

                SteerStateArray[i].msg_num++;
                SteerStateArray[i].last_update_time = common::FromUnixNow();
                SteerStateArray[i].updated = true;

            });

        }

        {
            // add sensor bit pdo
            uint16_t cob_id = 0x281+i;

            device.pdo.add_pdo_received_callback(cob_id,[&SteerStateArray,i,cob_id](const  kaco::Message& v){

//                std::cout << "device get pdo data from "<< cob_id << std::endl;
                to_common(v,SteerStateArray[i].signal);
                SteerStateArray[i].recv_state_message = v ;

                SteerStateArray[i].left_limit = SteerStateArray[i].signal.data[0];
                SteerStateArray[i].right_limit = SteerStateArray[i].signal.data[1];
                SteerStateArray[i].find_home = SteerStateArray[i].signal.data[2];

                /*
                     bool is_OverHeat = false;
    bool is_OverVolt = false;
    bool is_UnderVolt = false;
    bool is_Short = false;
    bool is_EStop = false;
    bool is_Motor_Sensor = false;
    bool is_MOSFail = false;
    bool is_DefConfig = false;
    bool is_STOFault = false;
                 */


                SteerStateArray[i].fault_flag = * (u_int64_t *)(&v.data[1]);
                SteerStateArray[i].is_OverHeat = SteerStateArray[i].fault_flag[15];
                SteerStateArray[i].is_OverVolt = SteerStateArray[i].fault_flag[14];
                SteerStateArray[i].is_UnderVolt = SteerStateArray[i].fault_flag[13];
                SteerStateArray[i].is_Short = SteerStateArray[i].fault_flag[12];
                SteerStateArray[i].is_EStop = SteerStateArray[i].fault_flag[11];
                SteerStateArray[i].is_Motor_Sensor = SteerStateArray[i].fault_flag[10];
                SteerStateArray[i].is_MOSFail = SteerStateArray[i].fault_flag[9];
                SteerStateArray[i].is_DefConfig = SteerStateArray[i].fault_flag[8];
                SteerStateArray[i].is_STOFault = SteerStateArray[i].fault_flag[7];
                SteerStateArray[i].ready = SteerStateArray[i].find_home && (SteerStateArray[i].fault_flag == 0);

            });
        }


    }



    //
    control::DoubleSteerController controller;
    control::SteerWheelBase wheel_1;
    wheel_1.enable_rot = true;
    wheel_1.mount_position_x = SteerStateArray[0].config.mount_x;
    wheel_1.mount_position_y = SteerStateArray[0].config.mount_y;

    wheel_1.max_forward_acc = forward_vel_acc;
    wheel_1.max_rot_vel = rotate_angle_vel;


    control::SteerWheelBase wheel_2;
    wheel_2.enable_rot = true;
    wheel_2.mount_position_x = SteerStateArray[1].config.mount_x;
    wheel_2.mount_position_y = SteerStateArray[1].config.mount_y;

    wheel_2.max_forward_acc = forward_vel_acc;
    wheel_2.max_rot_vel = rotate_angle_vel;

    controller.set_wheel(wheel_1,wheel_2);

    controller.smooth_stop = smooth_stop;

    float constrain_angle = std::atan2(SteerStateArray[1].config.mount_y - SteerStateArray[0].config.mount_y, SteerStateArray[1].config.mount_x - SteerStateArray[0].config.mount_x);

    float constrain_angle_vec[2] = { std::cos(constrain_angle),std::sin(constrain_angle)};



    lua.set_function("update_rot",[&](float k1, float b1, float k2, float b2){


        SteerStateArray[0].config.rot_angle_k = k1;
        SteerConfigArray[0].rot_angle_k = k1;

        SteerStateArray[0].config.rot_angle_b = b1;
        SteerConfigArray[0].rot_angle_b = b1;

        SteerStateArray[1].config.rot_angle_k = k2;
        SteerConfigArray[1].rot_angle_k = k2;

        SteerStateArray[1].config.rot_angle_b = b2;
        SteerConfigArray[1].rot_angle_b = b2;

    });
    lua.set_function("update_forward",[&](float k1,  float k2 ){


        SteerStateArray[0].config.forward_speed_k = k1;
        SteerConfigArray[0].forward_speed_k = k1;

        SteerStateArray[1].config.forward_speed_k = k2;
        SteerConfigArray[1].forward_speed_k = k2;


    });

    // calibration

    const int MAX_MESSAGE_SIZE = 1000;
    std::vector<kaco::Message> message_buffer(MAX_MESSAGE_SIZE);


    control::SmoothSimulator smoothSimulator;
    smoothSimulator.set_wheel(wheel_1,wheel_2);


    if(!use_sim){

        if (!device.start(busname, baudrate)) {
            std::cout << "Starting device failed." << std::endl;
            return EXIT_FAILURE;
        }

        taskManager.addTask([&]{
            device.recv_message(message_buffer,0, message_buffer.size());
            return true;
        },10*1000,0);


    }else{

        taskManager.addTask([&]{

            smoothSimulator.updateState(controller.m_steer_wheel[0].getCommandForwardVel(),controller.m_steer_wheel[0].getCommandRotateAngle(),
                                        controller.m_steer_wheel[1].getCommandForwardVel(),controller.m_steer_wheel[1].getCommandRotateAngle());

            for(int i = 0; i < CONTROLLER_NUM;i++){

                SteerStateArray[i].forward_vel = smoothSimulator.m_steer_wheel[i].actual_forward_vel  ;
                SteerStateArray[i].rot_angle = smoothSimulator.m_steer_wheel[i].actual_rot_angle;
                SteerStateArray[i].msg_num++;
                SteerStateArray[i].last_update_time = common::FromUnixNow();
                SteerStateArray[i].updated = true;


                SteerStateArray[i].find_home =  true;
                SteerStateArray[i].ready =  true;

            }

            return true;
        },5*1000,0);



    }







    if(do_calib){
        taskManager.addTask([&]{

            if(do_calib && !calib_done){

                const int id = calib_id ;

                if(!SteerStateArray[id].ready){
                    std::cout << "wait controller " << id   << "ready" << std::endl;
                    return true;
                }
                if (SteerStateArray[id].left_limit)
                    left_limit_feedback = SteerStateArray[id].state.num_2;

                if (SteerStateArray[id].right_limit)
                    right_limit_feedback = SteerStateArray[id].state.num_2;

                if(right_limit_feedback == 0){
                    SteerStateArray[id].command.num_2 = MAX_ROT_COMMAND_RAW;
                }else if(left_limit_feedback == 0){

                    SteerStateArray[id].command.num_2 = -MAX_ROT_COMMAND_RAW;
                }
                if(left_limit_feedback!=0 && right_limit_feedback!=0){
                    SteerStateArray[id].command.num_2 = 0.5*(left_limit_feedback +right_limit_feedback );
                }

                from_common(SteerStateArray[id].command, SteerStateArray[id].command_message);
                device.send(SteerStateArray[id].command_message,0);

                std::cout << "****************\n SteerStateArray[id].command_message\n ";
                SteerStateArray[id].command_message.print();


                /*

     angle = k * feedback + b

     k = angle_change/feed_back_change
     b = angle - k*feedback
     */

                if(left_limit_feedback!=0 && right_limit_feedback!=0){

                    SteerConfigArray[id].rot_angle_k = (left_limit_angle - right_limit_angle)/(left_limit_feedback - right_limit_feedback);
                    SteerConfigArray[id].rot_angle_b = left_limit_angle - SteerConfigArray[id].rot_angle_k*left_limit_feedback;
                    calib_done = true;
                    if(calib_done){
                        for(int i = 0 ; i < CONTROLLER_NUM;i++){
                            SteerStateArray[i].config = SteerConfigArray[i];
                        }

                        std::cout << "calib_k: " << SteerConfigArray[id].rot_angle_k  <<std::endl;
                        std::cout << "calib_b: " <<  SteerConfigArray[id].rot_angle_b <<std::endl;

                        nlohmann::json j3 = SteerConfigArray;
                        std::cout << "j3:\n" << j3.dump()<< std::endl;


                        std::ofstream ofs(config_file.c_str());
                        ofs << j3.dump() << std::endl;
                        ofs.close();

                    }
                }


            }



            return true;
        },500*1000,10);
    }



    const int STATUS_NUM = 11;
    const int FRAME_NUM = 60;

    common::Time record_start_time = common::FromUnixNow();
    int mqtt_msg_status_cnt = 0;
    std::vector<float> mqtt_msg_status(STATUS_NUM*FRAME_NUM);
    std::vector<int16_t> mqtt_msg_status_int(STATUS_NUM*FRAME_NUM);

    std::string mqtt_binary_str;
    std::string mqtt_base64_str;

    common::Suspend suspend;


    bool send_first = false;
    while (program_run && rosMessageManager.is_running()){
         taskManager.call();
         if(do_calib){
             continue;
         }

        bool is_fault = false;

         bool all_find_home = true;
         bool all_no_fault = true;
         bool skip = false;
//        for(int i = 0 ; i < CONTROLLER_NUM ;i++){

        for(auto& i :used_controller_id){
            all_find_home =  all_find_home && (SteerStateArray[i].msg_num >0)  &&
                             SteerStateArray[i].find_home;

            all_no_fault = all_no_fault && (SteerStateArray[i].fault_flag == 0);

        }

         if(!all_find_home) {
             std::cout<< "controller not ready or not receive feedback"<< std::endl;
             for(auto& i :used_controller_id){
                  PLOGD   << i  <<   ", recv_num: " << SteerStateArray[i].msg_num  << ", find home: "<<   SteerStateArray[i].find_home << ", fault: "<< SteerStateArray[i].fault_flag << ", is_short: " << SteerStateArray[i].is_Short;

             }
             continue;
         }

         bool timeout = false;
         common::Time now = common::FromUnixNow();
//        for(int i = 0 ; i < CONTROLLER_NUM ;i++){
         for(auto& i :used_controller_id){

            if(  common::ToMillSeconds(now - SteerStateArray[i].last_update_time) > feedback_timeout_ms){
                std::cout<< "controller"<< i <<  " feedback timeout"<< std::endl;
                SteerStateArray[i].msg_num = 0;
                SteerStateArray[i].ready = false;

                timeout = true;
            }
        }
        if(timeout){
            std::cout<< "controller feedback timeout"<< std::endl;
            skip = true;
            // reset cmd_vel


        }


        {

            for(auto& i :used_controller_id){
                if( timeout || (SteerStateArray[i].msg_num >0 && SteerStateArray[i].fault_flag != 0) || ( SteerStateArray[i].msg_num >0 && std::abs(controller.m_steer_wheel[i].actual_forward_vel ) < 0.0001   &&  std::abs( controller.m_steer_wheel[i].getCommandForwardVel() - controller.m_steer_wheel[i].actual_forward_vel ) > 0.0001 ) ){
                    is_fault = true;

                    std::copy(std::begin(SteerStateArray[i].recv_message.data),std::end(SteerStateArray[i].recv_message.data), std::begin(SteerStateArray[i].command_message.data));

                    for(int j = 0 ; j < 4;j++){
                        SteerStateArray[i].command_message.data[j] = 0;
                    }
                    device.send(SteerStateArray[i].command_message,0);
                    PLOGD << "send reset cmd";
                }

            }

        }



        rosMessageManager.recv_message("control_cmd_vel",10,0.001, cmd_vel_sub_cb);
        if(timeout || !all_no_fault){

            {
                recv_cmd_vel_msg.linear.x = 0.0;
                recv_cmd_vel_msg.linear.y = 0.0;
                recv_cmd_vel_msg.angular.z = 0.0;
            }

            for(auto& i :used_controller_id){

                SteerStateArray[i].forward_vel = 0.0;
//                SteerStateArray[i].forward_vel = 0.0;
            }
        }



        bool update = false;
        for(auto& i :used_controller_id){
            update =  update ||  SteerStateArray[i].updated;
        }
       if(update) {
           for(auto& i :used_controller_id){
               SteerStateArray[i].updated = false;
           }

           {
               //compute odom
               controller.updateState(SteerStateArray[0].forward_vel,SteerStateArray[0].rot_angle, SteerStateArray[1].forward_vel,SteerStateArray[1].rot_angle);

               PLOGD << "recv can message from controller1: " << int(SteerStateArray[0].recv_message.data[0]) << ", "
                     << int(SteerStateArray[0].recv_message.data[1]) << ", "
                     << int(SteerStateArray[0].recv_message.data[2]) << ", "
                     << int(SteerStateArray[0].recv_message.data[3]) << ", "
                     << int(SteerStateArray[0].recv_message.data[4]) << ", "
                     << int(SteerStateArray[0].recv_message.data[5]) << ", "
                     << int(SteerStateArray[0].recv_message.data[6]) << ", "
                     << int(SteerStateArray[0].recv_message.data[7]);

               PLOGD << "recv can message from controller2: " << int(SteerStateArray[1].recv_message.data[0]) << ", "
                     << int(SteerStateArray[1].recv_message.data[1]) << ", "
                     << int(SteerStateArray[1].recv_message.data[2]) << ", "
                     << int(SteerStateArray[1].recv_message.data[3]) << ", "
                     << int(SteerStateArray[1].recv_message.data[4]) << ", "
                     << int(SteerStateArray[1].recv_message.data[5]) << ", "
                     << int(SteerStateArray[1].recv_message.data[6]) << ", "
                     << int(SteerStateArray[1].recv_message.data[7]);

               PLOGD << "recv can state from controller1: " << int(SteerStateArray[0].recv_state_message.data[0]) << ", "
                     << int(SteerStateArray[0].recv_state_message.data[1]) << ", "
                     << int(SteerStateArray[0].recv_state_message.data[2]) << ", "
                     << int(SteerStateArray[0].recv_state_message.data[3]) << ", "
                     << int(SteerStateArray[0].recv_state_message.data[4]) << ", "
                     << int(SteerStateArray[0].recv_state_message.data[5]) << ", "
                     << int(SteerStateArray[0].recv_state_message.data[6]) << ", "
                     << int(SteerStateArray[0].recv_state_message.data[7]);

               PLOGD << "recv can state from controller2: " << int(SteerStateArray[1].recv_state_message.data[0]) << ", "
                     << int(SteerStateArray[1].recv_state_message.data[1]) << ", "
                     << int(SteerStateArray[1].recv_state_message.data[2]) << ", "
                     << int(SteerStateArray[1].recv_state_message.data[3]) << ", "
                     << int(SteerStateArray[1].recv_state_message.data[4]) << ", "
                     << int(SteerStateArray[1].recv_state_message.data[5]) << ", "
                     << int(SteerStateArray[1].recv_state_message.data[6]) << ", "
                     << int(SteerStateArray[1].recv_state_message.data[7]);
               PLOGD << "get feedback:[rotate_angle,forward_vel]\n"  << SteerStateArray[0].rot_angle << ", "<< SteerStateArray[0].forward_vel << ", "
                     << SteerStateArray[1].rot_angle<< ", " << SteerStateArray[1].forward_vel ;

               std::cout << "get steer_constrain_vel:\n" << controller.m_steer_wheel[0].steer_constrain_vel << ", " << controller.m_steer_wheel[1].steer_constrain_vel << std::endl;

               odom.header.stamp = common::FromUnixNow();


               const transform::Transform2d& robot_pose = controller.getPosition();
               odom.pose.pose.position.x = robot_pose.x();
               odom.pose.pose.position.y = robot_pose.y();
               odom.pose.pose.position.z = 0.0;

               math::yaw_to_quaternion(robot_pose.yaw(), odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w );

               odom.twist.twist.linear.x = controller.getActualForwardVel() * std::cos(controller.getActualForwardAngle());
               odom.twist.twist.linear.y = controller.getActualForwardVel() * std::sin(controller.getActualForwardAngle());

               odom.twist.twist.angular.z = controller.getActualRotateVel();



               rosMessageManager.send_message("odom",&odom, 1, 0.1 );
           }

           if( common::ToMillSeconds(now - recv_cmd_vel_time) > cmd_vel_timeout_ms){


               // reset cmd_vel
               {
                   recv_cmd_vel_msg.linear.x = 0.0;
                   recv_cmd_vel_msg.linear.y = 0.0;
                   recv_cmd_vel_msg.angular.z = 0.0;
               }

               controller.m_steer_wheel[0].command_forward_vel = 0.0;
               controller.m_steer_wheel[0].command_rotate_angle = 0.0;

               controller.m_steer_wheel[1].command_forward_vel = 0.0;
               controller.m_steer_wheel[1].command_rotate_angle = 0.0;


               float noise = common::uniform_real<float>(-0.001,0.001);
               SteerStateArray[0].createCommand(controller.m_steer_wheel[0].command_forward_vel,controller.m_steer_wheel[0].command_rotate_angle + noise );
               SteerCommand[0] = SteerStateArray[0].command_message;
//               device.send(SteerStateArray[0].command_message,0);

               SteerStateArray[1].createCommand(controller.m_steer_wheel[1].command_forward_vel,controller.m_steer_wheel[1].command_rotate_angle + noise);
               SteerCommand[1] = SteerStateArray[1].command_message;
//               device.send(SteerStateArray[1].command_message,0);

               device.send(SteerCommand.data(),2,0);

               recv_cmd_vel_time = now;
               PLOGD << "send cmd_vel timeout stop cmd";

           }

           {
               // mqtt
               if(mqtt_msg_status_cnt < FRAME_NUM ){
                   float ts = common::ToMicroSeconds(now - record_start_time) * 1e-6;
                   mqtt_msg_status[mqtt_msg_status_cnt*STATUS_NUM + 0 ] = ts;

                   mqtt_msg_status[mqtt_msg_status_cnt*STATUS_NUM + 1 ] = controller.m_steer_wheel[0].getCommandForwardVel();
                   mqtt_msg_status[mqtt_msg_status_cnt*STATUS_NUM + 2 ] = controller.m_steer_wheel[0].actual_forward_vel;
                   mqtt_msg_status[mqtt_msg_status_cnt*STATUS_NUM + 3 ] = controller.m_steer_wheel[0].getCommandRotateAngle();
                   mqtt_msg_status[mqtt_msg_status_cnt*STATUS_NUM + 4 ] = controller.m_steer_wheel[0].actual_rot_angle;
                   mqtt_msg_status[mqtt_msg_status_cnt*STATUS_NUM + 5 ] = controller.m_steer_wheel[0].steer_constrain_vel;

                   mqtt_msg_status[mqtt_msg_status_cnt*STATUS_NUM + 6 ] = controller.m_steer_wheel[1].getCommandForwardVel();
                   mqtt_msg_status[mqtt_msg_status_cnt*STATUS_NUM + 7 ] = controller.m_steer_wheel[1].actual_forward_vel;
                   mqtt_msg_status[mqtt_msg_status_cnt*STATUS_NUM + 8 ] = controller.m_steer_wheel[1].getCommandRotateAngle();
                   mqtt_msg_status[mqtt_msg_status_cnt*STATUS_NUM + 9 ] = controller.m_steer_wheel[1].actual_rot_angle;
                   mqtt_msg_status[mqtt_msg_status_cnt*STATUS_NUM + 10 ] = controller.m_steer_wheel[1].steer_constrain_vel;

               }
               mqtt_msg_status_cnt++;


               if(mqtt_msg_status_cnt == FRAME_NUM){
                   record_start_time = now;

                   for(int i = 0; i <FRAME_NUM*STATUS_NUM; i++ ){
                       mqtt_msg_status_int[i] =  int16_t (mqtt_msg_status[i]*1e4) ;
                   }

                   mqtt_binary_str.assign((char*)(&mqtt_msg_status_int[0])  , FRAME_NUM*STATUS_NUM*2 );

                   mqtt_base64_str= absl::Base64Escape(mqtt_binary_str);

                   mqttMessageManager.send_message(mqtt_pub_topic.c_str(), &mqtt_base64_str,1,0.1);


                   mqtt_msg_status_cnt = 0;
               }
           }


           if(is_fault){
           }

           {


               if(enable_fix_command){

                   mqttMessageManager.recv_message(mqtt_sub_topic.c_str(),10,0.01, lua_cb);


                   controller.m_steer_wheel[0].getCommandForwardVel() =  command_array[0];
                   controller.m_steer_wheel[0].getCommandRotateAngle() =  command_array[1];
                   controller.m_steer_wheel[1].getCommandForwardVel() =  command_array[2];
                   controller.m_steer_wheel[1].getCommandRotateAngle() =  command_array[3];

                   if(std::abs(controller.m_steer_wheel[0].actual_rot_angle - controller.m_steer_wheel[0].getCommandRotateAngle()) > 0.01){
                       controller.m_steer_wheel[0].getCommandForwardVel() = 0.0f;
                   }


                   if(std::abs(controller.m_steer_wheel[1].actual_rot_angle - controller.m_steer_wheel[1].getCommandRotateAngle()) > 0.01){
                       controller.m_steer_wheel[1].getCommandForwardVel() = 0.0f;
                   }

                   if(
                           (std::abs(controller.m_steer_wheel[0].getCommandForwardVel()) < 0.01  || std::abs(controller.m_steer_wheel[1].getCommandForwardVel()) < 0.01)
                           && (std::abs(controller.m_steer_wheel[0].actual_forward_vel) > 0.01  || std::abs(controller.m_steer_wheel[1].actual_forward_vel) > 0.01)


                   ){
//                       controller.m_steer_wheel[0].getCommandRotateAngle() =  controller.m_steer_wheel[0].actual_rot_angle;
//                       controller.m_steer_wheel[1].getCommandRotateAngle() =  controller.m_steer_wheel[1].actual_rot_angle;

                   }



                   PLOGD << "send fix command: " << command_array[0] << ", "<< command_array[1]  << ", "<< command_array[2] << ", " << command_array[3] ;

                   SteerStateArray[0].createCommand(controller.m_steer_wheel[0].getCommandForwardVel() ,controller.m_steer_wheel[0].getCommandRotateAngle());
//               device.send(SteerStateArray[0].command_message,0);

                   SteerStateArray[1].createCommand( controller.m_steer_wheel[1].getCommandForwardVel() ,controller.m_steer_wheel[1].getCommandRotateAngle());

                   SteerCommand[0] = SteerStateArray[0].command_message;

                   SteerCommand[1] = SteerStateArray[1].command_message;
//               device.send(SteerStateArray[1].command_message,0);

                   device.send(SteerCommand.data(),2,0);




                   PLOGD << "send can message to controller1: " << int(SteerStateArray[0].command_message.data[0]) << ", "
                         << int(SteerStateArray[0].command_message.data[1]) << ", "
                         << int(SteerStateArray[0].command_message.data[2]) << ", "
                         << int(SteerStateArray[0].command_message.data[3]) << ", "
                         << int(SteerStateArray[0].command_message.data[4]) << ", "
                         << int(SteerStateArray[0].command_message.data[5]) << ", "
                         << int(SteerStateArray[0].command_message.data[6]) << ", "
                         << int(SteerStateArray[0].command_message.data[7]);

                   PLOGD << "send can message to controller2: " << int(SteerStateArray[1].command_message.data[0]) << ", "
                         << int(SteerStateArray[1].command_message.data[1]) << ", "
                         << int(SteerStateArray[1].command_message.data[2]) << ", "
                         << int(SteerStateArray[1].command_message.data[3]) << ", "
                         << int(SteerStateArray[1].command_message.data[4]) << ", "
                         << int(SteerStateArray[1].command_message.data[5]) << ", "
                         << int(SteerStateArray[1].command_message.data[6]) << ", "
                         << int(SteerStateArray[1].command_message.data[7]);

                   continue;
               }

           }
           if(recv_new_cmd_vel){
               recv_new_cmd_vel = false;
               PLOGD<< "recv cmd_vel:[rotate_vel,forward_vel]: " << recv_cmd_vel_msg.angular.z << ", " << recv_cmd_vel_msg.linear.x;
           }


           {
               {
                   float vel = std::sqrt(recv_cmd_vel_msg.linear.x*recv_cmd_vel_msg.linear.x + recv_cmd_vel_msg.linear.y*recv_cmd_vel_msg.linear.y);

                   if (std::abs(vel) < 0.001){
                       controller.cmd_vel(0.0,recv_cmd_vel_msg.angular.z);
                   }else{
                       float vel_angle = std::atan2(recv_cmd_vel_msg.linear.y, recv_cmd_vel_msg.linear.x);
                       controller.cmd_vel(vel,recv_cmd_vel_msg.angular.z,vel_angle);
                   }
               }

//               controller.cmd_vel(recv_cmd_vel_msg.linear.x,recv_cmd_vel_msg.angular.z);




//               controller.m_steer_wheel[0].actual_rot_angle = SteerStateArray[0].rot_angle;
//               controller.m_steer_wheel[0].actual_forward_vel = SteerStateArray[0].forward_vel;
//               controller.m_steer_wheel[0].updateState( SteerStateArray[0].forward_vel,SteerStateArray[0].rot_angle );

//               controller.m_steer_wheel[1].actual_rot_angle = SteerStateArray[1].rot_angle;
//               controller.m_steer_wheel[1].actual_forward_vel = SteerStateArray[1].forward_vel;
//               controller.m_steer_wheel[1].updateState( SteerStateArray[1].forward_vel,SteerStateArray[1].rot_angle );




               controller.interpolate();

               PLOGD<< "recv Fault flag from controller1 : " << SteerStateArray[0].fault_flag.to_string();
               PLOGD<< "recv Fault flag from controller2 : " << SteerStateArray[1].fault_flag.to_string();




               PLOGD<< "get target   :[rotate_angle,forward_vel]\n" << controller.m_steer_wheel[0].target_rot_angle << ", "
                         << controller.m_steer_wheel[0].target_forward_vel << ", "
                         << controller.m_steer_wheel[1].target_rot_angle << ", "
                         << controller.m_steer_wheel[1].target_forward_vel;

               PLOGD << "get command:[rotate_angle,forward_vel]\n" << controller.m_steer_wheel[0].command_rotate_angle << ", "
                         << controller.m_steer_wheel[0].command_forward_vel << ", "
                         << controller.m_steer_wheel[1].command_rotate_angle << ", "
                         << controller.m_steer_wheel[1].command_forward_vel;

               PLOGD << "get interpolate command:[rotate_angle,forward_vel]\n" << controller.m_steer_wheel[0].getCommandRotateAngle() << ", "
                     << controller.m_steer_wheel[0].getCommandForwardVel() << ", "
                     << controller.m_steer_wheel[1].getCommandRotateAngle() << ", "
                     << controller.m_steer_wheel[1].getCommandForwardVel();








               SteerStateArray[0].createCommand(controller.m_steer_wheel[0].getCommandForwardVel(),controller.m_steer_wheel[0].getCommandRotateAngle() );
//               device.send(SteerStateArray[0].command_message,0);

               SteerStateArray[1].createCommand(controller.m_steer_wheel[1].getCommandForwardVel(),controller.m_steer_wheel[1].getCommandRotateAngle() );

               SteerCommand[0] = SteerStateArray[0].command_message;

               SteerCommand[1] = SteerStateArray[1].command_message;
//               device.send(SteerStateArray[1].command_message,0);

               device.send(SteerCommand.data(),2,0);




               PLOGD << "send can message to controller1: " << int(SteerStateArray[0].command_message.data[0]) << ", "
                     << int(SteerStateArray[0].command_message.data[1]) << ", "
                     << int(SteerStateArray[0].command_message.data[2]) << ", "
                     << int(SteerStateArray[0].command_message.data[3]) << ", "
                     << int(SteerStateArray[0].command_message.data[4]) << ", "
                     << int(SteerStateArray[0].command_message.data[5]) << ", "
                     << int(SteerStateArray[0].command_message.data[6]) << ", "
                     << int(SteerStateArray[0].command_message.data[7]);

               PLOGD << "send can message to controller2: " << int(SteerStateArray[1].command_message.data[0]) << ", "
                     << int(SteerStateArray[1].command_message.data[1]) << ", "
                     << int(SteerStateArray[1].command_message.data[2]) << ", "
                     << int(SteerStateArray[1].command_message.data[3]) << ", "
                     << int(SteerStateArray[1].command_message.data[4]) << ", "
                     << int(SteerStateArray[1].command_message.data[5]) << ", "
                     << int(SteerStateArray[1].command_message.data[6]) << ", "
                     << int(SteerStateArray[1].command_message.data[7]);


           }
       }else{
           suspend.sleep(5);
       }



    }

    {
        {
            command_array.resize(4);
            command_array[0] = 0.0;
            command_array[1] = 0.0;
            command_array[2] = 0.0;
            command_array[3] = 0.0;

            PLOGD << "send fix command: " << command_array[0] << ", "<< command_array[1]  << ", "<< command_array[0] << ", " << command_array[1] ;

            SteerStateArray[0].createCommand(command_array[0],command_array[1]);
//               device.send(SteerStateArray[0].command_message,0);

            SteerStateArray[1].createCommand( command_array[2],command_array[3] );

            SteerCommand[0] = SteerStateArray[0].command_message;

            SteerCommand[1] = SteerStateArray[1].command_message;
//               device.send(SteerStateArray[1].command_message,0);

            device.send(SteerCommand.data(),2,0);




            PLOGD << "send can message to controller1: " << int(SteerStateArray[0].command_message.data[0]) << ", "
                  << int(SteerStateArray[0].command_message.data[1]) << ", "
                  << int(SteerStateArray[0].command_message.data[2]) << ", "
                  << int(SteerStateArray[0].command_message.data[3]) << ", "
                  << int(SteerStateArray[0].command_message.data[4]) << ", "
                  << int(SteerStateArray[0].command_message.data[5]) << ", "
                  << int(SteerStateArray[0].command_message.data[6]) << ", "
                  << int(SteerStateArray[0].command_message.data[7]);

            PLOGD << "send can message to controller2: " << int(SteerStateArray[1].command_message.data[0]) << ", "
                  << int(SteerStateArray[1].command_message.data[1]) << ", "
                  << int(SteerStateArray[1].command_message.data[2]) << ", "
                  << int(SteerStateArray[1].command_message.data[3]) << ", "
                  << int(SteerStateArray[1].command_message.data[4]) << ", "
                  << int(SteerStateArray[1].command_message.data[5]) << ", "
                  << int(SteerStateArray[1].command_message.data[6]) << ", "
                  << int(SteerStateArray[1].command_message.data[7]);

        }
    }

    if(do_calib ){
        std::cout << "do_calib " <<  do_calib << std::endl;
        std::cout << "calib_id " <<  calib_id << std::endl;
        if(calib_done){
            const int id = calib_id ;

            std::cout << "calib_k: " << SteerConfigArray[id].rot_angle_k  <<std::endl;
            std::cout << "calib_b: " <<  SteerConfigArray[id].rot_angle_b <<std::endl;

            nlohmann::json j3 = SteerConfigArray;
            std::cout << "j3:\n" << j3.dump()<< std::endl;


            std::ofstream ofs(config_file.c_str());
            ofs << j3.dump() << std::endl;
            ofs.close();

        }else{
            std::cout << "calib failed"  <<std::endl;
        }
    }

    mqttMessageManager.stop();

    rosMessageManager.stop();
    device.stop();

    return 0;
}


int test_linde() {
    //void sigintHandler(int sig)
    std::atomic_bool program_run(true);
    auto my_handler = common::fnptr<void(int)>([&](int sig){ std::cout << "get sig " << sig;program_run = false;});
    common::set_signal_handler(my_handler);

	// ----------- //
	// Preferences //
	// ----------- //

	// The node ID of the slave we want to communicate with.
	const uint8_t node_id = 2;

	// Set the name of your CAN bus. "slcan0" is a common bus name
	// for the first SocketCAN device on a Linux system.
	const std::string busname = "USBCAN-II";

	// Set the baudrate of your CAN bus. Most drivers support the values
	// "1M", "500K", "125K", "100K", "50K", "20K", "10K" and "5K".
	const std::string baudrate = "500K:500K";

	// Set the object dictionary index to write to (download).
	// Here: CiA-401 (I/O device) digital output.
	const uint16_t index = 0x6200;

	// Alternative: CiA-402 (motor) control word:
	//const uint16_t index = 0x6040;

	// Set the object dictionary sub-index to write to (download).
	// Here: CiA-401 (I/O device) digital output - second byte.
	const uint8_t subindex = 0x01;

	// Alternative: CiA-402 (motor) control word:
	//const uint8_t subindex = 0x00;

	// Set the data to write (download).
	const std::vector<uint8_t> data { 0x7F };

	// Alternative: CiA-402 (motor) control word has two bytes. Command: shutdown (little-endian!)
	//const std::vector<uint8_t> data { 0x06, 0x00 };

	// -------------- //
	// Initialization //
	// -------------- //

    common::TaskManager taskManager;

    std::cout << "This is an example which shows the usage of the Core library." << std::endl;

	// Create core.
    kaco::Core device;

	// This will be set to true by the callback below.
	bool found_node = false;

	std::cout << "Registering a callback which is called when a device is detected via NMT..." << std::endl;


    device.register_receive_callback([](const  kaco::Message& v){
        std::cout  << "device get data" << std::endl;
        v.print();
    });
    device.pdo.add_pdo_received_callback(0x181,[](const  kaco::Message& v){

        std::cout << "device get pdo data" << std::endl;

    });





    unsigned short  AgwPDO_crc_start_value = 0xFFFF;

    kaco::Message HeartbtAgw={0x715,0,0,1,{0x05,0,0,0,0,0,0,0}} ;

    kaco::Message AgwPDO1 = {0x195,0,0,8,{0,0,0,0,0,0,0,0}};

    kaco::Message AgwPDO2 = {0x295,0,0,8,{0,0,0,0,0,0,0,0}};

    kaco::Message AgwPDO3 = {0x395,0,0,8,{0,0,0,0,0,0,0,0}};

    kaco::Message AgwPDO4 = {0x495,0,0,8,{0,0,0,0,0,0,0,0}};


    auto& AgwPDO1_auto_manual = AgwPDO1.data[0];
    auto& AgwPDO1_lift = AgwPDO1.data[2];
    auto& AgwPDO1_cnt = AgwPDO1.data[5];
    uint8_t AgwPDO1_cnt_real = 0;


    auto& AgwCmdTrVelocity_h = AgwPDO2.data[0];
    auto& AgwCmdTrVelocity_l = AgwPDO2.data[1];
    auto& AgwCmdSteerAngle_h = AgwPDO2.data[2];
    auto& AgwCmdSteerAngle_l = AgwPDO2.data[3];
    auto& AgwCmdStop  = AgwPDO2.data[4];
    auto& AgwPDO2_cnt = AgwPDO2.data[5];
    uint8_t AgwPDO2_cnt_real = 0;


    auto& AgwCmdID_DeviceVers = AgwPDO4.data[0];
    auto& AgwCmdID_CanIntVers = AgwPDO4.data[1];

    auto& AgwStatusFlag =  AgwPDO4.data[4];
    auto& AgwPDO4_cnt = AgwPDO4.data[5];
    uint8_t AgwPDO4_cnt_real = 0;
    AgwCmdID_CanIntVers = 0x100>>8 | 0x02<<2;
    AgwStatusFlag = 0;




    AgwPDO1_auto_manual = 1;
    AgwPDO1_cnt = 0x1;
    AgwPDO1_cnt = AgwPDO1_cnt_real<<4;

    AgwCmdStop = 0x3;
    AgwPDO2_cnt = AgwPDO2_cnt_real<<4;

    AgwPDO4_cnt = AgwPDO4_cnt_real<<4;

    auto AgwPDO_do_crc = [AgwPDO_crc_start_value](kaco::Message & msg){

        unsigned char data[8];
        data[0] = (msg.cob_id >> 8*(1)) & (0xff);
        data[1] = (msg.cob_id >> 8*(0)) & (0xff);

        for(size_t i = 0 ; i < 6 ;i ++){
            data[i+2] = msg.data[i];
        }
        unsigned short  crc = common::crc16_compute_tab(data,8,AgwPDO_crc_start_value );
        msg.data[6] = (crc >> (8*0)) & 0xff; // low
        msg.data[7] = (crc >> (8*1)) & 0xff; // hight
    };

    auto AgwPDO_do_add_cnt = [](kaco::Message & msg){

    };


    std::cout << "AgwPDO1:\n" << std::endl;

    AgwPDO1.print();

    taskManager.addTask([&]{

        AgwPDO1_cnt = AgwPDO1_cnt_real<<4;
        AgwPDO2_cnt = AgwPDO2_cnt_real<<4;
        AgwPDO_do_crc(AgwPDO1);
        AgwPDO_do_crc(AgwPDO2);

        device.send(AgwPDO1,0);
        device.send(AgwPDO2,0);

        AgwPDO1_cnt_real++;
        AgwPDO2_cnt_real++;

        if(AgwPDO1_cnt_real > 0xf){
            AgwPDO1_cnt_real = 0;
        }

        if(AgwPDO2_cnt_real > 0xf){
            AgwPDO2_cnt_real = 0;
        }

        return true;
    },10*1000,0);

    taskManager.addTask([&]{
        device.send(AgwPDO3,0);
        return true;
    },400*1000,0);
    taskManager.addTask([&]{
        AgwPDO4_cnt = AgwPDO4_cnt_real<<4;
        AgwPDO_do_crc(AgwPDO4);
        device.send(AgwPDO4,0);
        AgwPDO4_cnt_real++;
        if(AgwPDO4_cnt_real > 0xf){
            AgwPDO4_cnt_real = 0;
        }

        return true;
    },40*1000,0);

    taskManager.addTask([&]{
        device.send(HeartbtAgw,0);
        return true;
    },40*1000,1);

    std::cout << "Starting Core (connect to the driver and start the receiver thread)..." << std::endl;

    if (!device.start(busname, baudrate)) {
        std::cout << "Starting device failed." << std::endl;
        return EXIT_FAILURE;
    }

    while (program_run){
        taskManager.call();
    }

    return 0;




    std::cout << "Starting device ok." << std::endl;

    std::vector<kaco::Message> recv_mesg_buffer(100);
    std::cout << "check device.m_handle : " << device.m_handle <<std::endl;

    int recv_num = 10;
    device.recv_message(recv_mesg_buffer,0,recv_num);


    auto send_to_channel_1 = [&](const auto& msg){
        device.send(msg,0);
        return 1;
    };
    auto send_to_channel_2 = [&](const auto& msg){
        device.send(msg,1);
        return 1;
    };

    device.nmt.send_sync_message(send_to_channel_1);
    device.nmt.send_sync_message(send_to_channel_2);

    device.nmt.send_nmt_message(0x0, kaco::NMT::Command::start_node,send_to_channel_1);
    device.nmt.send_nmt_message(0x0, kaco::NMT::Command::start_node,send_to_channel_2);

    kaco::Message command[2];
    std::vector<kaco::Message> command_group;
    device.send(command_group,0);



    /*
     thread 1:
         while loop

         1. timed task, every 10ms, send sync
         2. receive rpdo from both servo node
         3. send tpdo to both servo node, if no new motion command computed, send old data
         4. get heartbeat status
         5. check heartbeat status and timeout
         6. timout control


     thread 2:
     while loop
     1. get cmd_vel, compute

     */



    device.stop();

    return 0;
    kaco::Core core;

    core.register_receive_callback([](const  kaco::Message& v){
        std::cout << "get data" << std::endl;
    });

    if (!core.start(busname, baudrate)) {
        std::cout << "Starting core failed." << std::endl;
        return EXIT_FAILURE;
    }


	std::cout << "Asking all devices to reset. You don't need to do that, but it makes"
		<< " sure all slaves are in a reproducible state." << std::endl;
//	core.nmt.reset_all_nodes();
    core.nmt.broadcast_nmt_message(kaco::NMT::Command::reset_node);
    core.nmt.send_nmt_message(1, kaco::NMT::Command::start_node);
    core.nmt.broadcast_nmt_message(kaco::NMT::Command::start_node);



    {
        /* SYNC message */
        const kaco::Message message = { 0x80, false, 0,1, {0x1,0,0,0,0,0,0,0} };
        core.send(message);
        core.nmt.send_sync_message();

    }
//    core.nmt.send_nmt_message(0x01, kaco::NMT::Command::)

    core.pdo.add_pdo_received_callback(0x581,[](const  kaco::Message& v){

        std::cout << "get data" << std::endl;

    });
    core.pdo.send(0x181,{0xA5,0,0,0,0,0,0,0});
    {
        /* HEARTBEAT message */
        /*
         Each node, upon completion of Initialization, transmits a Boot-Up Frame, and then
transitions to PreOperational state. This Boot-Up frame is of the same COB-ID and
form as the HeartBeat frame (described below), having a COB-ID of 1792 (700h) + CAN
ID, and having one byte of data to convey the NMT State. The “NMT State” field is zero
to indicate boot-up.



         */

//        const kaco::Message message = { 0x701, false, 1, {static_cast<uint8_t>(kaco::NMT::NodeState::Pre_operational),0,0,0,0,0,0,0} };
//        core.send(message);

        core.nmt.send_heartbeat_message(0x1, kaco::NMT::NodeState::Initialisation);
        core.nmt.send_heartbeat_message(0x2, kaco::NMT::NodeState::Stopped);

    }




	// As an alternative you can request the slaves to announce
	// themselves:
	// core.nmt.discover_nodes();

	std::cout << "Giving the devices one second time to respond..." << std::endl;
    for(int i = 0 ; i < 3; i++){
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    core.stop();
    device.stop();

	// Check if the device has been detected (see the callback above).
	if (!found_node) {
		std::cout << "Node with ID " << (unsigned) node_id << " has not been found."<< std::endl;
		return EXIT_FAILURE;
	}

	std::cout << "Asking the device to start up..." << std::endl;
	core.nmt.send_nmt_message(node_id,kaco::NMT::Command::start_node);

	std::cout << "Giving the devices one second time to boot up..." << std::endl;
	std::this_thread::sleep_for(std::chrono::seconds(1));

	// ------------ //
	// Device usage //
	// ------------ //

	std::cout << "Writing to a dictionary entry (CANopen speech: \"download\")..." << std::endl;
	core.sdo.download(node_id, index, subindex, data.size(), data);

	std::cout << "Reading the device type (\"upload\" 0x1000)... Little-endian!" << std::endl;
	std::vector<uint8_t> device_type = core.sdo.upload(node_id,0x1000,0x0);
	for (uint8_t device_type_byte : device_type) {
		std::cout << "  byte 0x" << std::hex << (unsigned) device_type_byte << std::endl;
	}

	std::cout << "Reading the device name (\"upload\" 0x1008 - usually using segmented transfer)..." << std::endl;
	std::vector<uint8_t> device_name = core.sdo.upload(node_id,0x1008,0x0);
	std::string result(reinterpret_cast<char const*>(device_name.data()), device_name.size());
	std::cout << "  " << result << std::endl;


	std::cout << "Finished." << std::endl;
	return EXIT_SUCCESS;

}

int main(int argc, char** argv){
//    test_roboteq(argc,argv );
    test_tec(argc,argv);

    return 0;
}
