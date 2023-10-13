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
#include "common/stamped_buffer.h"
#include "math/BezierGenerator.h"
#include "nlohmann/json.hpp"

#include "message/crc.h"

#include "message/impl/ros/RosMessageManager.h"
#include "message/impl/mqtt/MqttMessageManager.h"

#include "message/MessageManager.h"


#include "control/MobileRobotController.h"
#include "control/MotionPlanner.h"


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

    float predict_control_s = 0.02f;
    float forward_reach_thresh = 0.01f;
    float rotate_reach_thresh = 0.01f;
    float action_timeout_s = 0.2f;

    float comm_timeout_s = 0.2f;



    float max_forward_vel = 0.5;
    float max_forward_acc = 0.8;
    float max_rotate_angle = 1.9;
    float max_rotate_vel = 3.0;
    float min_forward_vel = 0.05;



    float actual_rot_angle = 0.0;
    float actual_forward_vel = 0.0;
    float actual_rot_abs_angle = 0.0;

    common::ValueStampedBuffer<float> angle_buffer;
    common::ValueStampedBuffer<float> angle_abs_buffer;

    common::Time forward_time;
    common::Time rotate_time;
    bool forward_feedback_timeout = false;
    bool rotate_feedback_timeout = false;

    
    

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
            rot_angle_abs_offset = 0.0;
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
        PLOGD << "query angle: " << angle;
        PLOGD << "query angle_abs: " << angle_abs;


        is_rot_angle_calib = ok1 && ok2;
        if(is_rot_angle_calib){

            rot_angle_abs_offset = angle_abs - angle;
        }
        PLOGD << "query rot_angle_abs_offset: " << rot_angle_abs_offset;

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
        common::property(&ControllerConfig::mount_y, "mount_y"),
        common::property(&ControllerConfig::max_forward_vel, "max_forward_vel"),
        common::property(&ControllerConfig::max_forward_acc, "max_forward_acc"),
        common::property(&ControllerConfig::max_rotate_angle, "max_rotate_angle"),
        common::property(&ControllerConfig::max_rotate_vel, "max_rotate_vel"),
        common::property(&ControllerConfig::min_forward_vel, "min_forward_vel"),

        common::property(&ControllerConfig::action_timeout_s, "action_timeout_s"),
        common::property(&ControllerConfig::predict_control_s, "predict_control_s"),
        common::property(&ControllerConfig::forward_reach_thresh, "forward_reach_thresh"),
        common::property(&ControllerConfig::rotate_reach_thresh, "rotate_reach_thresh")


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

constexpr auto PlannerConfig_properties = std::make_tuple(
        common::property(&control::PlannerConfig::stable_pose_dist_tolerance, "stable_pose_dist_tolerance"),
        common::property(&control::PlannerConfig::stable_pose_angle_tolerance, "stable_pose_angle_tolerance"),

        common::property(&control::PlannerConfig::curve_path_angle, "curve_path_angle"),
        common::property(&control::PlannerConfig::curve_path_window, "curve_path_window"),
        common::property(&control::PlannerConfig::curve_path_speed_ratio, "curve_path_speed_ratio"),


        common::property(&control::PlannerConfig::start_pose_dist, "start_pose_dist"),
        common::property(&control::PlannerConfig::first_rotate_angle_tolerance, "first_rotate_angle_tolerance"),
        common::property(&control::PlannerConfig::first_rotate_angle_p, "first_rotate_angle_p"),
        common::property(&control::PlannerConfig::first_rotate_vel, "first_rotate_vel"),


        //first_track_dist
        common::property(&control::PlannerConfig::first_track_dist, "first_track_dist"),

        //max_track_dist
        common::property(&control::PlannerConfig::max_track_dist, "max_track_dist"),

        common::property(&control::PlannerConfig::first_rotate_vel_min, "first_rotate_vel_min"),


        common::property(&control::PlannerConfig::first_rotate_acc, "first_rotate_acc"),

        common::property(&control::PlannerConfig::pursuit_path_forward_acc, "pursuit_path_forward_acc"),

        common::property(&control::PlannerConfig::pursuit_path_interpolate_step, "pursuit_path_interpolate_step"),
        common::property(&control::PlannerConfig::pursuit_path_angle_converge, "pursuit_path_angle_converge"),

        //pursuit_path_width
        common::property(&control::PlannerConfig::pursuit_path_width, "pursuit_path_width"),

        common::property(&control::PlannerConfig::pursuit_path_width_direction_adjust, "pursuit_path_width_direction_adjust"),


        common::property(&control::PlannerConfig::off_path_dist, "off_path_dist"),
        common::property(&control::PlannerConfig::steer_rotate_vel, "steer_rotate_vel"),


        common::property(&control::PlannerConfig::curve_interpolate_dist, "curve_interpolate_dist"),
        common::property(&control::PlannerConfig::follow_dist_min, "follow_dist_min"),
        common::property(&control::PlannerConfig::follow_dist_max, "follow_dist_max"),


        common::property(&control::PlannerConfig::pursuit_path_rotate_vel_angle_p, "pursuit_path_rotate_vel_angle_p"),
        common::property(&control::PlannerConfig::pursuit_path_rotate_vel_vel_p, "pursuit_path_rotate_vel_vel_p"),

        common::property(&control::PlannerConfig::pursuit_path_forward_vel, "pursuit_path_forward_vel"),

        common::property(&control::PlannerConfig::pursuit_goal_dist, "pursuit_goal_dist"),

        //pursuit_path_angle_rot_vel_max
        common::property(&control::PlannerConfig::pursuit_path_angle_rot_vel_max, "pursuit_path_angle_rot_vel_max"),

        //pursuit_goal_angle_rot_vel_max
        common::property(&control::PlannerConfig::pursuit_goal_angle_rot_vel_max, "pursuit_goal_angle_rot_vel_max"),

        //speed_up_acc
        common::property(&control::PlannerConfig::speed_up_acc, "speed_up_acc"),
//        speed_down_acc
        common::property(&control::PlannerConfig::speed_down_acc, "speed_down_acc"),

        common::property(&control::PlannerConfig::pursuit_goal_forward_vel, "pursuit_goal_forward_vel"),
        common::property(&control::PlannerConfig::pursuit_goal_angle_pid_p, "pursuit_goal_angle_pid_p"),
        common::property(&control::PlannerConfig::pursuit_direct_goal_dist, "pursuit_direct_goal_dist"),
        common::property(&control::PlannerConfig::pursuit_final_goal_dist, "pursuit_final_goal_dist"),
        common::property(&control::PlannerConfig::pursuit_goal_final_angle_pid_p, "pursuit_goal_final_angle_pid_p"),
        common::property(&control::PlannerConfig::pursuit_goal_final_forward_vel, "pursuit_goal_final_forward_vel"),

        common::property(&control::PlannerConfig::pursuit_goal_reach_tolerance, "pursuit_goal_reach_tolerance"),

        common::property(&control::PlannerConfig::pursuit_goal_forward_vel_pid_p, "pursuit_goal_forward_vel_pid_p"),
        common::property(&control::PlannerConfig::pursuit_goal_forward_vel_min, "pursuit_goal_forward_vel_min"),
        common::property(&control::PlannerConfig::pursuit_goal_forward_vel_max, "pursuit_goal_forward_vel_max"),
        common::property(&control::PlannerConfig::pursuit_goal_final_forward_vel_pid_p, "pursuit_goal_final_forward_vel_pid_p"),
        common::property(&control::PlannerConfig::pursuit_goal_final_forward_vel_min, "pursuit_goal_final_forward_vel_min"),
        common::property(&control::PlannerConfig::pursuit_goal_final_forward_vel_max, "pursuit_goal_final_forward_vel_max")


        );

namespace control{

    void to_json(nlohmann::json& j, const PlannerConfig& object)
    {

        constexpr auto nbProperties = std::tuple_size<decltype(PlannerConfig_properties)>::value;
        common::for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
            // get the property
            auto& property = std::get<i>(PlannerConfig_properties);
            // set the value to the member
            j[property.name] = object.*(property.member);
        });
    }


    void from_json(const nlohmann::json& j, PlannerConfig& object) {

        constexpr auto nbProperties = std::tuple_size<decltype(PlannerConfig_properties)>::value;
        common::for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
            // get the property
            auto& property = std::get<i>(PlannerConfig_properties);
            // set the value to the member
            if(j.contains(property.name))
                j.at(property.name).get_to(object.*(property.member));
            else{
                std::cerr << "from_json: key["<<property.name<<"] not exist"<< std::endl;
            }
        });
    }
}

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
    std::vector<uint32_t> recv_cob_id;
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

struct PathConfig{

    bool run = false;
    bool fix_direction = false;
    double direction = 0.0;
    std::vector<std::array<std::array<float,2>,4>> path;

};
constexpr auto PathConfig_properties = std::make_tuple(
        common::property(&PathConfig::run, "run"),
        common::property(&PathConfig::fix_direction, "fix_direction"),
        common::property(&PathConfig::direction, "direction"),
        common::property(&PathConfig::path, "path")
);


void to_json(nlohmann::json& j, const PathConfig& object)
{

    constexpr auto nbProperties = std::tuple_size<decltype(PathConfig_properties)>::value;
    common::for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
        // get the property
        auto& property = std::get<i>(PathConfig_properties);
        // set the value to the member
        j[property.name] = object.*(property.member);
    });
}

void from_json(const nlohmann::json& j, PathConfig& object) {

    constexpr auto nbProperties = std::tuple_size<decltype(PathConfig_properties)>::value;
    common::for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
        // get the property
        auto& property = std::get<i>(PathConfig_properties);
        // set the value to the member
        if(j.contains(property.name))
//            object.*(property.member) = j[property.name];
            j.at(property.name).get_to(object.*(property.member));
        else{
            std::cerr << "from_json: key["<<property.name<<"] not exist"<< std::endl;
        }
    });
}
namespace common{
    struct Variable{
        bool value = false;
        Variable(bool v): value(v){
        }
        explicit operator bool() {
            bool v = value;
            return value = false, v;
        }
    };
}


int test_tec(int argc, char** argv) {


    std::atomic_bool program_run(true);
    auto my_handler = common::fnptr<void(int)>([&](int sig){ std::cout << "get sig " << sig;program_run = false;});
    common::set_signal_handler(my_handler);

    plog::RollingFileAppender<plog::CsvFormatter> fileAppender("tec.csv", 20000000, 20); // Create the 1st appender.
    plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender; // Create the 2nd appender.



    std::string exe_name;
    bool get_help = false;
    bool use_sim = false;
    bool smooth_stop = true;
    bool enable_control = false;


    bool enable_fix_command = false;
    std::vector<float> command_array{0.0f, 0.0f, 0.0f, 0.0f};

    int cmd_vel_timeout_ms = 100;
    int feedback_timeout_ms = 100;
    int initialise_wait_s = 1;
    float command_recovery_time_s = 3.0f;

    std::vector<std::string> command;
    std::vector<std::string> extra_command;


    std::string mqtt_server = "192.168.1.101";
    std::string mqtt_pub_topic = "hello_robot";
    std::string mqtt_sub_topic = "hello_lua";
    std::string config_file;

    std::string can_forward_file;
    std::string planner_config_file;
    std::string path_config_file;


//    float forward_vel_acc = 0.8;
//    float rotate_angle_vel = 0.8;

    bool use_rot_angle_abs = false;

    bool use_dynamic_vel = false;

    bool log_to_file = false;

    int log_level = int(plog::Severity::fatal);

    int qos = 1;

    auto cli
            =  lyra::exe_name(exe_name)
               | lyra::help(get_help)
               | lyra::opt( config_file, "config_file" )["-c"]["--config_file"]("config_file")
               | lyra::opt( enable_control)["-e"]["--enable_control"]("enable_control")
               | lyra::opt( log_to_file)["-l"]["--log_to_file"]("log_to_file")
               | lyra::opt( log_level,"log_level")["-L"]["--log_level"]("log_level")
                 | lyra::opt( qos,"qos")["-q"]["--qos"]("qos")

               | lyra::opt( use_sim)["-S"]["--use_sim"]("use_sim")
               | lyra::opt( use_dynamic_vel)["-d"]["--use_dynamic_vel"]("use_dynamic_vel")
               | lyra::opt(smooth_stop,"smooth_stop")["-s"]["--smooth_stop"]("smooth_stop")
//               | lyra::opt(use_rot_angle_abs,"use_rot_angle_abs")["-a"]["--abs_rot"]("use_rot_angle_abs")
               | lyra::opt(cmd_vel_timeout_ms,"cmd_vel_timeout_ms")["-t"]["--cmdvel_timeout"]("cmd_vel_timeout_ms")
               | lyra::opt(feedback_timeout_ms,"feedback_timeout_ms")["-f"]["--feedback_timeout"]("feedback_timeout_ms")
               | lyra::opt(initialise_wait_s,"initialise_wait_s")["-i"]["--initialise_wait_s"]("initialise_wait_s")
                 | lyra::opt(command_recovery_time_s,"command_recovery_time_s")["-r"]["--recovery_time_s"]("command_recovery_time_s")
               | lyra::opt(mqtt_server,"mqtt_server")["-M"]["--mqtt_server"]("mqtt_server")
               | lyra::opt(can_forward_file,"can_forward_file")["-T"]["--can_forward_file"]("can_forward_file")
               | lyra::opt(planner_config_file,"planner_config_file")["-P"]["--planner_config_file"]("planner_config_file")
                 | lyra::opt(path_config_file,"path_config_file")["-p"]["--path_config_file"]("path_config_file")

//               | lyra::opt(forward_vel_acc,"forward_vel_acc")["-F"]["--forward_vel_acc"]("forward_vel_acc")
//               | lyra::opt(rotate_angle_vel,"rotate_angle_vel")["-R"]["--rotate_angle_vel"]("rotate_angle_vel")

               //---
             | lyra::opt( enable_fix_command)["-C"]["--command"]("enable_fix_command")

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

    if(log_to_file){
        plog::init(plog::Severity(log_level), & consoleAppender).addAppender(&fileAppender); // Initialize the logger with the both appenders.

    }else{
        plog::init(plog::Severity(log_level), & consoleAppender);//.addAppender(&fileAppender); // Initialize the logger with the both appenders.
    }



    if(config_file.empty()){
        std::cout << "config_file not set" << std::endl;
        return 0;

    }

    if(planner_config_file.empty()){
        std::cout << "planner_config_file not set" << std::endl;
        return 0;

    }



    PLOGD << "enable_fix_command: " << enable_fix_command;


    if(enable_fix_command &&command_array.size() !=4 ){
        std::cout << " enable_fix_command , command_array.size() = " << command_array.size() << std::endl;
        return 0;

    }

    const int CONTROLLER_NUM = 2;

    std::vector<ControllerConfig> SteerConfigArray;
    std::vector<CanMsgForward> CanForwardConfigArray;
    control::PlannerConfig planner_config;
    std::vector<PathConfig> path_config;


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

    if(!path_config_file.empty()){
        std::ifstream ifs(path_config_file.c_str());
        if(ifs.is_open()){
            try{
                nlohmann::json jf = nlohmann::json::parse(ifs);
                std::cout << "path_config_file: " << path_config_file<< "\n jf :\n" << jf << std::endl;
                path_config = jf;
            }catch(nlohmann::detail::parse_error & e) {
                std::cout << e.what() << std::endl;
                std::cout << "json format error in path_config_file: " << path_config_file<<  std::endl;
                return 0;
            }
        }else{
            std::cout << "path_config_file not exist" << std::endl;

            path_config.resize(2);
            for(auto& p: path_config){
                p.path.resize(8);
            }

            nlohmann::json j3 = path_config;
            std::cout << "path_config:\n" << j3.dump(2)<< std::endl;


            std::ofstream ofs(path_config_file.c_str());
            ofs << j3.dump(2) << std::endl;
            ofs.close();
            return 0;
        }
    }


    if(!planner_config_file.empty()){
        std::ifstream ifs(planner_config_file.c_str());
        if(ifs.is_open()){
            try{
                nlohmann::json jf = nlohmann::json::parse(ifs);
                std::cout << "planner_config_file: " << planner_config_file<< "\n jf :\n" << jf << std::endl;
                planner_config = jf;
            }catch(nlohmann::detail::parse_error & e) {
                std::cout << e.what() << std::endl;
                std::cout << "json format error in planner_config_file: " << planner_config_file<<  std::endl;
                return 0;
            }
        }else{
            std::cout << "planner_config_file not exist" << std::endl;
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
    std::string mqtt_pub_topic_arg = absl::StrFormat("PUB:%s:%i", mqtt_pub_topic.c_str(),qos);
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

    const int STATUS_NUM = 14;
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
    rosMessageManager.add_channel<common_message::Twist>("PUB:cmd_vel_planner:10");
    rosMessageManager.add_channel<common_message::Odometry>("PUB:odom:200");


    rosMessageManager.add_channel<common_message::Path>("PSUB:request_path:10");
    rosMessageManager.add_channel<common_message::PoseStamped>("PSUB:request_goal:10");
    rosMessageManager.add_channel<common_message::HeaderString>("PSUB:request_config:10");

    rosMessageManager.add_channel<common_message::Path>("PPUB:global_path:10");
    rosMessageManager.add_channel<common_message::Path>("PPUB:local_path:10");
    rosMessageManager.add_channel<common_message::Odometry>("PPUB:stable_pose:200");
    rosMessageManager.add_channel<common_message::HeaderString>("PPUB:status:200");
    rosMessageManager.add_channel<std::vector<uint16_t >>("PPUB:error_code:200");



    std::vector<common_message::Twist> cmd_vel_msgs(10);
    common_message::Twist send_cmd_vel;
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

    common_message::Path request_path;
    request_path.header.frame_id.assign("map");

    common_message::Path planner_global_path;
    planner_global_path.header.frame_id.assign("map");


    common_message::Path planner_local_path;
    planner_local_path.header.frame_id.assign("map");

    common_message::Odometry planner_stable_pose;
    planner_stable_pose.header.frame_id.assign("map");

    common_message::HeaderString planner_status;


    bool driver_comm_timeout = false;
    bool driver_action_timeout = false;

    common::Variable driver_init_command = false;
    std::vector<uint16_t > driver_error_code(4,0);

    std::vector<uint16_t > driver_error_code_template(19);
    driver_error_code_template[0] = 0x0001;
    driver_error_code_template[1] = 0x0002;
    driver_error_code_template[2] = 0x0004;
    driver_error_code_template[3] = 0x0008;

    driver_error_code_template[4] = 0x0010;
    driver_error_code_template[5] = 0x0020;
    driver_error_code_template[6] = 0x0040;
    driver_error_code_template[7] = 0x0080;

    driver_error_code_template[8] = 0x0100;
    driver_error_code_template[9] = 0x0200;
    driver_error_code_template[10] = 0x0400;
    driver_error_code_template[11] = 0x0800;


    driver_error_code_template[12] = 0x1000;
    driver_error_code_template[13] = 0x2000;
    driver_error_code_template[14] = 0x4000;
    driver_error_code_template[15] = 0x8001;

    driver_error_code_template[16] = 0x8002;
    driver_error_code_template[17] = 0x8004;
    driver_error_code_template[18] = 0x8005;




    if(!path_config.empty()){
        std::vector<std::array<float,2>>  path;
        float step = 0.1;
        common_message::PoseStamped pose;

        for(auto& P : path_config){
            if (P.run){

                transform::toQuaternion(pose.pose.orientation.w,pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z, P.direction);

                auto &p1 = P.path;
                for(auto & p2:p1){

                    float PA[2] = {p2[0][0],p2[0][1]};
                    float PB[2] = {p2[1][0],p2[1][1]};
                    float PC[2] = {p2[2][0],p2[2][1]};
                    float PD[2] = {p2[3][0],p2[3][1]};
                    math::buildBezier(PA, PB,PC, PD, step,path);
                    size_t last_len = planner_global_path.poses.size();
                    planner_global_path.poses.resize(last_len + path.size(),pose);

                    for(size_t i = 0 ; i < path.size();i++){
                        planner_global_path.poses[last_len + i].pose.position.x = path[i][0];
                        planner_global_path.poses[last_len + i].pose.position.y = path[i][1];
                    }
                }

                break;
            }

        }


    }




    // todo: callback may fail if topic and datetye not matched
    bool recv_new_cmd_vel = false;
    float dynamic_forward_vel = 0.0;
    float dynamic_rotate_vel = 0.0;

    common_message::Twist recv_cmd_vel_msg;
    common::Time  recv_cmd_vel_time = common::FromUnixNow();
    auto cmd_vel_sub_cb =[&recv_new_cmd_vel,&recv_cmd_vel_msg,&recv_cmd_vel_time](void* data){
        common_message::Twist * data_ptr = static_cast<common_message::Twist*>(data);
        recv_cmd_vel_msg = * data_ptr;
//        std::cout << "odom_sub_cb recv msg: " << data_ptr->linear.x << ", " << data_ptr->angular.z << std::endl;
        recv_cmd_vel_time = common::FromUnixNow();
        recv_new_cmd_vel = true;
    };



    control::DoubleSteerMotionPlanner motion_planner;
    bool motion_planner_error = false;
    // path callback
    // start: path size >= 1
    // pause: cmd_vel is 0
    // stop/cancel : path size = 0
    // velocity constrains: ros param or embed in frame_id string
    // ros
    auto planner_path_sub_cb = [&recv_cmd_vel_msg, &motion_planner,&motion_planner_error](void* data){
        if(motion_planner_error){
            return;
        }
        common_message::Path * data_ptr = static_cast<common_message::Path*>(data);

        motion_planner.requestPath(*data_ptr);

        { recv_cmd_vel_msg.linear.x = 0.0;
            recv_cmd_vel_msg.linear.y = 0.0;
            recv_cmd_vel_msg.angular.z = 0.0;
        }

    };


    // goal callbak
    auto planner_goal_sub_cb = [&driver_init_command, &recv_cmd_vel_msg, &motion_planner,&motion_planner_error](void* data){
        common_message::PoseStamped * data_ptr = static_cast<common_message::PoseStamped*>(data);
        {
            recv_cmd_vel_msg.linear.x = 0.0;
            recv_cmd_vel_msg.linear.y = 0.0;
            recv_cmd_vel_msg.angular.z = 0.0;
        }

        if(data_ptr->header.frame_id == "reset"){
            motion_planner.reset();
            driver_init_command = true;
            return ;
        }
        if(motion_planner_error){
            return;
        }
        motion_planner.requestGoal(*data_ptr);


    };
    auto planner_config_sub_cb = [&motion_planner](void* data){
        common_message::HeaderString * data_ptr = static_cast<common_message::HeaderString*>(data);
        control::PlannerConfig config = motion_planner.m_planner_config;
        if(!data_ptr->data.empty() && (motion_planner.m_task_state == control::MotionPlanner::TaskState::idle || motion_planner.m_task_state == control::MotionPlanner::TaskState::finished) ){
            nlohmann::json j = nlohmann::json::parse(data_ptr->data);
            from_json(j,config);
            motion_planner.m_planner_config = config;
        }
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
    bool device_open_successful = false;


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

    std::vector<kaco::Message> driver_node_guard_msg(1);
    std::vector<kaco::Message> forward_driver_reset_msg(1);
    std::vector<kaco::Message> rotate_driver_reset_msg(1);


    driver_node_guard_msg[0] = kaco::Message{0x700,true,false,0,{0x00,0x00,0x00,0,0,0,0,0}};
    forward_driver_reset_msg[0] = kaco::Message{0x200,false,false,3,{0x80,0x00,0x03,0,0,0,0,0}};
    rotate_driver_reset_msg[0] = kaco::Message{0x200,false,false,3,{0x80,0x00,0x01,0,0,0,0,0}};

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

        if(forward_driver_initializer_msg.empty()){
            return ;
        }
        forward_driver_initializer_msg[0].data[1] = driver_id;

        for(size_t i = 1; i < forward_driver_initializer_msg.size();i++){

            forward_driver_initializer_msg[i].cob_id = 0x200 + driver_id;
        }
        int channel = (driver_id <= 2) ? 0 : 1;
        device.send(forward_driver_initializer_msg.data(),forward_driver_initializer_msg.size(),channel);
    };

    auto initialise_rotate_driver = [&](size_t driver_id){

        PLOGD << "initialise_rotate_driver: " << driver_id;
        if(rotate_driver_initializer_msg.empty()){
            return ;
        }
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

    auto send_node_guard = [&](size_t driver_id){
        auto& msg = driver_node_guard_msg;
        for(auto& m:msg){
            m.cob_id = 0x700 + driver_id;
        }
        int channel = (driver_id <= 2) ? 0 : 1;
        device.send(msg.data(),msg.size(),channel);
    };
    auto reset_forward_driver= [&](size_t driver_id){
        auto& msg = forward_driver_reset_msg;
        for(auto& m:msg){
            m.cob_id = 0x200 + driver_id;
        }
        int channel = (driver_id <= 2) ? 0 : 1;
        device.send(msg.data(),msg.size(),channel);
    };
    auto reset_rotate_driver= [&](size_t driver_id){
        auto& msg = rotate_driver_reset_msg;
        for(auto& m:msg){
            m.cob_id = 0x200 + driver_id;
        }
        int channel = (driver_id <= 2) ? 0 : 1;
        device.send(msg.data(),msg.size(),channel);
    };

    bool is_all_alive = false;
    bool is_all_initialised = false;
    bool is_all_initialise_triggered = false;
    bool is_rot_sensor_ready = false;

    bool is_all_operational = false;
    bool is_any_fault_exist = false;
    bool is_stop_command = false;
    bool is_control_stop_command = false;
    bool is_command_need_recover = false;

    common::TimedCounter command_recovery_timer;
    command_recovery_timer.set(command_recovery_time_s);


    bool is_planner_running = false;

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

    };
    device.nmt.register_device_alive_callback(node_heartbeat_cb);

    device.nmt.register_device_emcy_callback(node_emcy_cb);

    std::vector<kaco::Message> send_message_array(6);
    bool is_stop_command_existed = false;
    bool is_enable_command_existed = false;

    auto control_wheel = [&](bool enable,bool stop,  float forward_vel_1, float rotate_angle_1, float forward_vel_2, float rotate_angle_2){


        if(!enable_control || use_sim){
            PLOGD << "enable_control is not set, or use_sim is set";
            return ;
        }
        if(!is_all_alive || !is_all_initialised){
            PLOGD << "not is_all_alive or not is_all_initialised";
            return;
        }

        PLOGD <<"control_wheel :[enable]: " <<enable;



        if(enable){

            PLOGD <<"control_wheel 1:[forward_vel, rotate_angle]: " << forward_vel_1 << ", " << rotate_angle_1;
            PLOGD <<"control_wheel 2:[forward_vel, rotate_angle]: " << forward_vel_2 << ", " << rotate_angle_2;

            PLOGD <<"SteerConfigArray[0].use_rot_angle_abs: " << SteerConfigArray[0].use_rot_angle_abs;
            PLOGD <<"SteerConfigArray[1].use_rot_angle_abs: " << SteerConfigArray[1].use_rot_angle_abs;

            PLOGD <<"SteerConfigArray[0].rot_angle_abs_offset: " << SteerConfigArray[0].rot_angle_abs_offset;
            PLOGD <<"SteerConfigArray[1].rot_angle_abs_offset: " << SteerConfigArray[1].rot_angle_abs_offset;


            *(int*)(&driver_command_msg_channel_1[0].data[0]) = static_cast<int>(forward_vel_1 / SteerConfigArray[0].forward_speed_k);
            *(int*)(&driver_command_msg_channel_1[1].data[0]) = static_cast<int>((rotate_angle_1 - SteerConfigArray[0].rot_angle_b - SteerConfigArray[0].rot_angle_abs_offset)/SteerConfigArray[0].rot_angle_k);
            *(int*)(&driver_command_msg_channel_2[0].data[0]) = static_cast<int>(forward_vel_2 / SteerConfigArray[1].forward_speed_k);
            *(int*)(&driver_command_msg_channel_2[1].data[0]) = static_cast<int>((rotate_angle_2 - SteerConfigArray[1].rot_angle_b - SteerConfigArray[1].rot_angle_abs_offset)/SteerConfigArray[1].rot_angle_k);
#if 0
            PLOGD << "send can data to channel 1";
            driver_command_msg_channel_1[0].print();
            driver_command_msg_channel_1[1].print();


            PLOGD << "send can data to channel 2";
            driver_command_msg_channel_2[0].print();
            driver_command_msg_channel_2[1].print();
#endif

            device.send(driver_command_msg_channel_1.data(),driver_command_msg_channel_1.size(),0);
            device.send(driver_command_msg_channel_2.data(),driver_command_msg_channel_2.size(),1);

            if(stop){
                disable_forward_driver(1);
                disable_forward_driver(4);
            }else {


                if(!is_enable_command_existed){
                    initialise_rotate_driver(2);
                    initialise_rotate_driver(5);
                    initialise_forward_driver(1);
                    initialise_forward_driver(4);

                }
                if(is_stop_command_existed){
                    initialise_forward_driver(1);
                    initialise_forward_driver(4);
                }
            }




        }else{
            disable_forward_driver(1);
            disable_rotate_driver(2);

            disable_forward_driver(4);
            disable_rotate_driver(5);

        }

        is_enable_command_existed = enable;
        is_stop_command_existed = stop;

    };

    bool state_update = false;
    auto update_wheel_forward = [&](size_t driver_id, const kaco::Message& message){

        PLOGD << "receive [" << driver_id << "]";

        int num = * (int*)(&message.data[0]);

        std::bitset<16> fault_code = * (int*)(&message.data[4]);

        SteerConfigArray[driver_id].actual_forward_vel = num * SteerConfigArray[driver_id].forward_speed_k;
        state_update = true;

        SteerConfigArray[driver_id].forward_time = common::FromUnixNow();

        //error code
        uint16_t current_code =  * (uint16_t *)(&message.data[4]);
        auto& code =  driver_error_code[driver_id*2 + 0];

        bool is_fault_last = (std::find(driver_error_code_template.begin(), driver_error_code_template.end(), code) != driver_error_code_template.end());

        bool is_fault_new = (std::find(driver_error_code_template.begin(), driver_error_code_template.end(),current_code) != driver_error_code_template.end());

        if(is_fault_new || !is_fault_last){
            code = current_code;
        }

        is_any_fault_exist = is_any_fault_exist || is_fault_new || is_fault_last;


        PLOGD << "actual_forward_vel: " << SteerConfigArray[driver_id].actual_forward_vel;
    };

    auto update_wheel_rotate = [&](size_t driver_id, const kaco::Message& message){
        PLOGD << "receive [" << driver_id << "]";
        int num = * (int*)(&message.data[0]);

        std::bitset<16> fault_code = * (int*)(&message.data[4]);

        float rot_angle = num * SteerConfigArray[driver_id].rot_angle_k +  SteerConfigArray[driver_id].rot_angle_b;

        SteerConfigArray[driver_id].addAngle(rot_angle);
        state_update = true;
        SteerConfigArray[driver_id].forward_time = common::FromUnixNow();

        //error code
        uint16_t current_code =  * (uint16_t *)(&message.data[4]);
        auto& code =  driver_error_code[driver_id*2 + 1];

        bool is_fault_last = (std::find(driver_error_code_template.begin(), driver_error_code_template.end(), code) != driver_error_code_template.end());

        bool is_fault_new = (std::find(driver_error_code_template.begin(), driver_error_code_template.end(),current_code) != driver_error_code_template.end());

        if(is_fault_new || !is_fault_last){
            code = current_code;
        }

        is_any_fault_exist = is_any_fault_exist || is_fault_new || is_fault_last;

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
    std::vector<common_message::CanMessageArray> can_channel_forward_msg(2);
    std::vector<std::vector<kaco::Message>> can_channel_send_msg(2);



    for(size_t i = 0 ; i < CanForwardConfigArray.size();i++){
        auto& c = CanForwardConfigArray[i];

        if(c.channel_id <0 || c.channel_id>1 || c.recv_cob_id.empty() || c.pub_topic.empty() || c.sub_topic.empty() ){
            continue;
        }

        // ros

//        rosMessageManager.add_channel<common_message::Odometry>("PUB:odom:200");
        std::string sub_topic = absl::StrFormat("SUB:%s:100", c.sub_topic.c_str());
        rosMessageManager.add_channel<common_message::CanMessageArray>(sub_topic.c_str());

        std::string pub_topic = absl::StrFormat("PUB:%s:100", c.pub_topic.c_str());
        rosMessageManager.add_channel<common_message::CanMessageArray>(pub_topic.c_str());



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
                can_channel_forward_msg[c.channel_id].messages.push_back(msg);
            }

        });
        auto can_message_sub_cb =[&c,&can_channel_send_msg](void* data){
            common_message::CanMessageArray * data_ptr = static_cast<common_message::CanMessageArray*>(data);

            kaco::Message msg;
            PLOGD << "recv data" << std::endl;

            for(size_t i = 0 ; i < data_ptr->messages.size();i++){
                msg.cob_id = data_ptr->messages.at(i).id;
                msg.rtr = data_ptr->messages.at(i).is_rtr;
                msg.ext = data_ptr->messages.at(i).is_extended;
                msg.len = data_ptr->messages.at(i).dlc;
                std::copy(std::begin(data_ptr->messages.at(i).data), std::end(data_ptr->messages.at(i).data), std::begin(msg.data));
                can_channel_send_msg[c.channel_id].push_back(msg);
            }

        };
        taskManager.addTask([&rosMessageManager,&c,&device,&can_channel_send_msg,can_message_sub_cb]{

            char key_buffer[100];
            sprintf(key_buffer,"SUB:%s",c.sub_topic.c_str());

            rosMessageManager.recv_message(key_buffer,10,0.001, can_message_sub_cb);

            if(can_channel_send_msg[c.channel_id].empty()){
                return true;
            }

            device.send( can_channel_send_msg[c.channel_id].data(), can_channel_send_msg[c.channel_id].size(),c.channel_id);
            can_channel_send_msg[c.channel_id].clear();

            return true;
        },100*1000,2);
        taskManager.addTask([&rosMessageManager,&c,&can_channel_forward_msg]{
            if(can_channel_forward_msg[c.channel_id].messages.empty()){
                return true;
            }

            char key_buffer[100];
            sprintf(key_buffer,"PUB:%s",c.pub_topic.c_str());
            rosMessageManager.send_message(key_buffer,&can_channel_forward_msg[c.channel_id], 1, 0.1 );
            can_channel_forward_msg[c.channel_id].messages.clear();
            return true;
        },100*1000,2);

    }
    // controller
    control::DoubleSteerController controller;


    //
    std::vector<control::SteerWheelBase> wheels_array(SteerConfigArray.size());
    for(size_t i = 0 ; i  < SteerConfigArray.size(); i++){

        wheels_array[i].enable_rot = true;
        wheels_array[i].mount_x = SteerConfigArray[i].mount_x;
        wheels_array[i].mount_y = SteerConfigArray[i].mount_y;

        wheels_array[i].max_forward_acc = SteerConfigArray[i].max_forward_acc;
        wheels_array[i].max_rotate_vel = SteerConfigArray[i].max_rotate_vel;

        wheels_array[i].min_forward_vel = SteerConfigArray[i].min_forward_vel;

        wheels_array[i].max_forward_vel = SteerConfigArray[i].max_forward_vel;
        wheels_array[i].max_rotate_angle = SteerConfigArray[i].max_rotate_angle;

        wheels_array[i].predict_control_s = SteerConfigArray[i].predict_control_s;
        wheels_array[i].forward_reach_thresh = SteerConfigArray[i].forward_reach_thresh;
        wheels_array[i].rotate_reach_thresh = SteerConfigArray[i].rotate_reach_thresh;
        wheels_array[i].action_timeout_s = SteerConfigArray[i].action_timeout_s;
    }


    controller.set_wheel(wheels_array[0],wheels_array[1]);

    controller.smooth_stop = smooth_stop;

    control::SmoothSimulator smoothSimulator;
    smoothSimulator.set_wheel(wheels_array[0],wheels_array[1]);

    motion_planner.m_planner_config = planner_config;
    motion_planner.initBase(wheels_array);

    common_message::TransformStamped planner_map_base_tf;
    planner_map_base_tf.base_frame.assign("map");
    planner_map_base_tf.target_frame.assign("base_link");

    common_message::TransformStamped planner_odom_base_tf;
    planner_odom_base_tf.base_frame.assign("odom");
    planner_odom_base_tf.target_frame.assign("base_link");


    taskManager.addTask([&]{

        // lookup tf map base_link
        // lookup tf odom base_link
        auto now = common::FromUnixNow();

        planner_map_base_tf.time = now;
        planner_odom_base_tf.time = now;

        long rt1 = rosMessageManager.recv_tf(planner_map_base_tf);
        long rt2 = rosMessageManager.recv_tf(planner_odom_base_tf);

        if(rt1 == 0){
            motion_planner.updateMapBasePose(planner_map_base_tf);

        }
        if(rt2 == 0){
            motion_planner.updateOdomBasePose(planner_odom_base_tf);
        }

        motion_planner.updateOdom(odom);

        bool get_pose = motion_planner.getStablePose();


        PLOGD << "motion planner: get_pose: " << get_pose;

        return true;
    },10*1000,0);

    taskManager.addTask([&]{
        auto now = common::FromUnixNow();






        if(!motion_planner.m_global_path.value.empty()){
            size_t pose_num = motion_planner.m_global_path.value.size();
            planner_global_path.poses.resize(pose_num);
            for(size_t i = 0 ; i < pose_num;i++){
                planner_global_path.poses[i].pose = common_message::Transform2dToPose(motion_planner.m_global_path.value[i]);
            }
            planner_global_path.header.stamp = now;
            rosMessageManager.send_message("PPUB:global_path",&planner_global_path, 1, 0.1 );
        }
        if(!motion_planner.m_local_path.value.empty()){
            size_t pose_num = motion_planner.m_local_path.value.size();
            planner_local_path.poses.resize(pose_num);
            for(size_t i = 0 ; i < pose_num;i++){
                planner_local_path.poses[i].pose = common_message::Transform2dToPose(motion_planner.m_local_path.value[i]);
            }
            planner_local_path.header.stamp = now;
            rosMessageManager.send_message("PPUB:local_path",&planner_local_path, 1, 0.1 );
        }

        if(motion_planner.m_stable_pose_get){

            planner_stable_pose.pose.pose = common_message::Transform2dToPose(motion_planner.m_actual_pose.value);
            planner_stable_pose.header.stamp = now;
            rosMessageManager.send_message("PPUB:stable_pose",&planner_stable_pose, 1, 0.1 );

        }

        planner_status.header.frame_id = motion_planner.getTaskFrame();
        if(motion_planner.getTaskState() == control::MotionPlanner::TaskState::idle){
            planner_status.data.assign("idle") ;
        }
        else if(motion_planner.getTaskState() == control::MotionPlanner::TaskState::finished){
            planner_status.data.assign("finished") ;
        }else if(motion_planner.getTaskState() == control::MotionPlanner::TaskState::error_path){
            planner_status.data.assign(motion_planner.getStatusMsg()) ;
        }else if(motion_planner.getTaskState() == control::MotionPlanner::TaskState::off_path){
            motion_planner_error = true;
            planner_status.data.assign(motion_planner.getStatusMsg()) ;
        }else{
            planner_status.data.assign("running") ;

            // suspend running task
            if(is_command_need_recover){
                planner_status.data.assign("suspend") ;
            }
        }

        if(driver_comm_timeout){
            char msg[500];
            sprintf(msg,"driver_error: driver communication timeout: [%i,%i], [%i,%i]",
                    SteerConfigArray[0].forward_feedback_timeout,
                    SteerConfigArray[0].rotate_feedback_timeout,
                    SteerConfigArray[1].forward_feedback_timeout,
                    SteerConfigArray[1].rotate_feedback_timeout
            );

            planner_status.data.assign(msg) ;

        }
        if(driver_action_timeout){

            char msg[500];
            sprintf(msg,"driver_error: driver action timeout: [%i,%i], [%i,%i]",
                    controller.m_steer_wheel[0].forward_action_timeout_timer.eval(),
                    controller.m_steer_wheel[0].rotate_action_timeout_timer.eval(),
                    controller.m_steer_wheel[1].forward_action_timeout_timer.eval(),
                    controller.m_steer_wheel[1].rotate_action_timeout_timer.eval()
            );
            planner_status.data.assign(msg) ;
        }
        if(!device_open_successful){
            planner_status.data.assign("driver_error: can device open failed") ;
            is_any_fault_exist = true;
        }
        if(is_any_fault_exist){
            motion_planner.reset();
            motion_planner_error = true;
        }


        bool get_pose = motion_planner.getStablePose();
        if(get_pose){
            rosMessageManager.recv_message("PSUB:request_goal",1,0.001, planner_goal_sub_cb);
            rosMessageManager.recv_message("PSUB:request_path",1,0.001, planner_path_sub_cb);
        }

        rosMessageManager.send_message("PPUB:status",&planner_status, 1, 0.1 );


        return true;
    },100*1000,1);

    common_message::TransformStamped send_map_odom_tf, send_odom_base_tf;
    send_map_odom_tf.base_frame.assign("map");
    send_map_odom_tf.target_frame.assign("odom");
    send_odom_base_tf.base_frame.assign("odom");
    send_odom_base_tf.target_frame.assign("base_link");
    double send_map_odom_tf_x = 0.0;
    double send_map_odom_tf_y = 0.0;

    send_map_odom_tf.transform = transform::createSe3<double>(send_map_odom_tf_x,send_map_odom_tf_y,0.0,0.0,0.0,0.0);


    Eigen::Transform<double,3,Eigen::Isometry> odom_base_transform;

    if(!use_sim){
        if (!device.start(busname, baudrate)) {
            std::cout << "Starting device failed." << std::endl;
            device_open_successful = false;
        }else{
            device_open_successful = true;

        }

        if(device_open_successful){
            taskManager.addTask([&]{

                check_all_state();
                // if not operational, send nmt start
//            if(!is_all_alive){
//                device.nmt.broadcast_nmt_message(kaco::NMT::Command::start_node,send_to_channel_1);
//                device.nmt.broadcast_nmt_message(kaco::NMT::Command::start_node,send_to_channel_2);
//            }
                device.nmt.broadcast_nmt_message(kaco::NMT::Command::start_node,send_to_channel_1);
                device.nmt.broadcast_nmt_message(kaco::NMT::Command::start_node,send_to_channel_2);



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
//            PLOGF.printf("driver error:is_all_alive : %i, driver_action_timeout : %i,  ",is_all_alive, driver_action_timeout);

                if(is_all_alive){

                    if(driver_init_command){
                        is_all_initialise_triggered = false;
                        is_all_initialised = false;

                    }


                    if(!is_all_initialise_triggered){

                        // run once

                        // clear error code
                        is_any_fault_exist = false;
                        std::fill(driver_error_code.begin(), driver_error_code.end(),0);

                        is_all_initialise_triggered = true;
                        is_all_initialised = false;
                        taskManager.addTask([&]{


                            if(!is_all_initialised){
                                initialise_forward_driver(1);
                                initialise_rotate_driver(2);

                                initialise_forward_driver(4);
                                initialise_rotate_driver(5);

                                taskManager.addTask([&]{
                                    is_any_fault_exist = false;
                                    motion_planner_error = false;
                                    driver_action_timeout = false;
                                    driver_comm_timeout = false;
                                    is_all_initialised = true;


                                    auto now = common::FromUnixNow();

                                    for(int i = 0; i < CONTROLLER_NUM;i++){

                                        SteerConfigArray[i].forward_time = now;
                                        SteerConfigArray[i].rotate_time = now;
                                    }



                                    std::fill(driver_error_code.begin(), driver_error_code.end(),0);
                                    return false;
                                }, initialise_wait_s*1000*1000, 0);
                            }




                            return false;
                        },100*1000, 0);


                    }

#if 0
                    auto now = common::FromUnixNow();
//                driver_comm_timeout = false;


                for(size_t i = 0 ; i < 2;i++){

                    driver_error_code[i*2+0] = std::abs(common::ToMillSeconds(now - SteerConfigArray[i].forward_time )) > 1000 ? 0x8005:driver_error_code[i*2+0]  ;
                    driver_error_code[i*2+1] = std::abs(common::ToMillSeconds(now - SteerConfigArray[i].rotate_time )) > 1000 ? 0x8005:driver_error_code[i*2+0]  ;
                    driver_comm_timeout = driver_comm_timeout || (std::abs(common::ToMillSeconds(now - SteerConfigArray[i].forward_time ))) || (std::abs(common::ToMillSeconds(odom.header.stamp - SteerConfigArray[i].rotate_time )) );

                    controller.m_steer_wheel[i].check();

                    driver_action_timeout = driver_action_timeout || controller.m_steer_wheel[i].forward_action_timeout.eval() || controller.m_steer_wheel[i].rotate_action_timeout.eval();
                }






                is_any_fault_exist = is_any_fault_exist || driver_comm_timeout || driver_action_timeout;
                //todo: remove in next version of driver
//                is_any_fault_exist = driver_comm_timeout;
                if(is_any_fault_exist){
                    for(size_t i = 0 ; i < 2;i++){
                        SteerConfigArray[i].actual_forward_vel = 0.0 ;
                        SteerConfigArray[i].actual_rot_abs_angle = 0.0;
                    }
                }
#endif


                }else{
                    return true;
                }


                return true;
            }, 50*1000,1);

            // rot
            taskManager.addTask([&]{
                is_rot_sensor_ready = true;
                for(size_t i = 0 ; i < SteerConfigArray.size();i++){
                    SteerConfigArray[i].angleCalib();
                    is_rot_sensor_ready = is_rot_sensor_ready && SteerConfigArray[i].isAngleCalib();
                }
                return true;
            }, 500*1000,1);
        }



    }else{

        device_open_successful = true;

        taskManager.addTask([&]{

            if(path_config.empty()){
                return false;
            }

            auto now = common::FromUnixNow();
            planner_global_path.header.stamp = now;
            motion_planner.requestPath(planner_global_path);

            rosMessageManager.send_message("PPUB:global_path",&planner_global_path, 1, 0.1 );


            return false;
        },2*1000*1000,1);






        taskManager.addTask([&]{


            smoothSimulator.updateState(controller.m_steer_wheel[0].getCommandForwardVel(),controller.m_steer_wheel[0].getCommandRotateAngle(),
                                        controller.m_steer_wheel[1].getCommandForwardVel(),controller.m_steer_wheel[1].getCommandRotateAngle());


            auto now = common::FromUnixNow();

            for(int i = 0; i < CONTROLLER_NUM;i++){

                SteerConfigArray[i].forward_time = now;
                SteerConfigArray[i].rotate_time = now;
                SteerConfigArray[i].actual_forward_vel = smoothSimulator.m_steer_wheel[i].actual_forward_vel  ;
                SteerConfigArray[i].actual_rot_abs_angle = smoothSimulator.m_steer_wheel[i].actual_rot_angle;

            }

            send_map_odom_tf.time = common::FromUnixNow();
            send_odom_base_tf.time = common::FromUnixNow();

            send_map_odom_tf.transform = transform::createSe3<double>(send_map_odom_tf_x,send_map_odom_tf_y,0.0,0.0,0.0,0.0);
            send_odom_base_tf.transform = odom_base_transform;

            rosMessageManager.send_tf(send_map_odom_tf);
            rosMessageManager.send_tf(send_odom_base_tf);


            is_all_alive = true;
            is_all_initialised = true;
            is_any_fault_exist = false;
            is_rot_sensor_ready = true;

            return true;
        },20*1000,0);

    }


#if 0
    // add map jump
    taskManager.addTask([&]{
        send_map_odom_tf_x +=0.2;
        send_map_odom_tf_y = 0.0;

        return false;}
        ,6*1000*1000,0);

    taskManager.addTask([&]{
                            send_map_odom_tf_x +=common::uniform_real<float>(-0.001,0.001);
                            send_map_odom_tf_y +=common::uniform_real<float>(-0.001,0.001);

                            return false;}
            ,10*1000,0);

#endif

    // odom
    taskManager.addTask([&]{


        PLOGD << "is_all_alive: " << is_all_alive;
        PLOGD << "is_rot_sensor_ready: " << is_rot_sensor_ready;
        PLOGD << "is_all_initialised: " << is_all_initialised;


        if(is_all_alive && is_rot_sensor_ready && is_all_initialised){
            auto now = common::FromUnixNow();


            PLOGD << "update driver0 feedback[forward_vel, rotate_angle], " << SteerConfigArray[0].actual_forward_vel << ", " << SteerConfigArray[0].actual_rot_abs_angle;
            PLOGD << "update driver1 feedback[forward_vel, rotate_angle], " << SteerConfigArray[1].actual_forward_vel << ", " << SteerConfigArray[1].actual_rot_abs_angle;

            {
                controller.updateState(SteerConfigArray[0].actual_forward_vel,SteerConfigArray[0].actual_rot_abs_angle, SteerConfigArray[1].actual_forward_vel,SteerConfigArray[1].actual_rot_abs_angle);
                state_update = false;
            }

            // todoo: check action timeout
            {
                for(size_t i = 0 ; i < CONTROLLER_NUM;i++){

                    controller.m_steer_wheel[i].check();
                    SteerConfigArray[i].forward_feedback_timeout = std::abs(common::ToMillSeconds(now - SteerConfigArray[i].forward_time )) >  SteerConfigArray[i].comm_timeout_s *1000;
                    SteerConfigArray[i].rotate_feedback_timeout = std::abs(common::ToMillSeconds(now - SteerConfigArray[i].rotate_time )) >  SteerConfigArray[i].comm_timeout_s*1000;


                    driver_comm_timeout = driver_comm_timeout || (
                                                                         SteerConfigArray[i].forward_feedback_timeout

                            )
                                    ||
                                    (
                                            SteerConfigArray[i].rotate_feedback_timeout

                                    );
//                    PLOGF.printf("driver_comm_timeout[%i] error: forward_time: %i, rotate_time : %i ",i, std::abs(common::ToMillSeconds(now - SteerConfigArray[i].forward_time )), std::abs(common::ToMillSeconds(now - SteerConfigArray[i].rotate_time )));

                    driver_action_timeout = driver_action_timeout || controller.m_steer_wheel[i].forward_action_timeout_timer.eval() || controller.m_steer_wheel[i].rotate_action_timeout_timer.eval();
                }

                is_any_fault_exist = is_any_fault_exist || driver_comm_timeout || driver_action_timeout;


//                PLOGF.printf("driver error: driver_comm_timeout: %i, driver_action_timeout : %i ",driver_comm_timeout, driver_action_timeout);
                if(is_any_fault_exist){
                    for(size_t i = 0 ; i < CONTROLLER_NUM;i++){
                        SteerConfigArray[i].actual_forward_vel = 0.0 ;
//                        SteerConfigArray[i].actual_rot_abs_angle = 0.0;
                    }
                }

            }



            odom.header.stamp = common::FromUnixNow();


            const transform::Transform2d& robot_pose = controller.getPosition();
//            odom.pose.pose.position.x = robot_pose.x();
//            odom.pose.pose.position.y = robot_pose.y();
//            odom.pose.pose.position.z = 0.0;




            odom_base_transform = transform::createSe3<double>(robot_pose.x(),robot_pose.y(),0.0f,0.0f,0.0f,robot_pose.yaw() );

            double tx,ty,tz,qw,qx,qy,qz;
//            transform::extractSe3<double>(pose,odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z, odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z );

            transform::extractSe3<double>(odom_base_transform,tx,ty,tz,qw,qx,qy,qz );

            odom.pose.pose.position.x = tx;
            odom.pose.pose.position.y = ty;
            odom.pose.pose.position.z = tz;
            odom.pose.pose.orientation.w = qw;
            odom.pose.pose.orientation.x = qx;
            odom.pose.pose.orientation.y = qy;
            odom.pose.pose.orientation.z = qz;

            odom.twist.twist.linear.x = controller.getActualForwardVel() * std::cos(controller.getActualForwardAngle());
            odom.twist.twist.linear.y = controller.getActualForwardVel() * std::sin(controller.getActualForwardAngle());

            odom.twist.twist.angular.z = controller.getActualRotateVel();



            rosMessageManager.send_message("PUB:odom",&odom, 1, 0.1 );
            rosMessageManager.send_message("PPUB:error_code",&driver_error_code, 1, 0.1 );
            rosMessageManager.send_message("PUB:cmd_vel_planner",&send_cmd_vel,1,0.01);


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
                    mqtt_msg_status[mqtt_msg_status_cnt*STATUS_NUM + 11 ] = recv_cmd_vel_msg.linear.x;
                    mqtt_msg_status[mqtt_msg_status_cnt*STATUS_NUM + 12 ] = recv_cmd_vel_msg.linear.y;
                    mqtt_msg_status[mqtt_msg_status_cnt*STATUS_NUM + 13 ] = recv_cmd_vel_msg.angular.z;

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

            //todo: remove
//            is_any_fault_exist = false;

            if((is_all_alive && is_all_initialised && is_rot_sensor_ready && ! is_any_fault_exist) && !motion_planner_error  ){

                rosMessageManager.recv_message("SUB:control_cmd_vel",10,0.001, cmd_vel_sub_cb);

                if(recv_new_cmd_vel){
                    dynamic_forward_vel = std::sqrt(recv_cmd_vel_msg.linear.x*recv_cmd_vel_msg.linear.x + recv_cmd_vel_msg.linear.y* recv_cmd_vel_msg.linear.y );
                    dynamic_rotate_vel = recv_cmd_vel_msg.angular.z;
                    recv_new_cmd_vel = false;
                    PLOGD<< "recv cmd_vel:[rotate_vel,forward_vel]: " << recv_cmd_vel_msg.angular.z << ", " << recv_cmd_vel_msg.linear.x;



                    if(use_dynamic_vel){
                        bool temp_stop = std::abs(recv_cmd_vel_msg.linear.x) < 0.001
                                         && std::abs(recv_cmd_vel_msg.linear.y) < 0.001
                                         && std::abs(recv_cmd_vel_msg.angular.z) < 0.001;

                        command_recovery_timer.on(!temp_stop);
                        if(temp_stop){
                            is_command_need_recover = is_planner_running;
                        }else{
                            if(is_command_need_recover)
                                is_command_need_recover = is_planner_running && !command_recovery_timer.eval();
                        }



                        is_control_stop_command = temp_stop;

                    }




                }




                if(motion_planner.getTaskState() == control::MotionPlanner::TaskState::idle || motion_planner.getTaskState() == control::MotionPlanner::TaskState::finished || motion_planner.getTaskState() == control::MotionPlanner::TaskState::error_path || motion_planner.getTaskState() == control::MotionPlanner::TaskState::off_path ){
                    is_planner_running = false;
                    auto now = common::FromUnixNow();
                    if( common::ToMillSeconds(now - recv_cmd_vel_time) > cmd_vel_timeout_ms){

                        PLOGD << "cmd_vel timeout, cmd_vel reset";
                        recv_cmd_vel_msg.linear.x = 0.0;
                        recv_cmd_vel_msg.linear.y = 0.0;
                        recv_cmd_vel_msg.angular.z = 0.0;
                        recv_cmd_vel_time = now;

                    }



                    {



#if 0
                        if(std::abs(recv_cmd_vel_msg.linear.y) < std::abs(recv_cmd_vel_msg.linear.x)) {
                            controller.setSteerPreference(0.0f);
                        }
                        else if(recv_cmd_vel_msg.linear.y > 0.001){
                            controller.setSteerPreference(M_PI_2f32);

                        }else if(recv_cmd_vel_msg.linear.y < -0.001){
                            controller.setSteerPreference(-M_PI_2f32);

                        }else{
                            controller.setSteerPreference(0.0f);
                        }

                        if (std::abs(vel) < 0.00001){

                            controller.setSteerPreference(0.0f);

                        }else{

                            float vel_angle = std::atan2(recv_cmd_vel_msg.linear.y, recv_cmd_vel_msg.linear.x);

                            if(vel_angle >0.0  && vel_angle <= M_PI_2f32 ){
                                controller.setSteerPreference(M_PI_2f32);
                            }else if(vel_angle > M_PI_2f32){
                                controller.setSteerPreference(-M_PI_2f32);
                            }else if(vel_angle >= -M_PI_2f32 && vel_angle < 0.0){
                                controller.setSteerPreference(-M_PI_2f32);
                            }else if(vel_angle < -M_PI_2f32){
                                controller.setSteerPreference(M_PI_2f32);
                            }else{
                                controller.setSteerPreference(0.0f);
                            }
                        }
#endif
                    }

                }else{
//                    PLOGF << "motion_planner.updateWheelState";
                    is_planner_running = true;
                    motion_planner.updateWheelState(SteerConfigArray[0].actual_forward_vel,SteerConfigArray[0].actual_rot_abs_angle, SteerConfigArray[1].actual_forward_vel,SteerConfigArray[1].actual_rot_abs_angle);

                    common::Time t1 = common::FromUnixNow();
//                    PLOGF << "motion_planner.go";

                    motion_planner.go();
                    auto& command = motion_planner.getCommand();

                    PLOGD << "go_time: " << common::ToMillSeconds(common::FromUnixNow() - t1) << " ms";

                    if(0){

                        if(!motion_planner.m_local_path.value.empty()){
                            size_t pose_num = motion_planner.m_local_path.value.size();
                            planner_local_path.poses.resize(pose_num);
                            for(size_t i = 0 ; i < pose_num;i++){
                                planner_local_path.poses[i].pose = common_message::Transform2dToPose(motion_planner.m_local_path.value[i]);
                            }
                            planner_local_path.header.stamp = common::FromUnixNow();
                            rosMessageManager.send_message("PPUB:local_path",&planner_local_path, 1, 0.1 );
                        }
                    }
                    if(command.command_type == control::MotionPlanner::CommandType::cmd_vel && command.command.size() == 3 ){

//                        PLOGF << "cmd_vel: " << command.command[0] << ", " << command.command[1] << ", " << command.command[2];

                        recv_cmd_vel_msg.linear.x = command.command[0];
                        recv_cmd_vel_msg.linear.y = command.command[1];
                        recv_cmd_vel_msg.angular.z = command.command[2];
                        controller.setSmoothStop();
//todo: remove
#if 0
                        controller.setPrefer(motion_planner.use_prefer_steer_angle());

                        controller.setSteerPreference(motion_planner.get_prefer_steer_angle());
#endif
                    }else if(command.command_type == control::MotionPlanner::CommandType::steer && command.command.size() == 1 ){



                        controller.m_steer_wheel[0].getCommandForwardVel() =  0.0f;
                        controller.m_steer_wheel[0].getCommandRotateAngle() =  std::max(std::min(1.918888889f,  command.command[0]),-1.918888889f);
                        controller.m_steer_wheel[1].getCommandForwardVel() =  0.0f;
                        controller.m_steer_wheel[1].getCommandRotateAngle() =  std::max(std::min(1.918888889f,  command.command[0]),-1.918888889f);

//                        PLOGF << "steer command: " << command.command[0];

                        control_wheel(true,false, controller.m_steer_wheel[0].getCommandForwardVel(),controller.m_steer_wheel[0].getCommandRotateAngle() ,controller.m_steer_wheel[1].getCommandForwardVel(),controller.m_steer_wheel[1].getCommandRotateAngle() );
                        recv_cmd_vel_msg.linear.x = 0.0;
                        recv_cmd_vel_msg.linear.y = 0.0;
                        recv_cmd_vel_msg.angular.z = 0.0;

                        controller.bypass();
                        return true;
                    }else{
//                        controller.setSteerPreference(0.0f);
                        recv_cmd_vel_msg.linear.x = 0.0;
                        recv_cmd_vel_msg.linear.y = 0.0;
                        recv_cmd_vel_msg.angular.z = 0.0;

                    }
                }

                // off_path
                recv_cmd_vel_msg.linear.x *= !motion_planner_error;
                recv_cmd_vel_msg.linear.y *= !motion_planner_error;
                recv_cmd_vel_msg.angular.z *= !motion_planner_error;


                send_cmd_vel = recv_cmd_vel_msg;



                // controller compute
                {
                    float vel = std::sqrt(recv_cmd_vel_msg.linear.x*recv_cmd_vel_msg.linear.x + recv_cmd_vel_msg.linear.y*recv_cmd_vel_msg.linear.y);


                    if(is_command_need_recover){
                        dynamic_forward_vel = 0.0f;
                        dynamic_rotate_vel = 0.0f;
                    }

                    if (std::abs(vel) < 0.00001){

                        if(use_dynamic_vel){
                            float ratio = 1.0;
                            if(std::abs(recv_cmd_vel_msg.angular.z) > 0.0001){
                                ratio = std::min(1.0f, std::abs(dynamic_rotate_vel)/std::abs(recv_cmd_vel_msg.angular.z));
                                recv_cmd_vel_msg.angular.z *= ratio;
                            }else{
                                recv_cmd_vel_msg.angular.z = 0.0f;
                            }
                            PLOGD << "dynamic, for_ward_vel: " << vel << ", rot_vel: " << recv_cmd_vel_msg.angular.z;

                        }
                        controller.cmd_vel(0.0,recv_cmd_vel_msg.angular.z);

                    }else{


                        if(use_dynamic_vel){
                            float ratio = 1.0;

                            ratio = std::min(1.0f,dynamic_forward_vel/vel);

                            vel *= ratio;
                            recv_cmd_vel_msg.angular.z *= ratio;

                            PLOGD << "dynamic, for_ward_vel: " << vel << ", rot_vel: " << recv_cmd_vel_msg.angular.z;
                        }
                        float vel_angle = std::atan2(recv_cmd_vel_msg.linear.y, recv_cmd_vel_msg.linear.x);
                        controller.cmd_vel(vel,recv_cmd_vel_msg.angular.z,vel_angle);
                    }
                }

//                PLOGF << "controller.interpolate";

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


                is_stop_command = std::abs(recv_cmd_vel_msg.linear.x) < 0.001
                                  && std::abs(recv_cmd_vel_msg.linear.y) < 0.001
                                  && std::abs(recv_cmd_vel_msg.angular.z) < 0.001;

                bool is_stopped = std::abs(odom.twist.twist.linear.x) < 0.001
                        && std::abs(odom.twist.twist.linear.y) < 0.001
                        && std::abs(odom.twist.twist.angular.z) < 0.001;


                // can send

                control_wheel(true,is_stop_command && is_stopped, controller.m_steer_wheel[0].getCommandForwardVel(),controller.m_steer_wheel[0].getCommandRotateAngle() ,controller.m_steer_wheel[1].getCommandForwardVel(),controller.m_steer_wheel[1].getCommandRotateAngle() );

                return true;
            }else{
                PLOGD << "driver not ready, cmd_vel reset";
                motion_planner.reset();
                controller.bypass();

                recv_cmd_vel_msg.linear.x = 0.0;
                recv_cmd_vel_msg.linear.y = 0.0;
                recv_cmd_vel_msg.angular.z = 0.0;
                send_cmd_vel.linear.x = 0.0;
                send_cmd_vel.linear.y = 0.0;
                send_cmd_vel.angular.z = 0.0;
                controller.m_steer_wheel[0].getCommandForwardVel() = 0.0f;
                controller.m_steer_wheel[1].getCommandForwardVel() = 0.0f;

                control_wheel(false,true, controller.m_steer_wheel[0].getCommandForwardVel(),controller.m_steer_wheel[0].getCommandRotateAngle() ,controller.m_steer_wheel[1].getCommandForwardVel(),controller.m_steer_wheel[1].getCommandRotateAngle() );

                return true;

            }



            return true;
        }, 5*1000,1);

    }else{
        PLOGD << "add fix command";

        taskManager.addTask([&controller, &control_wheel,&command_array,&mqttMessageManager,&mqtt_sub_topic,&lua_cb]{

            mqttMessageManager.recv_message(mqtt_sub_topic.c_str(),10,0.01, lua_cb);

            command_array[1] = std::max(std::min(1.918888889f, command_array[1]),-1.918888889f);
            command_array[3] = std::max(std::min(1.918888889f, command_array[3]),-1.918888889f);

            controller.m_steer_wheel[0].getCommandForwardVel() = command_array[0];
            controller.m_steer_wheel[0].getCommandForwardVel() = command_array[1];
            controller.m_steer_wheel[1].getCommandForwardVel() = command_array[2];
            controller.m_steer_wheel[1].getCommandForwardVel() = command_array[3];

            PLOGD << "send fix command: " << command_array[0] << ", " << command_array[1] << ", " <<command_array[2] << ", " << command_array[3];

            control_wheel(true, false, command_array[0],command_array[1],command_array[2],command_array[3] );

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

    control_wheel(false,true, 0.0, 0.0 , 0.0, 0.0);



    mqttMessageManager.stop();

    rosMessageManager.stop();
    device.stop();


    return 0;

}


int main(int argc, char** argv){
    test_tec(argc,argv);

    return 0;
}
