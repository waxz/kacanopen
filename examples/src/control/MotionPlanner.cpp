//
// Created by waxz on 6/24/23.
//

#include "MotionPlanner.h"
#include "math/BezierGenerator.h"
#include <plog/Log.h> // Step1: include the headers

namespace common_message{



}

namespace control{

    void MotionPlanner::updateMapBasePose(const common_message::TransformStamped &pose) {

        double tx,ty,tz, qw,qx,qy,qz,  roll,pitch,yaw;
        transform::extractSe3(pose.transform,tx,ty,tz,qw,qx,qy,qz);
        transform::toEulerAngle(yaw, pitch,roll,qw,qx,qy,qz );
        m_map_base_buffer.add(pose.time, transform::Transform2d(tx,ty,yaw));
    }

    void MotionPlanner::updateOdomBasePose(const common_message::TransformStamped &pose) {

        double tx,ty,tz, qw,qx,qy,qz,  roll,pitch,yaw;
        transform::extractSe3(pose.transform,tx,ty,tz,qw,qx,qy,qz);
        transform::toEulerAngle(yaw, pitch,roll,qw,qx,qy,qz );
        m_odom_base_buffer.add(pose.time, transform::Transform2d(tx,ty,yaw));
    }
    void MotionPlanner::updateOdom(const common_message::Odometry &odom) {

        m_actual_odom.time = common::FromUnixNow();
        m_actual_odom.value = odom;
    }

    void MotionPlanner::reset() {
        m_path_node_id = 0;
        m_path_node_id_interpolate = 0.0;
        m_closest_path_node_id = 0;
//        m_map_base_buffer.clear();
//        m_odom_base_buffer.clear();
        m_track_path_info.clear();
        m_stable_pose_get = false;
        m_task_state = TaskState::idle;
        m_map_odom_record.set(0.0,0.0,0.0);
        resetPlanner();
    }



    bool MotionPlanner::getStablePose() {


        if(m_map_base_buffer.size() < 3 || m_odom_base_buffer.size() < 3){
            return false;
        }
        common::Time check_point = m_map_base_buffer.back().time > m_odom_base_buffer.back().time ? m_odom_base_buffer.back().time : m_map_base_buffer.back().time;


        bool need_update = false;
        transform::Transform2d odom_base, map_base_predict, map_base;
        odom_base = m_odom_base_buffer.back().value;
        map_base = m_map_base_buffer.back().value;

        map_base_predict = m_map_odom_record * odom_base;
        float predict_error_dist = std::sqrt((map_base_predict.x() - map_base.x())*(map_base_predict.x() - map_base.x()) + (map_base_predict.y() - map_base.y())*(map_base_predict.y() - map_base.y()) );
        float predict_error_angle = std::abs(map_base_predict.yaw() - angle_normalise(map_base.yaw(), map_base_predict.yaw()));


        need_update = predict_error_dist > m_planner_config.stable_pose_dist_tolerance || predict_error_angle > m_planner_config.stable_pose_angle_tolerance;

        if(need_update || !m_stable_pose_get){

            bool ok1 = m_odom_base_buffer.query(check_point,odom_base);
            bool ok2 = m_map_base_buffer.query(check_point,map_base);

            m_stable_pose_get = ok1 && ok2;
            m_map_odom_record = map_base*odom_base.inverse();
        }

        m_actual_pose.value = m_map_odom_record*odom_base;

        m_actual_pose.time = check_point;

        return m_stable_pose_get;


    }

    const std::string &MotionPlanner::getTaskFrame() {
        return m_task_frame;
    }

    void MotionPlanner::requestGoal(const common_message::PoseStamped& goal) {


        m_task_frame = goal.header.frame_id;
        if(m_task_state != TaskState::idle ){
            stop();
        }
        if(goal.header.frame_id == "cancel"){

            stop();
            return;
        }
        m_global_path.time = goal.header.stamp;
        m_local_path.time = goal.header.stamp;

        bool ok = getStablePose();

        float time_diff = common::ToMillSeconds(m_global_path.time - m_actual_pose.time)*1e-3;
        if(!ok || ( std::abs( time_diff) > 0.5) ){
           std::cout << "reject goal , get_sable_pose: " << ok << ", check time_diff: " << time_diff<< std::endl;
            return;
        }

        std::vector<std::array<float,2>>  path;


        float PA[2] = {m_actual_pose.value.x(),m_actual_pose.value.y()};

        float PD[2] = {static_cast<float>(goal.pose.position.x), static_cast<float>(goal.pose.position.y) };
        float PB[2] = {0.5f*(PA[0] + PD[0]),0.5f*(PA[1] + PD[1])};
        float PC[2] = {0.5f*(PA[0] + PD[0]),0.5f*(PA[1] + PD[1])};
        float step = 0.1;
        math::buildBezier(PA, PB,PC, PD, step,path);

        size_t pose_num = path.size();
        m_global_path.value.resize(pose_num);

        float roll,pitch,yaw;
        transform::toEulerAngle<float>(yaw,pitch,roll,goal.pose.orientation.w, goal.pose.orientation.x,goal.pose.orientation.y,goal.pose.orientation.z);
        for(size_t i = 0 ; i < pose_num;i++){
            m_global_path.value[i].set(path[i][0],path[i][1],yaw);
        }


        bool path_ok = checkPath();
        if(path_ok){

            m_task_state = TaskState::running_init;
        }else{
            m_task_state = TaskState::error_path;

            m_global_path.value.clear();
            m_local_path.value.clear();
        }



    }

    void MotionPlanner::requestPath(const common_message::Path & path) {

        m_task_frame = path.header.frame_id;

        if(path.poses.empty()){
            std::cout << "cancel goal , path poses size : " << path.poses.size()  << std::endl;
            return;
        }
        if(path.header.frame_id == "cancel"){

            stop();
            return;
        }
        if(path.poses.size() == 1){
            auto goal = path.poses.front();
            goal.header = path.header;
            requestGoal(goal);
            return;
        }

        if(m_task_state != TaskState::idle ){
            stop();
        }
        m_global_path.time = path.header.stamp;
        m_global_path.value.clear();
        m_local_path.time = path.header.stamp;
        m_local_path.value.clear();

        // check path start pose distance to actual_pose

        bool ok = getStablePose();

        float time_diff = common::ToMillSeconds(m_global_path.time - m_actual_pose.time)*1e-3;
        if(!ok || ( std::abs( time_diff) > 0.5) ){
            std::cout << "reject goal , get_sable_pose: " << ok << ", check time_diff: " << time_diff<< std::endl;
            return;
        }

        float start_pose_diff = std::sqrt((m_actual_pose.value.x() -  path.poses[0].pose.position.x )*(m_actual_pose.value.x() -  path.poses[0].pose.position.x ) + (m_actual_pose.value.y() -  path.poses[0].pose.position.y )*(m_actual_pose.value.y() -  path.poses[0].pose.position.y ));

        if(start_pose_diff > m_planner_config.start_pose_dist){
            std::cout << "reject goal , start_pose_diff:" << start_pose_diff << std::endl;
            return;
        }


        size_t pose_num = path.poses.size();
        m_global_path.value.resize(pose_num);
        float roll,pitch,yaw;
        for(size_t i = 0 ; i < pose_num;i++){
            transform::toEulerAngle<float>(yaw,pitch,roll,  path.poses[i].pose.orientation.w, path.poses[i].pose.orientation.x,path.poses[i].pose.orientation.y,path.poses[i].pose.orientation.z);

            m_global_path.value[i].set(path.poses[i].pose.position.x,path.poses[i].pose.position.y,yaw);
        }


        bool path_ok = checkPath();
        if(path_ok){

            m_task_state = TaskState::running_init;
        }else{
            m_task_state = TaskState::error_path;
            m_global_path.value.clear();
            m_local_path.value.clear();
        }

    }

    void MotionPlanner::stop() {
        m_global_path.value.clear();
        m_local_path.value.clear();
        m_task_state = TaskState::idle;
        reset();
    }

    bool MotionPlanner::rotate(float actual, float target) {

        m_command.command_type = CommandType::cmd_vel;
        m_command.command.resize(3);
        float& command_linear_x = m_command.command[0];
        float& command_linear_y = m_command.command[1];
        float& command_angular_z = m_command.command[2];

        float actual_angular_z = m_actual_odom.value.twist.twist.angular.z;

        command_linear_x = 0.0;
        command_linear_y = 0.0;
        command_angular_z = 0.0;

        float angle_diff = target - angle_normalise(actual , target) ;
        std::cout << "angle_diff: " << angle_diff<< "\n";


        //
        float min_rotate_vel = angle_diff > 0.0 ?  m_planner_config.first_rotate_vel_min:- m_planner_config.first_rotate_vel_min;
        float max_rotate_vel = angle_diff > 0.0 ? m_max_base_rotate_vel:-m_max_base_rotate_vel;
        float rotate_acc = angle_diff > 0.0 ? m_planner_config.first_rotate_acc:-m_planner_config.first_rotate_acc;
        float predict_slop_stop_angle = 0.5f*(actual_angular_z + min_rotate_vel)*std::abs(actual_angular_z - min_rotate_vel)/m_max_base_rotate_acc;

        float update_s  = 0.01;
        common::Time now = common::FromUnixNow();
        interpolate_time_step_1 = now;

        update_s = common::ToMicroSeconds(interpolate_time_step_1 - interpolate_time)*1e-6;
        interpolate_time_step_0 = now;

        std::cout << "m_rotate_init_angle_diff: " << m_rotate_init_angle_diff<< "\n";
        std::cout << "m_planner_config.first_rotate_angle_tolerance: " << m_planner_config.first_rotate_angle_tolerance<< "\n";

        float command_feedback = min_rotate_vel + angle_diff* m_planner_config.first_rotate_angle_p ;
        if(m_rotate_init_angle_diff > 0.0f){

            if( (angle_diff) < m_planner_config.first_rotate_angle_tolerance ){
                command_angular_z = 0.0f;
                return std::abs(actual_angular_z) < 0.001;
            }else{
                command_angular_z = min_rotate_vel + update_s*rotate_acc;
//            command_angular_z =command_angular_z > 0.0 ?  std::min(command_angular_z, max_rotate_vel):std::max(command_angular_z, max_rotate_vel);
            }

        }else{

            //predict_slop_stop_angle - angle_diff
            if( ( angle_diff) >- m_planner_config.first_rotate_angle_tolerance ){
                command_angular_z = 0.0f;
                return std::abs(actual_angular_z) < 0.001;

            }else{
                command_angular_z = min_rotate_vel + update_s*rotate_acc;
//            command_angular_z =command_angular_z > 0.0 ?  std::min(command_angular_z, max_rotate_vel):std::max(command_angular_z, max_rotate_vel);
            }

        }

        std::cout << "command_angular_z orogin: " << command_angular_z<< "\n";
        std::cout << "m_planner_config.first_rotate_vel: " << m_planner_config.first_rotate_vel<< "\n";

        if(command_angular_z > 0.0){
            command_angular_z = std::max(std::min(std::min(command_feedback,command_angular_z ), m_planner_config.first_rotate_vel), m_planner_config.first_rotate_vel_min ) ;
        }else{
            command_angular_z = std::min(std::max(std::max(command_feedback,command_angular_z ), - m_planner_config.first_rotate_vel), -m_planner_config.first_rotate_vel_min);

        }
        std::cout << "actual_angular_z: " << actual_angular_z<< "\n";
        std::cout << "min_rotate_vel: " << min_rotate_vel<< "\n";
        std::cout << "update_s: " << update_s<< "\n";
        std::cout << "rotate_acc: " << rotate_acc<< "\n";


        std::cout << "actual_angular_z: " << actual_angular_z<< "\n";
        std::cout << "command_feedback: " << command_feedback<< "\n";
        std::cout << "command_angular_z: " << command_angular_z<< "\n";

        return false;
    }

    const MotionPlanner::TaskState &MotionPlanner::getTaskState() {

        return m_task_state;
    }

    const MotionPlanner::Command& MotionPlanner::go() {


        std::cout << "m_task_state: " << static_cast<int>(m_task_state) << std::endl;

        if(m_task_state == TaskState::running_init){
            m_command.command_type = CommandType::cmd_vel;
            m_command.command.resize(3);
            std::fill(m_command.command.begin(), m_command.command.end(),0.0f);

            if(m_global_path.value.empty()){
                std::cout << "global_path is empty"  << std::endl;

                return m_command;
            }
            // check robot direction
            std::cout << "running_init" << std::endl;

            float angle_diff = angle_normalise(m_global_path.value[0].yaw(), m_actual_pose.value.yaw()) - m_actual_pose.value.yaw();

            float dist_diff = transform::diff2(m_actual_pose.value, m_global_path.value.back());

            interpolate_time = common::FromUnixNow();
            interpolate_time_step_0 = interpolate_time;
            interpolate_time_step_1 = interpolate_time;

            if(std::sqrt(dist_diff) < m_planner_config.pursuit_goal_dist ){
                m_task_state = TaskState::running_adjust_prepare;
            }else{

                if(std::abs(angle_diff )< m_planner_config.first_rotate_angle_tolerance){
                    m_task_state = TaskState::running_adjust_prepare;
                }else{
                    m_rotate_init_angle_diff =angle_diff;
                    m_task_state = TaskState::running_adjust_rotate;
                }
            }
            return m_command;

        }

        if(m_task_state == TaskState::running_adjust_rotate){
            std::cout << "running_adjust_rotate" << std::endl;

            bool ok = rotate(m_actual_pose.value.yaw(), m_global_path.value[0].yaw());
            if(ok){
                float angle_diff = angle_normalise(m_global_path.value[0].yaw(), m_actual_pose.value.yaw()) - m_actual_pose.value.yaw();
                interpolate_time = common::FromUnixNow();
                interpolate_time_step_0 = interpolate_time;
                interpolate_time_step_1 = interpolate_time;
                if(std::abs(angle_diff )< m_planner_config.first_rotate_angle_tolerance){

                    m_task_state = TaskState::running_adjust_prepare;


                }else{
                    m_rotate_init_angle_diff =angle_diff;
                    m_task_state = TaskState::running_adjust_rotate;

                }

            }
            return m_command;

        }

        if(m_task_state == TaskState::running_adjust_prepare){
            std::cout << "running_adjust_prepare" << std::endl;

            bool ok = prepare();
            if(ok){
                interpolate_time = common::FromUnixNow();
                interpolate_time_step_0 = interpolate_time;
                interpolate_time_step_1 = interpolate_time;
                m_task_state = TaskState::running_pursuit_path;
            }
            return m_command;

        }

        if(m_task_state == TaskState::running_pursuit_path){
            std::cout << "running_pursuit_path" << std::endl;
            bool ok = pursuit_path();
            if(ok){
                interpolate_time = common::FromUnixNow();
                interpolate_time_step_0 = interpolate_time;
                interpolate_time_step_1 = interpolate_time;
                m_task_state = TaskState::running_pursuit_goal;
            }
            return m_command;

        }
        if(m_task_state == TaskState::off_path){
            std::cout << "off_path" << std::endl;
        }


        if(m_task_state == TaskState::running_pursuit_goal){
            std::cout << "running_pursuit_goal" << std::endl;
            bool ok = pursuit_goal();
            if(ok){
                interpolate_time = common::FromUnixNow();
                interpolate_time_step_0 = interpolate_time;
                interpolate_time_step_1 = interpolate_time;
                reset();
                m_task_state = TaskState::finished;
            }
            return m_command;

        }










        return m_command;
    }

    void DoubleSteerMotionPlanner::initBase(const std::vector<SteerWheelBase> &config) {

        if(config.empty()){
            return;
        }

        m_max_base_forward_vel = config[0].max_forward_vel;
        m_max_base_forward_acc = config[0].max_forward_acc;
        m_min_base_forward_vel = config[0].min_forward_vel;

        m_max_base_rotate_vel = config[0].max_forward_vel/std::sqrt(config[0].mount_position_x*config[0].mount_position_x + config[0].mount_position_y*config[0].mount_position_y  );

        m_max_base_rotate_acc = config[0].max_forward_acc/std::sqrt(config[0].mount_position_x*config[0].mount_position_x + config[0].mount_position_y*config[0].mount_position_y  );

        m_min_base_rotate_vel = config[0].min_forward_vel/std::sqrt(config[0].mount_position_x*config[0].mount_position_x + config[0].mount_position_y*config[0].mount_position_y  );


        m_max_base_forward_angle_vel = config[0].max_rot_vel;
        m_max_base_forward_angle_acc = config[0].max_rot_acc;

        m_max_wheel_rotate_angle = config[0].max_rot_angle;


        m_base_config_done = true;

    }

    void DoubleSteerMotionPlanner::updateWheelState(float forward_vel_1, float rotate_angle_1, float forward_vel_2,
                                                    float rotate_angle_2) {

        m_wheel_state.time = common::FromUnixNow();
        m_wheel_state.value[0] = forward_vel_1;
        m_wheel_state.value[1] = rotate_angle_1;
        m_wheel_state.value[2] = forward_vel_2;
        m_wheel_state.value[3] = rotate_angle_2;

    }
    bool DoubleSteerMotionPlanner::prepare() {

        m_command.command_type = CommandType::cmd_vel;
        m_command.command.resize(3);
        std::fill(m_command.command.begin(), m_command.command.end(),0.0f);


        PLOGD << "m_wheel_state.value[1]: " << m_wheel_state.value[1];
        PLOGD << "m_wheel_state.value[3]: " << m_wheel_state.value[3];

        //todo: remove spark point
//        return true;

        float actual_angular_z = m_actual_odom.value.twist.twist.angular.z;



        std::cout <<"actual_angular_z: " << actual_angular_z << std::endl;

        if(std::abs(actual_angular_z) < 1e-3
//        && std::abs(m_wheel_state.value[1] ) < 0.01
//        &&std::abs(m_wheel_state.value[3] ) < 0.01
        &&  std::abs(m_wheel_state.value[0]) < 0.001
        &&  std::abs(m_wheel_state.value[2]) < 0.001 ){


            return true;
        }else{
            return false;

        }


        return false;

    }

    bool DoubleSteerMotionPlanner::checkPath() {

        // todo: remove
#if 0
        m_start_wheel_angle = 0.0;
        return true;
#endif
        auto& path = m_global_path.value;
//
//        if ( std::abs(common::ToMillSeconds(m_actual_odom.time - m_global_path.time)) > 1000 ){
//            return false;
//        }

        size_t pose_num = path.size();
        if(pose_num < 2){
            return true;
        }

        float path_start_yaw_map = 0.0;
        path_start_yaw_map = std::atan2(m_global_path.value[1].y() - m_global_path.value[0].y() ,m_global_path.value[1].x() - m_global_path.value[0].x()  );

        float base_start_yaw_map = m_global_path.value[0].yaw();
        float wheel_start_yaw_base = path_start_yaw_map - base_start_yaw_map;

        wheel_start_yaw_base = angle_normalise_zero(wheel_start_yaw_base);
        float init_wheel_yaw_base[2] = {0.0,-M_PIf32};
//        init_wheel_yaw_base[1] = (wheel_start_yaw_base > 0.0) ? wheel_start_yaw_base- M_PIf32 :wheel_start_yaw_base + M_PIf32;
//        init_wheel_yaw_base[0] = angle_normalise_zero(init_wheel_yaw_base[0]);
//        init_wheel_yaw_base[1] = angle_normalise_zero(init_wheel_yaw_base[1]);

        bool init_wheel_yaw_ok[2] = {true, true};


        for(size_t i = 0 ; i < pose_num-1;i++){

            float path_dist = std::sqrt((m_global_path.value[i+1].y() - m_global_path.value[i].y())*(m_global_path.value[i+1].y() - m_global_path.value[i].y()) + (m_global_path.value[i+1].x() - m_global_path.value[i].x() )*(m_global_path.value[i+1].x() - m_global_path.value[i].x() ));
            if(path_dist < 0.0001){
                continue;
            }
            float path_yaw = std::atan2(m_global_path.value[i+1].y() - m_global_path.value[i].y() ,m_global_path.value[i+1].x() - m_global_path.value[i].x()  );

            float base_yaw = m_global_path.value[i].yaw();

            init_wheel_yaw_ok[0] = init_wheel_yaw_ok[0] && std::abs(angle_normalise_zero(init_wheel_yaw_base[0] + path_yaw - base_yaw)) < m_max_wheel_rotate_angle;
            init_wheel_yaw_ok[1] = init_wheel_yaw_ok[1] && std::abs(angle_normalise_zero(init_wheel_yaw_base[1] + path_yaw - base_yaw)) < m_max_wheel_rotate_angle;
        }
        PLOGD << "init_wheel_yaw_ok: " << init_wheel_yaw_ok[0] << ", " << init_wheel_yaw_ok[1];

        if(init_wheel_yaw_ok[0] && init_wheel_yaw_ok[1]){
            m_start_wheel_angle = 0.0f;
            PLOGD << "m_start_wheel_angle: " << m_start_wheel_angle;
            return true;
        }
        else if(init_wheel_yaw_ok[0] || init_wheel_yaw_ok[1] ){

            float angle_diff[2] = {100.0f,100.0f};

            for(size_t i = 0 ; i  < 2;i++){

                angle_diff[i] = init_wheel_yaw_ok[i] ? std::abs(init_wheel_yaw_base[i]) : 1000.0f;
            }


            m_start_wheel_angle = angle_diff[0] < angle_diff[1] ? M_PI_2f32 : -M_PI_2f32;
            PLOGD << "m_start_wheel_angle: " << m_start_wheel_angle;

            return true;
        }else{
            return false;
        }


        return false;
    }

//    bool DoubleSteerMotionPlanner::createLocalPath() {
//
//
//        return createLocalPath_v2();
//    }
//
    bool DoubleSteerMotionPlanner::createLocalPath(){

        // time
        common::Time now = common::FromUnixNow();
        interpolate_time_step_1 = now;

        float abs_update_s = common::ToMicroSeconds(interpolate_time_step_1 - interpolate_time)*1e-6;

        float relative_update_s = common::ToMicroSeconds(interpolate_time_step_1 - interpolate_time_step_0)*1e-6;

        interpolate_time_step_0 = now;

        //time





        size_t pose_num = m_global_path.value.size();

        auto & local_path = m_local_path.value;
        auto & global_path = m_global_path.value;

        auto& actual_pose = m_actual_pose.value;
        bool last_local_path_valid = !local_path.empty();
        bool last_local_path_goal_reach = false;

        local_path.clear();

        m_local_target.time = common::FromUnixNow();


        transform::Transform2d odom_twist(m_actual_odom.value.twist.twist.linear.x,m_actual_odom.value.twist.twist.linear.y);
        odom_twist = m_map_odom_record*odom_twist;
        float odom_twist_vel = std::sqrt(odom_twist.x()*odom_twist.x() + odom_twist.y()*odom_twist.y());


        float odom_forward_dist = odom_twist_vel * relative_update_s;
        PLOGD << "odom_twist_vel: " << odom_twist_vel;

        float wheel_max_angle_change = relative_update_s * m_max_base_forward_angle_vel;
        PLOGD << "wheel_max_angle_change: " << wheel_max_angle_change;

        /*
         slide window

         ------PS---------------PE------------------


       Path: P1 P2 P3 P4 P5 P6 P7 P8 P9 P10 P11

             R


        R: robot
        Path: global path
        P: node on Path
        PS: window start
        PE: window end

        if dist_R_to_PS < thresh, push PS forward


         m_path_node_id_interpolate represent PS between Path node
         */
        float slider_window_start_max_dist = 0.4;

        if(pose_num == 1){
            m_local_target.value = global_path.back();
            return true;
        }



        float first_track_dist = 0.1;

        PLOGD << "m_path_node_id: " << m_path_node_id;
        // deal with steer angle limitation

        if(m_path_node_id == 0){
            interpolate_time = now;

            const auto& PS = global_path[m_path_node_id];
            size_t next_id = m_path_node_id+1;
            for(size_t i =m_path_node_id+1; i<pose_num-1; i++ ){
                if(transform::diff2(global_path[m_path_node_id+1],global_path[m_path_node_id+1]) > 0.0001){
                    next_id = i;
                    break;
                }
            }
            PLOGD << "next_id: " << next_id;

            const auto& PS_1 = global_path[next_id];
            float path_yaw_map = std::atan2(PS_1.y() - PS.y(),PS_1.x() - PS.x() );

            float dist_base_PS = std::sqrt(transform::diff2(PS, actual_pose));
            float dist_PS_1 = std::sqrt(transform::diff2(PS, PS_1));

            float actual_yaw = actual_pose.yaw();
#if 0
            float wheel_yaw = angle_normalise(path_yaw_map, actual_yaw) - actual_yaw;

            PLOGD << "check inti steer, m_start_wheel_angle: " << m_start_wheel_angle;
            PLOGD << "check inti steer, wheel_yaw: " << wheel_yaw;
            PLOGD << "check inti steer, path_yaw_map: " << path_yaw_map;

            if(m_start_wheel_angle > 0.1)
            {
                if(std::abs(wheel_yaw) > std::abs(m_start_wheel_angle))
                {
                    // flip negative to positive
                    path_yaw_map = actual_yaw + wheel_yaw*0.99f*M_PI_2f32/std::abs(wheel_yaw);
                    PLOGD << "check inti steer, path_yaw_map: " << path_yaw_map;

                }else{
                    // do nothing
//                    path_yaw_map = actual_yaw + wheel_yaw*1.01f*M_PI_2f32/std::abs(wheel_yaw);
                    PLOGD << "check inti steer, path_yaw_map: " << path_yaw_map;

                }
            }else if(m_start_wheel_angle < -0.1) {

                if(std::abs(wheel_yaw) > std::abs(m_start_wheel_angle)){

                    // do notjing
//                    path_yaw_map = actual_yaw + wheel_yaw*0.99f*M_PI_2f32/std::abs(wheel_yaw);
                    PLOGD << "check inti steer, path_yaw_map: " << path_yaw_map;

                }else {
                    path_yaw_map = actual_yaw + wheel_yaw*1.01f*M_PI_2f32/std::abs(wheel_yaw);
                    PLOGD << "check inti steer, path_yaw_map: " << path_yaw_map;

                }
            }
            PLOGD << "check inti steer, path_yaw_map: " << path_yaw_map;
            m_local_target.value.set(actual_pose.x() + 0.1*std::cos(path_yaw_map),actual_pose.y() + 0.1*std::sin(path_yaw_map), actual_yaw) ;

#endif
#if 1
            // the first local goal should define correct steer angle
            float wheel_yaw = angle_normalise(path_yaw_map, actual_yaw) - actual_yaw;

            if(m_start_wheel_angle > 0.1){
                m_prefer_steer_angle =angle_normalise_zero(wheel_yaw) ;
            }else if(m_start_wheel_angle < -0.1){
                m_prefer_steer_angle = angle_normalise_zero(wheel_yaw + M_PIf32);
            }else{
                m_prefer_steer_angle =angle_normalise_zero(wheel_yaw) ;
            }
            m_local_target.value.set(actual_pose.x() + 0.1*std::cos(path_yaw_map),actual_pose.y() + 0.1*std::sin(path_yaw_map), actual_yaw) ;

#endif

            PLOGD << "actual_pose: " << actual_pose;
            PLOGD << "PS: " << PS;
            PLOGD << "PS_1: " << PS_1;

            PLOGD << "dist_PS_1: " << dist_PS_1;
            PLOGD << "path_yaw_map: " << path_yaw_map;
            PLOGD << "m_local_target.value: " << m_local_target.value;

            transform::Transform2d adjust_point(m_local_target.value.x(),m_local_target.value.y(), path_yaw_map);
            transform::Transform2d pose_in_adjust_point;

            if(odom_twist_vel > 0.01){

                bool find_valid_pose = false;
                size_t choose_id = m_path_node_id;

                float allow_angle_min = 0.0f;
                float allow_angle_max = m_max_wheel_rotate_angle;
                if(m_start_wheel_angle > 0.1){
                    allow_angle_min = path_yaw_map > 0.0f ? 0.0f : -m_max_wheel_rotate_angle;
                    allow_angle_max = path_yaw_map > 0.0f ? m_max_wheel_rotate_angle :0.0f;

                }else if(m_start_wheel_angle < -0.1){

                    allow_angle_min = path_yaw_map > 0.0f ? M_PIf32 - m_max_wheel_rotate_angle : -M_PIf32;
                    allow_angle_max = path_yaw_map > 0.0f ? M_PIf32 :-M_PIf32 + m_max_wheel_rotate_angle;

                }else{

                    allow_angle_min = path_yaw_map > 0.0f ? 0.0f : -m_max_wheel_rotate_angle;
                    allow_angle_max = path_yaw_map > 0.0f ? m_max_wheel_rotate_angle :0.0f;

                }
                    PLOGD << "check allow_angle_min: " << allow_angle_min;
                    PLOGD << "check allow_angle_max: " << allow_angle_max;
                for(size_t i = m_path_node_id + 1; i < pose_num;i++){
                    const auto& P = global_path[i];
                    pose_in_adjust_point = adjust_point.inverse() *P;
                    float base_to_target_yaw = std::atan2(P.y() - actual_pose.y(), P.x() - actual_pose.x());

                    float check_wheel_yaw = angle_normalise(base_to_target_yaw, actual_yaw) - actual_yaw;

                    float dist_to_base   = std::sqrt(transform::diff2(P,actual_pose));
//                    PLOGD << "check P: " << P;
//                    PLOGD << "check base_to_target_yaw: " << base_to_target_yaw;
//                    PLOGD << "check base_yaw: " << base_yaw;
//                    PLOGD << "check path_yaw_map: " << path_yaw_map;
                    PLOGD << "check check_wheel_yaw: " << check_wheel_yaw;

                    choose_id = i;

                    // the second local goal should follow steer angle constrain
                    if(pose_in_adjust_point.x() > m_planner_config.first_track_dist
                    && check_wheel_yaw > allow_angle_min
                    && check_wheel_yaw < allow_angle_max
//                    && std::abs(check_wheel_yaw) < 0.1f
//                    && dist_to_base > 0.4
                    ){
                        find_valid_pose = true;
                        break;
                    }
                }
                PLOGD << "find_valid_pose: " << find_valid_pose;

                if(find_valid_pose){
                    m_path_node_id = choose_id;
                    m_path_node_id_interpolate = m_path_node_id;
                    m_closest_path_node_id = m_path_node_id;
                }
                PLOGD << "m_path_node_id: " << m_path_node_id;

                return find_valid_pose;
            }else{
                PLOGD << "base is not moving!"  ;

            }
            return true;
        }
        // deal with steer angle limitation


        {
            size_t best_id = m_closest_path_node_id;
            float best_dist = 1000000.0;

            for(size_t i = m_closest_path_node_id; i < pose_num ;i++){
                float dist = transform::diff2(actual_pose,global_path[i]);
                best_dist = (dist < best_dist) ? (best_id = i, dist) : best_dist;
                 if(dist > 1.0) break;
            }
            m_closest_path_node_id = best_id;
            PLOGD << "m_closest_path_node_id: " << m_closest_path_node_id;
            best_dist = std::sqrt(best_dist);
            PLOGD << "best_dist: " << best_dist;
#if 0
            if(best_dist > 1.1){



                m_task_state = TaskState::off_path;
                return false;
            }
#endif

            m_track_path_info.clear();
            for(size_t i = m_closest_path_node_id; i < pose_num ;i++){
                float dist = transform::diff2(actual_pose,global_path[i]);
                best_dist = (dist < best_dist) ? (best_id = i, dist) : best_dist;
                if(dist > 1.0) break;
            }




        }

        PLOGD << "m_path_node_id: " << m_path_node_id;

        while(m_path_node_id < pose_num-1){
            const auto& PS = global_path[m_path_node_id];

            size_t next_id = m_path_node_id+1;
#if 1
            for(size_t i =m_path_node_id+1; i<pose_num-1; i++ ){
                float next_dist = transform::diff2(global_path[i],global_path[i-1]);
                if(next_dist < 0.0001){
//                    m_path_node_id = i;
                }
                if( next_dist> 0.0001){
                    next_id = i;
                    break;
                }
            }
#endif
            PLOGD << "next_id: " << next_id;

            const auto& PS_1 = global_path[next_id];

            size_t PE_id = std::min(m_path_node_id + 5,pose_num-1);
            const auto& PE = global_path[PE_id];



            float path_yaw_map = std::atan2(PS_1.y() - PS.y(),PS_1.x() - PS.x() );

            float path_yaw_map_2 = std::atan2(PE.y() - PS.y(),PE.x() - PS.x() );

            float base_to_path_yaw_map = std::atan2(PS.y() - actual_pose.y(), PS.x() - actual_pose.x() );

            transform::Transform2d adjust_point(actual_pose.x(),actual_pose.y(), path_yaw_map);
            transform::Transform2d pose_in_adjust_point;

            pose_in_adjust_point = adjust_point.inverse() *PS;




            float dist_base_PS = std::sqrt(transform::diff2(PS, actual_pose));
            float dist_PS_1 = std::sqrt(transform::diff2(PS, PS_1));

            float base_to_target_yaw = std::atan2(PS.y() - actual_pose.y(), PS.x() - actual_pose.x());

            float check_wheel_yaw = angle_normalise(base_to_target_yaw, path_yaw_map) - path_yaw_map;


            float relative_ratio = odom_forward_dist/dist_PS_1;

            PLOGD << "dist_base_PS: " << dist_base_PS;
            PLOGD << "pose_in_adjust_point: " << pose_in_adjust_point;

//            if(dist_PS_1< slider_window_start_max_dist)


            if( pose_in_adjust_point.x()  < m_planner_config.max_track_dist)
            {


                if( //dist_PS_1 < 1e-3 ||

                    (  m_path_node_id_interpolate += odom_forward_dist/dist_PS_1 , m_path_node_id_interpolate >=  next_id)){


                    m_path_node_id = next_id;
                    m_path_node_id_interpolate = std::max(static_cast<float>(m_path_node_id), m_path_node_id_interpolate) ;
//                    continue;
                }
                if(//dist_PS_1 > 1e-3 &&
                   pose_in_adjust_point.x() > m_planner_config.first_track_dist
                        )
                {
                    break;
                }else{
                    m_path_node_id = next_id;
                    m_path_node_id_interpolate = std::max(static_cast<float>(m_path_node_id), m_path_node_id_interpolate) ;
                }

            }else{

                break;
            }
#if 0
            // track to the future point,
            // the further the point is, the bigger offset will be
            if(dist_base_PS < slider_window_start_max_dist
            && (

                    dist_PS_1 < 1e-3
                    || pose_in_adjust_point.x() < 0.1
                    ||odom_forward_dist > dist_PS_1
                    || (  m_path_node_id_interpolate += odom_forward_dist/dist_PS_1 , m_path_node_id_interpolate >= m_path_node_id + 1)

                    )
            ){

                m_path_node_id = m_path_node_id +1;
                m_path_node_id_interpolate = std::max(static_cast<float>(m_path_node_id), m_path_node_id_interpolate) ;
            }
#endif

            if(pose_in_adjust_point.x() > m_planner_config.first_track_dist){
                break;
            }else{
                m_path_node_id = m_path_node_id +1;
                m_path_node_id_interpolate = std::max(static_cast<float>(m_path_node_id), m_path_node_id_interpolate) ;
            }

        }






        {
            float min_track_dist= 0.1*0.1;

            transform::Transform2d track_pose = actual_pose;


            size_t converge_id = m_path_node_id;
            PLOGD << "add local_path";
            PLOGD << "m_path_node_id: " << m_path_node_id;
            PLOGD << "m_path_node_id_interpolate: " << m_path_node_id_interpolate;

            PLOGD << "m_planner_config.pursuit_path_angle_converge:" << m_planner_config.pursuit_path_angle_converge;
#if 0
            {
                transform::Transform2d m_path_node_id_interpolate_pose = transform::interpolate(m_path_node_id_interpolate -m_path_node_id , global_path[m_path_node_id],global_path[m_path_node_id+1]);

                track_pose = transform::interpolate(m_planner_config.pursuit_path_interpolate_step, track_pose,m_path_node_id_interpolate_pose);

                local_path.push_back(track_pose);
            }
#endif
            for(size_t i = m_path_node_id  ; i < pose_num-1 ;i++){

#if 0
                for(size_t j = i; j < pose_num-1;j++){

                    float track_dist = (global_path[i].x() - track_pose.x())*(global_path[i].x() - track_pose.x()) + (global_path[i].y() - track_pose.y())*(global_path[i].y() - track_pose.y());
                    if(track_dist > min_track_dist){
                        break;
                    }
                    i = j;
                }
#endif
                track_pose = transform::interpolate(m_planner_config.pursuit_path_interpolate_step, track_pose,global_path[i]);
                PLOGD << "add node: " << i;

                local_path.push_back(track_pose);

                float path_yaw_map = std::atan2(global_path[i+1].y() - global_path[i].y(),global_path[i+1].x() - global_path[i].x() );
                float base_to_path_yaw_map = std::atan2(global_path[i].y() - track_pose.y(),global_path[i].x() - track_pose.x() );
                path_yaw_map = angle_normalise(path_yaw_map, base_to_path_yaw_map);
                float angle_diff = path_yaw_map - base_to_path_yaw_map;
                converge_id = i;
                PLOGD << "angle_diff" << angle_diff;

                if( std::abs(angle_diff) < m_planner_config.pursuit_path_angle_converge ){
                    break;
                }
            }



            bool plan_ok = !local_path.empty();
            if(plan_ok){
#if 0

                float local_goal_x = 0.0;
                float local_goal_y = 0.0;
                float local_goal_yaw_start = local_path.front().yaw();
                float local_goal_yaw = 0.0;

                for(size_t i = 0 ; i < local_path.size() ;i++){
                    local_goal_x += local_path[i].x();
                    local_goal_y += local_path[i].y();
                    local_goal_yaw += angle_normalise(local_path[i].yaw() ,local_goal_yaw_start ) ;
                }

                local_goal_x /= local_path.size();
                local_goal_y /= local_path.size();
                local_goal_yaw /= local_path.size();

                m_local_target.value.set(local_goal_x,local_goal_y,local_goal_yaw);
#endif

                float base_first_target_angle = std::atan2(local_path.front().y() - actual_pose.y(), local_path.front().x() - actual_pose.x());
                float base_last_target_angle = std::atan2(local_path.back().y() - actual_pose.y(), local_path.back().x() - actual_pose.x());
                PLOGD << "local_path.size(): " << local_path.size();

                PLOGD << "base_first_target_angle: " << base_first_target_angle;
                PLOGD << "base_last_target_angle: " << base_last_target_angle;

                m_local_target.value = local_path.front();
            }
#if 0

            for(size_t i = converge_id; i < pose_num;i++){
                local_path.push_back(global_path[i]);
            }
#endif
            return plan_ok;

        }


        return false;
    }
    bool DoubleSteerMotionPlanner::createLocalPath_v1()  {

        size_t pose_num = m_global_path.value.size();

        auto & local_path = m_local_path.value;
        auto & global_path = m_global_path.value;

        auto& actual_pose = m_actual_pose.value;
        local_path.clear();

        m_local_target.time = common::FromUnixNow();

        {
            float min_track_dist= 0.1*0.1;
            float min_split_dist= 0.01*0.01;
            float max_track_dist= 0.3*0.3;

            float max_base_dist= 0.5*0.5;


            if(m_path_node_id == 0){
                size_t choose_id = m_path_node_id;

                for(size_t i = m_path_node_id ; i < pose_num ;i++){

                    float base_dist = (global_path[i].x() - actual_pose.x())*(global_path[i].x() - actual_pose.x()) + (global_path[i].y() - actual_pose.y())*(global_path[i].y() - actual_pose.y());
                    if(base_dist > min_track_dist){
                        choose_id = i;
                        break;
                    }
                }
                m_path_node_id = std::max(choose_id , m_path_node_id);

            }else{
#if 1
                size_t choose_id = m_path_node_id;
                float best_base_dist = 1000.0;
                for(size_t i = m_path_node_id ; i < pose_num ;i++){

                    float base_dist = (global_path[i].x() - actual_pose.x())*(global_path[i].x() - actual_pose.x()) + (global_path[i].y() - actual_pose.y())*(global_path[i].y() - actual_pose.y());
                    float track_dist = (global_path[i].x() - global_path[m_path_node_id].x())*(global_path[i].x() - global_path[m_path_node_id].x()) + (global_path[i].y() - global_path[m_path_node_id].y())*(global_path[i].y() - global_path[m_path_node_id].y());

                    if(base_dist < best_base_dist){
                        choose_id = i;
                        best_base_dist = base_dist;
                    }
                    if(base_dist > max_base_dist ){
                        break;
                    }
                }
                m_path_node_id = std::max(choose_id , m_path_node_id);

#endif
            }



            size_t converge_id = m_path_node_id;

            transform::Transform2d track_pose = actual_pose;

            for(size_t i = m_path_node_id ; i < pose_num-1 ;i++){

                float dist = (global_path[i].x() - global_path[i+1].x())*(global_path[i].x() - global_path[i+1].x()) + ( global_path[i].y() - global_path[i+1].y())*(global_path[i].y() - global_path[i+1].y());

                if(dist < min_split_dist){
                    continue;
                }


                for(size_t j = i; j < pose_num-1;j++){

                    float track_dist = (global_path[i].x() - track_pose.x())*(global_path[i].x() - track_pose.x()) + (global_path[i].y() - track_pose.y())*(global_path[i].y() - track_pose.y());
                    if(track_dist > min_track_dist){
                        break;
                    }
                    i = j;
                }
                float base_dist = (global_path[i].x() - actual_pose.x())*(global_path[i].x() - actual_pose.x()) + (global_path[i].y() - actual_pose.y())*(global_path[i].y() - actual_pose.y());


                track_pose = transform::interpolate(m_planner_config.pursuit_path_interpolate_step, track_pose,global_path[i]);
                PLOGD << "add node: " << i;

                local_path.push_back(track_pose);

                float path_yaw_map = std::atan2(global_path[i+1].y() - global_path[i].y(),global_path[i+1].x() - global_path[i].x() );
                float base_to_path_yaw_map = std::atan2(global_path[i].y() - track_pose.y(),global_path[i].x() - track_pose.x() );
                path_yaw_map = angle_normalise(path_yaw_map, base_to_path_yaw_map);
                float angle_diff = path_yaw_map - base_to_path_yaw_map;
                converge_id = i;

                if(
                        //std::abs(angle_diff) < m_planner_config.pursuit_path_angle_converge
//                ||
                base_dist > max_track_dist
                ){
                    break;
                }
            }

            m_path_node_id = std::max(converge_id, m_path_node_id) ;


            bool plan_ok = !local_path.empty();

            if(plan_ok){

                float local_goal_x = 0.0;
                float local_goal_y = 0.0;
                float local_goal_yaw_start = local_path.front().yaw();
                float local_goal_yaw = 0.0;

                float local_goal_cnt = 0.0;
                for(size_t i = 0 ; i < local_path.size() ;i++){
                    local_goal_x += local_path[i].x();
                    local_goal_y += local_path[i].y();
                    local_goal_yaw += angle_normalise(local_path[i].yaw() ,local_goal_yaw_start ) ;
                }

                local_goal_x /= local_path.size();
                local_goal_y /= local_path.size();
                local_goal_yaw /= local_path.size();

                m_local_target.value.set(local_goal_x,local_goal_y,local_goal_yaw);
                m_local_target.value = local_path.front();

            }

            return plan_ok;
        }




        return false;
        //

//        local_path.push_back(actual_pose);
        float min_track_dist= 0.2*0.2;

        {

            size_t choose_id = m_path_node_id;
            float base_to_path_dist =  0.1;

            float min_predict_dist = 0.1;
            for(size_t i = m_path_node_id -1 ; i < pose_num-1; i++){

                float track_dist = (global_path[i].x() - global_path[i+1].x())*(global_path[i].x() - global_path[i+1].x()) + ( global_path[i].y() - global_path[i+1].y())*(global_path[i].y() - global_path[i+1].y());


                if(track_dist < 0.001){
                    choose_id = i;
                    continue;
                }

                float  dist = std::sqrt((global_path[i].y() - actual_pose.y())*(global_path[i].y() - actual_pose.y()) + (global_path[i].x() - actual_pose.x())*(global_path[i].x() - actual_pose.x()) );

                if( //(m_path_node_id == 0) &&
                (dist < base_to_path_dist ||dist <  min_predict_dist) ){
                    if(i == m_path_node_id){
                        base_to_path_dist = dist;
                    }
                    choose_id = i;

                    continue;
                }

                float path_yaw_map = std::atan2(global_path[i+1].y() - global_path[i].y(),global_path[i+1].x() - global_path[i].x() );
                float base_to_path_yaw_map = std::atan2(global_path[i].y() - actual_pose.y(),global_path[i].x() - actual_pose.x() );
                path_yaw_map = angle_normalise(path_yaw_map, base_to_path_yaw_map);
                float angle_diff = path_yaw_map - base_to_path_yaw_map;


                if(std::abs(angle_diff) < M_PI_2f32 ){
                    choose_id = i;
                    break;
                }

            }
            m_path_node_id = std::max(choose_id, m_path_node_id) ;

        }

        transform::Transform2d track_pose = actual_pose;


        size_t converge_id = m_path_node_id;
        PLOGD << "add local_path";
        PLOGD << "m_path_node_id: " << m_path_node_id;

        for(size_t i = m_path_node_id ; i < pose_num-1 ;i++){


            for(size_t j = i; j < pose_num-1;j++){

                float track_dist = (global_path[i].x() - track_pose.x())*(global_path[i].x() - track_pose.x()) + (global_path[i].y() - track_pose.y())*(global_path[i].y() - track_pose.y());
                if(track_dist > min_track_dist){
                    break;
                }
                i = j;
            }
            track_pose = transform::interpolate(m_planner_config.pursuit_path_interpolate_step, track_pose,global_path[i]);
            PLOGD << "add node: " << i;

            local_path.push_back(track_pose);

            float path_yaw_map = std::atan2(global_path[i+1].y() - global_path[i].y(),global_path[i+1].x() - global_path[i].x() );
            float base_to_path_yaw_map = std::atan2(global_path[i].y() - track_pose.y(),global_path[i].x() - track_pose.x() );
            path_yaw_map = angle_normalise(path_yaw_map, base_to_path_yaw_map);
            float angle_diff = path_yaw_map - base_to_path_yaw_map;
            converge_id = i;
            if( std::abs(angle_diff) < m_planner_config.pursuit_path_angle_converge ){
                break;
            }
        }


        for(size_t i = converge_id; i < pose_num;i++){
            local_path.push_back(global_path[i]);
        }
        return true;
    }
    bool DoubleSteerMotionPlanner::pursuit_path() {


        auto & local_path = m_local_path.value;
        auto & global_path = m_global_path.value;

        size_t pose_num = global_path.size();
        auto& actual_pose = m_actual_pose.value;

        m_command.command_type = CommandType::cmd_vel;
        m_command.command.resize(3);
        float& command_linear_x = m_command.command[0];
        float& command_linear_y = m_command.command[1];
        float& command_angular_z = m_command.command[2];


        float actual_angular_z = m_actual_odom.value.twist.twist.angular.z;
        float dist_to_goal = std::sqrt(transform::diff2(actual_pose, global_path.back()));

        float full_path_len = std::sqrt(transform::diff2(actual_pose, global_path[m_path_node_id]));
        for(size_t i = m_path_node_id; i < pose_num-1;i++){
            full_path_len += std::sqrt(transform::diff2(global_path[i], global_path[i+1]));
        }

        // use full_path_len to control dynamic speed
        // s = 0.5*a*t^2
        // t = sqrt(2s/a)
        // v = a*t
        float dynamic_acc =m_planner_config.pursuit_path_forward_acc;
        float dynamic_t = std::sqrt(2.0f*full_path_len/dynamic_acc);
        float dynamic_v = dynamic_acc*dynamic_t;
        PLOGD << "full_path_len: " << full_path_len;
        PLOGD << "dynamic v : " << dynamic_v;

        bool local_path_ok = createLocalPath();

        PLOGD << "createLocalPath: " << local_path_ok;

        if(!local_path_ok){

            command_linear_x = 0.0;
            command_linear_y = 0.0;

            command_angular_z = 0.0;
            return false;

        }

        // predict future
        // wheel angle change
        // base angle change
        // forward dist

        transform::Transform2d target_pose = m_local_target.value;

        float target_yaw_map = target_pose.yaw();
        float base_to_target_yaw_map = std::atan2(target_pose.y() - actual_pose.y(),target_pose.x() - actual_pose.x() );
        float base_yaw_map = actual_pose.yaw();
        float wheel_yaw_base = angle_normalise(base_to_target_yaw_map, base_yaw_map) - base_yaw_map;

        float base_to_target_dist = std::sqrt((actual_pose.x() -target_pose.x())*(actual_pose.x() - target_pose.x()) + (actual_pose.y() - target_pose.y())*(actual_pose.y() - target_pose.y()) );

        float base_yaw_error = angle_normalise(target_yaw_map, base_yaw_map)-base_yaw_map;

        float max_rot_vel =base_yaw_error>0.0  ? m_planner_config.pursuit_path_angle_rot_vel_max:-m_planner_config.pursuit_path_angle_rot_vel_max;

        float goal_forward_vel = std::min(dynamic_v,m_planner_config.pursuit_path_forward_vel);
        float goal_angle_pid_p = m_planner_config.pursuit_path_angle_pid_p;
        PLOGD << "goal_forward_vel : " << goal_forward_vel;



        command_linear_x = goal_forward_vel*std::cos(wheel_yaw_base);
        command_linear_y = goal_forward_vel*std::sin(wheel_yaw_base);
        command_angular_z = base_yaw_error*goal_angle_pid_p;

//        command_angular_z = goal_angle_pid_p*base_yaw_error/(std::abs(full_path_len - m_planner_config.pursuit_goal_dist)/goal_forward_vel+1e-6f);
        command_angular_z = std::max(std::min(command_angular_z,m_planner_config.pursuit_path_angle_rot_vel_max ),-m_planner_config.pursuit_path_angle_rot_vel_max  );

        // forbid rotate
        //1. start
        //2. at steer angle limit
        if(m_path_node_id == 0
        || std::abs(m_wheel_state.value[1] - m_max_wheel_rotate_angle) < 0.05
        || std::abs(m_wheel_state.value[3] - m_max_wheel_rotate_angle) < 0.05

        ){
            command_angular_z = 0.0f;
        }
        PLOGD << "goal_forward_vel: " << goal_forward_vel;
        PLOGD << "goal_angle_pid_p: " << goal_angle_pid_p;
        PLOGD << "wheel_yaw_base: " << wheel_yaw_base;
        PLOGD << "base_yaw_error: " << base_yaw_error;

        PLOGD << "command_linear_x: " << command_linear_x;
        PLOGD << "command_linear_y: " << command_linear_y;
        PLOGD << "command_angular_z: " << command_angular_z;


        // reach goal range
        //dist_to_goal
        if(full_path_len < m_planner_config.pursuit_goal_dist){
//            command_linear_x = 0.0;
//            command_linear_y = 0.0;
//
//            command_angular_z = 0.0;
            return true;
        }

        return false;
    }

    bool DoubleSteerMotionPlanner::pursuit_goal() {

        auto & local_path = m_local_path.value;
        auto & global_path = m_global_path.value;

        auto& actual_pose = m_actual_pose.value;

        m_command.command_type = CommandType::cmd_vel;
        m_command.command.resize(3);
        float& command_linear_x = m_command.command[0];
        float& command_linear_y = m_command.command[1];
        float& command_angular_z = m_command.command[2];



        bool local_path_ok  = true;




        // predict future
        // wheel angle change
        // base angle change
        // forward dist

        transform::Transform2d target_pose;

        // reach goal range

        float goal_forward_vel = m_planner_config.pursuit_goal_forward_vel;
        float goal_angle_pid_p = m_planner_config.pursuit_goal_angle_pid_p;

        float goal_forward_vel_pid_p = m_planner_config.pursuit_goal_forward_vel_pid_p;
        float goal_forward_vel_min = m_planner_config.pursuit_goal_forward_vel_min;
        float goal_forward_vel_max = m_planner_config.pursuit_goal_forward_vel_max;


        float dist_to_goal = std::sqrt(
                transform::diff2(actual_pose, global_path.back())
                );

        if(dist_to_goal < m_planner_config.pursuit_direct_goal_dist){
            target_pose = global_path.back();
        }else{
            local_path_ok = createLocalPath();
            PLOGD << "createLocalPath: " << local_path_ok;

            target_pose= m_local_target.value;

        }
        if(dist_to_goal < m_planner_config.pursuit_final_goal_dist){
            goal_forward_vel = m_planner_config.pursuit_goal_final_forward_vel;
            goal_angle_pid_p = m_planner_config.pursuit_goal_final_angle_pid_p;


            goal_forward_vel_pid_p = m_planner_config.pursuit_goal_final_forward_vel_pid_p;
            goal_forward_vel_min = m_planner_config.pursuit_goal_final_forward_vel_min;
            goal_forward_vel_max = m_planner_config.pursuit_goal_final_forward_vel_max;

        }

        if(!local_path_ok){

            command_linear_x = 0.0;
            command_linear_y = 0.0;

            command_angular_z = 0.0;
            return false;

        }

        float final_path_yaw_map = std::atan2(global_path[global_path.size()-1].y() - global_path[global_path.size()-2].y(), global_path[global_path.size()-1].x() - global_path[global_path.size()-2].x());
        float target_yaw_map = target_pose.yaw();

        float base_to_target_yaw_map = std::atan2(target_pose.y() - actual_pose.y(),target_pose.x() - actual_pose.x() );
        float base_yaw_map = actual_pose.yaw();
        float wheel_yaw_base = angle_normalise(base_to_target_yaw_map, base_yaw_map) - base_yaw_map;

        float base_to_target_dist = std::sqrt(
                transform::diff2(actual_pose,target_pose)
                );

        float base_yaw_error = angle_normalise(target_yaw_map, base_yaw_map)-base_yaw_map;


        float pid_goal_forward_vel =  base_to_target_dist*goal_forward_vel_pid_p;
        pid_goal_forward_vel = std::min(std::max(pid_goal_forward_vel,goal_forward_vel_min ),goal_forward_vel_max );
        goal_forward_vel = pid_goal_forward_vel;

        command_linear_x = goal_forward_vel*std::cos(wheel_yaw_base);
        command_linear_y = goal_forward_vel*std::sin(wheel_yaw_base);
        command_angular_z = base_yaw_error*goal_angle_pid_p;



        transform::Transform2d final_check_pose(global_path.back().x(), global_path.back().y(),final_path_yaw_map );
        transform::Transform2d actual_check_pose(actual_pose.x(), actual_pose.y(),final_path_yaw_map );

        transform::Transform2d actual_pose_in_final = final_check_pose.inverse()*actual_check_pose  ;

        float final_yaw = angle_normalise(final_path_yaw_map ,base_to_target_yaw_map) - base_to_target_yaw_map;
        float final_dist = dist_to_goal*std::cos(final_yaw);
        std::cout << "dist_to_goal: "<< dist_to_goal<< ", final_yaw: " << final_yaw << ", final_dist: " << final_dist << ", pursuit_goal_reach_tolerance: " <<  m_planner_config.pursuit_goal_reach_tolerance<< std::endl;
        if(
//                dist_to_goal < m_planner_config.pursuit_direct_goal_dist &&

//        final_dist < m_planner_config.pursuit_goal_reach_tolerance
                actual_pose_in_final.x() > -m_planner_config.pursuit_goal_reach_tolerance
        ){
            command_linear_x = 0.0;
            command_linear_y = 0.0;

            command_angular_z = 0.0;
            return true;
        }

        return false;


    }

    bool DoubleSteerMotionPlanner::use_prefer_steer_angle() {
        return m_path_node_id == 0;
    }

    float DoubleSteerMotionPlanner::get_prefer_steer_angle() {
        return m_prefer_steer_angle;
    }
    void DoubleSteerMotionPlanner::resetPlanner() {



    }
}