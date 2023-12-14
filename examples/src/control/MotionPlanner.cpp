//
// Created by waxz on 6/24/23.
//

#include "MotionPlanner.h"
#include "math/BezierGenerator.h"
#include "math/geometry.h"
#include <plog/Log.h> // Step1: include the headers


#include "absl/strings/str_format.h"
#include "absl/strings/escaping.h"
#include "absl/strings/string_view.h"

#include "absl/strings/match.h"


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
        off_width_once = false;
        m_task_state = TaskState::idle;
        m_planner_state = PlannerState::idle;

        m_map_odom_record.set(0.0,0.0,0.0);
        m_forward_vel = 0.0f;
        m_forward_diff = 0.0f;
        m_forward_angle = 0.0f;
        m_rotate_vel = 0.0f;
        m_rotate_diff = 0.0f;


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
            // robot may drift at steer wheel turning

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
        auto& global_path = m_global_path.value;

        m_global_path.time = goal.header.stamp;
        m_local_path.time = goal.header.stamp;

        bool ok = getStablePose();

        float time_diff = common::ToMillSeconds(m_global_path.time - m_actual_pose.time)*1e-3;

        std::string task_stamp, pose_stamp;
        common::formatTimestamp(m_global_path.time , task_stamp);
        common::formatTimestamp(m_actual_pose.time , pose_stamp);

        PLOGF << "get task: " <<m_task_frame <<", task_stamp: " << task_stamp << ", pose_stamp: " << pose_stamp;

#if 1
        float max_time_diff = 0.5;
        if(!ok || ( std::abs( time_diff) > max_time_diff) ){
            m_task_state = TaskState::error_path;

            char buffer[100];
            sprintf(buffer,"path_error: time is not in tolerance, %.3f > %.3f", time_diff,max_time_diff);
            m_status_msg.assign(buffer);
            PLOGF << m_status_msg;
            return;
        }
#endif


        double roll,pitch,yaw;
        transform::toEulerAngle(yaw,pitch,roll,goal.pose.orientation.w, goal.pose.orientation.x,goal.pose.orientation.y,goal.pose.orientation.z);

        float base_x = m_actual_pose.value.x();
        float base_y = m_actual_pose.value.y();


        float step = 0.05;
        float goal_dist = std::sqrt( ( base_x - goal.pose.position.x )*(base_x - goal.pose.position.x) + (base_y - goal.pose.position.y)*(base_y - goal.pose.position.y));



        if(absl::StrContains(m_task_frame,"rotate")){


            global_path.resize(1);
            global_path[0].set(base_x ,base_y , yaw);

            float max_dist = 0.1;

            if(goal_dist < max_dist){
                m_task_state = TaskState::running_rotate_init;

            }else{
                m_task_state = TaskState::error_path;


                char buffer[100];
                sprintf(buffer,"rotate_error: goal_dist is not in tolerance, %.3f > %.3f", goal_dist,max_dist);
                m_status_msg.assign(buffer);
                PLOGF << m_status_msg;

            }


            return;
        }

#if 1
        if(goal_dist < m_planner_config.pursuit_goal_reach_tolerance){
            m_task_state = TaskState::finished;
            return;
        }
#endif


        size_t pose_num = goal_dist/step;
        pose_num = std::max(size_t(1) ,pose_num);
        step = goal_dist/pose_num;

        float path_yaw = std::atan2(goal.pose.position.y - base_y,goal.pose.position.x - base_x);
        float cos_yaw = std::cos(path_yaw);
        float sin_yaw = std::sin(path_yaw);
        float cos_step = step*cos_yaw;
        float sin_step = step*sin_yaw;


        global_path.resize(pose_num + 1);
        global_path[0].set(base_x ,base_y , yaw);

        float px= base_x +cos_step , py = base_y+sin_step;
        for(size_t i = 1 ; i <= pose_num;i++){
            global_path[i].set(px,py, yaw);
            px += cos_step;
            py += sin_step;
        }


#if 0
        std::vector<std::array<float,2>>  path;


        float PA[2] = {m_actual_pose.value.x(),m_actual_pose.value.y()};

        float PD[2] = {static_cast<float>(goal.pose.position.x), static_cast<float>(goal.pose.position.y) };
        float PB[2] = {0.5f*(PA[0] + PD[0]),0.5f*(PA[1] + PD[1])};
        float PC[2] = {0.5f*(PA[0] + PD[0]),0.5f*(PA[1] + PD[1])};
        math::buildBezier(PA, PB,PC, PD, step,path);



        size_t pose_num = path.size();

        global_path.resize(pose_num);

        for(size_t i = 0 ; i < pose_num;i++){
            global_path[i].set(path[i][0],path[i][1],yaw);
        }
#endif


        bool path_ok = checkPath();






        if(path_ok){

            m_task_state = TaskState::running_init;
        }else{
            m_task_state = TaskState::error_path;
            m_local_path.value.clear();
        }



    }

    void MotionPlanner::requestPath(const common_message::Path & path) {

        m_task_frame = path.header.frame_id;

        if(path.poses.empty()){
            stop();
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

        std::string task_stamp, pose_stamp;
        common::formatTimestamp(m_global_path.time , task_stamp);
        common::formatTimestamp(m_actual_pose.time , pose_stamp);

        PLOGF << "get task: " <<m_task_frame <<", task_stamp: " << task_stamp << ", pose_stamp: " << pose_stamp;


        float time_diff = common::ToMillSeconds(m_global_path.time - m_actual_pose.time)*1e-3;
#if 1
        float max_time_diff = 0.5;
        if(!ok || ( std::abs( time_diff) > max_time_diff) ){
            m_task_state = TaskState::error_path;

            char buffer[100];
            sprintf(buffer,"path_error: time is not in tolerance, %.3f > %.3f", time_diff,max_time_diff);
            m_status_msg.assign(buffer);
            PLOGF << m_status_msg;

            return;

        }
#endif
        float start_pose_diff = std::sqrt((m_actual_pose.value.x() -  path.poses[0].pose.position.x )*(m_actual_pose.value.x() -  path.poses[0].pose.position.x ) + (m_actual_pose.value.y() -  path.poses[0].pose.position.y )*(m_actual_pose.value.y() -  path.poses[0].pose.position.y ));

#if 0
        if(start_pose_diff > m_planner_config.start_pose_dist){
            std::cout << "reject goal , start_pose_diff:" << start_pose_diff << std::endl;
            return;
        }
#endif

        float base_x = m_actual_pose.value.x();
        float base_y = m_actual_pose.value.y();


        auto& global_path = m_global_path.value;

        auto& goal =  path.poses.back();

        float goal_dist = std::sqrt( ( base_x - goal.pose.position.x )*(base_x - goal.pose.position.x) + (base_y - goal.pose.position.y)*(base_y - goal.pose.position.y));
#if 1
        if(goal_dist < m_planner_config.pursuit_goal_reach_tolerance){
            m_task_state = TaskState::finished;
            return;
        }
#endif


        // remove overlap point
        // fill large gap

        size_t pose_num = path.poses.size();
        global_path.resize(pose_num);
        double roll,pitch,yaw;

        transform::toEulerAngle(yaw,pitch,roll,  path.poses[0].pose.orientation.w, path.poses[0].pose.orientation.x,path.poses[0].pose.orientation.y,path.poses[0].pose.orientation.z);
        global_path[0].set(path.poses[0].pose.position.x,path.poses[0].pose.position.y,yaw);
        int valid_index = 1;
        for(size_t i = 1 ; i < pose_num;i++){
            transform::toEulerAngle(yaw,pitch,roll,  path.poses[i].pose.orientation.w, path.poses[i].pose.orientation.x,path.poses[i].pose.orientation.y,path.poses[i].pose.orientation.z);

            float last_x = global_path[valid_index - 1].x();
            float last_y = global_path[valid_index - 1].y();

            double dist = (path.poses[i].pose.position.x - last_x)*(path.poses[i].pose.position.x - last_x)
                    + (path.poses[i].pose.position.y - last_y)*(path.poses[i].pose.position.y - last_y);

            global_path[valid_index].set(path.poses[i].pose.position.x,path.poses[i].pose.position.y,yaw);
            bool valid = dist > 0.0001;
            valid_index += valid;
        }

#if 0
        transform::toEulerAngle<float>(yaw,pitch,roll,  path.poses.back().pose.orientation.w, path.poses.back().pose.orientation.x,path.poses.back().pose.orientation.y,path.poses.back().pose.orientation.z);

//        global_path[valid_index-1].set( 0.5f*(path.poses.back().pose.position.x + global_path[valid_index-1].x() ) , 0.5f*(path.poses.back().pose.position.y + global_path[valid_index-1].y()),yaw);
        global_path[valid_index-1].set(path.poses.back().pose.position.x,path.poses.back().pose.position.y,yaw);


#endif


#if 0
        global_path[valid_index].set(path.poses.back().pose.position.x,path.poses.back().pose.position.y,yaw);
        valid_index++;

#endif


        global_path.resize(valid_index);


        bool path_ok = checkPath();
        if(path_ok){

            m_task_state = TaskState::running_init;
        }else{
            m_task_state = TaskState::error_path;
            m_local_path.value.clear();
        }

    }

    void MotionPlanner::stop() {

        m_command.setCmd(0.0,0.0,0.0);
        m_global_path.value.clear();
        m_local_path.value.clear();
        m_task_state = TaskState::idle;
        reset();
    }

    bool MotionPlanner::rotate(float actual, float target) {

        m_command.setCmd(0.0,0.0,0.0);

        float& command_linear_x = m_command.command[0];
        float& command_linear_y = m_command.command[1];
        float& command_angular_z = m_command.command[2];

        float actual_angular_z = m_actual_odom.value.twist.twist.angular.z;

        command_linear_x = 0.0;
        command_linear_y = 0.0;
        command_angular_z = 0.0;

//        float angle_diff = target - angle_normalise(actual , target) ;
        float angle_diff = angle_normalise(target , actual) - actual ;
        angle_diff = angle_normal(angle_diff);
        std::cout << "angle_diff: " << angle_diff<< "\n";
        if( (angle_diff > m_planner_config.first_rotate_angle_tolerance &&  actual_angular_z<-0.001)
        || ((angle_diff < -m_planner_config.first_rotate_angle_tolerance &&  actual_angular_z >0.001)) ){
            std::cout << "stop first" << std::endl;
            command_angular_z = 0.0;
            return false;
        }

        if(m_rotate_init_angle_diff > 0.0f && (angle_diff) < m_planner_config.first_rotate_angle_tolerance ){
            command_angular_z = 0.0f;
            return std::abs(actual_angular_z) < 0.001;
        }
        if(m_rotate_init_angle_diff < 0.0f && (angle_diff) >- m_planner_config.first_rotate_angle_tolerance ){
            command_angular_z = 0.0f;
            return std::abs(actual_angular_z) < 0.001;
        }

#if 0
        {
            float min_rotate_vel = copysign(m_planner_config.first_rotate_vel_min, angle_diff) ;//angle_diff > 0.0 ?  m_planner_config.first_rotate_vel_min:- m_planner_config.first_rotate_vel_min;
            float max_rotate_vel =  copysign( m_max_base_rotate_vel, angle_diff) ;//angle_diff > 0.0 ? m_max_base_rotate_vel:-m_max_base_rotate_vel;
            float rotate_acc = copysign(m_planner_config.first_rotate_acc, angle_diff) ;// angle_diff > 0.0 ? m_planner_config.first_rotate_acc:-m_planner_config.first_rotate_acc;

            float update_s  = 0.01;
            common::Time now = common::FromUnixNow();
            interpolate_time_step_1 = now;

            update_s = common::ToMicroSeconds(interpolate_time_step_1 - interpolate_time_step_0)*1e-6;
            update_s = std::min(update_s, 0.02f);
            interpolate_time_step_0 = now;

            std::cout << "m_rotate_init_angle_diff: " << m_rotate_init_angle_diff<< "\n";
            std::cout << "m_planner_config.first_rotate_angle_tolerance: " << m_planner_config.first_rotate_angle_tolerance<< "\n";

            float command_feedback = min_rotate_vel + angle_diff* m_planner_config.first_rotate_angle_p ;
            if(m_rotate_init_angle_diff > 0.0f){

                if( (angle_diff) < m_planner_config.first_rotate_angle_tolerance ){
                    command_angular_z = 0.0f;
                    return std::abs(actual_angular_z) < 0.001;
                }else{

                    command_angular_z =  copysign(actual_angular_z, angle_diff) + update_s*rotate_acc;
//            command_angular_z =command_angular_z > 0.0 ?  std::min(command_angular_z, max_rotate_vel):std::max(command_angular_z, max_rotate_vel);
                }

            }else{

                //predict_slop_stop_angle - angle_diff
                if( ( angle_diff) >- m_planner_config.first_rotate_angle_tolerance ){
                    command_angular_z = 0.0f;
                    return std::abs(actual_angular_z) < 0.001;

                }else{
                    command_angular_z = copysign(actual_angular_z, angle_diff) + update_s*rotate_acc;
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
        }
#endif


        {
            float update_s  = 0.01;
            common::Time now = common::FromUnixNow();
            interpolate_time_step_1 = now;

            update_s = common::ToMicroSeconds(interpolate_time_step_1 - interpolate_time_step_0)*1e-6;
            update_s = std::min(update_s, 0.02f);
            interpolate_time_step_0 = now;

            float command_angular_z_abs = std::abs(command_angular_z);

            float min_rotate_vel = copysign(m_planner_config.first_rotate_vel_min, angle_diff);
            float rotate_acc = copysign(m_planner_config.first_rotate_acc, angle_diff) ;// angle_diff > 0.0 ? m_planner_config.first_rotate_acc:-m_planner_config.first_rotate_acc;


            float command_feedback = min_rotate_vel + angle_diff* m_planner_config.first_rotate_angle_p ;

            float command_feedback_abs = std::abs(command_feedback);

            float command_speed_up = copysign(actual_angular_z, angle_diff) + update_s*rotate_acc;
            float command_speed_up_abs = std::abs(command_speed_up);


            command_angular_z = std::min(std::min(std::max(m_planner_config.first_rotate_vel_min, command_speed_up_abs), command_feedback_abs), m_planner_config.first_rotate_vel)   ;
        }

        //

        command_angular_z = copysign(command_angular_z, angle_diff);
        std::cout << "actual_angular_z: " << actual_angular_z<< "\n";
        std::cout << "command_angular_z: " << command_angular_z<< "\n";
        return false;
    }

    const MotionPlanner::TaskState &MotionPlanner::getTaskState() {

        return m_task_state;
    }


    void MotionPlanner::go() {


        std::cout << "m_task_state: " << static_cast<int>(m_task_state) << std::endl;

        auto& global_path = m_global_path.value;

        float odom_twist_linear_x = m_actual_odom.value.twist.twist.linear.x;
        float odom_twist_linear_y = m_actual_odom.value.twist.twist.linear.y;
        float odom_twist_angular_z = m_actual_odom.value.twist.twist.angular.z;
        float odom_twist_linear = std::sqrt(odom_twist_linear_x*odom_twist_linear_x + odom_twist_linear_y*odom_twist_linear_y);

        bool odom_is_stopped = odom_twist_angular_z < 0.01 && odom_twist_linear < 0.01;

        if(m_task_state == TaskState::running_rotate_init){

            m_command.setCmd(0.0f,0.0f,0.0f);

            if(global_path.empty()){
                std::cout << "global_path is empty"  << std::endl;

                return ;
            }
            // check robot direction
            std::cout << "running_rotate_init" << std::endl;

            float angle_diff = angle_normalise(global_path[0].yaw(), m_actual_pose.value.yaw()) - m_actual_pose.value.yaw();
            angle_diff = angle_normal(angle_diff);

            float dist_diff = transform::diff2(m_actual_pose.value, global_path.back());

            interpolate_time = common::FromUnixNow();
            interpolate_time_step_0 = interpolate_time;
            interpolate_time_step_1 = interpolate_time;

            if(odom_is_stopped){

                if(std::abs(angle_diff )< m_planner_config.first_rotate_angle_tolerance){
                    m_task_state = TaskState::finished;
                }else{
                    m_rotate_init_angle_diff =angle_diff;
                    m_task_state = TaskState::running_rotate;
                }
            }
            return ;
        }

        if(m_task_state == TaskState::running_rotate){


            std::cout << "running_rotate" << std::endl;

            bool ok = rotate(m_actual_pose.value.yaw(), global_path[0].yaw());
            if(ok){
                float angle_diff = angle_normalise(global_path[0].yaw(), m_actual_pose.value.yaw()) - m_actual_pose.value.yaw();
                angle_diff = angle_normal(angle_diff);

                interpolate_time = common::FromUnixNow();
                interpolate_time_step_0 = interpolate_time;
                interpolate_time_step_1 = interpolate_time;
                if(std::abs(angle_diff )< m_planner_config.first_rotate_angle_tolerance){

                    m_task_state = TaskState::finished;


                }else{
                    m_rotate_init_angle_diff =angle_diff;
                    m_task_state = TaskState::running_rotate_init;

                }

            }
            return ;

        }

        if(m_task_state == TaskState::running_init){

            m_command.setCmd(0.0,0.0,0.0);

            if(global_path.empty()){
                std::cout << "global_path is empty"  << std::endl;

                return ;
            }
            // check robot direction
            std::cout << "running_init" << std::endl;

            float angle_diff = angle_normalise(global_path[0].yaw(), m_actual_pose.value.yaw()) - m_actual_pose.value.yaw();
            angle_diff = angle_normal(angle_diff);

            float dist_diff = transform::diff2(m_actual_pose.value, global_path.back());

            interpolate_time = common::FromUnixNow();
            interpolate_time_step_0 = interpolate_time;
            interpolate_time_step_1 = interpolate_time;

            if(odom_is_stopped){
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
            }

            return ;

        }

        if(m_task_state == TaskState::running_adjust_rotate){
            std::cout << "running_adjust_rotate" << std::endl;

            bool ok = rotate(m_actual_pose.value.yaw(), global_path[0].yaw());
            if(ok){
                float angle_diff = angle_normalise(global_path[0].yaw(), m_actual_pose.value.yaw()) - m_actual_pose.value.yaw();
                angle_diff = angle_normal(angle_diff);
                interpolate_time = common::FromUnixNow();
                interpolate_time_step_0 = interpolate_time;
                interpolate_time_step_1 = interpolate_time;
                if(std::abs(angle_diff )< m_planner_config.first_rotate_angle_tolerance){

                    m_task_state = TaskState::running_adjust_prepare;


                }else{
                    m_rotate_init_angle_diff =angle_diff;
                    m_task_state = TaskState::running_init;

                }

            }
            return ;

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
            return ;

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
            return ;

        }
        if(m_task_state == TaskState::off_path){
            std::cout << "off_path" << std::endl;

            m_command.setCmd(0.0,0.0,0.0);
            return ;
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
            return ;

        }

    }

    void DoubleSteerMotionPlanner::initBase(const std::vector<SteerWheelBase> &config) {

        if(config.size() != 2){
            return;
        }
        m_driver_controller.set_wheel(config[0],config[1]);

        m_max_base_forward_vel = config[0].max_forward_vel;
        m_max_base_forward_acc = config[0].max_forward_acc;
        m_min_base_forward_vel = config[0].min_forward_vel;

        m_max_base_rotate_vel = config[0].max_forward_vel/std::sqrt(config[0].mount_x * config[0].mount_x + config[0].mount_y * config[0].mount_y  );

        m_max_base_rotate_acc = config[0].max_forward_acc/std::sqrt(config[0].mount_x * config[0].mount_x + config[0].mount_y * config[0].mount_y  );

        m_min_base_rotate_vel = config[0].min_forward_vel/std::sqrt(config[0].mount_x * config[0].mount_x + config[0].mount_y * config[0].mount_y  );


        m_max_base_forward_angle_vel = config[0].max_rotate_vel;
        m_max_base_forward_angle_acc = config[0].max_rot_acc;

        m_max_wheel_rotate_angle = config[0].max_rotate_angle;


        m_base_config_done = true;

    }

    void DoubleSteerMotionPlanner::updateWheelState(float forward_vel_1, float rotate_angle_1, float forward_vel_2,
                                                    float rotate_angle_2) {

        m_wheel_state.time = common::FromUnixNow();
        m_wheel_state.value[0] = forward_vel_1;
        m_wheel_state.value[1] = rotate_angle_1;
        m_wheel_state.value[2] = forward_vel_2;
        m_wheel_state.value[3] = rotate_angle_2;

        m_driver_controller.updateState(forward_vel_1, rotate_angle_1, forward_vel_2, rotate_angle_2 );

    }
    bool DoubleSteerMotionPlanner::prepare() {

        m_command.setSteer( m_start_wheel_angle);

        std::cout << "actual steer angle  " << m_wheel_state.value[1] << ", " << m_wheel_state.value[3] << "\n";
        std::cout << "target steer angle  " << m_start_wheel_angle << "\n";

        //todo: remove spark point
//        return true;

        float actual_angular_z = m_actual_odom.value.twist.twist.angular.z;



        std::cout <<"actual_angular_z: " << actual_angular_z << std::endl;

        if(std::abs(actual_angular_z) < 1e-3
        && std::abs(m_wheel_state.value[1]  - m_start_wheel_angle) < 0.01
        && std::abs(m_wheel_state.value[3] - m_start_wheel_angle) < 0.01
        &&  std::abs(m_wheel_state.value[0]) < 0.001
        &&  std::abs(m_wheel_state.value[2]) < 0.001 ){


            return true;
        }else{
            return false;

        }


        return false;

    }

    bool DoubleSteerMotionPlanner::checkPath() {


        auto& global_path = m_global_path.value;

        auto& actual_pose = m_actual_pose.value;
        float actual_pose_yaw = actual_pose.yaw();
        float actual_pose_x = actual_pose.x();
        float actual_pose_y = actual_pose.y();

//
//        if ( std::abs(common::ToMillSeconds(m_actual_odom.time - m_global_path.time)) > 1000 ){
//            return false;
//        }

        size_t pose_num = global_path.size();

        // check path valid
        // check node num should >=2 to computer path direction
        if(pose_num < 2){

            return false;
        }

        // check actual_pose to target_pose distance is beyond target tolerance

        float actual_to_target_dist = std::sqrt(transform::diff2(actual_pose,global_path.back()));
#if 0
        if(actual_to_target_dist < m_planner_config.pursuit_goal_reach_tolerance){
            char buffer[100];
            sprintf(buffer,"path_error: goal is in tolerance, %.3f < %.3f", actual_to_target_dist,m_planner_config.pursuit_goal_reach_tolerance );
            m_status_msg.assign(buffer);
            return false;
        }
#endif
#if 0
        float path_start_yaw_map = 0.0;
        path_start_yaw_map = std::atan2(global_path[1].y() - global_path[0].y() ,global_path[1].x() - global_path[0].x()  );

        float base_start_yaw_map = global_path[0].yaw();
        float wheel_start_yaw_base = path_start_yaw_map - base_start_yaw_map;

        wheel_start_yaw_base = angle_normal(wheel_start_yaw_base);
        float init_wheel_yaw_base[2] = {0.0,-M_PIf32};
//        init_wheel_yaw_base[1] = (wheel_start_yaw_base > 0.0) ? wheel_start_yaw_base- M_PIf32 :wheel_start_yaw_base + M_PIf32;
//        init_wheel_yaw_base[0] = angle_normal(init_wheel_yaw_base[0]);
//        init_wheel_yaw_base[1] = angle_normal(init_wheel_yaw_base[1]);

        bool init_wheel_yaw_ok[2] = {true, true};



        for(size_t i = 0 ; i < pose_num-1;i++){

            float path_dist = std::sqrt((global_path[i+1].y() - global_path[i].y())*(global_path[i+1].y() - global_path[i].y()) + (global_path[i+1].x() - global_path[i].x() )*(global_path[i+1].x() - global_path[i].x() ));
            if(path_dist < 0.0001){
                return false;
            }
            float path_yaw = std::atan2(global_path[i+1].y() - global_path[i].y() ,global_path[i+1].x() - global_path[i].x()  );

            float base_yaw = global_path[i].yaw();

            init_wheel_yaw_ok[0] = init_wheel_yaw_ok[0] && std::abs(angle_normal(init_wheel_yaw_base[0] + path_yaw - base_yaw)) < m_max_wheel_rotate_angle;
            init_wheel_yaw_ok[1] = init_wheel_yaw_ok[1] && std::abs(angle_normal(init_wheel_yaw_base[1] + path_yaw - base_yaw)) < m_max_wheel_rotate_angle;
        }
        PLOGD << "init_wheel_yaw_ok: " << init_wheel_yaw_ok[0] << ", " << init_wheel_yaw_ok[1];

        if(init_wheel_yaw_ok[0] && init_wheel_yaw_ok[1]){
            m_start_wheel_angle = 0.0f;
            PLOGD << "m_start_wheel_angle: " << m_start_wheel_angle;
            steer_rotate_limit_ok = true;
        }
        else if(init_wheel_yaw_ok[0] || init_wheel_yaw_ok[1] ){

            float angle_diff[2] = {100.0f,100.0f};

            for(size_t i = 0 ; i  < 2;i++){

                angle_diff[i] = init_wheel_yaw_ok[i] ? std::abs(init_wheel_yaw_base[i]) : 1000.0f;
            }


            m_start_wheel_angle = angle_diff[0] < angle_diff[1] ? M_PI_2f32 : -M_PI_2f32;
            PLOGD << "m_start_wheel_angle: " << m_start_wheel_angle;

            steer_rotate_limit_ok = true;
        }else{
            steer_rotate_limit_ok = false;
        }

#endif
        bool steer_rotate_limit_ok = false;
        // todo:remove
        std::vector<transform::Transform2d> debug_path;

        {

            m_track_path_info.resize(pose_num);
            m_track_path_info[0].dist_from_last = std::sqrt(transform::diff2(actual_pose, global_path[0]));
            m_track_path_info[0].dist_from_start = std::sqrt(transform::diff2(actual_pose, global_path[0]));

            float path_direction_yaw = std::atan2(global_path[1].y() - global_path[0].y(), global_path[1].x() - global_path[0].x());
            m_track_path_info[0].direction = path_direction_yaw;
            m_track_path_info[0].direction = std::round(m_track_path_info[0].direction * 10000.0f) *0.0001f;

            m_track_path_info[0].pose.set(global_path[0].x(),global_path[0].y(), path_direction_yaw);

            m_track_path_info[0].segment_id = 1;


            int log_level = 4;

            PLOG(plog::Severity(log_level)) << "check m_track_path_info";
            for(size_t i = 1 ; i < pose_num  ;i++){
                m_track_path_info[i].dist_from_last = std::sqrt(transform::diff2(global_path[i], global_path[i-1]));
                m_track_path_info[i].dist_from_start = m_track_path_info[i-1].dist_from_start + m_track_path_info[i].dist_from_last;
                path_direction_yaw = std::atan2(global_path[i].y() - global_path[i-1].y(), global_path[i].x() - global_path[i-1].x());
                m_track_path_info[i].direction = angle_normalise(path_direction_yaw, m_track_path_info[i-1].direction);

                m_track_path_info[i].direction = std::round(m_track_path_info[i].direction * 10000.0f) *0.0001f;

                m_track_path_info[i].pose.set(global_path[i].x(),global_path[i].y(), path_direction_yaw);

                m_track_path_info[i].direction_change_from_last = m_track_path_info[i].direction - m_track_path_info[i-1].direction;

                m_track_path_info[i].direction_change_divide_dist = m_track_path_info[i].direction_change_from_last/m_track_path_info[i].dist_from_last;



                bool segment_change = (math::signum(m_track_path_info[i].direction_change_from_last) != math::signum(m_track_path_info[i-1].direction_change_from_last) )
                        || ( std::abs(m_track_path_info[i].direction_change_divide_dist) >  2.0f );

                m_track_path_info[i].segment_id = m_track_path_info[i-1].segment_id + segment_change;

            }
            float full_dist = m_track_path_info.back().dist_from_start;

            float start_pose_yaw = global_path[0].yaw();
            bool rotate_in_path = false;

            for(size_t i = 0 ; i < pose_num  ;i++){
                m_track_path_info[i].dist_to_end = full_dist - m_track_path_info[i].dist_from_start;

                m_track_path_info[i].segment_type = math::signum(m_track_path_info[i].direction_change_from_last);

                rotate_in_path = rotate_in_path || std::abs(global_path[i].yaw() - start_pose_yaw) > 0.05;

            }

            if(rotate_in_path){
                char buffer[100];
                sprintf(buffer,"path_error: path contains rotation");
                m_status_msg.assign(buffer);
                PLOGF << m_status_msg;

                return false;
            }

            float last_segment_dist =  m_track_path_info.back().dist_to_end;
            float last_segment_id =  m_track_path_info.back().segment_id;
            float last_segment_end_id =  m_track_path_info.size()-1;


            for(int i = pose_num -1 ; i >=0 ;i--){
                m_track_path_info[i].dist_to_segment_end = m_track_path_info[i].dist_to_end - last_segment_dist;


                if(m_track_path_info[i].segment_id !=last_segment_id ){

                    m_track_path_info[i].dist_to_segment_end = 0.0f;
                    last_segment_id = m_track_path_info[i].segment_id;
                    last_segment_dist =  m_track_path_info[i].dist_to_end;




                    last_segment_end_id = i;

                }

                m_track_path_info[i].segment_end_id = last_segment_end_id;
                if((i >0) && (i < pose_num -1) && (m_track_path_info[i-1].segment_id != m_track_path_info[i].segment_id)){
                    m_track_path_info[i].segment_end_id = m_track_path_info[i+1].segment_end_id;
                    m_track_path_info[i].segment_id = m_track_path_info[i+1].segment_id;
                }

            }

            size_t check_node_id = 0;
            size_t check_segment_id = 1;


            float curve_interpolate_dist = m_planner_config.curve_interpolate_dist;
            float curve_interpolate_dist_2 = 0.5f*curve_interpolate_dist;
            float steer_rotate_vel = m_planner_config.steer_rotate_vel;
            float pursuit_path_forward_vel= m_planner_config.pursuit_path_forward_vel;


            float dynamic_acc =m_planner_config.speed_down_acc;



            {
                // computer vel for each node in path
                // according to
                for(size_t i = 0 ; i < pose_num;i++){
                    float dist_from_start = m_track_path_info[i].dist_from_start;
                    float dist_to_segment_end = m_track_path_info[i].dist_to_segment_end;

                    if (   m_track_path_info[i].segment_type == 0   && dist_to_segment_end > curve_interpolate_dist_2){

                        m_track_path_info[i].direction_change_divide_dist = 0.0f;
                        m_track_path_info[i].forward_vel = pursuit_path_forward_vel;
                        m_track_path_info[i].steer_vel = 0.0f;
                        m_track_path_info[i].rotate_vel = 0.0f;

                        continue;
                    }


                    float dd = dist_from_start +  curve_interpolate_dist;
                    size_t choose_id = i;
                    for(size_t j = i + 1; j < pose_num;j++){
                        choose_id = j;
                        float dist_from_start_2 = m_track_path_info[j].dist_from_start;

                        if(dist_from_start_2 > dd){
                            break;
                        }
                    }
                    float direction_change = angle_normalise( m_track_path_info[choose_id].direction,  m_track_path_info[i].direction) - m_track_path_info[i].direction ;
                    float distance_chane = m_track_path_info[choose_id].dist_from_start - m_track_path_info[i].dist_from_start;

                    if(choose_id == i){
                        m_track_path_info[i].direction_change_divide_dist = 0.0f;
                        m_track_path_info[i].forward_vel = 0.0f;
                        m_track_path_info[i].steer_vel = 0.0f;
                        m_track_path_info[i].rotate_vel = 0.0f;

                    }else{
                        float direction_change_divide_dist = direction_change/distance_chane;
                        float forward_vel = (std::abs(direction_change_divide_dist) > 0.001f) ?steer_rotate_vel/std::abs(direction_change_divide_dist) : pursuit_path_forward_vel;

                        if(dist_to_segment_end < curve_interpolate_dist_2){

                            for(size_t j = i ; j <= choose_id;j++){
                                m_track_path_info[j].direction_change_divide_dist = direction_change_divide_dist;
                                m_track_path_info[j].forward_vel = forward_vel;
                                m_track_path_info[j].steer_vel = 0.0f;
                                m_track_path_info[j].rotate_vel = 0.0f;
                            }
                            i = choose_id;

                        }else{
                            m_track_path_info[i].direction_change_divide_dist = direction_change_divide_dist;
                            m_track_path_info[i].forward_vel = forward_vel;
                            m_track_path_info[i].steer_vel = 0.0f;
                            m_track_path_info[i].rotate_vel = 0.0f;

                        }
                    }


                }

                for(size_t i = 0 ; i < pose_num-1;i++){

                    float dynamic_t = std::sqrt(2.0f*m_track_path_info[i].dist_to_end/dynamic_acc);
                    float dynamic_v = dynamic_acc*dynamic_t;
                    m_track_path_info[i].forward_vel = std::min(m_track_path_info[i].forward_vel,dynamic_v)  ;

                }

                // set last node to zero
                m_track_path_info.back().forward_vel = m_planner_config.pursuit_goal_final_forward_vel_min;
                m_track_path_info.back().steer_vel = 0.0f;
                m_track_path_info.back().rotate_vel = 0.0f;

                float speed_down_acc = m_planner_config.speed_down_acc;

                for(int i = pose_num-2; i >= 0; i--){

                    float d = m_track_path_info[i].dist_to_end - m_track_path_info[i+1].dist_to_end;

                    float v1 = m_track_path_info[i+1].forward_vel;
                    float t1 = d/(v1);
                    float v2 = v1 + speed_down_acc*t1;
                    float v3 = 0.5f*(v1+v2);
                    float t2 = d/(v3);
                    float v4 = v1 + speed_down_acc*t2;
                    m_track_path_info[i].forward_vel = std::min(m_track_path_info[i].forward_vel,v4)  ;

                    if(m_track_path_info[i].dist_to_end  < m_planner_config.pursuit_final_goal_dist){
                        m_track_path_info[i].forward_vel = m_planner_config.pursuit_goal_final_forward_vel_min  ;
                    }else if(m_track_path_info[i].dist_to_end  < m_planner_config.pursuit_goal_dist){
                        m_track_path_info[i].forward_vel = m_planner_config.pursuit_goal_forward_vel_min  ;
                    }
                }

            }

#if 0


            while (check_node_id < pose_num){
                check_node_id = m_track_path_info[check_node_id].segment_end_id;
                float width_2 = 2*m_planner_config.pursuit_path_width;
                float width = m_planner_config.pursuit_path_width;

                if(check_node_id <  (pose_num - 1) ){

                    // search node , check distance to line
                    float search_step_1 = 0.1;
                    float search_step_2 = 0.1;

                    float search_step = 0.02;

                    float node_1_id_float = check_node_id - 0.001f;
                    float node_2_id_float = check_node_id + 0.001f;
                    transform::Transform2d interpolator_node_1;
                    transform::Transform2d interpolator_node_2;

                    float check_x = m_track_path_info[check_node_id].pose.x();
                    float check_y = m_track_path_info[check_node_id].pose.y();
                    PLOG(plog::Severity(log_level)) << "check_node_id: " << check_node_id;
                    PLOG(plog::Severity(log_level)) << "check_node pose: " << m_track_path_info[check_node_id].pose;

//                    search_step_1 = search_step/std::sqrt(transform::diff2(global_path[int(node_1_id_float)], global_path[int(node_1_id_float) + 1]));
//                    search_step_2 = search_step/std::sqrt(transform::diff2(global_path[int(node_2_id_float)], global_path[int(node_2_id_float) + 1]));

                    bool get_valid = false;
                    float interpolate_dist = 0.0f;
                    for(size_t i = 1 ; i < 100;i++){

                        if(node_1_id_float < 0.0f || node_2_id_float > pose_num-2){
                            break;
                        }
                        search_step_1 = search_step/std::sqrt(transform::diff2(global_path[int(node_1_id_float)], global_path[int(node_1_id_float) + 1]));
                        search_step_2 = search_step/std::sqrt(transform::diff2(global_path[int(node_2_id_float)], global_path[int(node_2_id_float) + 1]));


                        node_1_id_float -= search_step_1;
                        node_2_id_float += search_step_2;
                        if(node_1_id_float < 0.0f || node_2_id_float > pose_num-2){
                            break;
                        }
                        interpolator_node_1 = transform::interpolate((node_1_id_float - int(node_1_id_float)), global_path[int(node_1_id_float)], global_path[int(node_1_id_float) + 1]);
                        interpolator_node_2 = transform::interpolate((node_2_id_float - int(node_2_id_float)), global_path[int(node_2_id_float)], global_path[int(node_2_id_float) + 1]);


                        float check_dist = math::getPointDistToLine(check_x,check_y,interpolator_node_1.x(),interpolator_node_1.y(),interpolator_node_2.x(),interpolator_node_2.y());
                        PLOG(plog::Severity(log_level)) << "node_1_id_float: " << node_1_id_float;
                        PLOG(plog::Severity(log_level)) << "node_2_id_float: " << node_2_id_float;

                        PLOG(plog::Severity(log_level)) << "check_dist: " << check_dist;

                        interpolate_dist = (m_track_path_info[int(node_2_id_float)].dist_from_start + (node_2_id_float - int(node_2_id_float)) * (m_track_path_info[int(node_2_id_float) + 1].dist_from_start - m_track_path_info[int(node_2_id_float)].dist_from_start ))
                                -(m_track_path_info[int(node_1_id_float)].dist_from_start + (node_1_id_float - int(node_1_id_float)) * (m_track_path_info[int(node_1_id_float) + 1].dist_from_start - m_track_path_info[int(node_1_id_float)].dist_from_start ));
                        PLOG(plog::Severity(log_level)) << "interpolate_dist: " << interpolate_dist;

                        if(check_dist > width
                        || interpolate_dist  > m_planner_config.curve_interpolate_dist){
                            break;
                        }
                        get_valid = true;
                    }
                    PLOG(plog::Severity(log_level)) << "interpolator_node_1: " << interpolator_node_1;
                    PLOG(plog::Severity(log_level)) << "interpolator_node_2: " << interpolator_node_2;

                    if(get_valid){
                        float s = 0.03/std::sqrt(transform::diff2(interpolator_node_1,interpolator_node_2 ));
                        for(float r = 0.0; r < 1.0;r+=s){
                            debug_path.push_back(transform::interpolate(r,interpolator_node_1,interpolator_node_2 ));
                        }

                        m_track_path_info[check_node_id].direction_change_from_last = angle_normalise(m_track_path_info[int(node_2_id_float)].direction , m_track_path_info[int(node_1_id_float)].direction) -  m_track_path_info[int(node_1_id_float)].direction;
                        m_track_path_info[check_node_id].direction_change_divide_dist = m_track_path_info[check_node_id].direction_change_from_last/interpolate_dist;
                        PLOG(plog::Severity(log_level)) << "direction_change_from_last: " << m_track_path_info[check_node_id].direction_change_from_last;
                        PLOG(plog::Severity(log_level)) << "direction_change_divide_dist: " << m_track_path_info[check_node_id].direction_change_divide_dist;

                        m_track_path_info[check_node_id].interpolate_valid = true;
                        m_track_path_info[check_node_id].interpolate_node_in = interpolator_node_1;
                        m_track_path_info[check_node_id].interpolate_node_out = interpolator_node_2;
                        int in_id = std::min(std::max(0,int(node_1_id_float) ), int(pose_num-2));
                        int out_id = std::min(std::max(0,int(node_2_id_float) ), int(pose_num-2));

                        m_track_path_info[check_node_id].interpolate_node_in_id = in_id;
                        m_track_path_info[check_node_id].interpolate_node_out_id = out_id;
                        global_path[ in_id] = interpolator_node_1;
                        global_path[ out_id] = interpolator_node_2;


                    }


#if 0
                    //compute
                    // find two node
                    // node1    check_node_id  node2
                    // get distance and direction change between node1 and node2

                    PLOG(plog::Severity(log_level)) << "check_node_id: " << check_node_id;

                    size_t target_id = check_node_id;
                    auto check_pose_inv = m_track_path_info[check_node_id].pose.inverse();
                    PLOG(plog::Severity(log_level)) << "m_track_path_info[check_node_id].pose: " << m_track_path_info[check_node_id].pose;
                    PLOG(plog::Severity(log_level)) << "check_pose_inv: " << check_pose_inv;


                    transform::Transform2d target_pose_in_check_pose;

                    // if next segment is line , target_pose_in_check_pose.y() may not reach limit
                    // if it is far from goal , x_limix can be large , otherwise it should be small

                    // if next segment is curve ,target_pose_in_check_pose.y() should reach limit
                    float x_limix = 1.0f;

                    for(size_t i = check_node_id + 1; i < pose_num; i++){
                        target_id = i;
                        target_pose_in_check_pose = check_pose_inv * m_track_path_info[i].pose;
//                        PLOG(plog::Severity(log_level)) << "target_id: " << target_id;

//                        PLOG(plog::Severity(log_level)) << "target_pose_in_check_pose: " << target_pose_in_check_pose;

                        if( std::abs(target_pose_in_check_pose.y()) > width_2 || std::abs(target_pose_in_check_pose.x()) > x_limix ){
                            break;
                        }

                    }
                    PLOG(plog::Severity(log_level)) << "target_id: " << target_id;
                    PLOG(plog::Severity(log_level)) << "target_pose_in_check_pose: " << target_pose_in_check_pose;

#endif


                    //

                }else{
                    break;
                }
                check_node_id++;
            }
#endif

            for(size_t i = 0 ; i < pose_num  ;i++){
                PLOG(plog::Severity(log_level)) << "id: " << i;

                PLOG(plog::Severity(log_level)) << "pose: " << m_track_path_info[i].pose;
                PLOG(plog::Severity(log_level)) << "segment_id: " << m_track_path_info[i].segment_id;
                PLOG(plog::Severity(log_level)) << "segment_end_id: " << m_track_path_info[i].segment_end_id;
                PLOG(plog::Severity(log_level)) << "forward_vel: " << m_track_path_info[i].forward_vel;

                PLOG(plog::Severity(log_level)) << "dist_to_segment_end: " << m_track_path_info[i].dist_to_segment_end;

                PLOG(plog::Severity(log_level)) << "segment_type: " << m_track_path_info[i].segment_type;
                PLOG(plog::Severity(log_level)) << "direction: " << m_track_path_info[i].direction;

                PLOG(plog::Severity(log_level)) << "direction_change_from_last: " << m_track_path_info[i].direction_change_from_last;
                PLOG(plog::Severity(log_level)) << "dist_from_last: " << m_track_path_info[i].dist_from_last;
                PLOG(plog::Severity(log_level)) << "direction_change_divide_dist: " << m_track_path_info[i].direction_change_divide_dist;
                PLOG(plog::Severity(log_level)) << "dist_to_end: " << m_track_path_info[i].dist_to_end;

            }


            float check_wheel_zero_angle[2] = {0.0,-M_PIf32};
            bool check_wheel_zero_ok[2] = {true, true};

            for(size_t i = 0 ; i < pose_num  ;i++){
                check_wheel_zero_ok[0] = check_wheel_zero_ok[0] && std::abs(angle_normal(check_wheel_zero_angle[0] + m_track_path_info[i].pose.yaw() - global_path[i].yaw())) < m_max_wheel_rotate_angle;
                check_wheel_zero_ok[1] = check_wheel_zero_ok[1] && std::abs(angle_normal(check_wheel_zero_angle[1] + m_track_path_info[i].pose.yaw() - global_path[i].yaw())) < m_max_wheel_rotate_angle;
            }
            PLOG(plog::Severity(log_level)) << "check_wheel_zero_ok: " << check_wheel_zero_ok[0] << ", " << check_wheel_zero_ok[1];




            if(check_wheel_zero_ok[0] || check_wheel_zero_ok[1] ){

                float angle_diff[2] = {100.0f,100.0f};
                float init_wheel_angle[2] = {0.0f,0.0f};

                for(size_t i = 0 ; i  < 2;i++){
                    init_wheel_angle[i] = angle_normal(check_wheel_zero_angle[i] + m_track_path_info[0].pose.yaw() - global_path[0].yaw());
                    angle_diff[i] = check_wheel_zero_ok[i] ? std::abs( init_wheel_angle[i] - m_wheel_state.value[1]) : 1000.0f;
                }



                PLOG(plog::Severity(log_level))  << "init_wheel_angle: " << init_wheel_angle[0] << ", " << init_wheel_angle[1];

                m_start_wheel_angle = angle_diff[0] < angle_diff[1] ? init_wheel_angle[0] : init_wheel_angle[1];

                PLOG(plog::Severity(log_level))  << "m_start_wheel_angle: " << m_start_wheel_angle;

                if(angle_diff[0] < angle_diff[1]){

                    m_allow_angle_min[0] = -m_max_wheel_rotate_angle + 0.01f;
                    m_allow_angle_max[0] =  m_max_wheel_rotate_angle - 0.01f;
                    m_allow_angle_min[1] = -m_max_wheel_rotate_angle + 0.01f;
                    m_allow_angle_max[1] =  m_max_wheel_rotate_angle - 0.01f;
                }else{

                    m_allow_angle_min[0] = M_PIf32 - m_max_wheel_rotate_angle + 0.01f;
                    m_allow_angle_max[0] = M_PIf32 - 0.01f;
                    m_allow_angle_min[1] = -M_PIf32 + 0.01f;
                    m_allow_angle_max[1] = -M_PIf32 + m_max_wheel_rotate_angle - 0.01f;

                }
                PLOG(plog::Severity(log_level))  << "m_allow_angle_min: " << m_allow_angle_min[0] << ", " << m_allow_angle_min[1];
                PLOG(plog::Severity(log_level))  << "m_allow_angle_max: " << m_allow_angle_max[0] << ", " << m_allow_angle_max[1];

                steer_rotate_limit_ok = true;
            }else{
                steer_rotate_limit_ok = false;
            }


            if(steer_rotate_limit_ok){
                // find closest node
                size_t best_id = m_closest_path_node_id;
                float best_dist = 1000000.0;
                transform::Transform2d actual_in_closest;

                for(size_t i = m_closest_path_node_id; i < pose_num ;i++){
                    const auto& P = global_path[i];
                    float base_to_target_yaw = std::atan2(P.y() - actual_pose_y, P.x() - actual_pose_x);
                    float check_wheel_yaw = angle_normalise(base_to_target_yaw, actual_pose_yaw) - actual_pose_yaw;
                    actual_in_closest = m_track_path_info[i].pose.inverse()*actual_pose;


                    if(  (!(
                            (check_wheel_yaw > m_allow_angle_min[0]
                             && check_wheel_yaw < m_allow_angle_max[0])
                            ||(check_wheel_yaw > m_allow_angle_min[1]
                               && check_wheel_yaw < m_allow_angle_max[1])
                    ))||(
                            actual_in_closest.x() > -0.01f
                            )


                    ) continue;

                    float dist = std::sqrt(transform::diff2(actual_pose,P));
                    best_dist = (dist < best_dist) ? (best_id = i, dist) : best_dist;

                }
                m_closest_path_node_id = best_id;


                actual_in_closest = m_track_path_info[m_closest_path_node_id].pose.inverse()*actual_pose;

                float closest_node_offset_x = actual_in_closest.y();
                float closest_node_offset_y = actual_in_closest.y();
                float closest_dist = std::sqrt(transform::diff2(actual_pose, m_track_path_info[m_closest_path_node_id].pose));
                PLOG(plog::Severity(log_level))  << "closest_dist: " << closest_dist;

                if(closest_dist > m_planner_config.start_pose_dist){

                    char buffer[100];
                    sprintf(buffer,"path_error: closest_dist = %.3f > %.3f",closest_dist,m_planner_config.start_pose_dist);
                    m_status_msg.assign(buffer);
                    PLOGF << m_status_msg;

                    return false;
                }

                PLOG(plog::Severity(log_level))  << "actual_in_closest: " << actual_in_closest;

                PLOG(plog::Severity(log_level))  << "m_closest_path_node_id: " << m_closest_path_node_id;

                m_path_node_id = m_closest_path_node_id;
            }else{
                char buffer[100];
                sprintf(buffer,"path_error: steer_rotate_limit not ok");
                m_status_msg.assign(buffer);
                PLOGF << m_status_msg;

                return false;
            }


#if 0
            for(size_t i = 0 ; i < pose_num  ;i++){

                global_path[i] = m_track_path_info[i].pose;
            }
#endif


        }

        // todo:remove:
        m_local_path.value.clear();
        m_local_path.value.insert(m_local_path.value.end(), debug_path.begin(), debug_path.end());

        return steer_rotate_limit_ok;
    }

//    bool DoubleSteerMotionPlanner::createLocalPath() {
//
//
//        return createLocalPath_v2();
//    }
//
    bool DoubleSteerMotionPlanner::createLocalPath_v1(){

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


        float odom_twist_x = m_actual_odom.value.twist.twist.linear.x;
        float odom_twist_y = m_actual_odom.value.twist.twist.linear.y;

//        transform::Transform2d odom_twist(m_actual_odom.value.twist.twist.linear.x,m_actual_odom.value.twist.twist.linear.y);
//        odom_twist = m_map_odom_record*odom_twist;
        float odom_twist_vel = std::sqrt(odom_twist_x*odom_twist_x + odom_twist_y*odom_twist_y);


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

        float actual_yaw = actual_pose.yaw();

        float allow_angle_min[2] = {0.0f,0.0f};
        float allow_angle_max[2] = {0.0f,0.0f};
        if(m_start_wheel_angle > 0.1){
            allow_angle_min[0] = -m_max_wheel_rotate_angle;
            allow_angle_max[0] =  m_max_wheel_rotate_angle;
            allow_angle_min[1] = -m_max_wheel_rotate_angle;
            allow_angle_max[1] =  m_max_wheel_rotate_angle;


            PLOGD << "check allow_angle_min: " << allow_angle_min;
            PLOGD << "check allow_angle_max: " << allow_angle_max;
        }else if(m_start_wheel_angle < -0.1){

//                    allow_angle_min = path_yaw_map > 0.0f ? M_PIf32 - m_max_wheel_rotate_angle : -M_PIf32;
//                    allow_angle_max = path_yaw_map > 0.0f ? M_PIf32 :-M_PIf32 + m_max_wheel_rotate_angle;

            allow_angle_min[0] = M_PIf32 - m_max_wheel_rotate_angle;
            allow_angle_max[0] = M_PIf32;
            allow_angle_min[1] = -M_PIf32;
            allow_angle_max[1] = -M_PIf32 + m_max_wheel_rotate_angle;


            PLOGD << "check allow_angle_min: " << allow_angle_min;
            PLOGD << "check allow_angle_max: " << allow_angle_max;
        }else{

            allow_angle_min[0] = -m_max_wheel_rotate_angle;
            allow_angle_max[0] =  m_max_wheel_rotate_angle;

            allow_angle_min[1] = -m_max_wheel_rotate_angle;
            allow_angle_max[1] =  m_max_wheel_rotate_angle;

            PLOGD << "check allow_angle_min: " << allow_angle_min;
            PLOGD << "check allow_angle_max: " << allow_angle_max;
        }

        if(m_path_node_id == 0){


            const auto& PS = global_path[m_path_node_id];
            size_t next_id = m_path_node_id+1;
#if 0
            for(size_t i =m_path_node_id+1; i<pose_num; i++ ){
                if(transform::diff2(global_path[i],global_path[m_path_node_id]) > 0.0001){
                    next_id = i;
                    break;
                }
            }
            PLOGD << "next_id: " << next_id;
#endif
            const auto& PS_1 = global_path[next_id];
            float path_yaw_map = std::atan2(PS_1.y() - PS.y(),PS_1.x() - PS.x() );

            float dist_base_PS = std::sqrt(transform::diff2(PS, actual_pose));
            float dist_PS_1 = std::sqrt(transform::diff2(PS, PS_1));

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
                m_prefer_steer_angle =angle_normal(wheel_yaw) ;
            }else if(m_start_wheel_angle < -0.1){
                m_prefer_steer_angle = angle_normal(wheel_yaw + M_PIf32);
            }else{
                m_prefer_steer_angle =angle_normal(wheel_yaw) ;
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

            m_forward_vel = 0.05f;
            if(odom_twist_vel > 0.01){

                bool find_valid_pose = false;
                size_t choose_id = m_path_node_id;

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
                    && (
                                                      (check_wheel_yaw > allow_angle_min[0]
                                                       && check_wheel_yaw < allow_angle_max[0])
                                                       ||(check_wheel_yaw > allow_angle_min[1]
                                                          && check_wheel_yaw < allow_angle_max[1])
                            )
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
            float base_x = actual_pose.x();
            float base_y = actual_pose.y();
            float base_yaw = actual_pose.yaw();

            size_t best_id = m_closest_path_node_id;
            float best_dist = 1000000.0;

            for(size_t i = m_closest_path_node_id; i < pose_num ;i++){
                float dist = transform::diff2(actual_pose,global_path[i]);
                best_dist = (dist < best_dist) ? (best_id = i, dist) : best_dist;
                 if(dist > 1.0) break;
            }
            m_closest_path_node_id = best_id;

            size_t next_id = m_closest_path_node_id+1;
#if 0
            for(size_t i =m_closest_path_node_id+1; i<pose_num; i++ ){
                if(transform::diff2(global_path[i],global_path[m_closest_path_node_id]) > 0.0001){
                    next_id = i;
                    break;
                }
            }
#endif
            float path_yaw_map = std::atan2(global_path[next_id].y() - global_path[m_closest_path_node_id].y(),global_path[next_id].x() - global_path[m_closest_path_node_id].x() );


            PLOGD << "m_closest_path_node_id: " << m_closest_path_node_id;
            best_dist = std::sqrt(best_dist);
            transform::Transform2d adjust_point( global_path[m_closest_path_node_id].x(), global_path[m_closest_path_node_id].y(), path_yaw_map);
            transform::Transform2d  pose_in_adjust_point = adjust_point.inverse() *actual_pose;

            PLOGD << "best_dist: " << best_dist;
#if 0
            if(best_dist > 1.1){



                m_task_state = TaskState::off_path;
                return false;
            }
#endif


            float dist = math::getPointDistToLine(global_path[m_closest_path_node_id].x(),global_path[m_closest_path_node_id].y(), global_path[next_id].x(),global_path[next_id].y(), actual_pose.x(), actual_pose.y());



//            if(std::abs(pose_in_adjust_point.y()) > 0.1f)
            if(dist > 0.1f)
            {
                interpolate_time = now;
            }


            m_forward_vel = abs_update_s * m_planner_config.speed_up_acc +0.05f;





        }

        PLOGD << "m_path_node_id: " << m_path_node_id;

        while(m_path_node_id < pose_num-1){
            const auto& PS = global_path[m_path_node_id];

            size_t next_id = m_path_node_id+1;
#if 0
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

//            float check_wheel_yaw = angle_normalise(base_to_target_yaw, path_yaw_map) - path_yaw_map;
            float check_wheel_yaw = angle_normalise(base_to_target_yaw, actual_yaw) - actual_yaw;


            float relative_ratio = odom_forward_dist/dist_PS_1;

            PLOGD << "dist_base_PS: " << dist_base_PS;
            PLOGD << "pose_in_adjust_point: " << pose_in_adjust_point;

//            if(dist_PS_1< slider_window_start_max_dist)

#if 0

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
#endif
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

            if(pose_in_adjust_point.x() > m_planner_config.first_track_dist

               && (
                       (check_wheel_yaw > allow_angle_min[0]
                        && check_wheel_yaw < allow_angle_max[0])
                       ||(check_wheel_yaw > allow_angle_min[1]
                          && check_wheel_yaw < allow_angle_max[1])
               )

            ){
                break;
            }else{
                m_path_node_id = m_path_node_id +1;
                m_path_node_id = std::min(m_path_node_id, pose_num-1);


//                m_path_node_id_interpolate = std::max(static_cast<float>(m_path_node_id), m_path_node_id_interpolate) ;
            }

            if(m_path_node_id == pose_num-1){
                break;
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
            for(size_t i = m_path_node_id  ; i < pose_num ;i++){

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

                if( i < pose_num-1){
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

    bool DoubleSteerMotionPlanner::followLocalPath() {

        auto& local_path = m_local_path.value;

        auto& actual_pose = m_actual_pose.value;
        float actual_pose_x = actual_pose.x();
        float actual_pose_y = actual_pose.y();
        float actual_pose_yaw = actual_pose.yaw();


        size_t pose_num = local_path.size();

        if(pose_num <2){
            return false;
        }
        //
        float base_to_target_yaw_sum = 0.0;
        float target_path_yaw_sum = 0.0;
        float base_to_target_yaw_init = std::atan2(local_path[0].y() - actual_pose_y, local_path[0].x() - actual_pose_x );
        float target_path_yaw_init = angle_normal(m_track_path_info[m_closest_path_node_id + 1 ].direction) ;


        auto actual_in_closest = m_track_path_info[m_closest_path_node_id].pose.inverse()*actual_pose;

#if 0
        size_t valid_num = 0;

        //
        float angle_thresh = 2.5f;


        for(size_t i = 0 ; i < pose_num ; i++){
            float base_to_target_yaw = angle_normalise(std::atan2(local_path[i].y() - actual_pose_y, local_path[i].x() - actual_pose_x ), base_to_target_yaw_init );
            float target_path_yaw = angle_normalise(m_track_path_info[m_closest_path_node_id + i].direction, target_path_yaw_init );
            bool valid = std::abs(angle_normalise(base_to_target_yaw,   target_path_yaw_init )-target_path_yaw_init ) < angle_thresh;

            base_to_target_yaw_sum += base_to_target_yaw*valid;
            target_path_yaw_sum += target_path_yaw*valid;
            valid_num += valid;
        }

        if(valid_num == 0 ){
            return false;
        }

        base_to_target_yaw_sum /= float (valid_num);
        target_path_yaw_sum /= float (valid_num);

        float path_direction_weight = 0.1f;
        float base_to_goal_yaw = angle_normal(path_direction_weight*   angle_normalise(target_path_yaw_sum , base_to_target_yaw_sum) + (1.0f - path_direction_weight)* base_to_target_yaw_sum    )   ;
        m_forward_angle = angle_normalise(base_to_goal_yaw, actual_pose_yaw) - actual_pose_yaw;

#endif


#if 1
        m_forward_angle = base_to_target_yaw_init;
#endif


        {
            float best_diff = 10000.0f;

#if 0
            float best_id_interpolate = 0.1f;

            for(float f = 0.1f;f < pose_num - 1 ; f += 0.2f ){
                int i = int(best_id_interpolate);
                auto n = transform::interpolate(best_id_interpolate - i,local_path[i],local_path[i+1] );

                float base_to_target_yaw = angle_normalise(std::atan2(n.y() - actual_pose_y, n.x() - actual_pose_x ), base_to_target_yaw_init );
                float target_path_yaw = angle_normalise(m_track_path_info[m_closest_path_node_id + i].direction, target_path_yaw_init );

                float diff = std::abs(  angle_normalise(base_to_target_yaw, target_path_yaw) - target_path_yaw);
                best_diff = (diff < best_diff)? (best_id_interpolate = f  , diff) : best_diff;
            }
#endif

            size_t best_id = 0;
            for(size_t i = 0 ; i < pose_num ; i++){
                float base_to_target_yaw = angle_normalise(std::atan2(local_path[i].y() - actual_pose_y, local_path[i].x() - actual_pose_x ), base_to_target_yaw_init );
                float target_path_yaw = angle_normalise(m_track_path_info[m_closest_path_node_id + i].direction, target_path_yaw_init );

                float diff = std::abs(  angle_normalise(base_to_target_yaw, target_path_yaw) - target_path_yaw);
                best_diff = (diff < best_diff)? (best_id = i  , diff) : best_diff;

            }

            int best_weight = 3;

            int init_weight = 1;

            if(std::abs(actual_in_closest.y()) > 0.03){
                init_weight = 2;
            }

            float goal_y_sum = best_weight*local_path[best_id].y() + init_weight*local_path[0].y() ;
            float goal_x_sum = best_weight*local_path[best_id].x() + init_weight*local_path[0].x() ;


            for(size_t i = 0 ;i <pose_num;i++ ){
                goal_y_sum += local_path[i].y();
                goal_x_sum += local_path[i].x();
            }
//            goal_y_sum/=float(pose_num - best_id + best_weight - 1 + init_weight - 1);
//            goal_x_sum/=float(pose_num - best_id + best_weight - 1 + init_weight - 1);
            goal_y_sum/=float(pose_num  + best_weight  + init_weight );
            goal_x_sum/=float(pose_num  + best_weight  + init_weight );

            float temp_forward_angle = std::atan2(goal_y_sum - actual_pose_y, goal_x_sum - actual_pose_x );
            m_forward_angle = angle_normalise(temp_forward_angle, actual_pose_yaw) - actual_pose_yaw;

        }

        if(m_forward_angle > 0.0){
            m_forward_angle = std::min(m_allow_angle_max[0], std::max(m_forward_angle, m_allow_angle_min[0]));
        }else{
            m_forward_angle = std::min(m_allow_angle_max[1], std::max(m_forward_angle, m_allow_angle_min[1]));
        }

        PLOGI << "base_to_target_yaw_sum: " << base_to_target_yaw_sum;
        PLOGI << "target_path_yaw_sum: " << target_path_yaw_sum;
        PLOGI << "rotate_angle: " << m_forward_angle;

        return true;
    }

    bool DoubleSteerMotionPlanner::createLocalPath() {
        m_report_error.fill(0.0);

        int log_level = 4;

        // time
        common::Time now = common::FromUnixNow();
        interpolate_time_step_1 = now;

//        float abs_update_s = common::ToMicroSeconds(interpolate_time_step_1 - interpolate_time)*1e-6;

        float update_s = common::ToMicroSeconds(interpolate_time_step_1 - interpolate_time_step_0)*1e-6;
        update_s = std::min(update_s, 0.05f);

        interpolate_time_step_0 = now;

        //time





        size_t pose_num = m_global_path.value.size();

        auto & local_path = m_local_path.value;
        auto & global_path = m_global_path.value;

        auto& actual_pose = m_actual_pose.value;

        float actual_pose_yaw = actual_pose.yaw();
        float actual_pose_x = actual_pose.x();
        float actual_pose_y = actual_pose.y();


        bool last_local_path_valid = !local_path.empty();
        bool last_local_path_goal_reach = false;


        float curve_interpolate_dist = m_planner_config.curve_interpolate_dist;
        float curve_interpolate_dist_2 = 0.5f*curve_interpolate_dist;

        //todo: add
//        local_path.clear();

        m_local_target.time = common::FromUnixNow();


        float odom_twist_x = m_actual_odom.value.twist.twist.linear.x;
        float odom_twist_y = m_actual_odom.value.twist.twist.linear.y;

//        transform::Transform2d odom_twist(m_actual_odom.value.twist.twist.linear.x,m_actual_odom.value.twist.twist.linear.y);
//        odom_twist = m_map_odom_record*odom_twist;
        float odom_twist_vel = std::sqrt(odom_twist_x*odom_twist_x + odom_twist_y*odom_twist_y);

        float odom_forward_dist = odom_twist_vel * update_s;
        PLOGD << "odom_twist_vel: " << odom_twist_vel;

        float wheel_max_angle_change = update_s * m_max_base_forward_angle_vel;
        PLOGD << "wheel_max_angle_change: " << wheel_max_angle_change;


        float actual_yaw = actual_pose.yaw();



        // find closest node
        transform::Transform2d actual_in_closest;

        {
            // find closest node
            size_t best_id = m_closest_path_node_id;
            float best_dist = 1000000.0;


            for(size_t i = m_closest_path_node_id; i < pose_num ;i++){
                const auto& P = global_path[i];
                float base_to_target_yaw = std::atan2(P.y() - actual_pose_y, P.x() - actual_pose_x);
                float check_wheel_yaw = angle_normalise(base_to_target_yaw, actual_pose_yaw) - actual_pose_yaw;

                actual_in_closest = m_track_path_info[i].pose.inverse()*actual_pose;


                if(  (!(
                        (check_wheel_yaw > m_allow_angle_min[0]
                         && check_wheel_yaw < m_allow_angle_max[0])
                        ||(check_wheel_yaw > m_allow_angle_min[1]
                           && check_wheel_yaw < m_allow_angle_max[1])
                ))||(
                        actual_in_closest.x() > -0.01f
                        )


                        ) continue;


                float dist = std::sqrt(transform::diff2(actual_pose,P));
                best_dist = (dist < best_dist) ? (best_id = i, dist) : best_dist;

                if(dist > 1.0
                        ) break;
            }
            m_closest_path_node_id = best_id;

            actual_in_closest = m_track_path_info[m_closest_path_node_id].pose.inverse()*actual_pose;


            {
                // create local path
                // interpolate path with small step
                local_path.clear();

                float follow_dist_max = m_planner_config.follow_dist_max;
                float follow_dist_min = m_planner_config.follow_dist_min;

                for (size_t i = m_closest_path_node_id; i < pose_num; ++i) {
                    if((m_track_path_info[i].dist_from_start - m_track_path_info[m_closest_path_node_id].dist_from_start) >  follow_dist_min){
                        continue;
                    }
                    local_path.emplace_back(global_path[i]);

                    if((m_track_path_info[i].dist_from_start - m_track_path_info[m_closest_path_node_id].dist_from_start) >  follow_dist_max){
                        break;
                    }
                }

                if(local_path.empty()){
                    size_t end = std::min(pose_num, m_closest_path_node_id + 3);
                    for(size_t i = m_closest_path_node_id; i < end;i++){

                        local_path.emplace_back(global_path[i]);
                    }

                }

#if 0
                followLocalPath();

                // smooth stop vel
                m_forward_vel = m_track_path_info[m_closest_path_node_id].forward_vel;

                // smooth speed up
                float increase_forward_vel =  odom_twist_vel +   relative_update_s * m_planner_config.speed_up_acc;

                m_forward_vel = std::min(m_forward_vel,increase_forward_vel );

                return true;
#endif


            }


//            PLOG(plog::Severity(log_level))  << "actual_in_closest: " << actual_in_closest;
//            PLOG(plog::Severity(log_level))  << "m_closest_path_node_id: " << m_closest_path_node_id;

        }
        float closest_node_offset_x = actual_in_closest.x();
        float closest_node_offset_y = actual_in_closest.y();

        auto pose_diff = global_path[m_closest_path_node_id].inverse()*actual_pose;

        m_report_error[0] = closest_node_offset_x;
        m_report_error[1] = closest_node_offset_y;
        m_report_error[2] = pose_diff.yaw();

        // search track node

        if(m_planner_state == PlannerState::idle){


            // compute init command, switch to next state


            for(size_t i = m_closest_path_node_id; i < pose_num ;i++){
                const auto& P = global_path[i];
                float base_to_target_yaw = std::atan2(P.y() - actual_pose_y, P.x() - actual_pose_x);
                float check_wheel_yaw = angle_normalise(base_to_target_yaw, actual_pose_yaw) - actual_pose_yaw;

                actual_in_closest = m_track_path_info[i].pose.inverse()*actual_pose;

                m_path_node_id= i;

                if(std::abs(actual_in_closest.y()) < std::abs(actual_in_closest.x())
                 && (actual_in_closest.x() < -m_planner_config.pursuit_goal_reach_tolerance) ){

                    break;
                }
            }

            goToNode(m_path_node_id);


            m_planner_state = PlannerState::init_to_path;

        }

        if(m_planner_state == PlannerState::init_to_path){

            bool reach = false;
            if(odom_twist_vel < 0.04){
                m_rotate_vel = 0.0f;
                m_rotate_diff = 0.0f;
            }else {
                reach = goToNode(m_path_node_id);
            }

            m_forward_vel = m_forward_diff > 0.0f ? 0.05:0.0f;
            m_rotate_vel = 0.0f;

            if(reach){
                m_planner_state = PlannerState::follow_path;
            }

        }


        if(m_planner_state != PlannerState::init_to_path){


            // check control error
            if(std::abs(closest_node_offset_y) > m_planner_config.off_path_dist){

                char buffer[100];
                sprintf(buffer,"control_error: robot run off the path: %.3f > %.3f",closest_node_offset_y , m_planner_config.off_path_dist);
                m_status_msg.assign(buffer);
                PLOGF << m_status_msg;

                m_task_state = TaskState::off_path;
                return false;
            }

            //

            bool reach = false;

            bool is_on_line_path = m_track_path_info[m_closest_path_node_id].segment_type == 0;
            bool is_on_left_turn_path = m_track_path_info[m_closest_path_node_id].segment_type == 1;
            bool is_on_right_turn_path = m_track_path_info[m_closest_path_node_id].segment_type == -1;
            bool at_segment_end = m_track_path_info[m_closest_path_node_id].dist_to_segment_end < curve_interpolate_dist_2;

            float path_width = m_planner_config.pursuit_path_width*(at_segment_end ? 0.5f:1.0f);
            bool in_path_width = std::abs(closest_node_offset_y) < path_width ;


            bool in_path_close = std::abs(closest_node_offset_y) < 0.5f*path_width ;

            size_t segment_end_id = m_track_path_info[m_closest_path_node_id].segment_end_id;



            PLOG(plog::Severity(log_level))  << "run_on_segment_type: " << m_track_path_info[m_closest_path_node_id].segment_type;

            if(m_planner_state == PlannerState::follow_goal){
                size_t target_id = pose_num-1;
                m_closest_path_node_id = target_id;
                if(odom_twist_vel < 0.04){
                    m_rotate_vel = 0.0f;
                    m_rotate_diff = 0.0f;
                }else {
                    reach = goToNode(target_id);
                }

                m_rotate_vel = 0.0f;
                m_rotate_diff = 0.0f;



            }else if(is_on_line_path

//            && m_track_path_info[m_closest_path_node_id].dist_to_segment_end >curve_interpolate_dist_2

            ){
                size_t target_id = segment_end_id;


                float base_to_goal_yaw = angle_normal(m_track_path_info[target_id].direction);
                float goal_yaw = global_path[target_id].yaw();

                m_rotate_diff = angle_normalise(goal_yaw, actual_pose_yaw) - actual_pose_yaw;

                if(!in_path_width){
                    off_width_once = true;
                }
                if(in_path_close){
                    off_width_once = false;
                }



                if(in_path_width && !off_width_once// && beyond_end
                ){
                    m_forward_angle = angle_normalise(base_to_goal_yaw, actual_pose_yaw) - actual_pose_yaw;
                }else
                {
                    if(odom_twist_vel < 0.04){
                        m_rotate_vel = 0.0f;
                        m_rotate_diff = 0.0f;
                    }else {
                        reach = goToNode(target_id);
                    }
                }




            }else {
                // todo: followLocalPath is not stable
               bool ok =  followLocalPath();

               if(!ok){
                   char buffer[100];
                   sprintf(buffer,"control_error: followLocalPath fail");
                   m_status_msg.assign(buffer);
                   PLOGF << m_status_msg;

                   m_task_state = TaskState::off_path;
                   return false;
               }
            }


            if(m_planner_state != PlannerState::follow_goal)
            {
                m_forward_vel = m_track_path_info[m_closest_path_node_id].forward_vel;

                // smooth speed up
                float increase_forward_vel =  odom_twist_vel +   update_s * m_planner_config.speed_up_acc;
//                std::cout << "increase_forward_vel: " << increase_forward_vel << ", speed_up_acc: " << m_planner_config.speed_up_acc << ", update_s: " << update_s << ", odom_twist_vel: " << odom_twist_vel << "\n";

                m_forward_vel = std::min(m_forward_vel,increase_forward_vel );
            }

            if(reach){
                m_forward_vel = 0.0f;
                m_rotate_vel = 0.0f;
                m_rotate_diff = 0.0f;
            }
















            // detect segment type , use m_track_info
            // if closest node's type is line and distance to segment end is big enough , then use follow_line mode


            // in follow_line mode, robot has two action, follow_line and return_line, depends on path width and follow error offset
            // in follow_line mode, if node's distance to segment end is small, then switch to follow_curve mode
            // in follow_line mode, if node's distance to segment end is small, then switch to follow_curve mode
            // in follow_line mode, if node's distance to end is small, then switch to follow_goal mode





            PLOG(plog::Severity(log_level))  << "m_forward_vel: " << m_forward_vel;
            PLOG(plog::Severity(log_level))  << "m_forward_diff: " << m_forward_diff;
            PLOG(plog::Severity(log_level))  << "m_forward_angle: " << m_forward_angle;
            PLOG(plog::Severity(log_level))  << "m_rotate_vel: " << m_rotate_vel;
            PLOG(plog::Severity(log_level))  << "m_rotate_diff: " << m_rotate_diff;
        }





        return true;
    }
    bool DoubleSteerMotionPlanner::pursuit_path() {


        auto & local_path = m_local_path.value;
        auto & global_path = m_global_path.value;

        size_t pose_num = global_path.size();
        auto& actual_pose = m_actual_pose.value;

        m_command.setCmd(0.0f,0.0f,0.0f);
        float& command_linear_x = m_command.command[0];
        float& command_linear_y = m_command.command[1];
        float& command_angular_z = m_command.command[2];


        float actual_angular_z = m_actual_odom.value.twist.twist.angular.z;
        float dist_to_goal = std::sqrt(transform::diff2(actual_pose, global_path.back()));
#if 0
        float full_path_len = std::sqrt(transform::diff2(actual_pose, global_path[m_path_node_id]));
        for(size_t i = m_path_node_id; i < pose_num-1;i++){
            full_path_len += std::sqrt(transform::diff2(global_path[i], global_path[i+1]));
        }
#endif

        float full_path_len = std::min(m_track_path_info[m_closest_path_node_id].dist_to_end,dist_to_goal ) ;



        // use full_path_len to control dynamic speed
        // s = 0.5*a*t^2
        // t = sqrt(2s/a)
        // v = a*t
#if 0
        float dynamic_acc =m_planner_config.speed_down_acc;
        float dynamic_t = std::sqrt(2.0f*full_path_len/dynamic_acc);
        float dynamic_v = dynamic_acc*dynamic_t;
        PLOGD << "full_path_len: " << full_path_len;
        PLOGD << "dynamic v : " << dynamic_v;
#endif


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

//        transform::Transform2d target_pose = m_local_target.value;

//        float target_yaw_map = target_pose.yaw();
//        float base_to_target_yaw_map = std::atan2(target_pose.y() - actual_pose.y(),target_pose.x() - actual_pose.x() );
//        float base_yaw_map = actual_pose.yaw();
//        float wheel_yaw_base = angle_normalise(base_to_target_yaw_map, base_yaw_map) - base_yaw_map;

//        float base_to_target_dist = std::sqrt((actual_pose.x() -target_pose.x())*(actual_pose.x() - target_pose.x()) + (actual_pose.y() - target_pose.y())*(actual_pose.y() - target_pose.y()) );

//        float base_yaw_error = angle_normalise(target_yaw_map, base_yaw_map)-base_yaw_map;

//        float max_rot_vel =base_yaw_error>0.0  ? m_planner_config.pursuit_path_angle_rot_vel_max:-m_planner_config.pursuit_path_angle_rot_vel_max;
        float goal_angle_pid_p = m_planner_config.pursuit_path_rotate_vel_angle_p;
        float pursuit_path_rotate_vel_vel_p = m_planner_config.pursuit_path_rotate_vel_vel_p;

#if 0
        float goal_forward_vel = std::min(dynamic_v,m_planner_config.pursuit_path_forward_vel);
        PLOGD << "goal_forward_vel : " << goal_forward_vel;

        goal_forward_vel = std::min(goal_forward_vel, dynamic_v);
        goal_forward_vel = std::min(goal_forward_vel, m_forward_vel);
#endif



        command_linear_x = m_forward_vel*std::cos(m_forward_angle);
        command_linear_y = m_forward_vel*std::sin(m_forward_angle);
        command_angular_z = m_rotate_diff*goal_angle_pid_p;

//        command_angular_z = goal_angle_pid_p*base_yaw_error/(std::abs(full_path_len - m_planner_config.pursuit_goal_dist)/goal_forward_vel+1e-6f);
        command_angular_z = std::max(std::min(command_angular_z,m_planner_config.pursuit_path_angle_rot_vel_max ),-m_planner_config.pursuit_path_angle_rot_vel_max  );
        command_angular_z = std::max(std::min(command_angular_z,  std::abs(pursuit_path_rotate_vel_vel_p* m_forward_vel)),-std::abs(pursuit_path_rotate_vel_vel_p* m_forward_vel)  );

        // forbid rotate
        //1. start
        //2. at steer angle limit

        //todo: add a motor controller to check wheel angle

        m_driver_controller.cmd_vel(m_forward_vel,command_angular_z, m_forward_angle);
        if(
        std::abs(m_driver_controller.m_steer_wheel[0].command_rotate_angle - m_driver_controller.m_steer_wheel[1].command_rotate_angle  ) > M_PI_2f32
        ){
            command_angular_z = 0.0f;
            PLOGI << "force no rotate,  angle: " << m_driver_controller.m_steer_wheel[0].command_rotate_angle << ", " << m_driver_controller.m_steer_wheel[1].command_rotate_angle ;

        }
        PLOGI << "goal_forward_vel: " << m_forward_vel;
        PLOGI << "goal_angle_pid_p: " << goal_angle_pid_p;
//        PLOGD << "wheel_yaw_base: " << wheel_yaw_base;
//        PLOGD << "base_yaw_error: " << base_yaw_error;
        PLOGI << "m_rotate_diff: " << m_rotate_diff;
        PLOGI << "command_linear_x: " << command_linear_x;
        PLOGI << "command_linear_y: " << command_linear_y;
        PLOGI << "command_angular_z: " << command_angular_z;

        PLOGI << "full_path_len: " << full_path_len;

        // reach goal range
        //dist_to_goal
        if(full_path_len < m_planner_config.pursuit_goal_dist){

//            command_linear_x = 0.0;
//            command_linear_y = 0.0;
//
            command_angular_z = 0.0;
            return true;
        }

        return false;
    }

    bool DoubleSteerMotionPlanner::pursuit_goal() {


        auto & local_path = m_local_path.value;
        auto & global_path = m_global_path.value;
        auto& actual_pose = m_actual_pose.value;


        m_command.setCmd(0.0f,0.0f,0.0f);
        float& command_linear_x = m_command.command[0];
        float& command_linear_y = m_command.command[1];
        float& command_angular_z = m_command.command[2];

        float dist_to_goal = std::sqrt(transform::diff2(actual_pose, global_path.back()));

        float full_path_len = std::min(m_track_path_info[m_closest_path_node_id].dist_to_end,dist_to_goal ) ;



        float goal_forward_vel_pid_p = m_planner_config.pursuit_goal_forward_vel_pid_p;
        float goal_forward_vel_min = m_planner_config.pursuit_goal_forward_vel_min;
        float goal_forward_vel_max = m_planner_config.pursuit_goal_forward_vel_max;

        if(full_path_len < m_planner_config.pursuit_final_goal_dist){
            goal_forward_vel_min = m_planner_config.pursuit_goal_final_forward_vel_min;
            goal_forward_vel_max = m_planner_config.pursuit_goal_final_forward_vel_max;

        }




        {

            transform::Transform2d actual_pose_in_goal = m_track_path_info.back().pose.inverse() * actual_pose;
            m_report_error[0] = actual_pose_in_goal.x();
            m_report_error[1] = actual_pose_in_goal.y();
            m_report_error[2] = global_path.back().yaw() -actual_pose.yaw()  ;

            if(actual_pose_in_goal.x() > -m_planner_config.pursuit_goal_reach_tolerance
                    ){
                command_linear_x = 0.0;
                command_linear_y = 0.0;

                command_angular_z = 0.0;
                return true;
            }
        }


        size_t pose_num = global_path.size();

        m_planner_state = PlannerState::follow_goal;
        bool ok = createLocalPath();



        m_forward_vel = std::min(std::max(m_forward_vel,goal_forward_vel_min ),goal_forward_vel_max );

        m_forward_vel *= ok;

        command_linear_x = m_forward_vel*std::cos(m_forward_angle);
        command_linear_y = m_forward_vel*std::sin(m_forward_angle);
        command_angular_z = 0.0f;


        return false;



#if 0






        bool local_path_ok  = true;




        // predict future
        // wheel angle change
        // base angle change
        // forward dist

        transform::Transform2d target_pose;

        // reach goal range

        float goal_forward_vel = m_planner_config.pursuit_goal_forward_vel;
        float goal_angle_pid_p = m_planner_config.pursuit_goal_angle_pid_p;




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

        if(m_path_node_id == 0){

            if(std::abs(command_linear_x) < 1e-3 && std::abs(command_linear_y)<1e-3  ){
                command_linear_x = goal_forward_vel*std::cos(wheel_yaw_base);
                command_linear_y = goal_forward_vel*std::sin(wheel_yaw_base);
            }
        }else{
            command_linear_x = goal_forward_vel*std::cos(wheel_yaw_base);
            command_linear_y = goal_forward_vel*std::sin(wheel_yaw_base);
        }
        command_angular_z = base_yaw_error*goal_angle_pid_p;

        command_angular_z = std::max(std::min(command_angular_z,m_planner_config.pursuit_goal_angle_rot_vel_max ),-m_planner_config.pursuit_goal_angle_rot_vel_max  );


//        transform::Transform2d final_check_pose(global_path.back().x(), global_path.back().y(),final_path_yaw_map );
        transform::Transform2d& final_check_pose = m_track_path_info.back().pose;

//        transform::Transform2d actual_check_pose(actual_pose.x(), actual_pose.y(),final_path_yaw_map );

//        transform::Transform2d actual_pose_in_final = final_check_pose.inverse()*actual_check_pose  ;
        transform::Transform2d actual_pose_in_final = final_check_pose.inverse()*actual_pose  ;

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
#endif


    }

    bool DoubleSteerMotionPlanner::use_prefer_steer_angle() {
        return m_path_node_id == 0;
    }

    float DoubleSteerMotionPlanner::get_prefer_steer_angle() {
        return m_prefer_steer_angle;
    }
    void DoubleSteerMotionPlanner::resetPlanner() {



    }

    bool DoubleSteerMotionPlanner::goToNode(float node_id_float) {

        auto& actual_pose = m_actual_pose.value;
        float actual_pose_x = actual_pose.x();
        float actual_pose_y = actual_pose.y();
        float actual_pose_yaw = actual_pose.yaw();

        auto& global_path = m_global_path.value;

        node_id_float = std::min(std::max(node_id_float, 0.0f), float(global_path.size()-1));
        auto node_id = size_t(node_id_float);
        auto goal = transform::interpolate(node_id_float - node_id,global_path[node_id],global_path[node_id + 1]) ;

        float goal_x = goal.x();
        float goal_y = goal.y();
        float goal_yaw = goal.yaw();

        transform::Transform2d actual_pose_in_node = m_track_path_info[node_id].pose.inverse()*actual_pose;


        m_forward_diff = std::sqrt( transform::diff2(actual_pose,goal )  );

        bool reach = m_forward_diff < m_planner_config.pursuit_goal_reach_tolerance || actual_pose_in_node.x() > - m_planner_config.pursuit_goal_reach_tolerance;

        if(reach){
            m_forward_diff = 0.0f;
            m_forward_angle = 0.0f;
            m_rotate_diff = 0.0f;
            return true;
        }

        float base_to_goal_yaw = std::atan2(goal_y - actual_pose_y, goal_x - actual_pose_x);
        m_forward_angle = angle_normalise(base_to_goal_yaw, actual_pose_yaw) - actual_pose_yaw;
        m_rotate_diff = angle_normalise(goal_yaw, actual_pose_yaw) - actual_pose_yaw;

        if(m_forward_angle > 0.0){
            m_forward_angle = std::min(m_allow_angle_max[0], std::max(m_forward_angle, m_allow_angle_min[0]));
        }else{
            m_forward_angle = std::min(m_allow_angle_max[1], std::max(m_forward_angle, m_allow_angle_min[1]));
        }
        return false;
    }

    bool DoubleSteerMotionPlanner::goToNode(size_t node_id) {

        auto& actual_pose = m_actual_pose.value;
        float actual_pose_x = actual_pose.x();
        float actual_pose_y = actual_pose.y();
        float actual_pose_yaw = actual_pose.yaw();

        auto& global_path = m_global_path.value;

        auto& goal = global_path[node_id];

        float goal_x = goal.x();
        float goal_y = goal.y();
        float goal_yaw = goal.yaw();

        transform::Transform2d actual_pose_in_node = m_track_path_info[node_id].pose.inverse()*actual_pose;
        m_report_error[0] = actual_pose_in_node.x();
        m_report_error[1] = actual_pose_in_node.y();
        m_report_error[2] = angle_normalise(goal_yaw, actual_pose_yaw) - actual_pose_yaw;

        m_forward_diff = std::sqrt( transform::diff2(actual_pose,goal )  );

        bool reach = m_forward_diff < m_planner_config.pursuit_goal_reach_tolerance || actual_pose_in_node.x() > - m_planner_config.pursuit_goal_reach_tolerance;

        if(reach){
            m_forward_diff = 0.0f;
            m_forward_angle = 0.0f;
            m_rotate_diff = 0.0f;
            return true;
        }

        float base_to_goal_yaw = std::atan2(goal_y - actual_pose_y, goal_x - actual_pose_x);
        m_forward_angle = angle_normalise(base_to_goal_yaw, actual_pose_yaw) - actual_pose_yaw;
        m_rotate_diff = angle_normalise(goal_yaw, actual_pose_yaw) - actual_pose_yaw;

        if(m_forward_angle > 0.0){
            m_forward_angle = std::min(m_allow_angle_max[0], std::max(m_forward_angle, m_allow_angle_min[0]));
        }else{
            m_forward_angle = std::min(m_allow_angle_max[1], std::max(m_forward_angle, m_allow_angle_min[1]));
        }
        return false;
    }

    bool DoubleSteerMotionPlanner::gotoNode(const transform::Transform2d &goal) {

        auto& actual_pose = m_actual_pose.value;
        float actual_pose_x = actual_pose.x();
        float actual_pose_y = actual_pose.y();
        float actual_pose_yaw = actual_pose.yaw();


        float goal_x = goal.x();
        float goal_y = goal.y();
        float goal_yaw = goal.yaw();



        m_forward_diff = std::sqrt( transform::diff2(actual_pose,goal )  );


        float base_to_goal_yaw = std::atan2(goal_y - actual_pose_y, goal_x - actual_pose_x);
        m_forward_angle = angle_normalise(base_to_goal_yaw, actual_pose_yaw) - actual_pose_yaw;
        m_rotate_diff = angle_normalise(goal_yaw, actual_pose_yaw) - actual_pose_yaw;

        if(m_forward_angle > 0.0){
            m_forward_angle = std::min(m_allow_angle_max[0], std::max(m_forward_angle, m_allow_angle_min[0]));
        }else{
            m_forward_angle = std::min(m_allow_angle_max[1], std::max(m_forward_angle, m_allow_angle_min[1]));
        }
        return false;

    }
}