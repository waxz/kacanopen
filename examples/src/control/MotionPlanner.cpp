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
//        m_map_base_buffer.clear();
//        m_odom_base_buffer.clear();
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

        need_update = predict_error_dist > 0.03 || predict_error_angle > 0.05;

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


    void MotionPlanner::requestGoal(const common_message::PoseStamped& goal) {

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

        if(path.poses.size()<2){
            std::cout << "cancel goal , path poses size : " << path.poses.size()  << std::endl;
            stop();
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
        float min_rotate_vel = angle_diff > 0.0 ? m_min_base_rotate_vel:-m_min_base_rotate_vel;
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
            command_angular_z = std::min(std::min(command_feedback,command_angular_z ), m_planner_config.first_rotate_vel) ;
        }else{
            command_angular_z = std::max(std::max(command_feedback,command_angular_z ), - m_planner_config.first_rotate_vel) ;

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

            if(m_global_path.value.empty()){
                std::cout << "global_path is empty"  << std::endl;

                m_command.command_type = CommandType::cmd_vel;
                m_command.command.resize(3);
                std::fill(m_command.command.begin(), m_command.command.end(),0.0f);
                return m_command;
            }
            // check robot direction
            std::cout << "running_init" << std::endl;

            float angle_diff = angle_normalise(m_global_path.value[0].yaw(), m_actual_pose.value.yaw()) - m_actual_pose.value.yaw();

            interpolate_time = common::FromUnixNow();
            interpolate_time_step_0 = common::FromUnixNow();
            interpolate_time_step_1 = common::FromUnixNow();
            if(std::abs(angle_diff )< m_planner_config.first_rotate_angle_tolerance){

                m_task_state = TaskState::running_adjust_prepare;


            }else{
                m_rotate_init_angle_diff =angle_diff;
                m_task_state = TaskState::running_adjust_rotate;

            }


        }

        if(m_task_state == TaskState::running_adjust_rotate){
            std::cout << "running_adjust_rotate" << std::endl;

            bool ok = rotate(m_actual_pose.value.yaw(), m_global_path.value[0].yaw());
            if(ok){
                float angle_diff = angle_normalise(m_global_path.value[0].yaw(), m_actual_pose.value.yaw()) - m_actual_pose.value.yaw();
                interpolate_time = common::FromUnixNow();
                interpolate_time_step_0 = common::FromUnixNow();
                interpolate_time_step_1 = common::FromUnixNow();
                if(std::abs(angle_diff )< m_planner_config.first_rotate_angle_tolerance){

                    m_task_state = TaskState::running_adjust_prepare;


                }else{
                    m_rotate_init_angle_diff =angle_diff;
                    m_task_state = TaskState::running_adjust_rotate;

                }

            }
        }

        if(m_task_state == TaskState::running_adjust_prepare){
            std::cout << "running_adjust_prepare" << std::endl;

            bool ok = prepare();
            if(ok){
                m_task_state = TaskState::running_pursuit_path;
            }
        }

        if(m_task_state == TaskState::running_pursuit_path){
            std::cout << "running_pursuit_path" << std::endl;
            bool ok = pursuit_path();
            if(ok){
                m_task_state = TaskState::running_pursuit_goal;
            }
        }

        if(m_task_state == TaskState::running_pursuit_goal){
            std::cout << "running_pursuit_goal" << std::endl;
            bool ok = pursuit_goal();
            if(ok){
                reset();
                m_task_state = TaskState::finished;
            }
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

        m_command.command_type = CommandType::raw;
        m_command.command.resize(4);
        float& forward_vel_1 = m_command.command[0];
        float& rotate_angle_1 = m_command.command[1];
        float& forward_vel_2 = m_command.command[2];
        float& rotate_angle_2 = m_command.command[3];


        forward_vel_1 = 0.0f;
        rotate_angle_1 = m_wheel_state.value[1];

        forward_vel_2 = 0.0f;
        rotate_angle_2 = m_wheel_state.value[3];

        PLOGD << "m_wheel_state.value[1]: " << m_wheel_state.value[1];
        PLOGD << "m_wheel_state.value[3]: " << m_wheel_state.value[3];

        PLOGD << "m_start_wheel_angle: " << m_start_wheel_angle;
        //todo: remove spark point
        return true;

        float actual_angular_z = m_actual_odom.value.twist.twist.angular.z;


        rotate_angle_1 = m_start_wheel_angle;
        rotate_angle_2 = m_start_wheel_angle;

        std::cout <<"actual_angular_z: " << actual_angular_z << std::endl;

        if(std::abs(actual_angular_z) < 1e-3&& std::abs(m_wheel_state.value[1] - m_start_wheel_angle ) < 0.01&&std::abs(m_wheel_state.value[3] - m_start_wheel_angle ) < 0.01   &&  std::abs(m_wheel_state.value[0]) < 0.01 &&  std::abs(m_wheel_state.value[2]) < 0.01 ){
//            forward_vel_1  = 0.1;
//            forward_vel_2  = 0.1;

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
        float init_wheel_yaw_base[2] = {wheel_start_yaw_base,wheel_start_yaw_base};
        init_wheel_yaw_base[1] = (wheel_start_yaw_base > 0.0) ? wheel_start_yaw_base- M_PIf32 :wheel_start_yaw_base + M_PIf32;
        init_wheel_yaw_base[0] = angle_normalise_zero(init_wheel_yaw_base[0]);
        init_wheel_yaw_base[1] = angle_normalise_zero(init_wheel_yaw_base[1]);

        bool init_wheel_yaw_ok[2] = {true, true};


        for(size_t i = 0 ; i < pose_num-1;i++){

            float path_dist = std::sqrt((m_global_path.value[i+1].y() - m_global_path.value[i].y())*(m_global_path.value[i+1].y() - m_global_path.value[i].y()) + (m_global_path.value[i+1].x() - m_global_path.value[i].x() )*(m_global_path.value[i+1].x() - m_global_path.value[i].x() ));
            if(path_dist < 0.0001){
                continue;
            }
            float path_yaw = std::atan2(m_global_path.value[i+1].y() - m_global_path.value[i].y() ,m_global_path.value[i+1].x() - m_global_path.value[i].x()  );

            float base_yaw = m_global_path.value[i].yaw();

            init_wheel_yaw_ok[0] = init_wheel_yaw_ok[0] && std::abs(init_wheel_yaw_base[0] + path_yaw - base_yaw) < m_max_wheel_rotate_angle;
            init_wheel_yaw_ok[1] = init_wheel_yaw_ok[1] && std::abs(init_wheel_yaw_base[1] + path_yaw - base_yaw) < m_max_wheel_rotate_angle;
        }

        if(init_wheel_yaw_ok[0] || init_wheel_yaw_ok[1] ){

            float angle_diff[2] = {100.0f,100.0f};

            for(size_t i = 0 ; i  < 2;i++){

                angle_diff[i] = init_wheel_yaw_ok[i] ? std::abs(init_wheel_yaw_base[i]) : 1000.0f;
            }


            m_start_wheel_angle = angle_diff[0] < angle_diff[1] ? init_wheel_yaw_base[0] : init_wheel_yaw_base[1];

            return true;
        }else{
            return false;
        }


        return false;
    }

    bool DoubleSteerMotionPlanner::createLocalPath() {

        size_t pose_num = m_global_path.value.size();

        auto & local_path = m_local_path.value;
        auto & global_path = m_global_path.value;

        auto& actual_pose = m_actual_pose.value;
        local_path.clear();

//        local_path.push_back(actual_pose);

        {

            size_t choose_id = m_path_node_id;
            for(size_t i = m_path_node_id ; i < pose_num-1; i++){

                float path_yaw_map = std::atan2(global_path[i+1].y() - global_path[i].y(),global_path[i+1].x() - global_path[i].x() );
                float base_to_path_yaw_map = std::atan2(global_path[i].y() - actual_pose.y(),global_path[i].x() - actual_pose.x() );
                path_yaw_map = angle_normalise(path_yaw_map, base_to_path_yaw_map);
                float angle_diff = path_yaw_map - base_to_path_yaw_map;



                if(std::abs(angle_diff) < M_PI_2f32){
                    choose_id = i;
                    break;
                }

            }
            m_path_node_id = choose_id;

        }

        transform::Transform2d track_pose = actual_pose;

        float min_track_dist= 0.2*0.2;

        size_t converge_id = m_path_node_id;
        for(size_t i = m_path_node_id ; i < pose_num-1 ;i++){


            for(size_t j = i; j < pose_num-1;j++){

                float track_dist = (track_pose,global_path[i].x() - track_pose.x())*(track_pose,global_path[i].x() - track_pose.x()) + (track_pose,global_path[i].y() - track_pose.y())*(track_pose,global_path[i].y() - track_pose.y());
                if(track_dist > min_track_dist){
                    break;
                }
                i = j;
            }
            track_pose = transform::interpolate(m_planner_config.pursuit_path_interpolate_step, track_pose,global_path[i]);
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

        auto& actual_pose = m_actual_pose.value;

        m_command.command_type = CommandType::cmd_vel;
        m_command.command.resize(3);
        float& command_linear_x = m_command.command[0];
        float& command_linear_y = m_command.command[1];
        float& command_angular_z = m_command.command[2];


        float actual_angular_z = m_actual_odom.value.twist.twist.angular.z;


        bool local_path_ok = createLocalPath();

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

        transform::Transform2d& target_pose = local_path.front();

        float target_yaw_map = target_pose.yaw();
        float base_to_target_yaw_map = std::atan2(target_pose.y() - actual_pose.y(),target_pose.x() - actual_pose.x() );
        float base_yaw_map = actual_pose.yaw();
        float wheel_yaw_base = angle_normalise(base_to_target_yaw_map, base_yaw_map) - base_yaw_map;

        float base_to_target_dist = std::sqrt((actual_pose.x() -target_pose.x())*(actual_pose.x() - target_pose.x()) + (actual_pose.y() - target_pose.y())*(actual_pose.y() - target_pose.y()) );

        float base_yaw_error = angle_normalise(target_yaw_map, base_yaw_map)-base_yaw_map;


        float goal_forward_vel = m_planner_config.pursuit_path_forward_vel;
        float goal_angle_pid_p = m_planner_config.pursuit_path_angle_pid_p;


        command_linear_x = goal_forward_vel*std::cos(wheel_yaw_base);
        command_linear_y = goal_forward_vel*std::sin(wheel_yaw_base);
        command_angular_z = base_yaw_error*goal_angle_pid_p;


        PLOGD << "goal_forward_vel: " << goal_forward_vel;
        PLOGD << "goal_angle_pid_p: " << goal_angle_pid_p;
        PLOGD << "wheel_yaw_base: " << wheel_yaw_base;
        PLOGD << "base_yaw_error: " << base_yaw_error;

        PLOGD << "command_linear_x: " << command_linear_x;
        PLOGD << "command_linear_y: " << command_linear_y;
        PLOGD << "command_angular_z: " << command_angular_z;


        // reach goal range
        float dist_to_goal = std::sqrt((actual_pose.x() - global_path.back().x())*(actual_pose.x() - global_path.back().x()) + (actual_pose.y() - global_path.back().y())*(actual_pose.y() - global_path.back().y()) );
        if(dist_to_goal < m_planner_config.pursuit_goal_dist){
            command_linear_x = 0.0;
            command_linear_y = 0.0;

            command_angular_z = 0.0;
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



        bool local_path_ok = createLocalPath();

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

        transform::Transform2d target_pose = local_path.front();


        // reach goal range

        float goal_forward_vel = m_planner_config.pursuit_goal_forward_vel;
        float goal_angle_pid_p = m_planner_config.pursuit_goal_angle_pid_p;

        float dist_to_goal = std::sqrt((actual_pose.x() - global_path.back().x())*(actual_pose.x() - global_path.back().x()) + (actual_pose.y() - global_path.back().y())*(actual_pose.y() - global_path.back().y()) );
        if(dist_to_goal < m_planner_config.pursuit_direct_goal_dist){
            target_pose = global_path.back();
        }
        if(dist_to_goal < m_planner_config.pursuit_final_goal_dist){
            goal_forward_vel = m_planner_config.pursuit_goal_final_forward_vel;
            goal_angle_pid_p = m_planner_config.pursuit_goal_final_angle_pid_p;

        }



        float final_path_yaw_map = std::atan2(global_path[global_path.size()-1].y() - global_path[global_path.size()-2].y(), global_path[global_path.size()-1].x() - global_path[global_path.size()-2].x());
        float target_yaw_map = target_pose.yaw();

        float base_to_target_yaw_map = std::atan2(target_pose.y() - actual_pose.y(),target_pose.x() - actual_pose.x() );
        float base_yaw_map = actual_pose.yaw();
        float wheel_yaw_base = angle_normalise(base_to_target_yaw_map, base_yaw_map) - base_yaw_map;

        float base_to_target_dist = std::sqrt((actual_pose.x() -target_pose.x())*(actual_pose.x() - target_pose.x()) + (actual_pose.y() - target_pose.y())*(actual_pose.y() - target_pose.y()) );

        float base_yaw_error = angle_normalise(target_yaw_map, base_yaw_map)-base_yaw_map;



        command_linear_x = goal_forward_vel*std::cos(wheel_yaw_base);
        command_linear_y = goal_forward_vel*std::sin(wheel_yaw_base);
        command_angular_z = base_yaw_error*goal_angle_pid_p;



        float final_yaw = angle_normalise(final_path_yaw_map ,base_to_target_yaw_map) - base_to_target_yaw_map;
        float final_dist = dist_to_goal*std::cos(final_yaw);
        std::cout << "dist_to_goal: "<< dist_to_goal<< ", final_yaw: " << final_yaw << ", final_dist: " << final_dist << ", pursuit_goal_reach_tolerance: " <<  m_planner_config.pursuit_goal_reach_tolerance<< std::endl;
        if(dist_to_goal < m_planner_config.pursuit_direct_goal_dist && final_dist < m_planner_config.pursuit_goal_reach_tolerance ){
            command_linear_x = 0.0;
            command_linear_y = 0.0;

            command_angular_z = 0.0;
            return true;
        }

        return false;


    }
    void DoubleSteerMotionPlanner::resetPlanner() {


        m_path_node_id = 0;
    }
}