//
// Created by waxz on 5/12/23.
//

#include "MobileRobotController.h"

#define DEFINE_ASSERT_MODE AssertMode::THROW_
#define DEFINE_ASSERT_LEVEL 1

#include "dynamic_assert/assertion.h"
#include "common/string_logger.h"
#include "math/geometry.h"
#include "PathGenerator.h"

#include <plog/Log.h> // Step1: include the headers
#include <plog/Initializers/RollingFileInitializer.h>
#include <plog/Appenders/ColorConsoleAppender.h>

#include "math/math_basic.h"
#include "common/random.h"

namespace control {

    void SteerWheelBase::config() {
    }
    void SteerWheelBase::createCommand() {
        command_forward_vel = target_forward_vel;
        command_forward_vel = std::min(std::max(command_forward_vel, -max_forward_vel), max_forward_vel);


        //target rot angle in motor frame
        target_angle_in_motor = target_rot_angle ;

        target_angle_in_motor = angle_normalise_zero(target_angle_in_motor);
        // actual rot angle in motor frame
        actual_angle_in_motor = actual_rot_angle ;
        actual_rot_angle = angle_normalise_zero(actual_rot_angle);


        // choose possible rotate angle
        PLOGD << "target_angle_in_motor: " << target_angle_in_motor ;

        if(target_angle_in_motor > min_rot_angle && target_angle_in_motor < max_rot_angle){

            // two possible rot angle in motor frame
            float angle_1 = target_angle_in_motor > 0.0 ? target_angle_in_motor - M_PIf32: target_angle_in_motor;
            float angle_2 = target_angle_in_motor > 0.0 ? target_angle_in_motor: target_angle_in_motor + M_PIf32;

            if(angle_1 < min_rot_angle  ||  angle_1 > max_rot_angle ){
                angle_1 = target_angle_in_motor;
            }

            if(angle_2 < min_rot_angle  ||  angle_2 > max_rot_angle ){
                angle_2 = target_angle_in_motor;
            }


            float command_angle =  (std::abs(actual_angle_in_motor - angle_1) < std::abs(actual_angle_in_motor - angle_2)? angle_1:angle_2 );

            command_forward_vel = (std::abs(target_angle_in_motor - command_angle) < 0.1 ? command_forward_vel:-command_forward_vel);
            command_rotate_angle = command_angle;

            PLOGD << "command_rotate_angle: " << command_rotate_angle ;
        }else if(target_angle_in_motor < min_rot_angle ){


            command_rotate_angle =  target_angle_in_motor + M_PIf32;
            command_forward_vel = -command_forward_vel;
            PLOGD << "command_rotate_angle: " << command_rotate_angle ;

        }else  if(target_angle_in_motor > max_rot_angle ){

            command_rotate_angle  = target_angle_in_motor + M_PIf32;
            command_forward_vel = -command_forward_vel;
            PLOGD << "command_rotate_angle: " << command_rotate_angle ;

        }else{
            PLOGD << "command_rotate_angle: " << command_rotate_angle ;

        }

        command_rotate_angle = angle_normalise_zero(command_rotate_angle);
        PLOGD << "command_rotate_angle: " << command_rotate_angle ;

        // interpolate forward_vel and rot_angle

    }
    void DoubleSteerController::reset() {

        state_code = StateCode::Uninitialised;
        time = common::FromUnixNow();
        position.set(0.0, 0.0, 0.0);
        odom_flag = 0;
        actual_forward_vel = 0.0;
        actual_forward_angle = 0.0;
        actual_rotate_vel = 0.0;
    }

    const transform::Transform2d &DoubleSteerController::getPosition() {
        return position;
    }
    float DoubleSteerController::getActualForwardVel() {
        return actual_forward_vel;
    }

    float DoubleSteerController::getActualForwardAngle() {
        return actual_forward_angle;
    }

    float DoubleSteerController::getActualRotateVel() {
        return actual_rotate_vel;
    }
    void DoubleSteerController::cmd_vel(float forward_vel, float rot_vel) {
        cmd_vel(forward_vel, rot_vel, 0.0);
    }


    void DoubleSteerController::cmd_vel(float forward_vel, float rot_vel, float forward_angle) {

        m_cmd_forward_vel = forward_vel;
        m_cmd_rot_vel = rot_vel;
        m_cmd_forward_angle = forward_angle;

        float cos_forward_angle = std::cos(m_cmd_forward_angle);
        float sin_forward_angle = std::sin(m_cmd_forward_angle);

        bool is_command_rotate = std::abs(m_cmd_rot_vel) > 0.0001f;
        bool is_command_move = std::abs(m_cmd_forward_vel) > 0.0001f;

        MLOGI("is_command_move: %d, is_command_rotate: %d", is_command_move, is_command_rotate);

        if (!is_command_move) {

            if (!is_command_rotate) {
                m_steer_wheel[0].target_rot_angle = m_cmd_forward_angle;
                m_steer_wheel[1].target_rot_angle = m_cmd_forward_angle;
                m_steer_wheel[0].target_forward_vel = 0.0f;
                m_steer_wheel[1].target_forward_vel = 0.0f;
                return;
            } else {

                float command_rotate_radius = 0.5f * std::sqrt(
                        (m_steer_wheel[0].mount_position_x - m_steer_wheel[1].mount_position_x) *
                        (m_steer_wheel[0].mount_position_x - m_steer_wheel[1].mount_position_x)
                        + (m_steer_wheel[0].mount_position_y - m_steer_wheel[1].mount_position_y) *
                          (m_steer_wheel[0].mount_position_y - m_steer_wheel[1].mount_position_y));

                float command_rotate_center_x = 0.0f;
                float command_rotate_center_y = 0.0f;

                m_steer_wheel[0].target_rot_angle =
                        std::atan2(m_steer_wheel[0].mount_position_y - command_rotate_center_y,
                                   m_steer_wheel[0].mount_position_x - command_rotate_center_x) + M_PI_2f32;
                m_steer_wheel[1].target_rot_angle =
                        std::atan2(m_steer_wheel[1].mount_position_y - command_rotate_center_y,
                                   m_steer_wheel[1].mount_position_x - command_rotate_center_x) + M_PI_2f32;


                m_steer_wheel[0].target_forward_vel = m_cmd_rot_vel * command_rotate_radius;
                m_steer_wheel[1].target_forward_vel = m_cmd_rot_vel * command_rotate_radius;

            }

        }

        transform::Transform2d dummy_position(0.0f, 0.0f, m_cmd_forward_angle);

        if (is_command_rotate) {
            float command_rotate_radius = m_cmd_forward_vel / m_cmd_rot_vel;
            float command_rotate_angle = m_cmd_forward_angle + (command_rotate_radius > 0.0 ? M_PI_2f32 : -M_PI_2f32);

            MLOGI("command_rotate_radius: %.3f, command_rotate_angle: %.3f", command_rotate_radius,
                  command_rotate_angle);

            float command_rotate_center_x = std::abs(command_rotate_radius) * std::cos(command_rotate_angle);
            float command_rotate_center_y = std::abs(command_rotate_radius) * std::sin(command_rotate_angle);
            MLOGI("command_rotate_center_x: %.3f, command_rotate_center_y: %.3f", command_rotate_center_x,
                  command_rotate_center_y);

            m_steer_wheel[0].target_rot_angle = std::atan2(m_steer_wheel[0].mount_position_y - command_rotate_center_y,
                                                           m_steer_wheel[0].mount_position_x -
                                                           command_rotate_center_x) +
                                                (m_cmd_rot_vel > 0.0 ? M_PI_2f32 : -M_PI_2f32);
            m_steer_wheel[1].target_rot_angle = std::atan2(m_steer_wheel[1].mount_position_y - command_rotate_center_y,
                                                           m_steer_wheel[1].mount_position_x -
                                                           command_rotate_center_x) +
                                                (m_cmd_rot_vel > 0.0 ? M_PI_2f32 : -M_PI_2f32);
            MLOGI("m_steer_wheel[0].mount_position_x: %.3f, m_steer_wheel[0].mount_position_y: %.3f",
                  m_steer_wheel[0].mount_position_x, m_steer_wheel[0].mount_position_y);
            MLOGI("m_steer_wheel[1].mount_position_x: %.3f, m_steer_wheel[1].mount_position_y: %.3f",
                  m_steer_wheel[1].mount_position_x, m_steer_wheel[1].mount_position_y);

            float command_rotate_radius_1 = std::sqrt((m_steer_wheel[0].mount_position_x - command_rotate_center_x) *
                                                      (m_steer_wheel[0].mount_position_x - command_rotate_center_x) +
                                                      (m_steer_wheel[0].mount_position_y - command_rotate_center_y) *
                                                      (m_steer_wheel[0].mount_position_y - command_rotate_center_y));
            float command_rotate_radius_2 = std::sqrt((m_steer_wheel[1].mount_position_x - command_rotate_center_x) *
                                                      (m_steer_wheel[1].mount_position_x - command_rotate_center_x) +
                                                      (m_steer_wheel[1].mount_position_y - command_rotate_center_y) *
                                                      (m_steer_wheel[1].mount_position_y - command_rotate_center_y));
            MLOGI("command_rotate_radius_1: %.3f, command_rotate_radius_2: %.3f", command_rotate_radius_1,
                  command_rotate_radius_2);

            float wheel_to_rotate_center_angle_1 = std::atan2(
                    m_steer_wheel[0].mount_position_y - command_rotate_center_y,
                    m_steer_wheel[0].mount_position_x - command_rotate_center_x);
            float wheel_to_rotate_center_angle_2 = std::atan2(
                    m_steer_wheel[1].mount_position_y - command_rotate_center_y,
                    m_steer_wheel[1].mount_position_x - command_rotate_center_x);

            wheel_to_rotate_center_angle_1 = angle_normalise(wheel_to_rotate_center_angle_1,
                                                             m_steer_wheel[0].target_rot_angle);
            wheel_to_rotate_center_angle_2 = angle_normalise(wheel_to_rotate_center_angle_2,
                                                             m_steer_wheel[0].target_rot_angle);

            float rotate_diff_1 = m_steer_wheel[0].target_rot_angle - wheel_to_rotate_center_angle_1;
            float rotate_diff_2 = m_steer_wheel[1].target_rot_angle - wheel_to_rotate_center_angle_2;

            m_steer_wheel[0].target_forward_vel = std::abs(m_cmd_rot_vel) *
                                                  command_rotate_radius_1;// * (m_cmd_rot_vel*rotate_diff_1 > 0.0 ? (1.0f):(-1.0f));
            m_steer_wheel[1].target_forward_vel = std::abs(m_cmd_rot_vel) *
                                                  command_rotate_radius_2;//* (m_cmd_rot_vel*rotate_diff_2 > 0.0 ? (1.0f):(-1.0f));

        } else {

            m_steer_wheel[0].target_rot_angle = m_cmd_forward_angle;
            m_steer_wheel[1].target_rot_angle = m_cmd_forward_angle;
            m_steer_wheel[0].target_forward_vel = m_cmd_forward_vel;
            m_steer_wheel[1].target_forward_vel = m_cmd_forward_vel;
            return;
        }


    }

    void DoubleSteerController::feed_back(const std::vector<float> &wheel_states) {

        /*
       wheel_states : [forward_vel,rot_angle],[forward_vel,rot_angle],[forward_vel,rot_angle]
         */
        dynamic_assert(wheel_states.size() == 4, "DoubleSteerController::feed_back get wrong wheel_states size()");

        const float &forward_vel_1 = wheel_states[0];
        const float &rot_angle_1 = wheel_states[1];
        const float &forward_vel_2 = wheel_states[2];
        const float &rot_angle_2 = wheel_states[3];

        common::Time now = common::FromUnixNow();

        float interval_seconds = 0.001f * static_cast<float>(common::ToMicroSeconds(now - time));
        time = now;


        m_steer_wheel[0].updateState(forward_vel_1, rot_angle_1);
        m_steer_wheel[1].updateState(forward_vel_2, rot_angle_2);

        // update rotate_center_x
        // update rotate_center_x
        // update is_rotate

#if 0
        if(std::abs(m_steer_wheel[0].actual_forward_vel) < 0.0001f && std::abs(m_steer_wheel[1].actual_forward_vel) < 0.0001f   ){
            // two wheel stop, robot is not move
            m_actual_forward_angle = 0.0f;
            m_actual_rot_vel = 0.0f;
            m_actual_forward_vel = 0.0f;

            rotate_center_x = 0.0;
            rotate_center_y = 0.0;
            is_rotate = false;

            return;
        }else  if(std::abs(m_steer_wheel[0].actual_forward_vel) < 0.0001f    ){
            // one wheel stop, robot is rotating around m_steer_wheel[0]
            rotate_center_x = m_steer_wheel[0].mount_position_x;
            rotate_center_y = m_steer_wheel[0].mount_position_y;
            is_rotate = true;
        }else  if(std::abs(m_steer_wheel[1].actual_forward_vel) < 0.0001f   ){
            // one wheel stop, robot is rotating around m_steer_wheel[1]
            rotate_center_x = m_steer_wheel[1].mount_position_x;
            rotate_center_y = m_steer_wheel[1].mount_position_y;
            is_rotate = true;


        }else{
            // get rotate center


            float x1 = m_steer_wheel[0].mount_position_x ;
            float y1 = m_steer_wheel[0].mount_position_y;  //Line 1 start
            float x2 = x1 + cosf(m_steer_wheel[0].actual_rot_angle + M_PI_2f32);
            float y2 = y1 + sinf(m_steer_wheel[0].actual_rot_angle + M_PI_2f32);  //Line 1 end


            float x3 = m_steer_wheel[1].mount_position_x;
            float y3 = m_steer_wheel[1].mount_position_y; //Line 2 start
            float x4 = x3 + std::cos(m_steer_wheel[1].actual_rot_angle + M_PI_2f32);
            float y4 = y3 + std::sin(m_steer_wheel[1].actual_rot_angle + M_PI_2f32);  //Line 2 end

            float cx = rotate_center_x, cy = rotate_center_y;
            bool is_parallel = math::LineLineIntersect(x1,y1,x2,y2,x3,y3,x4,y4,cx ,cy);



            if(is_parallel){


                // two wheel move to same direction
                if(m_steer_wheel[0].actual_forward_vel * m_steer_wheel[1].actual_forward_vel > 0.0){
                    is_rotate = false;

                    if(std::abs(m_steer_wheel[0].actual_forward_vel - m_steer_wheel[1].actual_forward_vel) < 0.001){
                        // speed is synced


                    }else{
                        // speed is not synced

                        state_code = StateCode::FeedBackError;

                        return;

                    }

                }else{





                }


            }else{
                is_rotate = true;
                rotate_center_x = cx;
                rotate_center_y = cy;
            }


        }



        /*

         if two wheel is_parallel and has same velocity, then robot is moving toward wheel direction
         if two wheel is parallel , but has different velocity,

         */



        // para
//        is_rotate = is_parallel && ;
        float radius_1 = std::sqrt((x1 - rotate_center_x)*(x1 - rotate_center_x) + (y1 - rotate_center_y)*(y1 - rotate_center_y));
        float radius_2 = std::sqrt((x2 - rotate_center_x)*(x2 - rotate_center_x) + (y2 - rotate_center_y)*(y2 - rotate_center_y));

        float radius = std::sqrt(rotate_center_x*rotate_center_x + rotate_center_y*rotate_center_y);





        // update m_actual_rot_vel
        // update m_actual_forward_angle
        // update m_actual_forward_vel
        // w = v/r

        // check if rotate
        if(is_rotate){
            if(std::abs(radius_1) > 1e-6f && std::abs(radius_2) > 1e-6f  ){
                float rotate_vel_1 =  m_steer_wheel[0].actual_forward_vel /radius_1;
                float rotate_vel_2 =  m_steer_wheel[1].actual_forward_vel /radius_2;

                m_actual_rot_vel = 0.5f*(rotate_vel_1 + rotate_vel_2);
            }else  if(std::abs(radius_1) > 1e-6f){
                float rotate_vel_1 =  m_steer_wheel[0].actual_forward_vel /radius_1;

                m_actual_rot_vel = (rotate_vel_1 );
            }else  if(std::abs(radius_2) > 1e-6f  ){
                float rotate_vel_2 =  m_steer_wheel[1].actual_forward_vel /radius_2;

                m_actual_rot_vel = (rotate_vel_2);
            }

            // check if inplace rotate
            if(radius< 0.00001f){
                m_actual_forward_angle = 0.0f;

            }else{
                m_actual_forward_angle = std::atan2(rotate_center_y,rotate_center_x) + M_PI_2f32;
                if(m_actual_forward_angle < - M_PI_2f32 ){
                    m_actual_forward_angle += M_PIf32;
                }else if(m_actual_forward_angle > M_PI_2f32 ){
                    m_actual_forward_angle -= M_PIf32;
                }
            }
            m_actual_forward_vel = m_actual_rot_vel*radius;


        }else{

            m_actual_forward_angle = 0.0;
            m_actual_rot_vel = 0.0;
            m_actual_forward_vel = 0.5*(m_steer_wheel[0].actual_forward_vel  + m_steer_wheel[1].actual_forward_vel );
        }



        float cos_actual_forward_angle = std::cos(m_actual_forward_angle);
        float sin_actual_forward_angle = std::sin(m_actual_forward_angle);



        if(is_rotate){

            if(radius< 0.00001f){
                // rotate inplace
                float predict_relative_angle = interval_seconds*m_actual_rot_vel;
                float predict_relative_dist = interval_seconds*m_actual_forward_vel;

                transform::Transform2d relative_movement(predict_relative_dist*cos_actual_forward_angle, predict_relative_dist*sin_actual_forward_angle, predict_relative_angle);
                position =  position * relative_movement;

            }else{
                float curv = 1.0f/radius * (rotate_center_y > 0.0 ? 1.0 : -1.0);
                float dist = interval_seconds*m_actual_forward_vel;
                transform::Transform2d dummy_position(position.x(),position.y(),position.yaw() + m_actual_forward_angle);

                dummy_position = updateCurveDist(dummy_position,curv,dist);
                position.set(dummy_position.x(),dummy_position.y(),dummy_position.yaw() - m_actual_forward_angle);

            }

        }else{

            // move forward
            float predict_relative_angle = interval_seconds*m_actual_rot_vel;
            float predict_relative_dist = interval_seconds*m_actual_forward_vel;

            transform::Transform2d relative_movement(predict_relative_dist*cos_actual_forward_angle, predict_relative_dist*sin_actual_forward_angle, predict_relative_angle);
            position =  position * relative_movement;

        }


#endif


    }

    void
    DoubleSteerController::set_wheel(const control::SteerWheelBase &wheel_1, const control::SteerWheelBase &wheel_2) {
        m_steer_wheel[0] = wheel_1;
        m_steer_wheel[1] = wheel_2;

        for(size_t i = 0 ; i < m_steer_wheel.size();i++){
            m_steer_wheel[i].config();
        }

        constrain_angle = std::atan2(m_steer_wheel[1].mount_position_y - m_steer_wheel[0].mount_position_y,
                                     m_steer_wheel[1].mount_position_x - m_steer_wheel[0].mount_position_x);

        constrain_angle = std::abs(constrain_angle) < M_PI_2f32 ? constrain_angle : constrain_angle + (constrain_angle > 0.0f ? -M_PIf32 : M_PIf32  );

    }

    void
    DoubleSteerController::updateState(float forward_vel_1, float rot_angle_1, float forward_vel_2, float rot_angle_2) {
        m_steer_wheel[0].updateState(forward_vel_1, rot_angle_1);
        m_steer_wheel[1].updateState(forward_vel_2, rot_angle_2);

        m_steer_wheel[0].steer_constrain_vel =
                m_steer_wheel[0].actual_forward_vel * std::cos(m_steer_wheel[0].actual_rot_angle - constrain_angle);

        m_steer_wheel[1].steer_constrain_vel =
                m_steer_wheel[1].actual_forward_vel * std::cos(m_steer_wheel[1].actual_rot_angle - constrain_angle);

        // compute odom
        float constrain_vel_along = 0.5f*(m_steer_wheel[0].steer_constrain_vel + m_steer_wheel[1].steer_constrain_vel);


        float constrain_vel_vertical_1 =
                m_steer_wheel[0].actual_forward_vel * std::sin(m_steer_wheel[0].actual_rot_angle - constrain_angle);
        float constrain_vel_vertical_2 =
                m_steer_wheel[1].actual_forward_vel * std::sin(m_steer_wheel[1].actual_rot_angle - constrain_angle);


        float constrain_vel_vertical = 0.0f;

        float constrain_rotate_vel = 0.0;

        float constrain_rotate_radius_1 = std::sqrt(m_steer_wheel[0].mount_position_x*m_steer_wheel[0].mount_position_x + m_steer_wheel[0].mount_position_y*m_steer_wheel[0].mount_position_y);
        float constrain_rotate_radius_2 = std::sqrt(m_steer_wheel[1].mount_position_x*m_steer_wheel[1].mount_position_x + m_steer_wheel[1].mount_position_y*m_steer_wheel[1].mount_position_y);

        constrain_rotate_radius_1 = m_steer_wheel[0].mount_position_x > 0.0 ? constrain_rotate_radius_1 : -constrain_rotate_radius_1;
        constrain_rotate_radius_2 = m_steer_wheel[1].mount_position_x > 0.0 ? constrain_rotate_radius_2 : -constrain_rotate_radius_2;


        bool is_command_move = std::abs(m_steer_wheel[0].actual_forward_vel) > 0.001f || std::abs(m_steer_wheel[1].actual_forward_vel) > 0.001f;

        bool is_rotate = std::abs(constrain_vel_vertical_1 - constrain_vel_vertical_2) > 0.01;


        float base_vel = 0.0;
        float base_vel_x = 0.0;
        float base_vel_y = 0.0;
        float base_vel_yaw = 0.0;

        if(odom_flag == 0){

            time = common::FromUnixNow();

            position.set(0.0,0.0,0.0);
            odom_flag ++;
            return;
        }

        if(!is_command_move){
            // not move
            actual_forward_vel = 0.0;
            actual_forward_angle = 0.0;
            actual_rotate_vel = 0.0;
            time = common::FromUnixNow();
             new_time = Clock::now();
            PLOGD << "position:\n" << position;
            return;
        }

        if(is_rotate){


            float constrain_rotate_radius = (constrain_vel_vertical_1 * constrain_rotate_radius_2 - constrain_vel_vertical_2*constrain_rotate_radius_1)/(constrain_vel_vertical_1 - constrain_vel_vertical_2);

            float rot_vel_1 = 0.0;
            float rot_vel_2 = 0.0;
            if(std::abs(constrain_vel_vertical_1) > 0.001 && std::abs(constrain_vel_vertical_2) > 0.001 ){
                rot_vel_1 = constrain_vel_vertical_1/(constrain_rotate_radius_1 - constrain_rotate_radius);
                rot_vel_2 = constrain_vel_vertical_2/(constrain_rotate_radius_2 - constrain_rotate_radius);

            }else if (std::abs(constrain_vel_vertical_1) < 0.001 ){
                rot_vel_2 = constrain_vel_vertical_2/(constrain_rotate_radius_2 - constrain_rotate_radius);
                rot_vel_1 = rot_vel_2;

            }else if (std::abs(constrain_vel_vertical_2) < 0.001 ){
                rot_vel_1 = constrain_vel_vertical_1/(constrain_rotate_radius_1 - constrain_rotate_radius);
                rot_vel_2 =  rot_vel_1;
            }



            actual_rotate_vel = 0.5f*(rot_vel_1 + rot_vel_2);

            constrain_vel_vertical =actual_rotate_vel *  -constrain_rotate_radius;
            PLOGD << "check_vertical constrain_rotate_radius: "   << constrain_rotate_radius_1 << ", " << constrain_rotate_radius_2 << ", "<< constrain_rotate_radius;

            PLOGD << "check_vertical rot_vel: " << rot_vel_1 << ", " << rot_vel_2;
            PLOGD << "check_vertical vel_vertical: " << constrain_vel_vertical_1 << ", " << constrain_vel_vertical_2 << ", " << constrain_vel_vertical;


            base_vel_x = constrain_vel_along*std::cos(constrain_angle) + constrain_vel_vertical*std::cos(constrain_angle + M_PI_2f32);
            base_vel_y = constrain_vel_along*std::sin(constrain_angle) + constrain_vel_vertical*std::sin(constrain_angle + M_PI_2f32);


            actual_forward_vel = std::sqrt(base_vel_x*base_vel_x + base_vel_y*base_vel_y);
            actual_forward_angle = std::atan2(base_vel_y,base_vel_x);

            PLOGD << "actual_forward_vel: " << actual_forward_vel ;
            PLOGD << "actual_forward_angle: " << actual_forward_angle ;
            PLOGD << "actual_rotate_vel: " << actual_rotate_vel ;

        }else{

            float wheel_vel_x_1 = m_steer_wheel[0].actual_forward_vel * std::cos(m_steer_wheel[0].actual_rot_angle);
            float wheel_vel_y_1 = m_steer_wheel[0].actual_forward_vel * std::sin(m_steer_wheel[0].actual_rot_angle);


            float wheel_vel_x_2 = m_steer_wheel[1].actual_forward_vel * std::cos(m_steer_wheel[1].actual_rot_angle);
            float wheel_vel_y_2 = m_steer_wheel[1].actual_forward_vel * std::sin(m_steer_wheel[1].actual_rot_angle);


            base_vel_x = 0.5f*(wheel_vel_x_1 + wheel_vel_x_2);
            base_vel_y = 0.5f*(wheel_vel_y_1 + wheel_vel_y_2);

            actual_forward_vel = std::sqrt(base_vel_x*base_vel_x + base_vel_y*base_vel_y);
            actual_forward_angle = std::atan2(base_vel_y,base_vel_x);
            actual_rotate_vel = 0.0;
            PLOGD << "actual_forward_vel: " << actual_forward_vel ;
            PLOGD << "actual_forward_angle: " << actual_forward_angle ;
            PLOGD << "actual_rotate_vel: " << actual_rotate_vel ;
        }


        {
            // simulate movement



            common::Time  now =  common::FromUnixNow();
//            auto new_now = Clock::now();

            float update_s = common::ToMicroSeconds(now - time) * 1e-6f;
//            float new_update_s = (new_now - new_time).count() * 1e-9f;
            time = now;
//            new_time = new_now;

            float update_step = 1e-3f;
//            PLOGD << "update_s: " << update_s;
//            PLOGD << "new_update_s: " << new_update_s;
//            update_s = new_update_s;

//            PLOGD << "base_vel[x,y,w]: " << base_vel_x << ", " << base_vel_y<< ", " << actual_rotate_vel;
//            PLOGD << "origin position: " << position;

            transform::Transform2d relative_pose;
            while (update_s > 0.0001){
                float s = std::min(update_step, update_s);
                relative_pose.set(base_vel_x * s, base_vel_y * s,actual_rotate_vel*s );
//                PLOGD << "relative_pose: " << relative_pose;

                position = position * relative_pose;
                update_s -= update_step;
//                PLOGD << "update position:\n" << position;

            }
            PLOGD << "final position:\n" << position;



        }


//        position.set(position.x() + 0.001f, position.y() + 0.001f, position.yaw() + 0.001f);




    }

    void DoubleSteerController::interpolate() {

        float update_s  = control_period_sec;

        for(size_t i = 0 ; i < m_steer_wheel.size(); i++){
            m_steer_wheel[i].createCommand();
        }


        bool is_command_rotate = std::abs(m_cmd_rot_vel) > 0.0001f;
        bool is_command_move = std::abs(m_cmd_forward_vel) > 0.0001f;

        bool is_steer_forward_stopped_0 = std::abs(m_steer_wheel[0].actual_forward_vel) < forward_vel_reach_thresh;
        bool is_steer_forward_stopped_1 = std::abs(m_steer_wheel[1].actual_forward_vel) < forward_vel_reach_thresh;

        bool is_steer_rotate_zero_0 = std::abs(m_steer_wheel[0].actual_rot_angle) < rotate_angle_reach_thresh;
        bool is_steer_rotate_zero_1 = std::abs(m_steer_wheel[1].actual_rot_angle) < rotate_angle_reach_thresh;

        bool is_steer_forward_synced_0 =
                std::abs(m_steer_wheel[0].interpolate_command_forward_vel - m_steer_wheel[0].actual_forward_vel) <
                forward_vel_reach_thresh;
        bool is_steer_forward_synced_1 =
                std::abs(m_steer_wheel[1].interpolate_command_forward_vel - m_steer_wheel[1].actual_forward_vel) <
                forward_vel_reach_thresh;


        bool is_steer_rotate_synced_0 =
                std::abs(m_steer_wheel[0].interpolate_command_rotate_angle - m_steer_wheel[0].actual_angle_in_motor) <
                rotate_angle_reach_thresh;
        bool is_steer_rotate_synced_1 =
                std::abs(m_steer_wheel[1].interpolate_command_rotate_angle - m_steer_wheel[1].actual_angle_in_motor) <
                rotate_angle_reach_thresh;

        PLOGD << "is_steer_forward_stopped_0: " << is_steer_forward_stopped_0 << ", "
              << "is_steer_forward_stopped_1: " << is_steer_forward_stopped_0;

        PLOGD << "is_steer_rotate_zero_0: " << is_steer_rotate_zero_0 << ", "
              << "is_steer_rotate_zero_1: " << is_steer_rotate_zero_1;

        PLOGD << "is_steer_forward_synced_0: " << is_steer_forward_synced_0 << ", "
              << "is_steer_forward_synced_1: " << is_steer_forward_synced_1;

        PLOGD << "is_steer_rotate_synced_0: " << is_steer_rotate_synced_0 << ", "
              << "is_steer_rotate_synced_1: " << is_steer_rotate_synced_1;


        float rot_control_diff_1 = m_steer_wheel[0].command_rotate_angle - m_steer_wheel[0].actual_angle_in_motor;
        float rot_control_diff_2 = m_steer_wheel[1].command_rotate_angle - m_steer_wheel[1].actual_angle_in_motor;
        bool need_rotate_sync = is_steer_forward_stopped_0 && is_steer_forward_stopped_1;


        if (!is_command_move && !is_command_rotate) {
            move_cmd_type = MoveType::Stop;
            PLOGD << "move_cmd_type: " << "Stop";
        } else if (!is_command_move && is_command_rotate) {
            move_cmd_type = MoveType::Inplace_Rotate;
            PLOGD << "move_cmd_type: " << "Inplace_Rotate";

        } else if (is_command_move && is_command_rotate) {
            move_cmd_type = MoveType::Forward_And_Rotate;
            PLOGD << "move_cmd_type: " << "Forward_And_Rotate";

        } else if (is_command_move && !is_command_rotate) {
            move_cmd_type = MoveType::Forward_No_Rotate;
            PLOGD << "move_cmd_type: " << "Forward_No_Rotate";

        }

#if 0
        {
            if(std::abs(m_steer_wheel[0].actual_forward_vel) > 0.01){
                m_steer_wheel[0].is_forward_running = true;
            }
            if(std::abs(m_steer_wheel[1].actual_forward_vel) > 0.01){
                m_steer_wheel[1].is_forward_running = true;
            }


            m_steer_wheel[0].interpolate_command_forward_vel = ( m_steer_wheel[0].is_forward_running) ?  m_steer_wheel[0].command_forward_vel : ( (m_steer_wheel[0].command_forward_vel>0.0) ?  0.05f:-0.05f )  ;
            m_steer_wheel[1].interpolate_command_forward_vel = ( m_steer_wheel[1].is_forward_running) ?  m_steer_wheel[1].command_forward_vel : ( (m_steer_wheel[1].command_forward_vel>0.0) ?  0.05f:-0.05f )  ;

            m_steer_wheel[0].interpolate_command_rotate_angle = m_steer_wheel[0].command_rotate_angle;
            m_steer_wheel[1].interpolate_command_rotate_angle = m_steer_wheel[1].command_rotate_angle;


            if(move_cmd_type == MoveType::Stop){
                m_steer_wheel[0].is_forward_running = false;
                m_steer_wheel[1].is_forward_running = false;
                m_steer_wheel[0].interpolate_command_forward_vel = 0.0;
                m_steer_wheel[1].interpolate_command_forward_vel = 0.0;

            }

            return;

        }
#endif






        /*
         motion plan: smooth and sync
         target: reduce loop error
         */

        // **** state transform
        if (move_state_type == MoveType::Uninitialised) {
            move_state_type = MoveType::Stop;
            move_state_flag = 0;

        } else if (move_state_type == MoveType::Stop) {
            if(is_steer_forward_stopped_0 && is_steer_forward_stopped_1){

                move_state_type = move_cmd_type;
                move_state_flag = 0;
            }else{

                move_state_type = MoveType::Stop;
                move_state_flag = 0;
            }
        } else if (move_state_type == MoveType::Forward_No_Rotate) {
            if(move_cmd_type == MoveType::Inplace_Rotate){
                if(is_steer_forward_stopped_0 && is_steer_forward_stopped_1){

                    move_state_type = move_cmd_type;
                    move_state_flag = 0;
                }else{

                    move_state_type = MoveType::Stop;
                    move_state_flag = 0;
                }
            }else{
                move_state_type = move_cmd_type;
                move_state_flag = 0;
            }
        } else if (move_state_type == MoveType::Forward_And_Rotate) {
            if(move_cmd_type == MoveType::Inplace_Rotate){
                if(is_steer_forward_stopped_0 && is_steer_forward_stopped_1){

                    move_state_type = move_cmd_type;
                    move_state_flag = 0;
                }else{

                    move_state_type = MoveType::Stop;
                    move_state_flag = 0;
                }
            }else{
                move_state_type = move_cmd_type;
                move_state_flag = 0;
            }
        } else if (move_state_type == MoveType::Inplace_Rotate) {

            if(move_cmd_type != MoveType::Inplace_Rotate){
                if(is_steer_forward_stopped_0 && is_steer_forward_stopped_1){

                    move_state_type = move_cmd_type;
                    move_state_flag = 0;
                }else{

                    move_state_type = MoveType::Stop;
                    move_state_flag = 0;
                }
            }else{
                move_state_type = move_cmd_type;
                move_state_flag = 0;
            }
        }





        // **** state transform


        // **** state command

        if (move_state_type == MoveType::Stop) {
            PLOGD << "move_state_type: " << "Stop";
            m_steer_wheel[0].command_forward_vel = 0.0;
            m_steer_wheel[1].command_forward_vel = 0.0;


            if (smooth_stop) {
                // smooth forward_vel and rotate_angle

                bool need_update = move_state_flag == 0
                                   || ( is_steer_forward_synced_0 && is_steer_forward_synced_1);



                if(need_update){

                    float steer_constrain_ratio_1 = std::cos(m_steer_wheel[0].actual_rot_angle - constrain_angle);

                    float steer_constrain_ratio_2 = std::cos(m_steer_wheel[1].actual_rot_angle - constrain_angle);

                    // if previous state in inplace_rotate,then steer_constrain_ratio_1 = 0.0, steer_constrain_ratio_2 = 0.0
                    // if two wheel is parallel .and rotate angle is the same as  inplace_rotate mode
                    // keep two wheel forward_vel the same

                    //else in forward_rotate mode
                    // keep two wheel forward_vel same ratio

                    // v1*ratio_1 = v2*ratio_2
                    // ratio = ratio_1/ratio_2
                    // ratio = v2/v1

                    float ratio = 1;
                    if (std::abs(steer_constrain_ratio_1) < rotate_constrain_thresh
                        || std::abs(steer_constrain_ratio_2) < rotate_constrain_thresh) {

                        ratio = 1;
                        //                        ratio = 1;

//                        ratio = (std::abs(m_steer_wheel[0].actual_rot_angle - m_steer_wheel[1].actual_rot_angle) > M_PI_2f32) ? -1.0f:1.0;
                    } else {
                        ratio = steer_constrain_ratio_1 / steer_constrain_ratio_2;
                    }

                    // compute next forward_vel use forward_acc

                    // predict use acc
                    float valid_vel_1, valid_vel_2;

                    float constrain_vel_1, constrain_vel_2;

                    {
                        // predict
                        float vel_step = update_s * m_steer_wheel[0].max_forward_acc;

                        float vel_ratio_1 = vel_step / std::abs(
                                m_steer_wheel[0].command_forward_vel - m_steer_wheel[0].actual_forward_vel);
                        vel_ratio_1 = std::min(1.0f, vel_ratio_1);
                        valid_vel_1 = m_steer_wheel[0].actual_forward_vel + vel_ratio_1 *
                                                                            (m_steer_wheel[0].command_forward_vel -
                                                                             m_steer_wheel[0].actual_forward_vel);

                        float vel_ratio_2 = vel_step / std::abs(
                                m_steer_wheel[1].command_forward_vel - m_steer_wheel[1].actual_forward_vel);
                        vel_ratio_2 = std::min(1.0f, vel_ratio_2);
                        valid_vel_2 = m_steer_wheel[1].actual_forward_vel + vel_ratio_2 *
                                                                            (m_steer_wheel[1].command_forward_vel -
                                                                             m_steer_wheel[1].actual_forward_vel);
                    }
                    {
                        // constrain
                        constrain_vel_1 = std::abs(ratio) >= 1.0 ? (valid_vel_2 / ratio) : (valid_vel_1);
                        constrain_vel_2 = std::abs(ratio) >= 1.0 ? (valid_vel_2) : (ratio * valid_vel_1);
                    }

                    m_steer_wheel[0].interpolate_command_forward_vel = (m_steer_wheel[0].actual_forward_vel > 0.0) ? std::abs(constrain_vel_1) : -std::abs(constrain_vel_1) ;
                    m_steer_wheel[1].interpolate_command_forward_vel = (m_steer_wheel[1].actual_forward_vel > 0.0) ? std::abs(constrain_vel_2) : -std::abs(constrain_vel_2) ;
                    m_steer_wheel[0].interpolate_command_rotate_angle = m_steer_wheel[0].actual_angle_in_motor;
                    m_steer_wheel[1].interpolate_command_rotate_angle = m_steer_wheel[1].actual_angle_in_motor;

                }

                if( is_steer_forward_synced_0 && is_steer_forward_synced_1){
                    move_state_flag ++;
                }



            } else {
                // no smooth, direct set forward_vel and rotate_angle to 0
                m_steer_wheel[0].interpolate_command_forward_vel = 0.0;
                m_steer_wheel[0].interpolate_command_rotate_angle = m_steer_wheel[0].actual_angle_in_motor;

                m_steer_wheel[1].interpolate_command_forward_vel = 0.0;
                m_steer_wheel[1].interpolate_command_rotate_angle = m_steer_wheel[1].actual_angle_in_motor;

            }
            if (is_steer_forward_stopped_0 && is_steer_forward_stopped_1) {
                m_steer_wheel[0].interpolate_command_rotate_angle = 0.0f;
                m_steer_wheel[1].interpolate_command_rotate_angle = 0.0f;
            }

            return;
        } else if (move_state_type == MoveType::Forward_No_Rotate || move_state_type == MoveType::Forward_And_Rotate ) {
            PLOGD << "move_state_type: " << "Forward_No_Rotate/Forward_And_Rotate";

            /*
             1. control rot_angle, keep actual forward_vel
             rotate without stop, keep actual_forward_vel
             */
            if( std::abs(rot_control_diff_1) < rotate_angle_reach_thresh && std::abs(rot_control_diff_2) < rotate_angle_reach_thresh){
                need_rotate_sync = false;
            }

            float slow_ratio = 1.0;

            if(need_rotate_sync){
                // direct rotate, keep 0 vel

                bool need_update = move_state_flag == 0
                                   || ( is_steer_rotate_synced_0 &&is_steer_rotate_synced_1);


                if(need_update){
                    {
                        float rot_step = update_s * m_steer_wheel[0].max_rot_vel;

                        float rot_ratio_1 = rot_step / std::abs(
                                rot_control_diff_1);
                        rot_ratio_1 = std::min(1.0f, rot_ratio_1);
                        float valid_rot_1 = m_steer_wheel[0].actual_angle_in_motor + rot_ratio_1 *
                                                                                     (rot_control_diff_1);

                        float valid_rot_in_base_1 = valid_rot_1;

                        float rot_ratio_2 = rot_step / std::abs(
                                rot_control_diff_2);
                        rot_ratio_2 = std::min(1.0f, rot_ratio_2);
                        float valid_rot_2 = m_steer_wheel[1].actual_angle_in_motor + rot_ratio_2 *
                                                                                     (rot_control_diff_2);
                        float valid_rot_in_base_2 = valid_rot_2;

                        m_steer_wheel[0].interpolate_command_rotate_angle = valid_rot_1;
                        m_steer_wheel[1].interpolate_command_rotate_angle = valid_rot_2;

                        m_steer_wheel[0].interpolate_command_forward_vel = 0.0;
                        m_steer_wheel[1].interpolate_command_forward_vel = 0.0;

                    }

                }


                if(( is_steer_rotate_synced_0 &&is_steer_rotate_synced_1)){
                    move_state_flag++;
                }
            }else{

                bool need_update = move_state_flag == 0
                                   || ( is_steer_forward_synced_0 && is_steer_forward_synced_1 && is_steer_rotate_synced_0 &&is_steer_rotate_synced_1);

                {

                    float rot_step = update_s * m_steer_wheel[0].max_rot_vel;

                    float rot_ratio_1 = rot_step / std::abs(
                            rot_control_diff_1);
                    rot_ratio_1 = std::min(1.0f, rot_ratio_1);
                    float valid_rot_1 = m_steer_wheel[0].actual_angle_in_motor + rot_ratio_1 *
                                                                                 (rot_control_diff_1);

                    float valid_rot_in_base_1 = valid_rot_1 ;

                    float rot_ratio_2 = rot_step / std::abs(
                            rot_control_diff_2);
                    rot_ratio_2 = std::min(1.0f, rot_ratio_2);
                    float valid_rot_2 = m_steer_wheel[1].actual_angle_in_motor + rot_ratio_2 *
                                                                                 (rot_control_diff_2);
                    float valid_rot_in_base_2 = valid_rot_2;

                    //
                    float steer_constrain_ratio_1 = std::cos(valid_rot_in_base_1- constrain_angle);

                    float steer_constrain_ratio_2 = std::cos(valid_rot_in_base_2- constrain_angle);


                    float ratio = 1;
                    if (std::abs(steer_constrain_ratio_1) < rotate_constrain_thresh
                        || std::abs(steer_constrain_ratio_2) < rotate_constrain_thresh) {
                        ratio = (std::abs(valid_rot_in_base_1 - valid_rot_in_base_2) > M_PI_2f32) ? -1.0f : 1.0f;

                    } else {
                        ratio = steer_constrain_ratio_1 / steer_constrain_ratio_2;
                    }
                    float valid_vel_1, valid_vel_2;

                    float constrain_vel_1, constrain_vel_2;

                    {
                        // predict
                        float vel_step = update_s * m_steer_wheel[0].max_forward_acc;

                        float vel_ratio_1 = vel_step / std::abs(
                                m_steer_wheel[0].command_forward_vel - m_steer_wheel[0].actual_forward_vel);
                        vel_ratio_1 = std::min(1.0f, vel_ratio_1);
                        valid_vel_1 = m_steer_wheel[0].actual_forward_vel + vel_ratio_1 *
                                                                            (m_steer_wheel[0].command_forward_vel -
                                                                             m_steer_wheel[0].actual_forward_vel);

                        float vel_ratio_2 = vel_step / std::abs(
                                m_steer_wheel[1].command_forward_vel - m_steer_wheel[1].actual_forward_vel);
                        vel_ratio_2 = std::min(1.0f, vel_ratio_2);
                        valid_vel_2 = m_steer_wheel[1].actual_forward_vel + vel_ratio_2 *
                                                                            (m_steer_wheel[1].command_forward_vel -
                                                                             m_steer_wheel[1].actual_forward_vel);
                    }
                    {
                        // constrain
                        constrain_vel_1 = std::abs(ratio) <= 1.0 ? (valid_vel_2 / ratio) : (valid_vel_1);
                        constrain_vel_2 = std::abs(ratio) <= 1.0 ? (valid_vel_2) : (ratio * valid_vel_1);
                    }

                    m_steer_wheel[0].interpolate_command_forward_vel = constrain_vel_1;
                    m_steer_wheel[1].interpolate_command_forward_vel = constrain_vel_2;
                    m_steer_wheel[0].interpolate_command_rotate_angle = valid_rot_1;
                    m_steer_wheel[1].interpolate_command_rotate_angle = valid_rot_2;



                }
                if(is_steer_forward_synced_0 && is_steer_forward_synced_1&&is_steer_rotate_synced_0 &&is_steer_rotate_synced_1){
                    move_state_flag ++;
                }
            }

#if 0

            if(move_state_flag == 0){
                m_steer_wheel[0].interpolate_command_forward_vel = 0.0;
                m_steer_wheel[0].interpolate_command_rotate_angle = m_steer_wheel[0].command_rotate_angle;

                m_steer_wheel[1].interpolate_command_forward_vel = 0.0;
                m_steer_wheel[1].interpolate_command_rotate_angle = m_steer_wheel[1].command_rotate_angle;

            }else{

                float vel_step  = update_s * m_steer_wheel[0].max_forward_acc;

                float vel_ratio_1 = vel_step/std::abs(m_steer_wheel[0].command_forward_vel - m_steer_wheel[0].actual_forward_vel);
                vel_ratio_1 = std::min(1.0f,vel_ratio_1);
                float valid_vel_1 = m_steer_wheel[0].actual_forward_vel +vel_ratio_1* (m_steer_wheel[0].command_forward_vel - m_steer_wheel[0].actual_forward_vel);

                float vel_ratio_2 = vel_step/std::abs(m_steer_wheel[1].command_forward_vel - m_steer_wheel[1].actual_forward_vel);
                vel_ratio_2 = std::min(1.0f,vel_ratio_2);
                float valid_vel_2 = m_steer_wheel[1].actual_forward_vel +vel_ratio_2* (m_steer_wheel[1].command_forward_vel - m_steer_wheel[1].actual_forward_vel);

                float valid_vel = std::max( std::abs(valid_vel_1 - m_steer_wheel[0].actual_forward_vel), std::abs(valid_vel_1 - m_steer_wheel[1].actual_forward_vel)  )
                                  < std::max( std::abs(valid_vel_2 - m_steer_wheel[0].actual_forward_vel), std::abs(valid_vel_2 - m_steer_wheel[1].actual_forward_vel)  )

                                  ? valid_vel_1 : valid_vel_2;

                m_steer_wheel[0].interpolate_command_forward_vel = valid_vel;
                m_steer_wheel[1].interpolate_command_forward_vel = valid_vel;

                m_steer_wheel[0].interpolate_command_rotate_angle = m_steer_wheel[0].command_rotate_angle;
                m_steer_wheel[1].interpolate_command_rotate_angle = m_steer_wheel[1].command_rotate_angle;

            }
            if(is_steer_forward_synced_0 && is_steer_forward_synced_1){
                move_state_flag ++;
            }
#endif

        } else if (move_state_type == MoveType::Inplace_Rotate) {
            PLOGD << "move_state_type: " << "Inplace_Rotate";

            if( std::abs(rot_control_diff_1) < rotate_angle_reach_thresh && std::abs(rot_control_diff_2) < rotate_angle_reach_thresh){
                need_rotate_sync = false;
            }

            if(need_rotate_sync){
                // direct rotate, keep 0 vel

                bool need_update = move_state_flag == 0
                                   || ( is_steer_rotate_synced_0 &&is_steer_rotate_synced_1);


                if(need_update){
                    {
                        float rot_step = update_s * m_steer_wheel[0].max_rot_vel;

                        float rot_ratio_1 = rot_step / std::abs(
                                rot_control_diff_1);
                        rot_ratio_1 = std::min(1.0f, rot_ratio_1);
                        float valid_rot_1 = m_steer_wheel[0].actual_angle_in_motor + rot_ratio_1 *
                                                                                     (rot_control_diff_1);

                        float valid_rot_in_base_1 = valid_rot_1;

                        float rot_ratio_2 = rot_step / std::abs(
                                rot_control_diff_2);
                        rot_ratio_2 = std::min(1.0f, rot_ratio_2);
                        float valid_rot_2 = m_steer_wheel[1].actual_angle_in_motor + rot_ratio_2 *
                                                                                     (rot_control_diff_2);
                        float valid_rot_in_base_2 = valid_rot_2;

                        m_steer_wheel[0].interpolate_command_rotate_angle = valid_rot_1;
                        m_steer_wheel[1].interpolate_command_rotate_angle = valid_rot_2;

                        m_steer_wheel[0].interpolate_command_forward_vel = 0.0;
                        m_steer_wheel[1].interpolate_command_forward_vel = 0.0;

                    }

                }


                if(( is_steer_rotate_synced_0 &&is_steer_rotate_synced_1)){
                    move_state_flag++;
                }
            }else{
                bool need_update = move_state_flag == 0
                                   || ( is_steer_forward_synced_0 && is_steer_forward_synced_1);



               if(need_update) {

                    float steer_constrain_ratio_1 = std::cos(m_steer_wheel[0].actual_rot_angle - constrain_angle);

                    float steer_constrain_ratio_2 = std::cos(m_steer_wheel[1].actual_rot_angle - constrain_angle);

                    // if previous state in inplace_rotate,then steer_constrain_ratio_1 = 0.0, steer_constrain_ratio_2 = 0.0
                    // if two wheel is parallel .and rotate angle is the same as  inplace_rotate mode
                    // keep two wheel forward_vel the same

                    //else in forward_rotate mode
                    // keep two wheel forward_vel same ratio

                    // v1*ratio_1 = v2*ratio_2
                    // ratio = ratio_1/ratio_2
                    // ratio = v2/v1

                    float ratio = 1;
                    if (std::abs(steer_constrain_ratio_1) < rotate_constrain_thresh
                        || std::abs(steer_constrain_ratio_2) < rotate_constrain_thresh) {
                        ratio = (std::abs(m_steer_wheel[0].actual_rot_angle - m_steer_wheel[1].actual_rot_angle) > M_PI_2f32 ) ? 1.0f : -1.0f;
                    } else {
                        ratio = steer_constrain_ratio_1 / steer_constrain_ratio_2;
                    }

                    // compute next forward_vel use forward_acc

                    // predict use acc
                    float valid_vel_1, valid_vel_2;

                    float constrain_vel_1, constrain_vel_2;

                    {
                        // predict
                        float vel_step = update_s * m_steer_wheel[0].max_forward_acc;

                        float vel_ratio_1 = vel_step / std::abs(
                                m_steer_wheel[0].command_forward_vel - m_steer_wheel[0].actual_forward_vel);
                        vel_ratio_1 = std::min(1.0f, vel_ratio_1);
                        valid_vel_1 = m_steer_wheel[0].actual_forward_vel + vel_ratio_1 *
                                                                            (m_steer_wheel[0].command_forward_vel -
                                                                             m_steer_wheel[0].actual_forward_vel);

                        float vel_ratio_2 = vel_step / std::abs(
                                m_steer_wheel[1].command_forward_vel - m_steer_wheel[1].actual_forward_vel);
                        vel_ratio_2 = std::min(1.0f, vel_ratio_2);
                        valid_vel_2 = m_steer_wheel[1].actual_forward_vel + vel_ratio_2 *
                                                                            (m_steer_wheel[1].command_forward_vel -
                                                                             m_steer_wheel[1].actual_forward_vel);
                    }
                    {
                        // constrain
                        constrain_vel_1 = std::abs(ratio) >= 1.0 ? (valid_vel_2 / ratio) : (valid_vel_1);
                        constrain_vel_2 = std::abs(ratio) >= 1.0 ? (valid_vel_2) : (ratio * valid_vel_1);
                    }

                    m_steer_wheel[0].interpolate_command_forward_vel = constrain_vel_1;
                    m_steer_wheel[1].interpolate_command_forward_vel = constrain_vel_2;

                }

               if(( is_steer_forward_synced_0 && is_steer_forward_synced_1)){
                   move_state_flag++;
               }

            }



        }

        // **** state command


        return;

#if 0
        if(control_need_rot_sync){
            m_steer_wheel[0].command_forward_vel = 0.0;
            m_steer_wheel[1].command_forward_vel = 0.0;
            bool rot_angle_reach_1 = std::abs(m_steer_wheel[0].actual_angle_in_motor - m_steer_wheel[0].command_rotate_angle) < 0.01;
            bool rot_angle_reach_2 = std::abs(m_steer_wheel[1].actual_angle_in_motor - m_steer_wheel[1].command_rotate_angle) < 0.01;
            bool rot_angle_synced = rot_angle_reach_1&&rot_angle_reach_2;

            if( rot_angle_synced ){
                control_need_rot_sync = false;
            }
            return;
        }

        if(!is_command_move){

            if(is_command_rotate){
                // rotate inplace
                // if rot_angle not reach, disable forward speed


                bool rot_angle_reach_1 = std::abs(m_steer_wheel[0].actual_angle_in_motor - m_steer_wheel[0].command_rotate_angle) < 0.01;
                bool rot_angle_reach_2 = std::abs(m_steer_wheel[1].actual_angle_in_motor - m_steer_wheel[1].command_rotate_angle) < 0.01;
                PLOGD << "rotate in place , m_steer_wheel[0].actual_angle_in_motor: " << m_steer_wheel[0].actual_angle_in_motor
                << ", m_steer_wheel[0].command_rotate_angle: " <<  m_steer_wheel[0].command_rotate_angle;
                PLOGD << "rotate in place , m_steer_wheel[1].actual_angle_in_motor: " << m_steer_wheel[1].actual_angle_in_motor
                << ", m_steer_wheel[1].command_rotate_angle: " <<  m_steer_wheel[1].command_rotate_angle;
                PLOGD << "rotate in place , rot_angle_reach_1: " << rot_angle_reach_1 << ", rot_angle_reach_2: " << rot_angle_reach_2;

                bool rot_angle_synced = rot_angle_reach_1&&rot_angle_reach_2;


                bool vel_big_change = false;

                if(std::abs(m_steer_wheel[0].command_forward_vel - m_steer_wheel[0].actual_forward_vel) >  0.5&& ( std::abs(m_steer_wheel[0].actual_forward_vel) > 0.1 )  && (math::signum(m_steer_wheel[0].actual_forward_vel) != math::signum(m_steer_wheel[0].command_forward_vel))){

                    vel_big_change = true;
                }

                if(std::abs(m_steer_wheel[1].command_forward_vel - m_steer_wheel[1].actual_forward_vel) >  0.5&& ( std::abs(m_steer_wheel[1].actual_forward_vel) > 0.1 ) && (math::signum(m_steer_wheel[1].actual_forward_vel) != math::signum(m_steer_wheel[1].command_forward_vel))){

                    vel_big_change = true;
                }

                if(vel_big_change){
                    move_type = MoveType::Stop;

                    m_steer_wheel[0].command_forward_vel = 0.0;
                    m_steer_wheel[1].command_forward_vel = 0.0;
                    return;
                }





                if(!rot_angle_synced){
                    filter_time = common::FromUnixNow();
                }

                common::Time now = common::FromUnixNow();
                if(common::ToMillSeconds(now - filter_time) < 100){
                    move_type = MoveType::Stop;

                    m_steer_wheel[0].command_forward_vel = 0.0;
                    m_steer_wheel[1].command_forward_vel = 0.0;

                    return;
                }else{

                    float vel_step  = 0.1f;

                    float vel_ratio_1 = vel_step/std::abs(m_steer_wheel[0].command_forward_vel - m_steer_wheel[0].actual_forward_vel);

                    vel_ratio_1 = std::min(1.0f,vel_ratio_1);


                    float vel_ratio_2 = vel_step/std::abs(m_steer_wheel[1].command_forward_vel - m_steer_wheel[1].actual_forward_vel);

                    vel_ratio_2 = std::min(1.0f,vel_ratio_2);


                    //



                    float vel_1 = m_steer_wheel[0].actual_forward_vel +vel_ratio_1* (m_steer_wheel[0].command_forward_vel - m_steer_wheel[0].actual_forward_vel);
                    float vel_2 = m_steer_wheel[1].actual_forward_vel +vel_ratio_2* (m_steer_wheel[1].command_forward_vel - m_steer_wheel[1].actual_forward_vel);

                    float vel = std::min(std::abs(vel_1), std::abs(vel_2));


                    m_steer_wheel[0].command_forward_vel = vel_1 >0.0 ? vel :-vel;
                    m_steer_wheel[1].command_forward_vel = vel_2 >0.0 ? vel :-vel;
                }

                move_type = MoveType::Inplace_Rotate;

                return;

            }else{
                m_steer_wheel[0].command_forward_vel = 0.0f;
                m_steer_wheel[1].command_forward_vel = 0.0f;
                move_type = MoveType::Stop;

                return;
            }

        }

        if(is_command_move){

            if(is_command_rotate){
                // move and rotate

                // use current angle to replan


                bool rot_angle_reach_1 = std::abs(m_steer_wheel[0].actual_angle_in_motor - m_steer_wheel[0].command_rotate_angle) < 0.01;
                bool rot_angle_reach_2 = std::abs(m_steer_wheel[1].actual_angle_in_motor - m_steer_wheel[1].command_rotate_angle) < 0.01;
                bool rot_angle_synced = rot_angle_reach_1&&rot_angle_reach_2;

                PLOGD << "forward and rotate , m_steer_wheel[0].actual_angle_in_motor: " << m_steer_wheel[0].actual_angle_in_motor
                      << ", m_steer_wheel[0].command_rotate_angle: " <<  m_steer_wheel[0].command_rotate_angle;
                PLOGD << "forward and rotate, m_steer_wheel[1].actual_angle_in_motor: " << m_steer_wheel[1].actual_angle_in_motor
                      << ", m_steer_wheel[1].command_rotate_angle: " <<  m_steer_wheel[1].command_rotate_angle;
                PLOGD << "forward and rotate, rot_angle_reach_1: " << rot_angle_reach_1 << ", rot_angle_reach_2: " << rot_angle_reach_2;

                if(rot_angle_synced){
                    move_type = MoveType::Forward_And_Rotate;
                }

                if(move_type != MoveType::Forward_And_Rotate){

                    m_steer_wheel[0].command_forward_vel *= rot_angle_synced;
                    m_steer_wheel[1].command_forward_vel *= rot_angle_synced;
                }else{

                }

            }else{
                // move without rotate

                bool rot_angle_reach_1 = std::abs(m_steer_wheel[0].actual_angle_in_motor - m_steer_wheel[0].command_rotate_angle) < 0.01;
                bool rot_angle_reach_2 = std::abs(m_steer_wheel[1].actual_angle_in_motor - m_steer_wheel[1].command_rotate_angle) < 0.01;

                PLOGD << "forward no rotate , m_steer_wheel[0].actual_angle_in_motor: " << m_steer_wheel[0].actual_angle_in_motor
                      << ", m_steer_wheel[0].command_rotate_angle: " <<  m_steer_wheel[0].command_rotate_angle;
                PLOGD << "forward no rotate, m_steer_wheel[1].actual_angle_in_motor: " << m_steer_wheel[1].actual_angle_in_motor
                      << ", m_steer_wheel[1].command_rotate_angle: " <<  m_steer_wheel[1].command_rotate_angle;
                PLOGD << "forward no rotate, rot_angle_reach_1: " << rot_angle_reach_1 << ", rot_angle_reach_2: " << rot_angle_reach_2;


                bool rot_angle_synced = rot_angle_reach_1&&rot_angle_reach_2;

                if(rot_angle_synced){
                    move_type = MoveType::Forward_No_Rotate;

                }

                bool vel_big_change = false;

                if(std::abs(m_steer_wheel[0].command_forward_vel - m_steer_wheel[0].actual_forward_vel) >  0.5&& ( std::abs(m_steer_wheel[0].actual_forward_vel) > 0.1 )  && (math::signum(m_steer_wheel[0].actual_forward_vel) != math::signum(m_steer_wheel[0].command_forward_vel))){

                    vel_big_change = true;
                }

                if(std::abs(m_steer_wheel[1].command_forward_vel - m_steer_wheel[1].actual_forward_vel) >  0.5&& ( std::abs(m_steer_wheel[1].actual_forward_vel) > 0.1 ) && (math::signum(m_steer_wheel[1].actual_forward_vel) != math::signum(m_steer_wheel[1].command_forward_vel))){

                    vel_big_change = true;
                }

                if(vel_big_change){
                    move_type = MoveType::Stop;

                    m_steer_wheel[0].command_forward_vel = 0.0;
                    m_steer_wheel[1].command_forward_vel = 0.0;
                    return;
                }

                if(!rot_angle_synced){
                    filter_time = common::FromUnixNow();
                }

                common::Time now = common::FromUnixNow();
                if(common::ToMillSeconds(now - filter_time) < 100){
                    move_type = MoveType::Stop;

                    m_steer_wheel[0].command_forward_vel = 0.0;
                    m_steer_wheel[1].command_forward_vel = 0.0;

                    return;
                }else{
                    float vel_step  = 0.1f;

                    float vel_ratio_1 = vel_step/std::abs(m_steer_wheel[0].command_forward_vel - m_steer_wheel[0].actual_forward_vel);

                    vel_ratio_1 = std::min(1.0f,vel_ratio_1);


                    float vel_ratio_2 = vel_step/std::abs(m_steer_wheel[1].command_forward_vel - m_steer_wheel[1].actual_forward_vel);

                    vel_ratio_2 = std::min(1.0f,vel_ratio_2);


                    //



                    float vel_1 = m_steer_wheel[0].actual_forward_vel +vel_ratio_1* (m_steer_wheel[0].command_forward_vel - m_steer_wheel[0].actual_forward_vel);
                    float vel_2 = m_steer_wheel[1].actual_forward_vel +vel_ratio_2* (m_steer_wheel[1].command_forward_vel - m_steer_wheel[1].actual_forward_vel);

                    float vel = std::min(std::abs(vel_1), std::abs(vel_2));


                    m_steer_wheel[0].command_forward_vel = vel_1 >0.0 ? vel :-vel;
                    m_steer_wheel[1].command_forward_vel = vel_2 >0.0 ? vel :-vel;
                }
                move_type = MoveType::Forward_No_Rotate;


                return;
            }

        }






        return;

        /*
        sync principle
         1. if wheel forward_vel is 0, rot angle must reach target before forward_vel start increase
         2. if wheel forward_vel is > 0,  forward_vel change according to rot_angle difference, the difference is bigger, the forward_vel is smaller
         3. constrain must be satisfied
         */



        // sync forward and rotation angle
        float rot_1 =   rot_control_diff_1 ;
        float rot_2 =   rot_control_diff_2 ;



        if(
                std::abs(rot_1 ) > 0.2
                ||std::abs( rot_2 ) > 0.2
                ){
           m_steer_wheel[0].command_forward_vel = 0.0;
            m_steer_wheel[1].command_forward_vel = 0.0;

        }




        float interval_seconds = 1e-6f*static_cast<float>(common::ToMicroSeconds(now - time)) ;
        time = now;


        float valid_rot_angle_1 = 0.0;
        float valid_rot_angle_2 = 0.0;


        float predict_rot_time_1 = 0.0f;
        float predict_rot_time_2 = 0.0f;



        {
            predict_rot_time_1 = std::abs(rot_1)/m_steer_wheel[0].max_rot_vel;

            if(predict_rot_time_1 > predict_time){
                m_steer_wheel[0].command_rotate_angle =  m_steer_wheel[0].actual_angle_in_motor + (predict_time/predict_rot_time_1)*rot_1;
            }else{
                m_steer_wheel[0].command_rotate_angle =  m_steer_wheel[0].actual_angle_in_motor + rot_1;

            }

        }

        {
            predict_rot_time_2 = std::abs(rot_2)/m_steer_wheel[1].max_rot_vel;

            if(predict_rot_time_2 > predict_time){
                m_steer_wheel[1].command_rotate_angle =  m_steer_wheel[1].actual_angle_in_motor + (predict_time/predict_rot_time_2)*rot_2;
            }else{
                m_steer_wheel[1].command_rotate_angle =  m_steer_wheel[1].actual_angle_in_motor + rot_2;

            }
        }
        //

#endif


    }


    void SmoothSimulator::set_wheel(const control::SteerWheelBase &wheel_1, const control::SteerWheelBase &wheel_2) {
        m_steer_wheel[0] = wheel_1;
        m_steer_wheel[1] = wheel_2;

        m_steer_wheel[0].time = common::FromUnixNow();
        m_steer_wheel[1].time = common::FromUnixNow();

        constrain_angle = std::atan2(m_steer_wheel[1].mount_position_y - m_steer_wheel[0].mount_position_y,
                                     m_steer_wheel[1].mount_position_x - m_steer_wheel[0].mount_position_x);

    }

    void SmoothSimulator::updateState(float forward_vel_1, float rot_angle_1, float forward_vel_2, float rot_angle_2) {

        common::Time now = common::FromUnixNow();

        {
            m_steer_wheel[0].target_forward_vel = forward_vel_1;
            m_steer_wheel[0].target_rot_angle =  m_steer_wheel[0].enable_rot * rot_angle_1;


            float update_s = common::ToMicroSeconds(now - m_steer_wheel[0].time) * 1e-6;
            float vel_step = update_s * m_steer_wheel[0].max_forward_acc;
            m_steer_wheel[0].time = now;


            float vel_ratio_1 =
                    vel_step / std::abs(m_steer_wheel[0].target_forward_vel - m_steer_wheel[0].actual_forward_vel);

            vel_ratio_1 = std::min(1.0f, vel_ratio_1);

            float vel_1 = m_steer_wheel[0].actual_forward_vel +
                          vel_ratio_1 * (m_steer_wheel[0].target_forward_vel - m_steer_wheel[0].actual_forward_vel);


            m_steer_wheel[0].actual_forward_vel = vel_1;//  +common::uniform_real<float>(-0.005f,0.05f);

            float rot_step = 0.01;

            float rot_ratio_1 =
                    rot_step / std::abs(m_steer_wheel[0].target_rot_angle - m_steer_wheel[0].actual_rot_angle);
            rot_ratio_1 = std::min(1.0f, rot_ratio_1);
            float rot_1 = m_steer_wheel[0].actual_rot_angle +
                          rot_ratio_1 * (m_steer_wheel[0].target_rot_angle - m_steer_wheel[0].actual_rot_angle);

            m_steer_wheel[0].actual_rot_angle = rot_1;


        }
        {
            m_steer_wheel[1].target_forward_vel = forward_vel_2;
            m_steer_wheel[1].target_rot_angle = m_steer_wheel[1].enable_rot * rot_angle_2;

            float update_s = common::ToMicroSeconds(now - m_steer_wheel[1].time) * 1e-6;
            float vel_step = update_s * m_steer_wheel[1].max_forward_acc;
            m_steer_wheel[1].time = now;

            float vel_ratio_1 =
                    vel_step / std::abs(m_steer_wheel[1].target_forward_vel - m_steer_wheel[1].actual_forward_vel);

            vel_ratio_1 = std::min(1.0f, vel_ratio_1);

            float vel_1 = m_steer_wheel[1].actual_forward_vel +
                          vel_ratio_1 * (m_steer_wheel[1].target_forward_vel - m_steer_wheel[1].actual_forward_vel);


            m_steer_wheel[1].actual_forward_vel = vel_1;// + common::uniform_real<float>(-0.005f,0.05f);

            float rot_step = 0.01;

            float rot_ratio_1 =
                    rot_step / std::abs(m_steer_wheel[1].target_rot_angle - m_steer_wheel[1].actual_rot_angle);
            rot_ratio_1 = std::min(1.0f, rot_ratio_1);
            float rot_1 = m_steer_wheel[1].actual_rot_angle +
                          rot_ratio_1 * (m_steer_wheel[1].target_rot_angle - m_steer_wheel[1].actual_rot_angle);

            m_steer_wheel[1].actual_rot_angle = rot_1;
        }

        m_steer_wheel[0].steer_constrain_vel =
                m_steer_wheel[0].actual_forward_vel * std::cos(m_steer_wheel[0].actual_rot_angle - constrain_angle);

        m_steer_wheel[1].steer_constrain_vel =
                m_steer_wheel[1].actual_forward_vel * std::cos(m_steer_wheel[1].actual_rot_angle - constrain_angle);

        // compute odom

//        position.set(position.x() + 0.001f, position.y()+0.001f,position.yaw() + 0.001f);

    }
}