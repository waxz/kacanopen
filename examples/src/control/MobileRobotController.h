//
// Created by waxz on 5/12/23.
//

#ifndef CMAKE_SUPER_BUILD_MOBILEROBOTCONTROLLER_H
#define CMAKE_SUPER_BUILD_MOBILEROBOTCONTROLLER_H

#include "transform/transform.h"
#include "common/clock_time.h"
#include "math/math_basic.h"
namespace control {


    /*
     MobileRobotControllerBase:
     define: steering wheel ,
     */


    struct SteerWheelBase {

        // time
        common::Time time;

        bool is_forward_running = false;

        // steering wheel or fix wheel
        bool enable_rot = false;

        // mount position, and rot angle
        float mount_position_x = 0.0;
        float mount_position_y = 0.0;
        float mount_position_yaw = 0.0;

//        transform::Transform2d position;

        // rot range
        float min_rot_angle = -1.1 * M_PI_2;
        float max_rot_angle = 1.1 * M_PI_2;

        // acc/vel limit
        float max_forward_acc = 0.8;  // vel change limit , unit m/s2
        float max_forward_vel = 1.2;  // vel limit, unit m/s
        float max_rot_acc = 2*M_PIf32;      // vel change limit, unit rag/s2
        float max_rot_vel = 0.5*M_PIf32;      // vel limit , unit rag/s // 3000rpm ==> 1 rotate per second



        // actual
        float actual_rot_angle = 0.0;
        float actual_forward_acc = 0.0;
        float actual_rot_acc = 0.0;
        float actual_forward_vel = 0.0;
        float actual_rot_vel = 0.0;

        //
        float target_angle_in_motor = 0.0;
        float actual_angle_in_motor = 0.0;

        float steer_constrain_vel = 0.0;
        // target
        float target_rot_angle = 0.0;
        float target_forward_vel = 0.0;

        // output command
        float command_forward_vel = 0.0;
        float command_rotate_angle = 0.0;

        float interpolate_command_forward_vel = 0.0;
        float interpolate_command_rotate_angle = 0.0;


        float& getCommandForwardVel(){
            return interpolate_command_forward_vel;
        }
        float& getCommandRotateAngle(){
            return interpolate_command_rotate_angle;

        }

        void reset() {
            createCommand(0.0, 0.0);
        }

        ///actual_rot_angle = mount_position_yaw + enable_rot*rot_angle;
        /// \param forward_vel  : sensor feedback forward velocity
        /// \param rot_angle  : sensor feedback angle
        /// \return
        bool updateState(float forward_vel, float rot_angle) {
            float forward_vel_change = std::abs(forward_vel - actual_forward_vel);
            float rotate_angle_change = std::abs(actual_rot_angle - mount_position_yaw + enable_rot * rot_angle);


            actual_forward_vel = forward_vel;
            actual_rot_angle = mount_position_yaw + enable_rot * rot_angle;
            return enable_rot;
        }

        ///
        /// \param forward_vel
        /// \param rot_angle
        void createCommand(float forward_vel, float rot_angle) {
            command_forward_vel = forward_vel;
            command_rotate_angle = rot_angle - mount_position_yaw;
            command_rotate_angle = std::min(std::max(command_rotate_angle, min_rot_angle), max_rot_angle);
            command_forward_vel = std::min(std::max(command_forward_vel, -max_forward_vel), max_forward_vel);
        }



        // transform command to motor frame
        void createCommand();
    };




    class MobileRobotControllerBase {


    public:
        enum class StateCode{
            Uninitialised = 0,
            OK = 1,
            FeedBackError
        };

        enum class MoveType{
            Uninitialised = 0,
            Stop = 1,
            Forward_No_Rotate,
            Forward_And_Rotate,
            Inplace_Rotate
        };


    private:

    protected:
        // time stamp
        common::Time time;
        float control_period_sec = 0.01;

        bool control_need_forward_zero = false;
        bool control_need_forward_sync = false;
        bool control_need_rot_zero = false;
        bool control_need_rot_sync = false;


        // fault
        StateCode state_code = StateCode::Uninitialised;

        MoveType move_cmd_type = MoveType::Uninitialised;
        MoveType move_state_type = MoveType::Uninitialised;
        size_t move_state_flag = 0;


        common::Time filter_time;

        // robot position in odom frame, start from [0.0,0.0,0.0]
        transform::Transform2d position;

        // rotate center
        // relative to base_link
        bool is_rotate = false;
        float rotate_center_x = 0.0;
        float rotate_center_y = 0.0;

        // velocity
        float m_actual_forward_vel = 0.0;
        float m_actual_rot_vel = 0.0;
        float m_actual_forward_angle = 0.0;

        float m_cmd_forward_vel = 0.0;
        float m_cmd_rot_vel = 0.0;
        float m_cmd_forward_angle = 0.0;

        //
        float forward_vel_reach_thresh = 0.01;
        float rotate_angle_reach_thresh = 0.01;

        //
        float rotate_constrain_thresh = 0.01;

    public:
        float predict_time = 0.01;

        bool smooth_stop = true;

        const transform::Transform2d& getPosition(){
            return position;
        }
        virtual void reset() = 0;

        ///
        /// \param forward_vel: robot linear velocity, m/s
        /// \param rot_vel : robot rotate velocity, rad/s
        virtual void cmd_vel(float forward_vel, float rot_vel) = 0;

        ///
        /// \param forward_vel: robot linear velocity, m/s
        /// \param rot_vel : robot rotate velocity, rad/s
        /// \param forward_angle : linear velocity direction relative to base_link's X axis
        virtual void cmd_vel(float forward_vel, float rot_vel, float forward_angle) = 0;


        ///
        /// \param wheel_states : [forward_vel,rot_angle],[forward_vel,rot_angle],[forward_vel,rot_angle]
        virtual void feed_back(const std::vector<float> &wheel_states) = 0;

    };

    class SmoothSimulator{
    public:

        std::array<SteerWheelBase,2> m_steer_wheel;
        float constrain_angle = 0.0;
        void set_wheel(const SteerWheelBase &wheel_1, const SteerWheelBase &wheel_2);
        void updateState(float forward_vel_1, float rot_angle_1, float forward_vel_2, float rot_angle_2);


    };

    class DoubleSteerController : public MobileRobotControllerBase {

    public:
//        SteerWheelBase m_steer_wheel_1;
//        SteerWheelBase m_steer_wheel_2;

        std::array<SteerWheelBase,2> m_steer_wheel;
        float constrain_angle = 0.0;
    public:
        void reset() override;

        void cmd_vel(float forward_vel, float rot_vel) override;

        void cmd_vel(float forward_vel, float rot_vel, float forward_angle) override;

        void feed_back(const std::vector<float> &wheel_states) override;

        // set SteerWheel

        void set_wheel(const SteerWheelBase &wheel_1, const SteerWheelBase &wheel_2);

        void interpolate();

        void updateState(float forward_vel_1, float rot_angle_1, float forward_vel_2, float rot_angle_2);
    };

}


#endif //CMAKE_SUPER_BUILD_MOBILEROBOTCONTROLLER_H
