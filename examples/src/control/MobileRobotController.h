//
// Created by waxz on 5/12/23.
//

#ifndef CMAKE_SUPER_BUILD_MOBILEROBOTCONTROLLER_H
#define CMAKE_SUPER_BUILD_MOBILEROBOTCONTROLLER_H
#include <chrono>
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

        void config();


//        transform::Transform2d position;

        // rot range
        float max_rot_angle = 1.1 * M_PI_2;

        // acc/vel limit
        float max_forward_acc = 0.8;  // vel change limit , unit m/s2
        float max_forward_vel = 1.2;  // vel limit, unit m/s
        float min_forward_vel = 0.05;
        float max_rot_acc = 2*M_PIf32;      // vel change limit, unit rag/s2
        float max_rot_vel = 0.5*M_PIf32;      // vel limit , unit rag/s // 3000rpm ==> 1 rotate per second



        // actual
        size_t flag = 0;
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
        }

        ///actual_rot_angle = mount_position_yaw + enable_rot*rot_angle;
        /// \param forward_vel  : sensor feedback forward velocity
        /// \param rot_angle  : sensor feedback angle
        /// \return
        bool updateState(float forward_vel, float rot_angle) ;




        // transform command to motor frame
        void createCommand();
        void setPrefer(bool prefer);
        void setPreferAngle(float prefer_angle);

        bool use_prefer = false;
        float m_prefer_angle = 0.0f;
    };




    class MobileRobotControllerBase {


    public:
        typedef std::chrono::high_resolution_clock Clock;
        std::chrono::system_clock::time_point new_time;

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

        float control_period_sec = 0.02;

        bool control_need_forward_zero = false;
        bool control_need_forward_sync = false;
        bool control_need_rot_zero = false;
        bool control_need_rot_sync = false;


        size_t odom_flag = 0;
        // fault
        StateCode state_code = StateCode::Uninitialised;

        MoveType move_cmd_type = MoveType::Uninitialised;
        MoveType move_state_type = MoveType::Uninitialised;
        size_t move_state_flag = 0;


        common::Time filter_time;

        // robot position in odom frame, start from [0.0,0.0,0.0]
        transform::Transform2d position;
        // veclocity
        float actual_forward_vel = 0.0;
        float actual_forward_angle = 0.0;
        float actual_rotate_vel = 0.0;


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
        float forward_vel_reach_thresh = 0.001;
        float rotate_angle_reach_thresh = 0.01;

        bool wheel_angle_reach = false;

        //
        float rotate_constrain_thresh = 0.01;

    public:
        float predict_time = 0.01;

        bool smooth_stop = true;
        bool smooth_stop_temp = false;

        void setSmoothStop();


        virtual const transform::Transform2d& getPosition() = 0;
        virtual float getActualForwardVel() = 0;
        virtual float getActualForwardAngle() = 0;
        virtual float getActualRotateVel()=0;
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
        float m_prefer_steer_angle = 0.0;
        void setSteerPreference(float angle);

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
        float m_prefer_steer_angle = 0.0f;
        bool m_use_prefer_angle = false;

    public:

        void setPrefer(bool prefer);
        void setSteerPreference(float angle);
        void reset() override;

        void cmd_vel(float forward_vel, float rot_vel) override;

        void cmd_vel(float forward_vel, float rot_vel, float forward_angle) override;

        void feed_back(const std::vector<float> &wheel_states) override;

        // set SteerWheel

        void set_wheel(const SteerWheelBase &wheel_1, const SteerWheelBase &wheel_2);

        common::Time interpolate_time;
        common::Time interpolate_time_step_0;
        common::Time interpolate_time_step_1;

        size_t interpolate_cnt = 0;
        void interpolate();

        void updateState(float forward_vel_1, float rot_angle_1, float forward_vel_2, float rot_angle_2);

        float getActualForwardVel() override;
        float getActualForwardAngle() override;
        float getActualRotateVel() override;
        const transform::Transform2d & getPosition() override;



    };

}


#endif //CMAKE_SUPER_BUILD_MOBILEROBOTCONTROLLER_H
