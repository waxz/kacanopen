//
// Created by waxz on 6/24/23.
//

#ifndef KACANOPEN_MOTIONPLANNER_H
#define KACANOPEN_MOTIONPLANNER_H

#include "message/common_message.h"
#include "transform/transform.h"
#include "MobileRobotController.h"
#include "common/stamped_buffer.h"

namespace control{


    // base type: double_steering, single_steering, single_differential
    // path : [p1,p2,p3,... pt], p : [x,y,yaw]
    // goal: [x,y,yaw]
    // target tolerance
    // path tolerance
    // speed constrain
    // footprint constrain: used in single_steering base's pallet docking task

    // input request_path, odom pose, base_link pose
    // output robot pose
    // output control command: raw steering command:[forward_vel, rotate_angle], cmd_vel:[]


    struct PlannerConfig{
        // base actual pose to first pose in path
        float start_pose_dist = 1.0;

        float first_rotate_acc = 0.1;
        // adjust base direction at beginning
        float first_rotate_angle_tolerance = 0.05;
        //pid, p
        float first_rotate_angle_p = 0.6;

        // first rotate vel
        float first_rotate_vel = 0.5;

        // pursuit path
        float pursuit_path_interpolate_step = 0.2;

        float pursuit_path_angle_converge = 0.1;

        float pursuit_path_angle_pid_p = 0.6;

        float pursuit_path_forward_vel = 0.6;



        // when to start pursuit_goal
        float pursuit_goal_dist = 1.0f;


        float pursuit_goal_angle_pid_p = 0.2;
        float pursuit_goal_forward_vel = 0.1;

        float pursuit_direct_goal_dist = 0.5f;

        float pursuit_final_goal_dist = 0.2f;

        float pursuit_goal_final_angle_pid_p = 0.2;
        float pursuit_goal_final_forward_vel = 0.1;


        float pursuit_goal_reach_tolerance = 0.05;



        float pursuit_path_dist_tolerance = 0.1;
        float pursuit_path_angle_tolerance = 0.1;
        float target_tolerance_along = 0.03;
        float target_tolerance_vertical = 0.03;
    };

    // foresee insight prediction
    struct MotionPlanner{

        PlannerConfig m_planner_config;
        //input odom and tf, get robot_pose


        // input request_path, get global_path

        // use robot_pose and global_path, get local_path

        // task manager
        // initializer : steering wheel angle, initial robot pose
        // start pause stop finish

        // each task has different tolerance, footprint constrain
        enum class TaskType{
            free_stop = 0,
            pickup_stop = 1,
            pallet_docking = 2
        };

        // at beginning of each task
        // robot should do initialising prepare, adjust robot pose or steering wheel angle

        // if new task comes but an old task is running
        // stop old task, stop robot
        // and reinitialise new task

        // if pause command comes but an old task is running
        // pause old task, stop robot

        enum class TaskCommand{

            start = 0,
            pause = 1,
            resume = 2,
            stop = 3
        };


        enum class TaskState{

            error_path = -1,
            idle = 0,
            running_init = 1,
            running_adjust_rotate = 2,
            running_adjust_prepare = 3,
            running_pursuit_path = 4,
            running_pursuit_goal = 5,
            suspended = 5,
            finished = 6
        };


        enum class PlannerState{
            uninitialised = 0,
            global_plan_ok = 1,
            local_plan_ok = 2 ,
            target_reached = 3
        };

        enum class CommandType{
            uninitialised = 0,
            raw = 1,
            cmd_vel = 2
        };

        struct Command{
            CommandType command_type = CommandType::uninitialised;
            // cmd_vel: [vx,vy,rz]
            // raw: [forward_vel_1, rotate_angle_1, forward_vel_2, rotate_angle_2]
            std::vector<float> command;
        };

        Command m_command;
        // base interface

        // task manager

        // stable localization
        // map base_link
        /*
         sync map_base and odom_base
         record map_base_sync and odom_base_sync at time_sync like task begin
         odom_base_change = odom_base_sync.inverse() * odom_base_latest
         stable_pose = map_base_sync * odom_base_change
         compare difference between map_base and stable_pose,

         */
        common::ValueStampedBuffer<transform::Transform2d> m_map_base_buffer;
        common::ValueStampedBuffer<transform::Transform2d> m_odom_base_buffer;
        common::ValueStamped<common_message::Odometry> m_actual_odom;
        common::ValueStamped<transform::Transform2d> m_actual_pose;
        transform::Transform2d m_map_odom_record;
        bool m_stable_pose_get = false;

        void updateMapBasePose(const common_message::TransformStamped& pose);
        void updateOdomBasePose(const common_message::TransformStamped& pose);
        bool getStablePose();

        // /odom
        void updateOdom(const common_message::Odometry& odom);

        void reset();

        void stop();


        // task state
        TaskState m_task_state = TaskState::idle;
        const TaskState& getTaskState();

        // request_goal, request_path
        common::ValueStamped<std::vector<transform::Transform2d>> m_global_path;
        common::ValueStamped<std::vector<transform::Transform2d>> m_local_path;
        void requestGoal(const common_message::PoseStamped& );
        void requestPath(const common_message::Path&);



        common::Time interpolate_time;
        common::Time interpolate_time_step_0;
        common::Time interpolate_time_step_1;

        virtual bool checkPath() = 0;
        // rotate
        float m_rotate_init_angle_diff = 0.0;

        bool rotate(float actual, float target);

        virtual bool prepare() = 0;

        virtual bool createLocalPath() = 0;
        virtual bool pursuit_path() = 0;
        virtual bool pursuit_goal() = 0;


        virtual void resetPlanner() = 0;
        // command type


        // go
        const Command& go();




        // plan path result
        void getGlobalPath();
        void getLocalPath();


        // plan command result
        // cmd_vel: forward_vel, rotate_vel
        void getCmdVel();

        // virtual interface

        // initialise robot pose at beginning of task
        // 1. stop base
        // 2. adjust steer wheel angle
        bool m_base_config_done = false;
        // rotate around base center
        float m_max_base_rotate_vel = 0.0;
        float m_min_base_rotate_vel = 0.0;
        float m_max_base_rotate_acc = 0.0;
        // forward alongside with forward_angle
        float m_min_base_forward_vel = 0.0;
        float m_max_base_forward_vel = 0.0;
        float m_max_base_forward_acc = 0.0;
        // forward_angle change,
        float m_max_base_forward_angle_vel = 0.0;
        float m_max_base_forward_angle_acc = 0.0;

        float m_max_wheel_rotate_angle = 0.0;



    };


    struct DoubleSteerMotionPlanner: public MotionPlanner{


        void initBase(const std::vector<SteerWheelBase>& config);

        void updateWheelState(float forward_vel_1, float rotate_angle_1, float forward_vel_2, float rotate_angle_2);
        float m_start_wheel_angle = 0.0f;
        common::ValueStamped<std::array<float,4>> m_wheel_state;
        // check
        bool checkPath() override;
        bool prepare() override;

        void resetPlanner() override;
        size_t m_path_node_id = 0;
        bool createLocalPath() override;
        bool pursuit_path() override;
        bool pursuit_goal() override;

    };

}

#endif //KACANOPEN_MOTIONPLANNER_H
