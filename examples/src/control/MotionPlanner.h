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

        // check curve path,
        float curve_path_angle = 0.2;
        size_t curve_path_window = 5;
        float curve_path_speed_ratio = 0.5;

        // localization smooth
        float stable_pose_dist_tolerance = 0.03;
        float stable_pose_angle_tolerance = 0.05;

        // base actual pose to first pose in path
        float start_pose_dist = 1.0;

        float first_track_dist = 0.1;
        float max_track_dist = 0.5;
        float first_rotate_vel_min = 0.02;
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

        float pursuit_path_rotate_vel_angle_p = 0.6;
        float pursuit_path_rotate_vel_vel_p = 0.01;

        float pursuit_path_angle_rot_vel_max = 0.01;

        float pursuit_path_forward_vel = 0.6;

//        float follow_line_vel = 0.6;

        // virtual path width, single side
        float pursuit_path_width = 0.1f;

        float pursuit_path_width_direction_adjust = 0.05f;

        // off path distance, single side
        float off_path_dist = 0.2f;


        // steer rotate vel
        float steer_rotate_vel = 0.5;
        // follow path
        float follow_dist_min = 0.2f;
        float follow_dist_max = 0.5f;


        float curve_interpolate_dist = 1.0f;

        float speed_up_acc = 0.3;
        float speed_down_acc = 0.3;

        float pursuit_path_forward_acc = 0.3;


        // when to start pursuit_goal
        float pursuit_goal_dist = 1.0f;


        float pursuit_goal_angle_pid_p = 0.2;
        float pursuit_goal_forward_vel = 0.1;

        float pursuit_goal_forward_vel_pid_p = 0.1;
        float pursuit_goal_forward_vel_min = 0.1;
        float pursuit_goal_forward_vel_max = 0.1;


        float pursuit_direct_goal_dist = 0.5f;

        float pursuit_final_goal_dist = 0.2f;

        float pursuit_goal_final_angle_pid_p = 0.2;
        float pursuit_goal_final_forward_vel = 0.1;

        float pursuit_goal_final_forward_vel_pid_p = 0.1;

        float pursuit_goal_angle_rot_vel_max = 0.001;
        float pursuit_goal_final_forward_vel_min = 0.01;
        float pursuit_goal_final_forward_vel_max = 0.05;

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
            off_path = -2,
            error_path = -1,
            idle = 0,
            running_init = 1,

            // only rotate
            running_rotate_init = 2,
            running_rotate = 3,


            // rotate in pursuit path
            running_adjust_rotate = 4,
            running_adjust_prepare = 5,
            running_pursuit_path = 6,
            running_pursuit_goal = 7,
            suspended = 8,
            finished = 9
        };


        enum class PlannerState{
            // task start
            idle = 0,
            // base go to the closest node on path from start. if tolerance is reached, state change to follow_path
            init_to_path = 1,

            // dynamic switch to follow_line or follow curve
            follow_path = 2,

            //curve
            follow_curve =3,

            return_to_curve = 4,

            // follow line path with width
            follow_line = 5 ,
            // follow line path, error beyond tolerance, must return to path
            return_to_line = 6,

            follow_turn = 7,

            follow_goal = 8
        };

        PlannerState m_planner_state = PlannerState::idle;

        enum class CommandType{
            uninitialised = 0,
            steer = 1,
            cmd_vel = 2
        };

        struct Command{
            CommandType command_type = CommandType::uninitialised;
            // cmd_vel: [vx,vy,rz]
            // raw: [forward_vel_1, rotate_angle_1, forward_vel_2, rotate_angle_2]
            std::vector<float> command;
            void setSteer(float rotate_angle){

                command_type = CommandType::steer;
                command.resize(1);
                command[0] = rotate_angle;

            }
            void setCmd(float vx, float vy, float rz){

                command_type = CommandType::cmd_vel;
                command.resize(3);
                std::fill(command.begin(), command.end(),0.0f);
                command[0] = vx;
                command[1] = vy;
                command[2] = rz;

            }

        };

        Command m_command;
        const Command& getCommand()const{
            return m_command;
        }
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
        common::ValueStamped<transform::Transform2d> m_local_target;

        common::ValueStamped<transform::Transform2d> m_local_target_last;

        transform::Transform2d m_map_odom_record;
        bool m_stable_pose_get = false;

        void updateMapBasePose(const common_message::TransformStamped& pose);
        void updateOdomBasePose(const common_message::TransformStamped& pose);
        bool getStablePose();

        // /odom
        void updateOdom(const common_message::Odometry& odom);

        //
        size_t m_path_node_id = 0;
        float m_path_node_id_interpolate = 0.0;
        size_t m_closest_path_node_id = 0;


        void reset();

        void stop();


        // task state
        TaskState m_task_state = TaskState::idle;
        const TaskState& getTaskState();

        // request_goal, request_path
        common::ValueStamped<std::vector<transform::Transform2d>> m_global_path;
        common::ValueStamped<std::vector<transform::Transform2d>> m_local_path;
        std::string m_task_frame;

        // create global path from request_goal
        // makesure global_path contains at least two nodes, two nodes represent path direction
        // check target tolerance, skip task
        void requestGoal(const common_message::PoseStamped& );

        // create global path from request_path
        // makesure global_path contains at least two nodes, two nodes represent path direction
        // check target tolerance, skip task
        void requestPath(const common_message::Path&);

        const std::string& getTaskFrame();


        common::Time interpolate_time;
        common::Time interpolate_time_step_0;
        common::Time interpolate_time_step_1;

        // base velocity
        float m_forward_vel = 0.0f;
        float m_forward_diff = 0.0f;

        float m_forward_angle = 0.0f;
        float m_rotate_vel = 0.0f;
        float m_rotate_diff = 0.0f;
        bool off_width_once = false;


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
        void go();

        // track path info
        struct TrackPointInfo{

            // node pose in path direction
            // the x-axis indicates path direction
            // the y-axis can be used to compute segmentation and robot control error
            transform::Transform2d pose;

            // segment id, indicate segment type change
            // line segment
            // left turn curve segment
            // right turn curve segment
            size_t segment_id = 0;

            size_t segment_end_id = 0;

            // segment type
            // 0: line
            // 1: left turn curve
            // -1: right turn curve
            int segment_type = 0;

            float direction = 0.0;

            // direction from last node
            // 0.0 : line segment
            // otherwise: curve segment
            // positive : turn left
            // negative: turn right
            float direction_change_from_last = 0.0;

            float direction_change_divide_dist = 0.0;

            // planned forward vel base on direction change speed
            float forward_vel = 0.0;
            // planned wheel's steer vel base on direction change speed
            float steer_vel = 0.0;
            // planned base rotate vel base on direction change speed
            float rotate_vel = 0.0;


            // relative dist compare to last node
            float dist_from_last = 0.0;

            // distance accumulation from the first node
            float dist_from_start = 0.0;

            // distance to end node
            float dist_to_end = 0.0;

            // distance_to_segment_end
            float dist_to_segment_end = 0.0f;


            // interpolate node
            bool interpolate_valid = false;
            transform::Transform2d interpolate_node_in;
            transform::Transform2d interpolate_node_out;
            size_t interpolate_node_in_id = 0 ;
            size_t interpolate_node_out_id = 0 ;

        };
        std::vector<TrackPointInfo> m_track_path_info;


        std::string m_status_msg;
        const std::string& getStatusMsg()const{
            return m_status_msg;
        }

        // plan path result
        void getGlobalPath();
        void getLocalPath();

        // compute forward_vel, forward_angle and rotate_vel to target node
        virtual bool goToNode(float node_id_float) = 0;
        virtual bool goToNode(size_t node_id) = 0;
        virtual bool gotoNode(const transform::Transform2d& node) = 0;

        virtual bool followLocalPath() = 0;


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

        DoubleSteerController m_driver_controller;

        void initBase(const std::vector<SteerWheelBase>& config);

        void updateWheelState(float forward_vel_1, float rotate_angle_1, float forward_vel_2, float rotate_angle_2);

        // start wheel angle
        float m_start_wheel_angle = 0.0f;

        // steer wheel angle limit
        float m_allow_angle_min[2] = {0.0f,0.0f};
        float m_allow_angle_max[2] = {0.0f,0.0f};

        //float forward_vel_1, float rotate_angle_1, float forward_vel_2, float rotate_angle_2
        common::ValueStamped<std::array<float,4>> m_wheel_state;
        // check path validation
        // 1. distance to goal
        // 2. rotate in path
        // 3. closest distance to path
        // 4. steer limit
        bool checkPath() override;
        bool prepare() override;

        void resetPlanner() override;

        bool createLocalPath() override;
        bool pursuit_path() override;
        bool pursuit_goal() override;

        bool use_prefer_steer_angle();
        float m_prefer_steer_angle = 0.0f;
        float get_prefer_steer_angle();

        bool goToNode(float node_id_float) override;
        bool goToNode(size_t node_id) override;
        bool gotoNode(const transform::Transform2d &node) override;

        bool followLocalPath() override;

        bool createLocalPath_v1();
//        bool createLocalPath_v2();



    };

}

#endif //KACANOPEN_MOTIONPLANNER_H
