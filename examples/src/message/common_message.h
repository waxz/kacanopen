//
// Created by waxz on 5/6/23.
//

#ifndef CMAKE_SUPER_BUILD_COMMON_MESSAGE_H
#define CMAKE_SUPER_BUILD_COMMON_MESSAGE_H
#include "transform/eigen_transform.h"
#include "common/clock_time.h"

namespace common_message{

    struct Int16Array{
        std::vector<int16_t> data;
    };
    struct Header{
        u_int32_t seq = 0;
        common::Time  stamp;
        std::string frame_id;
    };
    struct Vector3{
        float x = 0.0;
        float y = 0.0;
        float z = 0.0;
    };

    struct Twist{
        Vector3 linear;
        Vector3 angular;
    };

    struct TwistWithCovariance{
        Twist twist;
        float covariance[36] ={0.0};
    };

    struct Point{
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct Quaternion{
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double w = 1.0;
    };

    struct Pose{
        Point position;
        Quaternion orientation;
    };

    struct PoseStamped{
        Header header;
        Pose pose;
    };

    struct Path{
        Header header;
        std::vector<PoseStamped> poses;

    };
    struct PoseWithCovariance{
        Pose pose;
        float covariance[36] = {0.0};
    };

    struct Odometry{
        Header header;
        std::string child_frame_id;
        PoseWithCovariance pose;
        TwistWithCovariance twist;
    };

    struct TransformStamped{
        common::Time time;
        std::string base_frame;
        std::string target_frame;
        Eigen::Transform<double,3,Eigen::Isometry> transform;
    };

    struct CanMessage{
        u_int32_t id;
        bool is_rtr;
        bool is_extended;
        bool is_error;
        u_int8_t dlc;
        u_int8_t data[8];
    };
    struct CanMessageArray{
       std::vector<CanMessage> messages;
    };
}

#endif //CMAKE_SUPER_BUILD_COMMON_MESSAGE_H
