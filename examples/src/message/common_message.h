//
// Created by waxz on 5/6/23.
//

#ifndef CMAKE_SUPER_BUILD_COMMON_MESSAGE_H
#define CMAKE_SUPER_BUILD_COMMON_MESSAGE_H
#include "transform/eigen_transform.h"
#include "transform/transform.h"
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

    struct HeaderString{
        Header header;
        std::string data;
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



    inline Eigen::Transform<double,3,Eigen::Isometry> PoseToEigen(const  Pose &pose){
        return transform::createSe3<double>(pose.position.x,pose.position.y,pose.position.z, pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z );
    }
    inline transform::Transform2d PoseToTransform2d(const Pose &pose){
        double roll, pitch, yaw;

        transform::toEulerAngle<double >(yaw,  pitch ,roll, pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

        return transform::Transform2d(pose.position.x, pose.position.y, yaw);
    }
    inline Pose Transform2dToPose(const transform::Transform2d& pose){

        Pose result;
        transform::toQuaternion(result.orientation.w,result.orientation.x,result.orientation.y,result.orientation.z, double(pose.yaw()));
        result.position.x = pose.x();
        result.position.y = pose.y();
        result.position.z = 0.0;

        return result;
    }
}

#endif //CMAKE_SUPER_BUILD_COMMON_MESSAGE_H
