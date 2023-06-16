//
// Created by waxz on 5/11/23.
//

#ifndef CMAKE_SUPER_BUILD_ROSMESSAGECONVERT_H
#define CMAKE_SUPER_BUILD_ROSMESSAGECONVERT_H

#include "message/common_message.h"
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <yocs_msgs/canMessageArray.h>

namespace common_message{


    inline Twist to_common(const geometry_msgs::Twist& data){

        Twist target;
        target.linear.x = data.linear.x;
        target.linear.y = data.linear.y;
        target.linear.z = data.linear.z;


        target.angular.x = data.angular.x;
        target.angular.y = data.angular.y;
        target.angular.z = data.angular.z;

        return target;
    }

    inline geometry_msgs::Twist from_common(const Twist& data){
        geometry_msgs::Twist target;
        target.linear.x = data.linear.x;
        target.linear.y = data.linear.y;
        target.linear.z = data.linear.z;


        target.angular.x = data.angular.x;
        target.angular.y = data.angular.y;
        target.angular.z = data.angular.z;

        return target;
    }


    inline Header to_common(const std_msgs::Header& data){
        Header target;
        target.frame_id.assign(data.frame_id);
        target.seq = data.seq;
        target.stamp = common::FromRos(data.stamp);
        return target;
    }

    inline std_msgs::Header from_common(const Header& data){
        std_msgs::Header target;
        target.frame_id.assign(data.frame_id);
        target.seq = data.seq;
        common::ToRos(data.stamp,target.stamp);

        return target;
    }

    inline TwistWithCovariance to_common(const geometry_msgs::TwistWithCovariance & data   ){

        TwistWithCovariance target;
        target.twist = to_common(data.twist);
        std::copy(std::begin(data.covariance),std::end(data.covariance), std::begin(target.covariance));

        return target;
    }

    inline geometry_msgs::TwistWithCovariance from_common(const TwistWithCovariance & data   ){

        geometry_msgs::TwistWithCovariance target;
        target.twist = from_common(data.twist);
        std::copy(std::begin(data.covariance),std::end(data.covariance), std::begin(target.covariance));

        return target;
    }


    inline Point to_common(const geometry_msgs::Point& data){
        Point target;
        target.x = data.x;
        target.y = data.y;
        target.z = data.z;
        return target;
    }

    inline geometry_msgs::Point from_common(const Point& data){
        geometry_msgs::Point target;
        target.x = data.x;
        target.y = data.y;
        target.z = data.z;
        return target;
    }


    inline Quaternion to_common(const geometry_msgs::Quaternion& data){
        Quaternion target;
        target.x = data.x;
        target.y = data.y;
        target.z = data.z;
        target.w = data.w;
        return target;
    }

    inline geometry_msgs::Quaternion from_common(const Quaternion& data){
        geometry_msgs::Quaternion target;
        target.x = data.x;
        target.y = data.y;
        target.z = data.z;
        target.w = data.w;
        return target;
    }
    inline Pose to_common(const geometry_msgs::Pose& data){
        Pose target;
        target.position = to_common(data.position);
        target.orientation = to_common(data.orientation);
        return target;
    }
    inline geometry_msgs::Pose from_common(const Pose& data){
        geometry_msgs::Pose target;
        target.position = from_common(data.position);
        target.orientation = from_common(data.orientation);
        return target;
    }

    inline PoseWithCovariance to_common(const geometry_msgs::PoseWithCovariance& data){
        PoseWithCovariance target;

        target.pose = to_common(data.pose);
        std::copy(std::begin(data.covariance),std::end(data.covariance), std::begin(target.covariance));

        return target;
    }
    inline geometry_msgs::PoseWithCovariance from_common(const PoseWithCovariance& data){
        geometry_msgs::PoseWithCovariance target;

        target.pose = from_common(data.pose);
        std::copy(std::begin(data.covariance),std::end(data.covariance), std::begin(target.covariance));

        return target;
    }

    inline Odometry to_common(const nav_msgs::Odometry& data){

        Odometry target;
        target.header = to_common(data.header);
        target.child_frame_id.assign(data.child_frame_id);
        target.pose = to_common( data.pose);
        target.twist = to_common( data.twist);


        return target;
    }


    inline nav_msgs::Odometry from_common(const Odometry& data){

        nav_msgs::Odometry target;
        target.header = from_common(data.header);
        target.child_frame_id.assign(data.child_frame_id);
        target.pose = from_common( data.pose);
        target.twist = from_common( data.twist);


        return target;
    }


    inline TransformStamped to_common(const tf::StampedTransform& data){
        TransformStamped target;

        target.base_frame.assign(data.frame_id_);
        target.target_frame.assign(data.child_frame_id_);

        target.time = common::FromRos(data.stamp_);

        target.transform =  transform::createSe3<float>(data.getOrigin().x(),data.getOrigin().y(),data.getOrigin().z(),
                                                        data.getRotation().getW(), data.getRotation().getX(), data.getRotation().getY(), data.getRotation().getZ());

        return target;
    }

    inline void to_common(const tf::StampedTransform& data,TransformStamped& target){

        target.base_frame.assign(data.frame_id_);
        target.target_frame.assign(data.child_frame_id_);

        target.time = common::FromRos(data.stamp_);

        target.transform =  transform::createSe3<float>(data.getOrigin().x(),data.getOrigin().y(),data.getOrigin().z(),
                                                        data.getRotation().getW(), data.getRotation().getX(), data.getRotation().getY(), data.getRotation().getZ());

    }
    inline tf::StampedTransform from_common(const TransformStamped& data){
        tf::StampedTransform  target;

        target.frame_id_.assign(data.base_frame);
        target.child_frame_id_.assign(data.target_frame);

        float tx, ty,tz,qw,qx,qy,qz;
        transform::extractSe3<float>(data.transform, tx, ty,tz,qw,qx,qy,qz);
        target.setOrigin(tf::Vector3(tx, ty,tz));
        target.setRotation(tf::Quaternion(qx,qy,qz,qw));
        common::ToRos(data.time, target.stamp_);

        return target;
    }
    inline void from_common(const TransformStamped& data,tf::StampedTransform&  target){

        target.frame_id_.assign(data.base_frame);
        target.child_frame_id_.assign(data.target_frame);

        float tx, ty,tz,qw,qx,qy,qz;
        transform::extractSe3<float>(data.transform, tx, ty,tz,qw,qx,qy,qz);
        target.setOrigin(tf::Vector3(tx, ty,tz));
        target.setRotation(tf::Quaternion(qx,qy,qz,qw));
        common::ToRos(data.time, target.stamp_);

    }

    inline common_message::CanMessage to_common(const yocs_msgs::canMessage& data){
        common_message::CanMessage target;
        target.id = data.id;
        target.is_rtr = data.is_rtr;
        target.is_extended = data.is_extended;
        target.is_error = data.is_error;
        target.dlc = data.dlc;
        std::copy(std::begin(data.data),std::end(data.data), std::begin(target.data));

        return target;
    }

    inline yocs_msgs::canMessage from_common(const  common_message::CanMessage & data){
        yocs_msgs::canMessage  target;
        target.id = data.id;
        target.is_rtr = data.is_rtr;
        target.is_extended = data.is_extended;
        target.is_error = data.is_error;
        target.dlc = data.dlc;
        std::copy(std::begin(data.data),std::end(data.data), std::begin(target.data));
        return target;

    }

    inline std::vector<common_message::CanMessage> to_common(const yocs_msgs::canMessageArray & data){
        std::vector<common_message::CanMessage> target;
        target.resize(data.messages.size());
        for(size_t i = 0 ; i < target.size(); i++){
            target[i] = to_common(data.messages[i]);
        }
        return target;

    }

    inline yocs_msgs::canMessageArray from_common(const std::vector<common_message::CanMessage>& data){
        yocs_msgs::canMessageArray target;
        target.messages.resize(data.size());
        for(size_t i = 0 ; i < target.messages.size(); i++){
            target.messages[i] = from_common(data[i]);
        }
        return target;

    }
}

#endif //CMAKE_SUPER_BUILD_ROSMESSAGECONVERT_H
