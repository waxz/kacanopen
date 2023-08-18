//
// Created by waxz on 23-2-23.
//

#ifndef SCAN_REPUBLISHER_EIGEN_TRANSFORM_H
#define SCAN_REPUBLISHER_EIGEN_TRANSFORM_H

#include <Eigen/Core>
#include <Eigen/Geometry>

// transformation should only use double

// if you use non-normalised float matrix ,  Eigen::Quaternion<FloatType> Q(transform.rotation()) , may block program

//typedef Transform<double,3,Isometry> Isometry3d;

namespace transform{
    //https://stackoverflow.com/questions/63341630/angle-axis-to-quaternion-using-eigen
    template<typename FloatType>
    Eigen::Quaternion<FloatType> createQuaternion(FloatType roll, FloatType pitch, FloatType yaw){

        Eigen::Matrix<FloatType,3,1> rotation(roll, pitch, yaw);
        FloatType angle = rotation.norm();
        Eigen::Matrix<FloatType,3,1> axis = rotation.normalized();
        return Eigen::Quaternion<FloatType> (Eigen::AngleAxis<FloatType>(angle, axis));

    }



    template<typename FloatType>
    Eigen::Transform<FloatType,3,Eigen::Isometry> createSe3(FloatType tx, FloatType ty, FloatType tz, FloatType qw, FloatType qx, FloatType qy, FloatType qz){

        Eigen::Transform<FloatType,3,Eigen::Isometry> result = Eigen::Transform<FloatType,3,Eigen::Isometry>::Identity();;
//    Eigen::Quaternion<FloatType> Q = Eigen::Quaternion<FloatType>(qw, qx, qy, qz).normalized();
        Eigen::Quaternion<FloatType> Q(qw, qx, qy, qz);

        Q.normalize();
        Eigen::Matrix<FloatType,3,1> T(tx, ty, tz);

        result.rotate(Q.toRotationMatrix());
        result.pretranslate(T);
        return result;

    }

    template<typename FloatType>
    Eigen::Transform<FloatType,3,Eigen::Isometry> createSe3(FloatType tx, FloatType ty, FloatType tz, FloatType roll, FloatType pitch, FloatType yaw){

        Eigen::Transform<FloatType,3,Eigen::Isometry> result = Eigen::Transform<FloatType,3,Eigen::Isometry>::Identity();;
//    Eigen::Quaternion<FloatType> Q = Eigen::Quaternion<FloatType>(qw, qx, qy, qz).normalized();
        Eigen::Quaternion<FloatType> Q = createQuaternion(roll, pitch, yaw);
        Eigen::Matrix<FloatType,3,1> T(tx, ty, tz);

        result.rotate(Q.toRotationMatrix());
        result.pretranslate(T);
        return result;

    }


    template<typename FloatType>
    void extractSe3(const Eigen::Transform<FloatType,3,Eigen::Isometry>& transform, FloatType& tx, FloatType& ty, FloatType& tz, FloatType& qw, FloatType& qx, FloatType& qy, FloatType& qz){

        tx = transform.translation()(0);
        ty = transform.translation()(1);
        tz = transform.translation()(2);
        Eigen::Quaternion<FloatType> Q(transform.rotation());
        qw = Q.w();
        qx = Q.x();
        qy = Q.y();
        qz = Q.z();

    }

//https://math.stackexchange.com/questions/90081/quaternion-distance
    template<typename FloatType>
    FloatType computeQuaternionDiff(const  Eigen::Quaternion<FloatType>& q1, const  Eigen::Quaternion<FloatType>& q2){
        FloatType qd1 = q1.x()*q2.x() + q1.y()*q2.y() + q1.z() * q2.z() + q1.w()*q2.w();
        FloatType diff = 2*acos(abs(qd1));
        return diff;
    }


    template<typename FloatType>
    Eigen::Quaternion<FloatType> createQuaternion( FloatType qw, FloatType qx, FloatType qy, FloatType qz){
        Eigen::Quaternion<FloatType> Q(qw, qx, qy, qz);
        Q.normalize();
        return Q;

    }
}

#endif //SCAN_REPUBLISHER_EIGEN_TRANSFORM_H
