//
// Created by waxz on 22-10-11.
//

#ifndef SCAN_REPUBLISHER_TRANSFORM_H
#define SCAN_REPUBLISHER_TRANSFORM_H

#include <cmath>
#include <vector>
#include <array>
#include <iostream>
#include "math/math_basic.h"

namespace transform{

    struct Transform2d{

        std::array<std::array<float,3>,3> matrix;
        Transform2d(float x=0.0, float y=0.0,float yaw=0.0){
            set(x,y,yaw);
        }
        void set(float x=0.0, float y=0.0,float yaw=0.0){
            // Calculate rotation about z axis
            /*
                     cos(yaw),   -sin(yaw),      0,
                     sin(yaw),   cos(yaw),       0,
                     0,          0,              1
                 */
            matrix[0][0] = cos(yaw);
            matrix[0][1]  = -sin(yaw);
            matrix[1][0] = sin(yaw);
            matrix[1][1]  = cos(yaw);

            matrix[0][2]  = x;
            matrix[1][2]  = y;

            matrix[2][0]  = 0.0;
            matrix[2][1]  = 0.0;
            matrix[2][2]  = 1.0;

        }

        float x() const {
            return matrix[0][2];
        }
        float y() const {
            return matrix[1][2];
        }
        float yaw() const {
            return atan2(matrix[1][0],matrix[0][0]);
        }


        Transform2d mul(const Transform2d& rhv)const {
            Transform2d result;

            auto& a = this->matrix;
            auto& b = rhv.matrix;
            auto& c = result.matrix;
            // Calculate the j-th column of the result in-place (in B) using the helper array

//            std::cout << "check matrix mul:\n";
            for(int i=0 ; i<3 ; i++)
            {
                for(int j=0 ; j<3 ; j++)
                {

                    c[i][j]=0;
                    for(int k=0 ; k<3 ; k++)
                    {
//                        std::cout <<" [ " <<  a[i][k] << " * " << b[k][j] << " ] " ;
                        c[i][j]+=a[i][k]*b[k][j];
                        //--^-- should be k
                    }
//                    std::cout << " = " << c[i][j] << "\n";

                }
            }
            return result;

        }

        Transform2d operator*(const Transform2d& rhv)const{
            Transform2d result;

            auto& a = this->matrix;
            auto& b = rhv.matrix;
            auto& c = result.matrix;
            // Calculate the j-th column of the result in-place (in B) using the helper array

//            std::cout << "check matrix multiply:\n";
            for(int i=0 ; i<3 ; i++)
            {
                for(int j=0 ; j<3 ; j++)
                {

                    c[i][j]=0;
                    for(int k=0 ; k<3 ; k++)
                    {
//                        std::cout <<" [ " <<  a[i][k] << " * " << b[k][j] << " ] " ;
                        c[i][j]+=a[i][k]*b[k][j];
                        //--^-- should be k
                    }
//                    std::cout << " = " << c[i][j] << "\n";

                }
            }
            return result;

        }
        void mul(const std::vector<float>& points, std::vector<float>& result){

            /*
         r00 r01 r02 tx     x0        x1
         r10 r11 r12 ty  X  y0   =>   y1
         r20 r21 r22 tz     z0        z1
         0   0   0   1      1         1
        */

            if(points.size()%2 != 0 ){
                std::cerr << __FUNCTION__ << " ERROR : " << " points.size() = " << points.size() << std::endl;

            }
            result.resize(points.size());

            float r00 = this->matrix[0][0];
            float r01 = this->matrix[0][1];

            float r10 = this->matrix[1][0];
            float r11 = this->matrix[1][1];

            float tx = this->matrix[0][2];
            float ty = this->matrix[1][2];

            int n_dim = points.size()/2;

            const float *p_data_x = &(points[0]);
//            const float *p_data_y = p_data_x + n_dim;

            float *p_x = &(result[0]);
//            float *p_y = p_x + n_dim;
//            std::cout << "tf, tx = " << tx << ", ty = " << ty << std::endl;

//            std::cout << "tf, r00 = " << r00 << ", r01 = " << r01 << ", r10 = " << r10  << ", r11 = " << r11 << std::endl;


            for (int i = 0; i < n_dim; i++) {
                p_x[i + i] = r00 * p_data_x[i+i] + r01 * p_data_x[i+i+1] + tx;
                p_x[i + i + 1] = r10 * p_data_x[i+i] + r11 * p_data_x[i+i+1] + ty;
//                std::cout << "== i = " << i << std::endl;
//                std::cout << "== tf p_x = " << p_data_x[i+i] << ", p_y = " << p_data_x[i + i + 1] << std::endl;
//                std::cout << "== tf d_x = " << r00 * p_data_x[i+i] + r01 * p_data_x[i+i+1] + tx << ", d_y = " << r10 * p_data_x[i+i] + r11 * p_data_x[i+i+1] + ty << std::endl;
            }

        }
        void mul(const std::vector<float>& points, size_t n_dim, std::vector<float>& result){

            /*
         r00 r01 r02 tx     x0        x1
         r10 r11 r12 ty  X  y0   =>   y1
         r20 r21 r22 tz     z0        z1
         0   0   0   1      1         1
        */

            if(points.size() < (n_dim+n_dim) ){
                std::cerr << __FUNCTION__ << " ERROR : " << " points.size() = " << points.size() << std::endl;

            }
            result.resize(n_dim+n_dim);

            float r00 = this->matrix[0][0];
            float r01 = this->matrix[0][1];

            float r10 = this->matrix[1][0];
            float r11 = this->matrix[1][1];

            float tx = this->matrix[0][2];
            float ty = this->matrix[1][2];


            const float *p_data_x = &(points[0]);
//            const float *p_data_y = p_data_x + n_dim;

            float *p_x = &(result[0]);
//            float *p_y = p_x + n_dim;
//            std::cout << "tf, tx = " << tx << ", ty = " << ty << std::endl;

//            std::cout << "tf, r00 = " << r00 << ", r01 = " << r01 << ", r10 = " << r10  << ", r11 = " << r11 << std::endl;


            for (size_t i = 0; i < n_dim; i++) {
                p_x[i + i] = r00 * p_data_x[i+i] + r01 * p_data_x[i+i+1] + tx;
                p_x[i + i + 1] = r10 * p_data_x[i+i] + r11 * p_data_x[i+i+1] + ty;
//                std::cout << "== i = " << i << std::endl;
//                std::cout << "== tf p_x = " << p_data_x[i+i] << ", p_y = " << p_data_x[i + i + 1] << std::endl;
//                std::cout << "== tf d_x = " << r00 * p_data_x[i+i] + r01 * p_data_x[i+i+1] + tx << ", d_y = " << r10 * p_data_x[i+i] + r11 * p_data_x[i+i+1] + ty << std::endl;
            }

        }
        void mul( float* points, int n_dim, float*  result){

            /*
         r00 r01 r02 tx     x0        x1
         r10 r11 r12 ty  X  y0   =>   y1
         r20 r21 r22 tz     z0        z1
         0   0   0   1      1         1
        */


            float r00 = this->matrix[0][0];
            float r01 = this->matrix[0][1];

            float r10 = this->matrix[1][0];
            float r11 = this->matrix[1][1];

            float tx = this->matrix[0][2];
            float ty = this->matrix[1][2];


            const float *p_data_x = points;
//            const float *p_data_y = p_data_x + n_dim;

            float *p_x = result;
//            float *p_y = p_x + n_dim;
//            std::cout << "tf, tx = " << tx << ", ty = " << ty << std::endl;

//            std::cout << "tf, r00 = " << r00 << ", r01 = " << r01 << ", r10 = " << r10  << ", r11 = " << r11 << std::endl;


            for (int i = 0; i < n_dim; i++) {
                p_x[i + i] = r00 * p_data_x[i+i] + r01 * p_data_x[i+i+1] + tx;
                p_x[i + i + 1] = r10 * p_data_x[i+i] + r11 * p_data_x[i+i+1] + ty;
//                std::cout << "== i = " << i << std::endl;
//                std::cout << "== tf p_x = " << p_data_x[i+i] << ", p_y = " << p_data_x[i + i + 1] << std::endl;
//                std::cout << "== tf d_x = " << r00 * p_data_x[i+i] + r01 * p_data_x[i+i+1] + tx << ", d_y = " << r10 * p_data_x[i+i] + r11 * p_data_x[i+i+1] + ty << std::endl;
            }

        }

        Transform2d inverse() const{
            Transform2d transform_inv;
            float determinant = 0;

            auto & mat  =this->matrix;
            //finding determinant
            for(int i = 0; i < 3; i++)
                determinant += (mat[0][i] * (mat[1][(i+1)%3] * mat[2][(i+2)%3] - mat[1][(i+2)%3] * mat[2][(i+1)%3]));

            auto & mat_inv  =transform_inv.matrix;
            float determinant_inv = 1.0f/determinant;

            for(int i = 0; i < 3; i++){
                for(int j = 0; j < 3; j++)
                    mat_inv[i][j]= ((mat[(j+1)%3][(i+1)%3] * mat[(j+2)%3][(i+2)%3]) - (mat[(j+1)%3][(i+2)%3] * mat[(j+2)%3][(i+1)%3]))* determinant_inv ;

            }

            return transform_inv;

        }

    };
    inline std::ostream& operator <<(std::ostream& out,const Transform2d& rhv ){
        out << "Transform2d:\n";
//    out.unsetf ( std::ios::floatfield );                // floatfield not set
        out.precision(5);
        out.setf( std::ios::fixed, std:: ios::floatfield ); // floatfield set to fixed

        out << "x-y-yaw:\n[ " << rhv.x() << ", " << rhv.y() << ", " << rhv.yaw() << " ]\n";

        out << "matrix:\n[" << rhv.matrix[0][0] << ", " << rhv.matrix[0][1] << ", " << rhv.matrix[0][2]<<"\n"
            << " " << rhv.matrix[1][0] << ", " << rhv.matrix[1][1] << ", " << rhv.matrix[1][2]<<"\n"
            <<" " << rhv.matrix[2][0] << ", " << rhv.matrix[2][1] << ", " << rhv.matrix[2][2]<<"]\n"
            << std::endl;
        out.unsetf ( std::ios::floatfield );                // floatfield not set

        return out;
    }


    template<typename FloatType>
    struct MatrixSE2{
        std::array<std::array<FloatType,3>,3> matrix;
        MatrixSE2(FloatType x = 0.0, FloatType y = 0.0, FloatType yaw = 0.0){
            set(x,y,yaw);
        }

        MatrixSE2(const Transform2d& rhv){
            set(rhv.x(),rhv.y(),rhv.yaw());
        }

        FloatType x() const {
            return matrix[0][2];
        }
        FloatType y() const {
            return matrix[1][2];
        }
        FloatType yaw() const {
            return atan2(matrix[1][0],matrix[0][0]);
        }
        void set(FloatType x, FloatType y, FloatType yaw){
            matrix[0][0] = cos(yaw);
            matrix[0][1]  = -sin(yaw);
            matrix[1][0] = sin(yaw);
            matrix[1][1]  = cos(yaw);

            matrix[0][2]  = x;
            matrix[1][2]  = y;

            matrix[2][0]  = 0.0;
            matrix[2][1]  = 0.0;
            matrix[2][2]  = 1.0;
        }

        MatrixSE2<FloatType> operator*(const MatrixSE2<FloatType>& rhv)const{
            MatrixSE2<FloatType> result;

            auto& a = this->matrix;
            auto& b = rhv.matrix;
            auto& c = result.matrix;
            // Calculate the j-th column of the result in-place (in B) using the helper array

//            std::cout << "check matrix mul:\n";
            for(int i=0 ; i<3 ; i++)
            {
                for(int j=0 ; j<3 ; j++)
                {

                    c[i][j]=0;
                    for(int k=0 ; k<3 ; k++)
                    {
//                        std::cout <<" [ " <<  a[i][k] << " * " << b[k][j] << " ] " ;
                        c[i][j]+=a[i][k]*b[k][j];
                        //--^-- should be k
                    }
//                    std::cout << " = " << c[i][j] << "\n";

                }
            }
            return result;
        }
    };



    /// compute squared distance between tow Transform2d
    /// \param v_start: Transform2d 1
    /// \param v_end: Transform2d 2
    /// \return squared distance
    inline float diff2(const Transform2d & v_start,const Transform2d& v_end ){
        return (v_start.x() -v_end.x())*(v_start.x() -v_end.x()) + (v_start.y() -v_end.y())*(v_start.y() -v_end.y());
    }

    inline Transform2d interpolate(double factor, const Transform2d & v_start,const Transform2d& v_end  ){
        float tx, ty, yaw;
        tx = v_start.x() + factor*(v_end.x() - v_start.x());
        ty = v_start.y() + factor*(v_end.y() - v_start.y());

        float yaw_1 = angle_normalise_zero(v_start.yaw());
        float yaw_2 = angle_normalise(v_end.yaw(), yaw_1);

        yaw = yaw_1 + factor*(yaw_2 - yaw_1);

        return Transform2d(tx,ty,yaw);
    }

    // Quaternion

    template<typename T>
    void toQuaternion(T &qw, T &qx, T &qy, T &qz, T yaw, T pitch = 0.0, T roll = 0.0) // yaw (Z), pitch (Y), roll (X)
    {
        // Abbreviations for the various angular functions
        T cy = cos(yaw * 0.5);
        T sy = sin(yaw * 0.5);
        T cp = cos(pitch * 0.5);
        T sp = sin(pitch * 0.5);
        T cr = cos(roll * 0.5);
        T sr = sin(roll * 0.5);

        qw = cy * cp * cr + sy * sp * sr;
        qx = cy * cp * sr - sy * sp * cr;
        qy = sy * cp * sr + cy * sp * cr;
        qz = sy * cp * cr - cy * sp * sr;

        /* 2d condition only yaw
            q.qw = cos(yaw * 0.5) ;
            q.qx = 0 ;
            q.qy = 0 ;
            q.qz = sin(yaw * 0.5) ;
             * */
    }

    //https://stackoverflow.com/questions/11667783/quaternion-and-normalization
    template<typename T>
    bool toEulerAngle(T &yaw, T &pitch, T &roll,  T qw, T qx, T qy, T qz) {

        double qmagsq = qw*qw + qx*qx + qy*qy + qz*qz;

        double scale = 1.0;

        if (std::abs(1.0 - qmagsq) < 2.107342e-08) {
            scale = 2.0 / (1.0 + qmagsq);
        }
        else {
            scale = 1.0 / std::sqrt(qmagsq);
        }

        qw *= scale;
        qx *= scale;
        qy *= scale;
        qz *= scale;

        // We choose the quaternion with positive 'qw', i.e., the one with a smaller
        // angle that represents this orientation.
#if 1
        if (qw < 0.0) {
            // Multiply by -1. http://eigen.tuxfamily.org/bz/show_bug.cgi?id=560
            qw = -1. * qw;
            qx = -1. * qx;
            qy = -1. * qy;
            qz = -1. * qz;
        }
#endif

        // roll (qx-axis rotation)
        T sinr_cosp = +2.0 * (qw * qx + qy * qz);
        T cosr_cosp = +1.0 - 2.0 * (qx * qx + qy * qy);
        roll = atan2(sinr_cosp, cosr_cosp);

        // pitch (qy-axis rotation)
        T sinp = +2.0 * (qw * qy - qz * qx);
        if (fabs(sinp) >= 1)
            pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            pitch = asin(sinp);

        // yaw (qz-axis rotation)
        T siny_cosp = +2.0 * (qw * qz + qx * qy);
        T cosy_cosp = +1.0 - 2.0 * (qy * qy + qz * qz);
        yaw = atan2(siny_cosp, cosy_cosp);

        return true;

    }

}
#endif //SCAN_REPUBLISHER_TRANSFORM_H
