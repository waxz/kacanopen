//
// Created by waxz on 5/8/23.
//

#ifndef CMAKE_SUPER_BUILD_GEOMETRY_H
#define CMAKE_SUPER_BUILD_GEOMETRY_H

#include <cmath>

namespace math{

    //https://stackoverflow.com/a/12132746
    inline void getLine(double x1, double y1, double x2, double y2, double &a, double &b, double &c)
    {
        // (x- p1X) / (p2X - p1X) = (y - p1Y) / (p2Y - p1Y)
        a = y1 - y2; // Note: this was incorrectly "y2 - y1" in the original answer
        b = x2 - x1;
        c = x1 * y2 - x2 * y1;
    }

    // point(pct1) to line (pct2-pct3)
    inline double getPointDistToLine(double pct1X, double pct1Y, double pct2X, double pct2Y, double pct3X, double pct3Y)
    {
        double a, b, c;
        getLine(pct2X, pct2Y, pct3X, pct3Y, a, b, c);
        return abs(a * pct1X + b * pct1Y + c) / sqrt(a * a + b * b);
    }




    //https://gist.github.com/TimSC/47203a0f5f15293d2099507ba5da44e6#file-linelineintersect-cpp
/** Calculate determinant of matrix:
	[a b]
	[c d]
*/
    inline float Det(float a, float b, float c, float d)
    {
        return a*d - b*c;
    }
///Calculate intersection of two lines.
///\return true if found, false if not found or error
    inline bool LineLineIntersect(float x1, float y1, //Line 1 start
                           float x2, float y2, //Line 1 end
                           float x3, float y3, //Line 2 start
                           float x4, float y4, //Line 2 end
                           float &ixOut, float &iyOut) //Output
    {
        //http://mathworld.wolfram.com/Line-LineIntersection.html

        float detL1 = Det(x1, y1, x2, y2);
        float detL2 = Det(x3, y3, x4, y4);
        float x1mx2 = x1 - x2;
        float x3mx4 = x3 - x4;
        float y1my2 = y1 - y2;
        float y3my4 = y3 - y4;

        float xnom = Det(detL1, x1mx2, detL2, x3mx4);
        float ynom = Det(detL1, y1my2, detL2, y3my4);
        float denom = Det(x1mx2, y1my2, x3mx4, y3my4);
        if(denom == 0.0)//Lines don't seem to cross
        {
            ixOut = NAN;
            iyOut = NAN;
            return false;
        }

        ixOut = xnom / denom;
        iyOut = ynom / denom;
        if(!std::isfinite(ixOut) || !std::isfinite(iyOut)) //Probably a numerical issue
            return false;

        return true; //All OK
    }



    inline void to_yaw( float x,  float y,  float z,  float w, float &yaw) {
// roll (x-axis rotation)
        float sinr_cosp = +2.0 * (w * x + y * z);
        float cosr_cosp = +1.0 - 2.0 * (x * x + y * y);

// pitch (y-axis rotation)
        float sinp = +2.0 * (w * y - z * x);


// yaw (z-axis rotation)
        float siny_cosp = +2.0 * (w * z + x * y);
        float cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
        yaw = atan2(siny_cosp, cosy_cosp);
//    return yaw;
    }
    inline void toEulerAngle(const float x, const float y, const float z, const float w, float &roll, float &pitch, float &yaw) {
// roll (x-axis rotation)
        float sinr_cosp = +2.0 * (w * x + y * z);
        float cosr_cosp = +1.0 - 2.0 * (x * x + y * y);
        roll = atan2(sinr_cosp, cosr_cosp);

// pitch (y-axis rotation)
        float sinp = +2.0 * (w * y - z * x);
        if (fabs(sinp) >= 1)
            pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            pitch = asin(sinp);

// yaw (z-axis rotation)
        float siny_cosp = +2.0 * (w * z + x * y);
        float cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
        yaw = atan2(siny_cosp, cosy_cosp);
//    return yaw;
    }


    inline void yaw_to_quaternion(float yaw, double &qx, double &qy, double &qz, double &qw) {
        float roll = 0.0;
        float pitch = 0.0;

        qx = 0.0;
        qy = 0.0;
        qz = sin(yaw * 0.5f);
        qw = cos(yaw * 0.5f);

        double sin_roll_2 = sin(roll / 2);
        double cos_pitch_2 = cos(pitch / 2);
        double cos_roll_2 = cos(roll / 2);
        double sin_yaw_2 = sin(yaw / 2);
        double cos_yaw_2 = cos(yaw / 2);
        double sin_pitch_2 = sin(pitch / 2);


        
        qx = sin_roll_2 * cos_pitch_2 * cos_yaw_2 - cos_roll_2 * sin_pitch_2 * sin_yaw_2;
        qy = cos_roll_2 * sin_pitch_2 * cos_yaw_2 + sin_roll_2 * cos_pitch_2 * sin_yaw_2;
        qz = cos_roll_2 * cos_pitch_2 * sin_yaw_2 - sin_roll_2 * sin_pitch_2 * cos_yaw_2;
        qw = cos_roll_2 * cos_pitch_2 * cos_yaw_2 + sin_roll_2 * sin_pitch_2 * sin_yaw_2;

    }
    inline void yaw_to_quaternion(float yaw, float &qx, float &qy, float &qz, float &qw) {
        float roll = 0.0;
        float pitch = 0.0;

        qx = 0.0;
        qy = 0.0;
        qz = sin(yaw * 0.5f);
        qw = cos(yaw * 0.5f);

        float sin_roll_2 = sin(roll / 2);
        float cos_pitch_2 = cos(pitch / 2);
        float cos_roll_2 = cos(roll / 2);
        float sin_yaw_2 = sin(yaw / 2);
        float cos_yaw_2 = cos(yaw / 2);
        float sin_pitch_2 = sin(pitch / 2);

        qx = sin_roll_2 * cos_pitch_2 * cos_yaw_2 - cos_roll_2 * sin_pitch_2 * sin_yaw_2;
        qy = cos_roll_2 * sin_pitch_2 * cos_yaw_2 + sin_roll_2 * cos_pitch_2 * sin_yaw_2;
        qz = cos_roll_2 * cos_pitch_2 * sin_yaw_2 - sin_roll_2 * sin_pitch_2 * cos_yaw_2;
        qw = cos_roll_2 * cos_pitch_2 * cos_yaw_2 + sin_roll_2 * sin_pitch_2 * sin_yaw_2;

    }
}

#endif //CMAKE_SUPER_BUILD_GEOMETRY_H
