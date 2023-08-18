//
// Created by waxz on 5/16/23.
//

#ifndef CMAKE_SUPER_BUILD_BEZIERGENERATOR_H
#define CMAKE_SUPER_BUILD_BEZIERGENERATOR_H

#include <vector>
#include <array>

namespace math{

    /*

     */
    /*

             float PA[2] = {-1.2,0.5};
        float PB[2] = {-1.0,0.0};
        float PC[2] = {-0.8,0.0};
        float PD[2] = {0.0,0.0};

     */
    inline bool buildBezier(float PA[2] ,float PB[2],float PC[2],float PD[2], float step, std::vector<std::array<float,2>> & path){


        float simple_len =
                   sqrt((PA[0] -PB[0])*(PA[0] -PB[0]) +(PA[1] -PB[1])*(PA[1] -PB[1]))
                +  sqrt((PC[0] -PB[0])*(PC[0] -PB[0]) +(PC[1] -PB[1])*(PC[1] -PB[1]))
                +  sqrt((PC[0] -PD[0])*(PC[0] -PD[0]) +(PC[1] -PD[1])*(PC[1] -PD[1]));
        step = std::max(std::min(step, 0.05f), 0.01f);
        size_t t_num = simple_len/step;
        step = 1.0f/t_num;
        path.resize(t_num + 1);
        float t = 0.0;

        float PAB[2];
        float PBC[2];
        float PCD[2];
        float PABC[2];
        float PBCD[2];

        for(size_t i = 0 ; i < t_num ; i++){


            PAB[0] = (1-t) *PA[0] + t*PB[0];
            PAB[1] = (1-t) *PA[1] + t*PB[1];

            PBC[0] = (1-t) *PB[0] + t*PC[0];
            PBC[1] = (1-t) *PB[1] + t*PC[1];

            PCD[0] = (1-t) *PC[0] + t*PD[0];
            PCD[1] = (1-t) *PC[1] + t*PD[1];


            PABC[0] = (1-t) *PAB[0] + t*PBC[0];
            PABC[1] = (1-t) *PAB[1] + t*PBC[1];

            PBCD[0] = (1-t) *PBC[0] + t*PCD[0];
            PBCD[1] = (1-t) *PBC[1] + t*PCD[1];


            path[i][0] = (1-t) *PABC[0] + t*PBCD[0];
            path[i][1] =  (1-t) *PABC[1] + t*PBCD[1];
            t += step;
        }
        path[t_num][0] = PD[0];
        path[t_num][1] = PD[1];

        return true;
    }
}

#endif //CMAKE_SUPER_BUILD_BEZIERGENERATOR_H
