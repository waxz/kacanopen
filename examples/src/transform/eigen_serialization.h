//
// Created by waxz on 23-2-23.
//

#ifndef SCAN_REPUBLISHER_EIGEN_SERIALIZATION_H
#define SCAN_REPUBLISHER_EIGEN_SERIALIZATION_H

#include "eigen_transform.h"
#include "transform.h"
#include "nlohmann/json.hpp"


namespace Eigen{

    template<typename FloatType>
    void to_json(nlohmann::json& j, const Eigen::Transform<FloatType,3,Eigen::Isometry> & p)
    {
        FloatType tx, ty,tz, qw,qx,qy,qz;

        transform::extractSe3(p, tx, ty,tz, qw,qx,qy,qz);

        j = {{"tx",tx},
             {"ty",ty},
             {"tz",tz},

             {"qw",qw},
             {"qx",qx},
             {"qy",qy},
             {"qz",qz}
             };
    }
    template<typename FloatType>
    void from_json(const nlohmann::json& j, Eigen::Transform<FloatType,3,Eigen::Isometry>& p) {

        FloatType tx, ty,tz, qw,qx,qy,qz;

        j.at("tx").get_to(tx);
        j.at("ty").get_to(ty);
        j.at("tz").get_to(tz);


        j.at("qw").get_to(qw);
        j.at("qx").get_to(qx);
        j.at("qy").get_to(qy);
        j.at("qz").get_to(qz);

        p = transform::createSe3(tx, ty,tz, qw,qx,qy,qz);

}


}



#endif //SCAN_REPUBLISHER_EIGEN_SERIALIZATION_H
