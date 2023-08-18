//
// Created by waxz on 23-2-23.
//

#ifndef SCAN_REPUBLISHER_TRANSFORM_SERIALIZATION_H
#define SCAN_REPUBLISHER_TRANSFORM_SERIALIZATION_H
#include "transform.h"
#include "nlohmann/json.hpp"


namespace transform{

    inline void to_json(nlohmann::json& j, const transform::Transform2d& p)
    {
        j = {{"matrix",p.matrix}};
    }

    inline void from_json(const nlohmann::json& j, transform::Transform2d& p) {
        j.at("matrix").get_to(p.matrix);
    }


}

#endif //SCAN_REPUBLISHER_TRANSFORM_SERIALIZATION_H
