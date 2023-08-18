//
// Created by waxz on 4/30/23.
//

#ifndef CMAKE_SUPER_BUILD_TYPE_ID_H
#define CMAKE_SUPER_BUILD_TYPE_ID_H

#include <typeinfo>

namespace common{
    // default implementation
    template <typename T>
    struct TypeName
    {
        static const char* Get()
        {
            return typeid(T).name();
        }
    };

// a specialization for each type of those you want to support
// and don't like the string returned by typeid
    template <>
    struct TypeName<int>
    {
        static const char* Get()
        {
            return "int";
        }
    };
}

#endif //CMAKE_SUPER_BUILD_TYPE_ID_H
