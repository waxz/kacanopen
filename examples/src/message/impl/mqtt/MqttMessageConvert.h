//
// Created by waxz on 5/13/23.
//

#ifndef CMAKE_SUPER_BUILD_MQTTMESSAGECONVERT_H
#define CMAKE_SUPER_BUILD_MQTTMESSAGECONVERT_H
#include "message/common_message.h"
#include "nlohmann/json.hpp"

namespace common_message{



    // mqtt <--> json
    // mqtt <--> string

    template <typename T, typename std::enable_if<std::is_same<T,std::string>{} , bool>::type = true>
    T to_common(const char* data){
        return data;
    }
    template <typename T, typename std::enable_if<!std::is_same<T,std::string>{} , bool>::type = true>
    T to_common(const char* data){
        T target =  nlohmann::json::parse(data);
        return target;
    }

    template <typename T, typename std::enable_if<std::is_same<T,std::string>{} , bool>::type = true>
    std::string from_common(const T& data){
        return std::string(data);
    }

    template <typename T, typename std::enable_if<!std::is_same<T,std::string>{} , bool>::type = true>
    std::string from_common(const T& data){
        nlohmann::json  j = data;
        return j.dump();
    }

    template <typename T, typename std::enable_if<std::is_same<T,Int16Array>{} , bool>::type = true>
    std::string from_common(const T& data){
        std::string target;
        target.assign((char*)data.data(), data.size()*2);
        return target;
    }


    // common

    constexpr auto Vector3_properties = std::make_tuple(
            common::property(&Vector3::x, "x"),
            common::property(&Vector3::y, "y"),
            common::property(&Vector3::z, "z")
    );



    constexpr auto Twist_properties = std::make_tuple(
            common::property(&Twist::linear, "linear"),
            common::property(&Twist::angular, "angular")
    );

    void to_json(nlohmann::json& j, const Vector3& object)
    {

        constexpr auto nbProperties = std::tuple_size<decltype(Vector3_properties)>::value;
        common::for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
            // get the property
            auto& property = std::get<i>(Vector3_properties);
            // set the value to the member
            j[property.name] = object.*(property.member);
        });
    }
    void to_json(nlohmann::json& j, const Twist& object)
    {

        constexpr auto nbProperties = std::tuple_size<decltype(Twist_properties)>::value;
        common::for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
            // get the property
            auto& property = std::get<i>(Twist_properties);
            // set the value to the member
            j[property.name] = object.*(property.member);
        });
    }

    void from_json(const nlohmann::json& j, Vector3& object) {

        constexpr auto nbProperties = std::tuple_size<decltype(Vector3_properties)>::value;
        common::for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
            // get the property
            auto& property = std::get<i>(Vector3_properties);
            // set the value to the member
            object.*(property.member) = j[property.name];
        });
    }
    void from_json(const nlohmann::json& j, Twist& object) {

        constexpr auto nbProperties = std::tuple_size<decltype(Twist_properties)>::value;
        common::for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
            // get the property
            auto& property = std::get<i>(Twist_properties);
            // set the value to the member
            object.*(property.member) = j[property.name];
        });
    }



}

#endif //CMAKE_SUPER_BUILD_MQTTMESSAGECONVERT_H
