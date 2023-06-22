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

    void to_json(nlohmann::json&j, const Header& object){

        j["stamp"] = common::ToUniversal(object.stamp);
        j["frame_id"] = object.frame_id;
        j["seq"] = object.seq;

    }

    void from_json(const nlohmann::json&j,  Header& object){

        j.at("seq").get_to(object.seq);
        j.at("frame_id").get_to(object.frame_id);
        int64_t stamp;
        j.at("stamp").get_to(stamp);
        object.stamp = common::FromUniversal(stamp);

    }



    constexpr auto Vector3_properties = std::make_tuple(
            common::property(&Vector3::x, "x"),
            common::property(&Vector3::y, "y"),
            common::property(&Vector3::z, "z")
    );



    constexpr auto Twist_properties = std::make_tuple(
            common::property(&Twist::linear, "linear"),
            common::property(&Twist::angular, "angular")
    );

    constexpr auto Point_properties = std::make_tuple(
            common::property(&Point::x, "x"),
            common::property(&Point::y, "y"),
            common::property(&Point::z, "z")
    );

    constexpr auto Quaternion_properties = std::make_tuple(
            common::property(&Quaternion::x, "x"),
            common::property(&Quaternion::y, "y"),
            common::property(&Quaternion::z, "z"),
            common::property(&Quaternion::w, "w")

    );


    constexpr auto Pose_properties = std::make_tuple(
            common::property(&Pose::position, "position"),
            common::property(&Pose::orientation, "orientation")
    );

    constexpr auto PoseStamped_properties = std::make_tuple(
            common::property(&PoseStamped::header, "header"),
            common::property(&PoseStamped::pose, "pose")
    );

    constexpr auto Path_properties = std::make_tuple(
            common::property(&Path::header, "header"),
            common::property(&Path::poses, "poses")
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

    void to_json(nlohmann::json& j, const Point& object)
    {

        constexpr auto nbProperties = std::tuple_size<decltype(Point_properties)>::value;
        common::for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
            // get the property
            auto& property = std::get<i>(Point_properties);
            // set the value to the member
            j[property.name] = object.*(property.member);
        });
    }

    void to_json(nlohmann::json& j, const Quaternion& object)
    {

        constexpr auto nbProperties = std::tuple_size<decltype(Quaternion_properties)>::value;
        common::for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
            // get the property
            auto& property = std::get<i>(Quaternion_properties);
            // set the value to the member
            j[property.name] = object.*(property.member);
        });
    }

    void to_json(nlohmann::json& j, const Pose& object)
    {

        constexpr auto nbProperties = std::tuple_size<decltype(Pose_properties)>::value;
        common::for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
            // get the property
            auto& property = std::get<i>(Pose_properties);
            // set the value to the member
            j[property.name] = object.*(property.member);
        });
    }



    void to_json(nlohmann::json& j, const PoseStamped& object)
    {

        constexpr auto nbProperties = std::tuple_size<decltype(PoseStamped_properties)>::value;
        common::for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
            // get the property
            auto& property = std::get<i>(PoseStamped_properties);
            // set the value to the member
            j[property.name] = object.*(property.member);
        });
    }

    void to_json(nlohmann::json& j, const Path& object)
    {

        constexpr auto nbProperties = std::tuple_size<decltype(Path_properties)>::value;
        common::for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
            // get the property
            auto& property = std::get<i>(Path_properties);
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
            if(j.contains(property.name))
                j.at(property.name).get_to(object.*(property.member));
            else{
                std::cerr << "from_json: key["<<property.name<<"] not exist"<< std::endl;
            }
        });
    }

    void from_json(const nlohmann::json& j, Twist& object) {

        constexpr auto nbProperties = std::tuple_size<decltype(Twist_properties)>::value;
        common::for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
            // get the property
            auto& property = std::get<i>(Twist_properties);
            // set the value to the member
            if(j.contains(property.name))
                j.at(property.name).get_to(object.*(property.member));
            else{
                std::cerr << "from_json: key["<<property.name<<"] not exist"<< std::endl;
            }
        });
    }
    void from_json(const nlohmann::json& j, Point& object) {

        constexpr auto nbProperties = std::tuple_size<decltype(Point_properties)>::value;
        common::for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
            // get the property
            auto& property = std::get<i>(Point_properties);
            // set the value to the member
            if(j.contains(property.name))
                j.at(property.name).get_to(object.*(property.member));
            else{
                std::cerr << "from_json: key["<<property.name<<"] not exist"<< std::endl;
            }
        });
    }

    void from_json(const nlohmann::json& j, Quaternion& object) {

        constexpr auto nbProperties = std::tuple_size<decltype(Quaternion_properties)>::value;
        common::for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
            // get the property
            auto& property = std::get<i>(Quaternion_properties);
            // set the value to the member
            if(j.contains(property.name))
                j.at(property.name).get_to(object.*(property.member));
            else{
                std::cerr << "from_json: key["<<property.name<<"] not exist"<< std::endl;
            }
        });
    }

    void from_json(const nlohmann::json& j, Pose& object) {

        constexpr auto nbProperties = std::tuple_size<decltype(Pose_properties)>::value;
        common::for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
            // get the property
            auto& property = std::get<i>(Pose_properties);
            // set the value to the member
            if(j.contains(property.name))
                j.at(property.name).get_to(object.*(property.member));
            else{
                std::cerr << "from_json: key["<<property.name<<"] not exist"<< std::endl;
            }
        });
    }


    void from_json(const nlohmann::json& j, PoseStamped& object) {

        constexpr auto nbProperties = std::tuple_size<decltype(PoseStamped_properties)>::value;
        common::for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
            // get the property
            auto& property = std::get<i>(PoseStamped_properties);
            // set the value to the member
            if(j.contains(property.name))
                j.at(property.name).get_to(object.*(property.member));
            else{
                std::cerr << "from_json: key["<<property.name<<"] not exist"<< std::endl;
            }
        });
    }


    void from_json(const nlohmann::json& j, Path& object) {

        constexpr auto nbProperties = std::tuple_size<decltype(Path_properties)>::value;
        common::for_sequence(std::make_index_sequence<nbProperties>{}, [&](auto i) {
            // get the property
            auto& property = std::get<i>(Path_properties);
            // set the value to the member
            if(j.contains(property.name))
                j.at(property.name).get_to(object.*(property.member));
            else{
                std::cerr << "from_json: key["<<property.name<<"] not exist"<< std::endl;
            }
        });
    }

}

#endif //CMAKE_SUPER_BUILD_MQTTMESSAGECONVERT_H
