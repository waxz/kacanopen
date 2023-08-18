//
// Created by waxz on 6/24/23.
//

#ifndef KACANOPEN_STAMPED_BUFFER_H
#define KACANOPEN_STAMPED_BUFFER_H

#include "clock_time.h"
#include <deque>
#include <algorithm>
namespace common{

    template<typename T>
    struct ValueStamped{

        common::Time time = common::FromUnixNow();
        T value ;
    };

    inline float interpolate(double factor,const float& v_start,const float& v_end  ){

        return v_start + factor*(v_end - v_start);
    }

    template<typename T>
    struct ValueStampedBuffer{
        std::deque<ValueStamped<T>> buffer;
        size_t MAX_SIZE = 2000;
        void add(const T & value){

            buffer.push_back(ValueStamped<T>{common::FromUnixNow(), value});

            if(buffer.size() > MAX_SIZE){
                buffer.erase(buffer.begin(),buffer.begin() + MAX_SIZE/2);
            }
        }
        void add(const common::Time & t, const T & value){

            if(buffer.empty() || t != buffer.back().time){
                buffer.push_back(ValueStamped<T>{t, value});
            }

            if(buffer.size() > MAX_SIZE){
                buffer.erase(buffer.begin(),buffer.begin() + MAX_SIZE/2);
            }
        }
        void clear(){
            buffer.clear();
        }

        bool empty(){
            return buffer.empty();
        }

        size_t size(){
            return buffer.size();
        }
        bool query(const common::Time& time, T & value){

            if(buffer.size() < 3){
                return false;
            }

            if(time == buffer.back().time){

                value = buffer.back().value;
                return true;
            }

            // first v > t

            auto pred = [time](auto&x){return x.time < time;};
            auto mid = std::partition (buffer.begin(),buffer.end(),pred);


//        auto it = std::stable_partition(buffer.begin(),buffer.end(), [time](auto& v) { return v.time > time; });





            if(mid!= buffer.end()){
                auto low_it = std::prev(mid);
                auto up_it = mid;


//            std::cout << "interpolate, up: "  << std::distance(buffer.begin(), up_it)  << ", low:  " << std::distance(buffer.begin(), low_it) << std::endl;

//            std::cout << "interpolate, value : " << up_it->value << ", " << low_it->value << std::endl;

//            std::cout << "interpolate: time " << s1*1e-6 << ", " << s2*1e-6<< std::endl;


                auto s1 =  common::ToMicroSeconds(up_it->time - low_it->time);
                auto s2 =  common::ToMicroSeconds(time - low_it->time);
                double factor = s2/s1;
                value = interpolate(factor, low_it->value, up_it->value);

                return true;
            }else{
                return false;
            }

            return false;
        }
        const ValueStamped<T>& back(){
            return buffer.back();
        }

    };
}
#endif //KACANOPEN_STAMPED_BUFFER_H
