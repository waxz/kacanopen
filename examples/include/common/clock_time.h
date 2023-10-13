//
// Created by waxz on 19-6-4.
//

#ifndef COMMON_CLOCK_TIME_H
#define COMMON_CLOCK_TIME_H

#include <chrono>
#include <ostream>
#include <ratio>
#include <thread>
#include <iostream>
#include <vector>
#include <tuple>
#include <ctime>
#include <locale>
#include <iostream>
#include <iomanip>
#include <iterator>
#include <locale>


namespace common {
    using int8 = int8_t;
    using int16 = int16_t;
    using int32 = int32_t;
    using int64 = int64_t;
    using uint8 = uint8_t;
    using uint16 = uint16_t;
    using uint32 = uint32_t;
    using uint64 = uint64_t;

//https://codereview.stackexchange.com/questions/132852/easy-to-use-c-class-for-asking-current-time-stamp-in-milli-micro-and-nanose/132863
    class CurrentTime {
        std::chrono::high_resolution_clock m_clock;

    public:
        uint64_t milliseconds() {
            return std::chrono::duration_cast<std::chrono::milliseconds>(m_clock.now().time_since_epoch()).count();
        }

        uint64_t microseconds() {
            return std::chrono::duration_cast<std::chrono::microseconds>(m_clock.now().time_since_epoch()).count();
        }

        uint64_t nanoseconds() {
            return std::chrono::duration_cast<std::chrono::nanoseconds>(m_clock.now().time_since_epoch()).count();
        }
    };

//================================
// timer
#if 0
    class Timer {
    public:
        Timer():m_StartTime (std::chrono::system_clock::now())
        {

        }
        std::vector<std::tuple<std::string, double, double>> m_time_record;



        void start() {
            m_StartTime = std::chrono::system_clock::now();
            m_bRunning = true;
        }

        double getTime() const {
            std::chrono::time_point<std::chrono::system_clock> endTime;

            if (m_bRunning) {
                endTime = std::chrono::system_clock::now();
            } else {
                endTime = m_EndTime;
            }
            return std::chrono::duration_cast<std::chrono::microseconds>(endTime - m_StartTime).count();
        }

        double microSeconds() const {

            return std::chrono::duration_cast<std::chrono::microseconds>(m_EndTime - m_StartTime).count();
        }

        double seconds() const {
            return microSeconds() / 1000000.0;
        }

        void stop() {
            m_EndTime = std::chrono::system_clock::now();
            m_bRunning = false;
            getTime();
        }

        void record(const std::string& msg){
            double t = getTime() * 0.001;

            if(m_time_record.empty()){
                m_time_record.push_back(std::make_tuple(msg,t,0.0));

            }else{
                double t0 = std::get<1>(m_time_record.back());

                m_time_record.push_back(std::make_tuple(msg,t,t - t0));

            }

        }
    private:
        std::chrono::time_point<std::chrono::system_clock> m_StartTime;
        std::chrono::time_point<std::chrono::system_clock> m_EndTime;
        bool m_bRunning = false;
    };

    inline std::ostream &operator<<(std::ostream &s, const Timer &timer) {
        s << std::endl
          << "timer record:\n";
        s << "timer count duration\n"
                    << timer.seconds() << " seconds, "
                    << timer.microSeconds() << " microSeconds"
                    << std::endl;


        if(!timer.m_time_record.empty()){
            for(auto& r: timer.m_time_record){

                std::cout <<"** msg:" <<std::get<0>(r) << ", all time: " << std::get<1>(r) << " ms, segment time: " << std::get<2>(r) << " ms" << std::endl;
            }
        }
        return s;
    }

   //================================
// rate

    class Rate {
    private:
        float m_rate;
        size_t m_ms;

    public:
        Rate(float rate) : m_rate(rate), m_ms(1000 / m_rate) {};

        void sleep() {
            std::this_thread::sleep_for(std::chrono::milliseconds(m_ms));
        };
    };
#endif

//=============================================
// from cartographer/common/time.h
////719162 是0001年1月1日到1970年1月1日所经历的天数
    constexpr int64 kUtsEpochOffsetFromUnixEpochInSeconds =
            (719162ll * 24ll * 60ll * 60ll);

    struct UniversalTimeScaleClock {
        using rep = int64;
        using period = std::ratio<1, 10000000>; // 1e-7 s = 1e-1us = 100ns
        using duration = std::chrono::duration<rep, period>;
        using time_point = std::chrono::time_point<UniversalTimeScaleClock>;
        static constexpr bool is_steady = true;
    };

// Represents Universal Time Scale durations and timestamps which are 64-bit
// integers representing the 100 nanosecond ticks since the Epoch which is
// January 1, 1 at the start of day in UTC.
    using Duration = UniversalTimeScaleClock::duration;
    using Time = UniversalTimeScaleClock::time_point;

// Convenience functions to create common::Durations.
    inline Duration FromSeconds(double seconds) {
        return std::chrono::duration_cast<Duration>(
                std::chrono::duration<double>(seconds));
    }

    inline Duration FromMilliseconds(int64 milliseconds){
        return std::chrono::duration_cast<Duration>(
                std::chrono::milliseconds(milliseconds));
    }
    inline Duration FromMicroseconds(int64 microseconds){
        return std::chrono::duration_cast<Duration>(
                std::chrono::microseconds (microseconds));
    }
// Returns the given duration in seconds.
    inline long ToMillSeconds(Duration duration){
        return std::chrono::duration_cast<std::chrono::milliseconds>(duration)
                .count();
    }

    // Returns the given duration in seconds.
    inline long ToMicroSeconds(Duration duration){
        return std::chrono::duration_cast<std::chrono::microseconds>(duration)
                .count();
    }

// Creates a time from a Universal Time Scale.
    inline Time FromUniversal(int64 ticks) { return Time(Duration(ticks)); }

// Outputs the Universal Time Scale timestamp for a given Time.
//将c++的time_point对象转为TUC时间,单位是us
    inline int64 ToUniversal(Time time) { return time.time_since_epoch().count(); }

// For logging and unit tests, outputs the timestamp integer.
    inline std::ostream &operator<<(std::ostream &os, Time time){
        os  << std::to_string(ToUniversal(time));
        return os;
    }

// utc time <---> unix time
//https://stackoverflow.com/questions/31255486/c-how-do-i-convert-a-stdchronotime-point-to-long-and-back
    inline Time FromUnixNow(){
        using namespace std::chrono;
        // Get current time with precision of milliseconds
        auto now = time_point_cast<Duration>(system_clock::now());

        // Convert time_point to signed integral type
        auto integral_duration = now.time_since_epoch().count(); // 1ms = 1e3us = 1e4 * 100ns

        return FromUniversal(integral_duration );
    }

    inline Time FromTimePoint(std::chrono::time_point<std::chrono::system_clock,
                                      std::chrono::nanoseconds> timestamp){
        using namespace std::chrono;

        // Get current time with precision of milliseconds
        auto now = time_point_cast<Duration>(timestamp);

        // Convert time_point to signed integral type
        auto integral_duration = now.time_since_epoch().count(); // 1ms = 1e3us = 1e4 * 100ns

        return FromUniversal(integral_duration );
    }

    template<typename TimeType>
    void ToRos(Time time,TimeType& target_time)
    {
        int64_t uts_timestamp = ToUniversal(time);
//        int64_t ns_since_unix_epoch = (uts_timestamp - kUtsEpochOffsetFromUnixEpochInSeconds *10000000ll) * 100ll;
        int64_t ns_since_unix_epoch = (uts_timestamp ) * 100ll;

        target_time.fromNSec(ns_since_unix_epoch);
    }
    template<typename TimeType>
    Time FromRos(const TimeType& time)
    {
//        auto integral_duration = (time.sec + common::kUtsEpochOffsetFromUnixEpochInSeconds) * 10000000ll  + (time.nsec + 50) / 100;
        auto integral_duration = (time.sec ) * 10000000ll  + (time.nsec + 50) / 100;

        return FromUniversal(integral_duration);
    }


    // Get current date/time, format is YYYY-MM-DD HH:mm:ss, format = "%Y-%m-%d %X" or "%Y-%m-%d-%H-%M-%S"
    inline std::string getCurrentDateTime(const std::string & format = "%Y-%m-%d %X") {
        time_t     now = time(0);
        struct tm  tstruct;
        char       buf[80];
        tstruct = *localtime(&now);
        // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
        // for more information about date/time format
        strftime(buf, sizeof(buf), format.c_str(), &tstruct);

        return buf;
    }

    inline void formatTimestamp(const Time& timestamp, std::string& out)
    {
        std::time_t time = std::chrono::system_clock::to_time_t(
                std::chrono::time_point_cast<Duration>(
                        std::chrono::system_clock::time_point{Duration {ToUniversal(timestamp)}}));
        std::chrono::microseconds u_s = std::chrono::duration_cast<std::chrono::microseconds>(
                timestamp.time_since_epoch() - std::chrono::duration_cast<std::chrono::seconds>(
                        timestamp.time_since_epoch()));

        std::ostringstream s;
        s << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S")
          << ":" << std::setw(6) << std::setfill('0') << u_s.count()<<" us";
        out.assign(s.str());
    }

    struct TimedCounter{
        Time t;
        long interval_ms= 100;
        TimedCounter():t(FromUnixNow()),interval_ms(100){

        }
        TimedCounter(float interval_s):t(FromUnixNow()),interval_ms(interval_s * 1000){
        }

        void set(float interval_s){
            interval_ms = 1000*interval_s;
            t = FromUnixNow();
        }


        /// start counter is s is on
        /// \param s
        void on(bool s){
            if(!s){
                t = FromUnixNow();
            }
        }
        bool eval() const{
            return ToMillSeconds(FromUnixNow() - t) > interval_ms;
        }
    };
} // namespace common
#endif //COMMON_CLOCK_TIME_H
