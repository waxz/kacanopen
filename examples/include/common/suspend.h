//
// Created by waxz on 6/28/22.
//

#ifndef DOCKER_DEMO_SUSPEND_H
#define DOCKER_DEMO_SUSPEND_H

#include <mutex>
#include <condition_variable>
#include <chrono>
#include <cmath>
#include <thread>

namespace common{

    //https://stackoverflow.com/questions/51817018/can-you-implement-a-timer-without-a-sleep-in-it-using-standard-c-c11-only
    // repeatedly sleeping for small amounts as in your example is a bad idea. This will burn a lot of CPU on needlessly rescheduling and waking threads
    struct Suspend{
        std::mutex mtx;
        std::unique_lock<std::mutex> locker;
        std::condition_variable cv;
//        std::chrono::duration<int, std::ratio<1, 1000>> time_count(1000);
        Suspend():locker(mtx){

        }
        void sleep(float ms){
            cv.wait_for(locker, std::chrono::duration<float, std::ratio<1, 1000>> (ms), [] {  return false;  });

        }
    };
    inline void preciseSleep(double seconds) {
        using namespace std;
        using namespace std::chrono;

        static double estimate = 5e-3;
        static double mean = 5e-3;
        static double m2 = 0;
        static int64_t count = 1;

        while (seconds > estimate) {
            auto start = high_resolution_clock::now();
            this_thread::sleep_for(milliseconds(1));
            auto end = high_resolution_clock::now();

            double observed = (end - start).count() / 1e9;
            seconds -= observed;

            ++count;
            double delta = observed - mean;
            mean += delta / count;
            m2   += delta * (observed - mean);
            double stddev = sqrt(m2 / (count - 1));
            estimate = mean + stddev;
        }

        // spin lock
        auto start = high_resolution_clock::now();
        while ((high_resolution_clock::now() - start).count() / 1e9 < seconds);
    }

}
#endif //DOCKER_DEMO_SUSPEND_H
