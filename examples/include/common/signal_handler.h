#ifndef COMMON_SYSTEM_SIGNAL_H
#define COMMON_SYSTEM_SIGNAL_H

#include <signal.h>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <iostream>

namespace common {

    inline bool &getSystemTerminalFlag() {
        static bool SystemTerminalFlag = true;
        return SystemTerminalFlag;
    }
    static bool SignalOK() {
        return getSystemTerminalFlag();
    }

    inline void my_handler(int signum) {
        std::cerr << "  Caught signal: " << signum << std::endl;
        if ( (2 == signum ) or ( 15 == signum)) {
            bool &flag = getSystemTerminalFlag();
            flag = false;
            std::cerr << "  Set SignalOK to : " << SignalOK() << std::endl;

        }
    }



    //https://stackoverflow.com/questions/7376228/linux-c-catching-kill-signal-for-graceful-termination
    //respond to  SIGINT, SIGTERM
    //Don't try to respond to all signals and don't try to "clean up" after signal that indicates an error in your program.
    // SIGKILL, SIGSEGV, SIGBUS can't be caught.

    //123
#if 0
    class SystemSignalHandler {
    public:
        struct sigaction sigIntHandler;

        SystemSignalHandler() {

            sigemptyset(&sigIntHandler.sa_mask);
            sigIntHandler.sa_flags = 0;
            sigIntHandler.sa_handler = my_handler;

            sigaction(SIGINT, &sigIntHandler, nullptr);
            sigaction(SIGTERM, &sigIntHandler, nullptr);

        }


    };
#endif
    // if use in ros app, this function should be called after ros::init
    inline void initSystemSignalHandler() {
//        static SystemSignalHandler s;

        struct sigaction sigIntHandler;
        sigemptyset(&sigIntHandler.sa_mask);
        sigIntHandler.sa_flags = 0;
        sigIntHandler.sa_handler = my_handler;

        sigaction(SIGINT, &sigIntHandler, nullptr);
        sigaction(SIGTERM, &sigIntHandler, nullptr);

    }



    inline bool spin() {
        initSystemSignalHandler();
        std::mutex mtx;
        std::unique_lock<std::mutex> locker(mtx);
        std::condition_variable cv;
        std::chrono::duration<int, std::ratio<1, 1000>> time_count(1000);

        while (SignalOK()) {
            cv.wait_for(locker, time_count, [] {  return !SignalOK();  });
        }
        return 0;
    }
} // namespace common

#endif