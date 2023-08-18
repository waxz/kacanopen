//
// Created by waxz on 4/28/23.
//

#ifndef KACANOPEN_SIGNALS_H
#define KACANOPEN_SIGNALS_H
#include <functional>
#include <signal.h>
#include "functions.h"

namespace common{

    /// \param my_handler\n
    /// usage:\n
    ///std::atomic_bool program_run(true);
    ///auto my_handler = common::fnptr<void(int)>([&](int sig){ std::cout << "get sig " << sig;program_run = false;});
    ///common::set_signal_handler(my_handler);
    void set_signal_handler(void (*my_handler)(int)){

        struct sigaction sigIntHandler;
        sigemptyset(&sigIntHandler.sa_mask);
        sigIntHandler.sa_flags = 0;
        sigIntHandler.sa_handler = my_handler;

        sigaction(SIGINT, &sigIntHandler, nullptr);
        sigaction(SIGTERM, &sigIntHandler, nullptr);
    }

}
#endif //KACANOPEN_SIGNALS_H
