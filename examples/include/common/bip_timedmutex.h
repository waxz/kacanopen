//
// Created by waxz on 6/28/22.
//

#ifndef BOOST_INTERPROCESS_TIMEDMUTEX_BIP_TIMEDMUTEX_H
#define BOOST_INTERPROCESS_TIMEDMUTEX_BIP_TIMEDMUTEX_H



#define BOOST_DATE_TIME_NO_LIB
#define BOOST_ALL_NO_LIB
#define BOOST_INTERPROCESS_MSG_QUEUE_CIRCULAR_INDEX
//https://stackoverflow.com/questions/6212138/boostinterprocess-message-queue-performance-rather-slow
//#define BOOST_INTERPROCESS_ENABLE_TIMEOUT_WHEN_LOCKING
//#define BOOST_INTERPROCESS_TIMEOUT_WHEN_LOCKING_DURATION_MS 1000
//https://stackoverflow.com/a/17542184
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>


#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/named_recursive_mutex.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/date_time.hpp>
#include <boost/atomic.hpp>

#include <boost/interprocess/smart_ptr/shared_ptr.hpp>
#include <boost/interprocess/smart_ptr/unique_ptr.hpp>
#include <boost/interprocess/smart_ptr/deleter.hpp>

//boost:interprocess::vector is the implementation of std::vector ready to be used in managed memory segments like shared memory
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/containers/set.hpp>

//basic_string is the implementation of std::basic_string
#include <boost/interprocess/containers/string.hpp>
//boost:interprocess::deque is the implementation of std::deque
#include <boost/interprocess/containers/deque.hpp>
//list is the implementation of std::list
#include <boost/interprocess/containers/list.hpp>

#include <boost/lockfree/spsc_queue.hpp> // ring buffer

#if (BOOST_ATOMIC_INT_LOCK_FREE != 2)
#error "This code requires lock-free boost::atomic<int>"
#endif


#include "clock_time.h"
namespace common{

    struct BipTimedMutex{
        common::Time t;
        boost::interprocess::interprocess_mutex mtx;
        boost::atomic<bool> flag;
        int Dead_ms = 3000;
        const int wait_ms = 10;

        BipTimedMutex(int dead_ms = 3000):Dead_ms(dead_ms){
            t = common::FromUnixNow();
            flag = false;
        }
        void lock(){


            bool lock_done = mtx.timed_lock(boost::posix_time::microsec_clock::universal_time() +
                                            boost::posix_time::milliseconds(wait_ms));
            common::Time now ;

            while(!lock_done ){
                now = common::FromUnixNow();

                if (( common::ToMillSeconds(now -t)  > Dead_ms)
                    and ( ! flag)

                        ){
                    flag = true;

                    std::thread::id this_id = std::this_thread::get_id();
                    std::cout << "dead lock , unlock, thread::id: " << this_id << std::endl;
                    unlock();

                    flag = false;

                }
                lock_done = mtx.timed_lock(boost::posix_time::microsec_clock::universal_time() +
                                           boost::posix_time::milliseconds(wait_ms));


            }
            t = common::FromUnixNow();
        }
        void unlock(){
            t = common::FromUnixNow();

            mtx.unlock();

        }

    };

}

#endif //BOOST_INTERPROCESS_TIMEDMUTEX_BIP_TIMEDMUTEX_H
