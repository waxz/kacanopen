//
// Created by waxz on 22-11-14.
//

#ifndef COMMON_TASK_H
#define COMMON_TASK_H

#include <functional>
#include <vector>
#include <algorithm>

#include "clock_time.h"
#include "suspend.h"

namespace common{


    /*

     # prio
     task with low prio should run forever
     one time task should  have prio lager than 10

     # base tick
     base = 5ms

     tic increase at every call()

     |prio| interval|
     |----|---------|
     |0   | base    |
     |1   | 2*base, each task should be equally distributed to 2 slot|
     |2   | 3*base, each task should be equally distributed to 3 slot|
     ...
     |10   | only compare time stamp|



     */

    struct TaskManager{
        struct Task{
            //  lower Task.prio value means higher priority.
            int prio = 10;
            int slot = 0;
            bool valid = true;
            common::Time time;
            long delay_us = 100;
            long run_time_us = 0;
            long max_run_time_us = 0;
            std::string name;
            long run_counter = 0;


            std::function<bool()> func;
            Task(const char* name_, const std::function<bool()>& func_,long delay_us_ = 100,int prio_ = 10, int slot_ = 0 ):name(name_), prio(prio_),valid(true), time(common::FromUnixNow()),delay_us(delay_us_),func(func_),slot(slot_){
            }
            Task(const Task& rhv){
                name = rhv.name;
                valid = rhv.valid;
                prio = rhv.prio;
                slot = rhv.slot;
                time = rhv.time;
                delay_us = rhv.delay_us;
                func = rhv.func;
                run_time_us = rhv.run_time_us;
                max_run_time_us = rhv.max_run_time_us;
                run_counter = rhv.run_counter;

            }

        };

        void set_tick(long delay_us){

        }
        common::LoopHelper loop_helper;

        std::vector<Task> task_queue;
        std::vector<int> task_queue_index;
        common::Suspend suspend;

        std::vector<int> task_counter;
        uint16 max_prio=10;
        size_t m_tic = 0;

        TaskManager(uint16 max_prio_):task_counter(max_prio_ + 1,0),max_prio(max_prio_){

            std::cout<<"task_counter.size() = " << task_counter.size() << "\n";
        }

        void set_loop(float fps, long accuracy){
            loop_helper.set_fps(fps);
            loop_helper.set_accuracy(accuracy);
        }
        /*
         const std::function<bool()>& func
         return true, keep running at given rate
         return false, run once
         */
        void add_task(const char* name,  const std::function<bool()>& func, uint16 prio , long  delay_us){


            prio = std::max(uint16(0), std::min(prio,max_prio));

            int slot = 0;

            {
                slot = task_counter[prio] % (prio + 1);
                task_counter[prio] = task_counter[prio] + 1;
            }

            task_queue.emplace_back(name, func,delay_us,prio,slot);


            task_queue_index.resize(task_queue.size());
            std::generate(task_queue_index.begin(), task_queue_index.end(), [n = 0]() mutable { return n++; });


//            std::sort(task_queue.begin(),task_queue.end(),[&](auto& v1, auto& v2){
//                return v1.prio < v2.prio || (v1.prio == v2.prio  && v1.delay_us < v2.delay_us);
//            });
            std::sort(task_queue_index.begin(),task_queue_index.end(),[&](auto& v1, auto& v2){
                return task_queue[v1].prio < task_queue[v2].prio || (task_queue[v1].prio == task_queue[v2].prio  && task_queue[v1].delay_us < task_queue[v2].delay_us);
            });


        }

        bool call(){
            loop_helper.start();
            common::Time now = common::FromUnixNow();
            bool need_remove = false;
            for(size_t i = 0 ; i < task_queue_index.size(); i++){
                auto& t = task_queue[task_queue_index[i]];
                int toc = m_tic%(t.prio + 1) ;

                bool toc_run = toc == t.slot;

                long time_diff = common::ToMicroSeconds( now -t.time);


                bool is_lazy_task = t.prio == max_prio;
//                std::cout << "task id: " << i << ", toc: " << toc << ", run:" << toc_run <<  ", prio: " << t.prio <<", slot: " << t.slot << "\n";

//                if (is_lazy_task){
//                    std::cout << "is_lazy_task, time_diff: " << time_diff << ", t.delay_us: " << t.delay_us << "\n";
//                }

                if( (toc_run && !is_lazy_task) || ( is_lazy_task && time_diff > t.delay_us )){
                    auto t1 = common::FromUnixNow();
                    t.valid = t.func();
                    auto t2 = common::FromUnixNow();
                    t.run_time_us = common::ToMicroSeconds(t2-t1);
                    t.max_run_time_us = std::max(t.run_time_us, t.max_run_time_us);
                    t.time = now;
                    t.run_counter++;
                    need_remove = need_remove || t.valid;
                }

            }
//            std::cout << "run task use time: " << common::ToMicroSeconds(common::FromUnixNow() - now) << "\n";



            if(need_remove){

                auto it  = std::remove_if(task_queue.begin(),task_queue.end(),[](auto& e){
                    return !e.valid;
                });

                task_queue.erase(it, task_queue.end());

                task_queue_index.resize(task_queue.size());

                std::generate(task_queue_index.begin(), task_queue_index.end(), [n = 0]() mutable { return n++; });

//                std::sort(task_queue.begin(),task_queue.end(),[&](auto& v1, auto& v2){
//                    return v1.prio < v2.prio || (v1.prio == v2.prio  && v1.delay_us < v2.delay_us);
//                });
                std::sort(task_queue_index.begin(),task_queue_index.end(),[&](auto& v1, auto& v2){
                    return task_queue[v1].prio < task_queue[v2].prio || (task_queue[v1].prio == task_queue[v2].prio  && task_queue[v1].delay_us < task_queue[v2].delay_us);
                });

            }

            m_tic++;
            loop_helper.sleep();

            return true;

        }

        std::stringstream  report( ){
            std::stringstream s;
            s << "\nTaskManager monitor:\n";
            s << "\n|name|run_time_us|max_run_time_us|run_counter|\n";


            for(auto& t: task_queue){
                s << "|" <<t.name << " | " << t.run_time_us << " | " << t.max_run_time_us << " | " << t.run_counter <<  "|\n";
            }

            return s;
        }


    };
}
#endif //COMMON_TASK_H
