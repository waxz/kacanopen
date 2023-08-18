//
// Created by waxz on 23-2-5.
//

#ifndef CMAKE_SUPER_BUILD_STRING_LOGGER_H
#define CMAKE_SUPER_BUILD_STRING_LOGGER_H

//https://stackoverflow.com/questions/3585846/color-text-in-terminal-applications-in-unix
#include <stdio.h>
#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"
#define KRESET "\x1B[0m"
#define MLOGW(__format,...) {printf(KRED "%s:%i @%s:" __format " \n" KRESET, __FILE__, __LINE__, __FUNCTION__ , __VA_ARGS__);}
#define MLOGI(__format,...) {printf(KGRN "%s:%i @%s:" __format " \n" KRESET, __FILE__, __LINE__, __FUNCTION__ , __VA_ARGS__);}
#define MLOGD(__format,...) {printf(KYEL "%s:%i @%s:" __format " \n" KRESET, __FILE__, __LINE__, __FUNCTION__ , __VA_ARGS__);}

#endif //CMAKE_SUPER_BUILD_STRING_LOGGER_H
