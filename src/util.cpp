//
// Created by Spencer Powers on 1/1/22.
//

#include "util.h"
#include "System.h"

f_out f(func_in in_struct){
    float tmp = in_struct.x.at(0);

    f_out ret;
    ret.x.push_back(tmp);
    return ret;
}