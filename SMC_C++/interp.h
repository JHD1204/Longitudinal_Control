#ifndef INTERP_H
#define INTERP_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <cmath>
#include "ros/ros.h"
#include <vector>

#include "my_typedef.h"

namespace longitude_control
{
class Interp
{
public:
    Interp();
    ~Interp();
    void ClassInit();
    float64 Interp2Thr(float64 v,float64 Ft);
	float64 Interp2Brk(float64 Fb);

private:
	std::vector<float64> brk_map;    // brake map
	float64 brk_des;        // desired brake
	std::vector<std::vector<float64>> thr_map;    // throttle map
	float64 thr_des;        // desired brake
	std::vector<int32> Ft_map_thr;  // force column of throttle map (10*acceleration)
	std::vector<int32> v_map;         // speed row of brake map and throttle map
	std::vector<int32> Fb_map_brk;  // force column of brake map (10*acceleration)

};

}
#endif
