//
//  config.h
//  Path_Planning
//
//  Created by Travis on 2018/12/5.
//

#ifndef config_h
#define config_h

enum Lane
{
    left_lane = 0,
    middle_lane,
    right_lane
};


namespace config
{
    int init_lane = middle_lane;
    double normal_vel = 49.5; //mph
    double T = 0.02; // second
    double vision_distance = 60;
}

#endif /* config_h */
