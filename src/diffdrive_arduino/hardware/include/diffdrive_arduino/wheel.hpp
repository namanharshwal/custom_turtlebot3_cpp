/*
 * ==============================================================================
 * Creator & Maintainer: Naman Harshwal (namanharshwal@gmail.com)
 * Owner/Supervisor:     Pradeep Singh Chauhan
 * Workspace:            psnh_turtlebot_ws
 * License:              Apache License 2.0 (c) 2026 Naman Harshwal
 *
 * Description: Custom TurtleBot Burger 2.0 robotics and autonomous navigation stack.
 * ==============================================================================
 */

#ifndef DIFFDRIVE_ARDUINO_WHEEL_HPP
#define DIFFDRIVE_ARDUINO_WHEEL_HPP

#include <string>
#include <cmath>


class Wheel
{
    public:

    std::string name = "";
    int enc = 0;
    double cmd = 0;
    double pos = 0;
    double vel = 0;
    double rads_per_count = 0;

    Wheel() = default;

    Wheel(const std::string &wheel_name, int counts_per_rev)
    {
      setup(wheel_name, counts_per_rev);
    }

    
    void setup(const std::string &wheel_name, int counts_per_rev)
    {
      name = wheel_name;
      rads_per_count = (2*M_PI)/counts_per_rev;
    }

    double calc_enc_angle()
    {
      return enc * rads_per_count;
    }



};


#endif // DIFFDRIVE_ARDUINO_WHEEL_HPP
