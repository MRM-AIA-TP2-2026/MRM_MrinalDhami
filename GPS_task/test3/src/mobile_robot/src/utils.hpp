#ifndef utils_hpp
#define utils_hpp

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath>
#include <iostream>


// To make the code wait in few areas to debuggg better
#include <thread>
#include <chrono>

#define DEG2RAD(deg) ((deg) * M_PI / 180.0)
#define RAD2DEG(rad) ((rad) * 180.0 / M_PI)
#define EARTH_RADIUS 6371000.0 // meters

// Compute initial bearing using Haversine formula
double for_azimuth(double lat1, double lon1, double lat2, double lon2) {
    lat1 = DEG2RAD(lat1);
    lon1 = DEG2RAD(lon1);
    lat2 = DEG2RAD(lat2);
    lon2 = DEG2RAD(lon2);

    double dLon = lon2 - lon1;
    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);

    RCLCPP_INFO(rclcpp::get_logger("imu_to_gps"), " bearing has beem calculated %f",atan2(y,x)); // Bearing in radians

    
    return  M_PI + M_PI_2 - atan2(y, x);
}

// Compute distance using Haversine formula
double haversine_distance(double lat1, double lon1, double lat2, double lon2) {
    lat1 = DEG2RAD(lat1);
    lon1 = DEG2RAD(lon1);
    lat2 = DEG2RAD(lat2);
    lon2 = DEG2RAD(lon2);

    double dLat = lat2 - lat1;
    double dLon = lon2 - lon1;
    double a = sin(dLat/2) * sin(dLat/2) + cos(lat1) * cos(lat2) * sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return EARTH_RADIUS * c; // Distance in meters
}

#endif