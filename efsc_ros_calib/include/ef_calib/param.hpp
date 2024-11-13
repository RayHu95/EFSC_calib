
#ifndef _CALIB_TOOLS_param_HPP_
#define _CALIB_TOOLS_param_HPP_

#include <ros/ros.h>

namespace ef_calib { namespace tools{

    //#define DEBUG_EDS_PARAM_PRINTS 1

    template<typename T>
    T param(const ros::NodeHandle &nh, const std::string &name, const T &defaultValue){
        if (nh.hasParam(name))
        {
            T v;
            nh.param<T>(name, v, defaultValue);
            #ifdef DEBUG_EDS_PARAM_PRINTS
                ROS_INFO_STREAM("Found parameter: " << name << " = " << v);
            #endif

            return v;
        }
        ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
        return defaultValue;
    }
    
    template<typename T>
    std::vector<T> vectorparam(const ros::NodeHandle &nh, const std::string &name, const std::vector<T> &defaultValue){
        if (nh.hasParam(name))
        {
            std::vector<T> v;
            nh.getParam(name, v);
            #ifdef DEBUG_EDS_PARAM_PRINTS
                ROS_INFO_STREAM("Found parameter: " << name << "[0] = " << v[0]<< ", " << name << "[1] = " << v[1]);
            #endif

            return v;
        }
        ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default : zero matrix");
        return defaultValue;
    }

} //tools namespace
} // end namespace

#endif
