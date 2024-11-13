
#ifndef _CALIB_PICKER_CONFIG_HPP_
#define _CALIB_PICKER_CONFIG_HPP_

#include <stdint.h>
#include <ros/ros.h>
#include <ef_calib/param.hpp>

namespace ef_calib { namespace picker{

    struct Config
    {
        uint16_t motionTimeStep;
        uint16_t frameEventNumThreshold;
        double dbscan_eps;           // pixel unit
        int dbscan_startMinSample;
        int clusterMinSample;        // related with total event num.
        int knn_num;                 // k nearest neighbor
        bool fitCircle = false;
    };

    inline ::ef_calib::picker::Config readPickerConfig(ros::NodeHandle &nh)
    {
        ::ef_calib::picker::Config picker_config;

        picker_config.motionTimeStep         = ::ef_calib::tools::param(nh, "picker/motionTimeStep", 500);
        picker_config.frameEventNumThreshold = ::ef_calib::tools::param(nh, "picker/frameEventNumThreshold", 8000);

        //detect
        picker_config.dbscan_eps            = ::ef_calib::tools::param(nh, "picker/detect/dbscan_eps", 4);
        picker_config.dbscan_startMinSample = ::ef_calib::tools::param(nh, "picker/detect/dbscan_startMinSample", 2);
        picker_config.clusterMinSample      = ::ef_calib::tools::param(nh, "picker/detect/clusterMinSample", 5);
        picker_config.knn_num               = ::ef_calib::tools::param(nh, "picker/detect/knn_num", 3);
        
        return picker_config;
    };

    // leave it for now
    struct pattern_Config
    {
        bool isAsymmetric;
        int rows, cols;
        double circleRadius          = 1.75;
        double squareSize            = 5.5;
        double RThreshold            = 0.0;
    };

    inline ::ef_calib::picker::pattern_Config readPatternConfig(ros::NodeHandle &nh)
    {
        ::ef_calib::picker::pattern_Config pattern_config;

        pattern_config.isAsymmetric     = ::ef_calib::tools::param(nh, "pattern/isAsymmetric", true);
        pattern_config.circleRadius     = ::ef_calib::tools::param(nh, "pattern/circleRadius", 0.0);
        pattern_config.squareSize       = ::ef_calib::tools::param(nh, "pattern/squareSize", 0.0);

        std::vector<int> grid(2, 0);
        grid = ::ef_calib::tools::vectorparam(nh, "pattern/grid", grid);
        pattern_config.rows = grid[0]; pattern_config.cols = grid[1];
        ROS_INFO_STREAM("The pattern grid, rows[" << pattern_config.rows << "], cols[" << pattern_config.cols <<"]");   
        ROS_INFO_STREAM("                  circleRadius["<< pattern_config.circleRadius <<"], squareSize["<< pattern_config.squareSize <<"]");
        
        return pattern_config;
    };

} //picker namespace
} // end namespace

#endif
