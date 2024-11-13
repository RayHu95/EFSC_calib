
#ifndef _CALIB_CALIB_CONFIG_HPP_
#define _CALIB_CALIB_CONFIG_HPP_

#include <opencv2/opencv.hpp>
#include <ef_calib/param.hpp>
#include <ros/ros.h>

namespace ef_calib { namespace paramcalib{

    struct Config
    {
        int NumOfFrameToUse_;
        float aspectRatio_;           // The aspect ratio
        bool calibZeroTangentDist_;   // Assume zero tangential distortion
        bool calibFixPrincipalPoint_; // Fix the principal point at the center

        bool fixK1_;                  // fix K1 distortion coefficient
        bool fixK2_;                  // fix K2 distortion coefficient
        bool fixK3_;                  // fix K3 distortion coefficient
        bool fixK4_;                  // fix K4 distortion coefficient
        bool fixK5_;                  // fix K5 distortion coefficient

        int flag_;

        //g2o huber
        float huber_es_;
        float huber_fr_;
        float huber_esTofr_;
        float huber_frToes_;
        bool calib_flag_;
    };

    inline ::ef_calib::paramcalib::Config readCalibConfig(ros::NodeHandle &nh)
    {
        ::ef_calib::paramcalib::Config calib_config;

        calib_config.calibFixPrincipalPoint_    = ::ef_calib::tools::param(nh, "calib/FixAspectRatio", 1);
        calib_config.calibZeroTangentDist_      = ::ef_calib::tools::param(nh, "calib/AssumeZeroTangentialDistortion", 1);
        calib_config.aspectRatio_               = ::ef_calib::tools::param(nh, "calib/FixPrincipalPointAtTheCenter", 1);
        calib_config.fixK1_               = ::ef_calib::tools::param(nh, "calib/FixK1", false);
        calib_config.fixK2_               = ::ef_calib::tools::param(nh, "calib/FixK2", false);
        calib_config.fixK3_               = ::ef_calib::tools::param(nh, "calib/FixK3", true);
        calib_config.fixK4_               = ::ef_calib::tools::param(nh, "calib/FixK4", true);
        calib_config.fixK5_               = ::ef_calib::tools::param(nh, "calib/FixK5", true);
        
        calib_config.flag_ = 0;

        // validate
        if (calib_config.calibFixPrincipalPoint_) calib_config.flag_ |= cv::CALIB_FIX_PRINCIPAL_POINT; //fix cx, cy
        if (calib_config.calibZeroTangentDist_) calib_config.flag_ |= cv::CALIB_ZERO_TANGENT_DIST;
        if (calib_config.aspectRatio_) calib_config.flag_ |= cv::CALIB_FIX_ASPECT_RATIO; // fixed fx/fy
        if (calib_config.fixK1_) calib_config.flag_ |= cv::CALIB_FIX_K1;
        if (calib_config.fixK2_) calib_config.flag_ |= cv::CALIB_FIX_K2;
        if (calib_config.fixK3_) calib_config.flag_ |= cv::CALIB_FIX_K3;
        // if (calib_config.fixK4_) calib_config.flag_ |= cv::CALIB_FIX_K4;
        // if (calib_config.fixK5_) calib_config.flag_ |= cv::CALIB_FIX_K5;
        // calib_config.flag_ |= cv::CALIB_FIX_K6;

        calib_config.huber_es_               = ::ef_calib::tools::param(nh, "g2o/huber_es", 0.1);
        calib_config.huber_fr_               = ::ef_calib::tools::param(nh, "g2o/huber_fr", 0.1);
        calib_config.huber_esTofr_           = ::ef_calib::tools::param(nh, "g2o/huber_estofr", 0.5);
        calib_config.huber_frToes_           = ::ef_calib::tools::param(nh, "g2o/huber_frtoes", 1.0);
        calib_config.calib_flag_             = ::ef_calib::tools::param(nh, "g2o/calib_flag", true);

        return calib_config;
    };

}//Namespace ParamCalib
}//Namespace ef_calib

#endif