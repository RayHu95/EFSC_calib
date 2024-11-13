#ifndef _TOOLS_CAMERAINFO_HPP_
#define _TOOLS_CAMERAINFO_HPP_

/** Opencv  **/
#include <opencv2/opencv.hpp>

#include <vector>
#include <tools/param.hpp>

 namespace tools {

    struct CameraInfo
    {
        uint16_t height;// input image height
        uint16_t width;// input image width
        std::string distortion_model; // distortion model name
        std::vector<double> D; // distortion coefficients
        std::vector<double> intrinsics; // fx, fy, cx, cy intrinsics
        std::vector<double> R; //3x3 row-major rectification matrix
        /* this matrix specifies the intrinsic (camera) matrix
        of the processed (rectified) image. That is, the left 3x3 portion
        is the normal camera intrinsic matrix for the rectified image. **/
        std::vector<double> P; //3x4 row major 
    };


    inline ::tools::CameraInfo readCameraInfo(ros::NodeHandle &nh, std::string name){
        ::tools::CameraInfo cam;

        //camra_info
        std::vector<int> resolution;
        nh.getParam(name + "/resolution", resolution);
        resolution = ::tools::vectorparam(nh, name + "/resolution", resolution);
        cam.width = resolution[0]; cam.height = resolution[1]; 
        ROS_INFO_STREAM("The camera size of "<< name <<", width[" << cam.width << "], height[" << cam.height <<"]");           

        //distortion_model
        // cam.distortion_model = ::tools::param(nh, name + "/distortion_model", std::string("radtan"));
        std::vector<double> dis_coeffs(5, 0.0);
        cam.D = ::tools::vectorparam(nh, name + "/distortion_coeffs", dis_coeffs);

        //intrinsics
        std::vector<double> intri(4, 0.0); 
        cam.intrinsics = ::tools::vectorparam(nh, name + "/intrinsics", intri);          
        
        // // Projection matrix 
        // XmlRpc::XmlRpcValue Projection;
        // if(nh.hasParam("cam0/P")){
        //     nh.getParam("cam0/P", Projection);
        //     #ifdef DEBUG_PARAM_PRINTS
        //     ROS_INFO_STREAM("Found parameter: cam0/P[0] = " << Projection[0]);
        //     #endif
        //     for (int row=0; row<3; ++row)
        //         for (int col=0; col<4; ++col)
        //             cam.P.push_back(Projection[row][col]);
        // }
        // else{
        //     ROS_WARN_STREAM("Cannot find value for parameter: cam0/P");
        // }

        return cam;
    }

} // tools namespace

#endif // _TOOLS_CAMERA_HPP_
