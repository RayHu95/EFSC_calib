#ifndef _main_node_
#define _main_node_

// #include <dv-processing/processing.hpp>
//#include <sensor_msgs/Image.h>

// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>

/** Calib library **/
#include <ef_calib/picker/EventPicker.hpp>
#include <ef_calib/picker/EventSlice.hpp>
#include <ef_calib/picker/PairSlice.hpp>
#include <ef_calib/ParamCalib.hpp>

#include <ef_calib/calib/CameraBase.hpp>
#include <ef_calib/calib/EventCalib.hpp>
#include <ef_calib/calib/PoseCalib_g2o.hpp>

/** tools library **/
#include <tools/tools.hpp>

/** standard & ros library **/
#include <geometry_msgs/PoseStamped.h>
#include <dv-processing/core/core.hpp>
#include <dv_ros_messaging/messaging.hpp>

#include <boost/lockfree/spsc_queue.hpp>
#include <thread>
#include <chrono>

namespace ef_calib_node {
	
	struct Configuration
    {
		::ef_calib::picker::Config picker;
		::ef_calib::picker::pattern_Config pattern;
		::ef_calib::paramcalib::Config calib;

    };

	//using Transformation = kindr::minimal::QuatTransformation;

	enum State {
		Null,
		INITIALIZING,
		RUNNING,
		CHECK,
		OPTIMIZE
	};


	/// ****   Task  class  *****///
	class Task{

	// Dynamic tuning parameters
	struct DataLoaderConfig
	{
		size_t num_events;
		double overlap;
	};
	

	public:
		Task(){};

		/** Camera **/
		::ef_calib::calib::CameraBase::Ptr camera_fr;
		::ef_calib::calib::CameraBase::Ptr camera_es;

        /** Configuration **/
		::tools::CameraInfo eventcam;
		::tools::CameraInfo framecam;
        ef_calib_node::Configuration config;
		DataLoaderConfig data_loader;

        /** Counters **/
        uint64_t es_idx, fr_idx, kf_idx, init_frames;
		uint64_t event_num_k;


		// /** Shared-form pointer for operation **/
		std::shared_ptr<::ef_calib::picker::EventPicker> picker;

		::ef_calib::calib::EventFramePairMap::Ptr pairMap;
		std::shared_ptr<::ef_calib::calib::EventCalib> caliber;

		/** Buffer of events **/
        dv::EventStore events_buff;
		/** Image frame in opencv format **/
        cv::Mat img_frame;

		ef_calib_node::State state(){ return condition; }
		void stateInitial(){ condition = ef_calib_node::State::INITIALIZING; }
		void stateRunning(){ condition = ef_calib_node::State::RUNNING; }
		void stateCheck(){ condition = ef_calib_node::State::CHECK; }
		void stateOptimize(){ condition = ef_calib_node::State::OPTIMIZE; }
		void stateNull()   { condition = ef_calib_node::State::Null; }

	private:
		ef_calib_node::State condition;

	public:
		// EventImageWithMotion
		void drawEventsubpixel(Eigen::MatrixXd &Im, 
							   ::ef_calib::picker::EventPathes::Ptr ePatch, 
							   const Eigen::Ref<const Eigen::Vector2d> &param);
		bool findcircle(::ef_calib::picker::EventPathes::Ptr ePatch, const Eigen::Ref<const Eigen::Vector2d> &param, cv::Mat &image_MC, 
						std::vector<cv::Point3d> &candidate, Eigen::Vector2d &center, double &radius);
		void ImageFromPatch(::ef_calib::picker::EventPathes::Ptr ePatch, cv::Mat &image3, const Eigen::Ref<const Eigen::Vector2d> &param);

		// task.cpp
		void calcBoardCornerPositions(std::vector<cv::Point3f> &corners);
		void publishpose(ros::Publisher &pose_pub, cv::Mat &rvec, cv::Mat &tvec, Eigen::Vector3d &twb, Eigen::Quaterniond &unitQwb);
		void PrintdepthOfImage(std::vector<cv::Point2f> &Points, cv::Mat &rvec, cv::Mat &tvec, ::ef_calib::calib::CameraBase::Ptr camera);
		double computeReprojectionErrors(const std::vector<std::vector<cv::Point3f> > &objectPoints,
                                        const std::vector<std::vector<cv::Point2f> > &imagePoints,
                                        const std::vector<cv::Mat> &rvecs, const std::vector<cv::Mat> &tvecs,
                                        const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
                                        std::vector<float> &perViewErrors);

		// Two View Rcover
		void TranslationBetween(cv::Mat &rvec, cv::Mat &tvec, cv::Mat &rvec2, cv::Mat &tvec2, Eigen::Vector3d &tef, Eigen::Matrix3d &Ref, Eigen::Quaterniond &Qef);
		bool FindFundamental(const std::vector<cv::Point2f> &vPn1i,const std::vector<cv::Point2f> &vPn2i, 
		      				 ::ef_calib::calib::CameraBase::Ptr camera1, ::ef_calib::calib::CameraBase::Ptr camera2, 
							 Eigen::Matrix3f &F21, Eigen::Matrix3f &R21, Eigen::Vector3f &t21);
	};

}

#endif
