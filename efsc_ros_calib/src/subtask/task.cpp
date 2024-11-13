#include <calib_node.hpp>

namespace ef_calib_node {

	void Task::publishpose(ros::Publisher &pose_pub, cv::Mat &rvec, cv::Mat &tvec, Eigen::Vector3d &twb, Eigen::Quaterniond &unitQwb)
	{
		cv::Mat cvRsw;
		cv::Rodrigues(rvec, cvRsw);
		Eigen::Matrix3d Rsw;
		Eigen::Vector3d tsw;
		cv::cv2eigen(cvRsw, Rsw);
		cv::cv2eigen(tvec, tsw);
		Eigen::Quaterniond Qsw(Rsw);
		Qsw.normalize();

		// std::cout<<"Rotation:"<<Qsw<<std::endl;

		// set poses
		// Eigen::Quaterniond Qsb(1, 0, 0, 0);
		// Eigen::Vector3d    tsb(0, 0, 0);

		//      twb = Qsw.conjugate() * (tsb - tsw);
		//  unitQwb = Qsw.conjugate() * Qsb;
			 twb = tsw;
		 unitQwb = Qsw;
	
		geometry_msgs::PoseStampedPtr ps_ptr(new geometry_msgs::PoseStamped());
		// This only is suitable for visualization, not the real time.
		ps_ptr->header.stamp = ros::Time::now(); 
		ps_ptr->header.frame_id = "world";
		ps_ptr->pose.position.x = tsw(0);
		ps_ptr->pose.position.y = tsw(1);
		ps_ptr->pose.position.z = tsw(2);
		// tf2::Quaternion q;
		// q.setRPY(0, 0, 0.1);	
		ps_ptr->pose.orientation.x = Qsw.x();
		ps_ptr->pose.orientation.y = Qsw.y();
		ps_ptr->pose.orientation.z = Qsw.z();
		ps_ptr->pose.orientation.w = Qsw.w();
		pose_pub.publish(ps_ptr);
	}

	double Task::computeReprojectionErrors(const std::vector<std::vector<cv::Point3f> > &objectPoints,
                                        const std::vector<std::vector<cv::Point2f> > &imagePoints,
                                        const std::vector<cv::Mat> &rvecs, const std::vector<cv::Mat> &tvecs,
                                        const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
                                        std::vector<float> &perViewErrors) {
		std::vector<cv::Point2f> imagePoints2;
		size_t totalPoints = 0;
		double totalErr = 0, err;
		perViewErrors.resize(objectPoints.size());

		for (size_t i = 0; i < objectPoints.size(); ++i) {
			projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
			
			err = norm(imagePoints[i], imagePoints2, cv::NORM_L2);

			size_t n = objectPoints[i].size();
			perViewErrors[i] = (float) std::sqrt(err * err / n);
			totalErr += err * err;
			totalPoints += n;
		}

		return std::sqrt(totalErr / totalPoints);
	}
    
    void Task::calcBoardCornerPositions(std::vector<cv::Point3f> &corners) {
		corners.clear();
		//isAsymmetric_ 7*2 corners=[ (0,0),(2p,0), (p,p),(3p,p), (0,2p),(2p,2p)......]
		if (this->config.pattern.isAsymmetric) {
			for (int i = 0; i < this->config.pattern.rows; i++)
				for (int j = 0; j < this->config.pattern.cols; j++)
					corners.emplace_back( i * this->config.pattern.squareSize, (2 * j + i % 2) * this->config.pattern.squareSize, 0);
					//changed xy direction into yx direction.
		} else {
			for (int i = 0; i < this->config.pattern.rows; ++i)
				for (int j = 0; j < this->config.pattern.cols; ++j)
					corners.emplace_back( j * this->config.pattern.squareSize, i * this->config.pattern.squareSize, 0);
		}
	}

	void Task::PrintdepthOfImage(std::vector<cv::Point2f> &Points, cv::Mat &rvec, cv::Mat &tvec, ::ef_calib::calib::CameraBase::Ptr camera)
	{
		std::cout<<" ***** empty function : task.PrintdepthOfImage ***** "<<std::endl;
		// // from world to the camera optical center 
		// cv::Mat cvRsw;
		// cv::Rodrigues(rvec, cvRsw);
		// Eigen::Matrix3d Rsw;
		// cv::cv2eigen(cvRsw, Rsw);
		// Eigen::Quaterniond Qsw(Rsw);
		// Qsw.normalize();

		// Eigen::Vector3d tsw;
		// cv::cv2eigen(tvec, tsw);

		// // from the camera body to world
		// Eigen::Vector3d        twb = Qsw.conjugate() * (- tsw);
		// Eigen::Quaterniond unitQwb = Qsw.conjugate() ;
		// // std::cout<<"** [Project] point "<<std::endl<<twb<<std::endl;

		// const Eigen::VectorXd &distCoeffs = camera->distCoeffs();
		// if (distCoeffs.size() != 5) {
		// 	throw std::logic_error("Sorry! Only radial distortion considered for now.");
		// }

		// const Eigen::Matrix3d &K_ = camera->K();
		// Eigen::Vector4d radialDistortion(distCoeffs(0), distCoeffs(1), distCoeffs(4), 0);
		// Eigen::VectorXd inverseRadial = camera->inverseRadialDistortion(radialDistortion);
		// const double k1 = inverseRadial[0];
		// const double k2 = inverseRadial[1];
		// const double k3 = inverseRadial[2];
		// const double k4 = inverseRadial[3];
		// const double k5 = inverseRadial[4];

		// double depth_sum = 0;
		// for(int k = 0; k < Points.size(); k++ ){

		// 	cv::Point2f p = Points[k];
		// 	Eigen::Vector3d pp = camera->pixel2cam(p);

		// 	double Qx = 2. * unitQwb.x();
		// 	double Qy = 2. * unitQwb.y();
		// 	double Qz = 2. * unitQwb.z();
		// 	double Qwx = Qx * unitQwb.w(); //Name: Q + Q(wxy) + t(xyz)
		// 	double Qwy = Qy * unitQwb.w();
		// 	double Qxx = Qx * unitQwb.x();
		// 	double Qxz = Qz * unitQwb.x();
		// 	double Qyy = Qy * unitQwb.y();
		// 	double Qyz = Qz * unitQwb.y();
		// 	Eigen::Vector3d Rws_r2(Qxz - Qwy, Qyz + Qwx, 1.0 - (Qxx + Qyy));
		// 	// double depth = -twb[2] / (Rws_r2.dot(pp));

		// 	// Eigen::Vector3d op;
		// 	// op = unitQwb * (pp*depth) + twb;
			
		// 	// std::cout<<"** [Project] point "<<k<<" ("<<Points[k].x<<", "<<Points[k].y<<"),   after K and discoff: ("<<std::setprecision(3)<<pp.transpose()<<"),   after R,t = ("<<op.transpose()<<"), depth = "<<std::setprecision(6)<<depth<<std::endl;
			

		// 	double px = (p.x - K_(0, 2)) / K_(0, 0);
		// 	double py = (p.y - K_(1, 2)) / K_(1, 1);
		// 	double xx = px * px;
        //     double yy = py * py;
        //     double r2 = xx + yy;
        //     double r4 = r2 * r2;
        //     double r6 = r4 * r2;
        //     double r8 = r6 * r2;
        //     double r10 = r8 * r2;
        //    	double r_coeff = 1.0 + k1 * r2 + k2 * r4 + k3 * r6 + k4 * r8 + k5 * r10;
        //     px *= r_coeff;
        //     py *= r_coeff;
		// 	Eigen::Vector3d pp2( px, py, 1.0 );
		// 	double depth2 = -twb[2] / (Rws_r2.dot(pp2));
		// 	Eigen::Vector3d op2;
		// 	op2 = unitQwb * (pp2*depth2) + twb;
		// 	// std::cout<<"** point "<<k<<" ("<<Points[k].x<<", "<<Points[k].y<<"),   after K and discoff: ("<<std::setprecision(3)<<pp2.transpose()<<"),   after R,t = ("<<op2.transpose()<<"), depth = "<<std::setprecision(6)<<depth2<<std::endl;
		// 	// std::cout<<"** [Project] point "<<k<<" ("<<Points[k].x<<", "<<Points[k].y<<"), depth = "<<depth2<<std::endl;
		// 	depth_sum += depth2;
		// }
		// std::cout<<"** [Project] average depth = "<<depth_sum/(Points.size()*1.0)<<std::endl;
	}


} //Namespace ef_calib_node