#include <calib_node.hpp>

namespace ef_calib_node {

	void drawMatrix(Eigen::MatrixXd &Im, 
					const int x_p2, const int y_p2, const double x_dx, const double y_dy, 
					const int type){
		if( type == 1 || type == 4 || type == 7 ){
			int row = (type-1)%3;
			int col = (type-1)/3;
			Eigen::VectorXd Vect_x(9);
			Eigen::VectorXd Vect_y(9);
			Vect_x <<  -0.5, 0.0, 0.5,   -0.5, 0.0, 0.5,   -0.5, 0.0, 0.5 ;
			Vect_y <<  -0.5,-0.5,-0.5,    0.0, 0.0, 0.0,    0.5, 0.5, 0.5 ;
			Vect_x += row*0.5*Eigen::VectorXd::Ones(9);
			Vect_y += col*0.5*Eigen::VectorXd::Ones(9);
			// std::cout<<Vect_y.transpose()<<std::endl;

			Eigen::VectorXd weight = Eigen::VectorXd::Zero(9);
			double weightSum = 0.0;
			for(int i=0; i<weight.size(); i++){
				weight[i] = (Vect_x[i] - x_dx)*(Vect_x[i] - x_dx) + (Vect_y[i] - y_dy)*(Vect_y[i] - y_dy);
				if( weight[i] <= 0.0002 ) weight[i] = 50000;
				else weight[i] = 1.0/weight[i];
				weightSum += weight[i];
			}
			Im(y_p2 -1 +col, x_p2 -1 +row) += weight[0] / weightSum;
			Im(y_p2 -1 +col, x_p2    +row) += weight[1] / weightSum;
			Im(y_p2 -1 +col, x_p2 +1 +row) += weight[2] / weightSum;
			Im(y_p2    +col, x_p2 -1 +row) += weight[3] / weightSum;
			Im(y_p2    +col, x_p2    +row) += weight[4] / weightSum;
			Im(y_p2    +col, x_p2 +1 +row) += weight[5] / weightSum;
			Im(y_p2 +1 +col, x_p2 -1 +row) += weight[6] / weightSum;
			Im(y_p2 +1 +col, x_p2    +row) += weight[7] / weightSum;
			Im(y_p2 +1 +col, x_p2 +1 +row) += weight[8] / weightSum;

			// for(int i=0; i<weight.size(); i++){
			// 	std::cout<<" "<<weight[i]/weightSum;
			// 	if(i%3 == 2){std::cout<<std::endl;}
			// }
			// std::cout<<std::endl;	 
		}

	}

    void Task::drawEventsubpixel(Eigen::MatrixXd &Im, ::ef_calib::picker::EventPathes::Ptr ePatch, const Eigen::Ref<const Eigen::Vector2d> &param){
		int range = ePatch->maxRange *4;
		// std::cout<<" range = "<<range<<std::endl;
		Eigen::VectorXd xp = ePatch->xp_;
		Eigen::VectorXd yp = ePatch->yp_;
		Eigen::VectorXd tp = ePatch->delta_time_;
		for(int i = 0 ; i< xp.size() ; i++){
			if(xp(i) > 0.0 && yp(i) > 0.0){
				double x_pp = xp(i) - param(0) * tp(i);
				double y_pp = yp(i) - param(1) * tp(i);
				int x_p = std::floor(x_pp);
				int y_p = std::floor(y_pp);
				if( x_p > 0 && y_p > 0 && x_p*2 < range-2 && y_p*2 < range-2 ){ // keep the border of the Im out
					double x_dx = x_pp - x_p;
					double y_dy = y_pp - y_p;
					int type = 0;
					if( x_dx >= 0.0 && x_dx < 0.25 ){
						if( y_dy >= 0.0 && y_dy < 0.25 ) type = 1;
						else if(y_dy < 0.75) 			 type = 2;
						else if(y_dy < 1.0 ) 			 type = 3;
					}
					else if( x_dx < 0.75 ){
						if( y_dy >= 0.0 && y_dy < 0.25 ) type = 4;
						else if(y_dy < 0.75) 			 type = 5;
						else if(y_dy < 1.0 ) 			 type = 6;
					}					
					else if( x_dx  < 1.0 ){
						if( y_dy >= 0.0 && y_dy < 0.25 ) type = 7;
						else if(y_dy < 0.75) 			 type = 8;
						else if(y_dy < 1.0 ) 			 type = 9;
					}
					if( type == 0 ){
						std::cout<<"  *********************    "<<std::endl
								 <<" something is wrong with the std::floor(); draw subpixel fails. exit "<<std::endl
								 <<"  *********************    "<<std::endl;
						break;
					}
					// std::cout<<"Data x,y: "<<x_pp<<", "<<y_pp<<"   dx,dy = "<<x_dx<<", "<<y_dy<<"      type("<<type<<")     "<<std::endl;

					// std::cout<<Im.block<10,10>(y_p*2-2, x_p*2-2)<<std::endl;
					drawMatrix(Im, x_p*2, y_p*2, x_dx, y_dy, type);
					// std::cout<<Im.block<10,10>(y_p*2-2, x_p*2-2)<<std::endl;

				} else { 
					//std::cout<<"Data x,y: "<<x_pp<<", "<<y_pp<<std::endl;
				}
			}
		}
		// std::cout<<Im.block<50, 50>(30, 30)<<std::endl;
		
	}

	bool Task::findcircle(::ef_calib::picker::EventPathes::Ptr ePatch, const Eigen::Ref<const Eigen::Vector2d> &param, cv::Mat &image_MC,
							std::vector<cv::Point3d> &candidate, Eigen::Vector2d &center, double &radius){

		Eigen::MatrixXd Im = Eigen::MatrixXd::Zero(ePatch->maxRange*4, ePatch->maxRange*4);
		this->drawEventsubpixel(Im, ePatch, param);

		
		// cv::circle(image_MC, cv::Point2f(                     ePatch->bias_x*2,                      ePatch->bias_y*2), 1, cv::Vec3b(255, 255, 255));
		// cv::circle(image_MC, cv::Point2f(                     ePatch->bias_x*2, ePatch->maxRange*4 + ePatch->bias_y*2), 1, cv::Vec3b(255, 255, 255));
		// cv::circle(image_MC, cv::Point2f(ePatch->maxRange*4 + ePatch->bias_x*2,                      ePatch->bias_y*2), 1, cv::Vec3b(255, 255, 255));
		// cv::circle(image_MC, cv::Point2f(ePatch->maxRange*4 + ePatch->bias_x*2, ePatch->maxRange*4 + ePatch->bias_y*2), 1, cv::Vec3b(255, 255, 255));

		for(int j = 0; j<ePatch->maxRange*4; j++){
			for(int i = 0; i<ePatch->maxRange*4; i++){
				double Imvalue = Im(j, i);

				// draw image_MC
				int pos_x = i + ePatch->bias_x*2;
				int pos_y = j + ePatch->bias_y*2;
				if( pos_x >0 && pos_y >0 &&  pos_y < this->eventcam.height*2 && pos_x < this->eventcam.width*2){
					// image_MC.at<cv::Vec3b>(pos_y , pos_x) += cv::Vec3b(0, 0, std::floor(Imvalue*30));
					if( Imvalue > 5.0 ){
						candidate.push_back(cv::Point3d(i, j, Imvalue));
						// image_MC.at<cv::Vec3b>(pos_y , pos_x) += cv::Vec3b(255, std::floor(Imvalue*100), 255);
						image_MC.at<cv::Vec3b>(pos_y , pos_x) += cv::Vec3b(0, std::floor(Imvalue*100), 0);
					}
				}
			}
		}

		// std::cout<<"  ** candidate.size() "<<candidate.size()<<std::endl;
		if( candidate.size() < 10 )
			return false;
		this->picker->fitCircleSubpixel(candidate, center, radius);
		// std::cout<<"  ** the center is "<<center(0)*0.5<<", "<<center(1)*0.5<<", radius is "<<radius*0.5<<std::endl;
		cv::circle(image_MC, cv::Point2f(center(0) + ePatch->bias_x*2, center(1) + ePatch->bias_y*2), radius, cv::Vec3b(150, 150, 150));
		
		return true;
	}

    void Task::ImageFromPatch(::ef_calib::picker::EventPathes::Ptr ePatch, cv::Mat &image3, const Eigen::Ref<const Eigen::Vector2d> &param){
		//Normal pixel resolution Image with color
		// cv::Mat	image3 = cv::Mat(this->eventcam.height, this->eventcam.width, CV_8UC3, cv::Vec3b(0, 0, 0));
		Eigen::VectorXd xp = ePatch->xp_;
		Eigen::VectorXd yp = ePatch->yp_;
		Eigen::VectorXd tp = ePatch->delta_time_;
		for(int i = 0 ; i< xp.size() ; i++){
			int x_p = std::round(xp(i) - param(0) * tp(i));
			int y_p = std::round(yp(i) - param(1) * tp(i));
			if( x_p>0 && x_p< ePatch->maxRange *2 && y_p>0 && y_p< ePatch->maxRange *2 )
				image3.at<cv::Vec3b>(y_p+ ePatch->bias_y, x_p + ePatch->bias_x) += cv::Vec3b(0, 0, 15);
		}
		// cv::Mat	image3 = cv::Mat(range *2, range*2, CV_8UC1, cv::Scalar(0));
		// double min3, max3;
		// cv::minMaxLoc(image_MC_before, &min3, &max3);
		// image3 = 300 * (image_MC_before - min3)/(max3-min3);
	}
}//Namespace ef_calib_node