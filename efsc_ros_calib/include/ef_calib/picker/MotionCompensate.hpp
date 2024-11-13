#ifndef ef_calib_MotionCompensate_HPP
#define ef_calib_MotionCompensate_HPP

#include <ceres/ceres.h>

/** Opencv  **/
#include <opencv2/opencv.hpp>

namespace ef_calib {
    namespace picker {

    class MotionLine : public ceres::SizedCostFunction<1, 2> {
    public:
        MotionLine(const double x, const double y) : x_(x), y_(y) {};

        virtual ~MotionLine() {}
        virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
            const double a = parameters[0][0];
            const double b = parameters[0][1];

            std::cout<<"** [MotionLine]  a,b = "<<a<<", "<<b<<"     ";
            residuals[0] = a * x_ + b - y_;
            std::cout<<" residuals = "<<residuals[0]<<std::endl;

            if(!jacobians) return true;
            double* jacobian = jacobians[0];
            if(!jacobian) return true;

            jacobian[0] = x_;
            jacobian[1] = 1;
            std::cout<<"** [MotionLine] jacobians = "<<jacobian[0]<<", "<<jacobian[1]<<std::endl;
            return true;
        }

    private:
        const double x_;
        const double y_;

    };

    void sobel_Eigen(Eigen::MatrixXd &I, Eigen::MatrixXd &I_sobel, int flag, int max)
    {
        if(flag == 1){
            Eigen::MatrixXd sobel_x(3,3);
            sobel_x << -1, 0, 1, 
                       -2, 0, 2, 
                       -1, 0, 1;
            for(int i = 1; i < max-1; i++){
                for(int j = 1; j < max-1; j++){
                    Eigen::Matrix3d sobel_value = sobel_x * I.block(i-1,j-1,3,3);
                    // std::cout<<" i,j = "<<i<<", "<<j<<std::endl<<sobel_value<<std::endl;
                    I_sobel(i,j) = sobel_value.sum() / 9 ;
                }
            }
        }
        else{
            Eigen::MatrixXd sobel_y(3,3);
            sobel_y << -1, -2, -1, 
                        0,  0,  0, 
                        1,  2,  1;
            for(int i = 1; i < max-1; i++){
                for(int j = 1; j < max-1; j++){
                    Eigen::Matrix3d sobel_value = I.block(i-1,j-1,3,3) * sobel_y;
                    I_sobel(i,j) = sobel_value.sum() / 9 ;
                }
            }
        }
    }

    void diff_Eigen(Eigen::MatrixXd &I, Eigen::MatrixXd &Ix, Eigen::MatrixXd &Iy, int max)
    {
        Ix.block(0,0,max,max-1) = I.block(0,1,max,max-1) - I.block(0,0,max,max-1);
        Iy.block(0,0,max-1,max) = I.block(1,0,max-1,max) - I.block(0,0,max-1,max);
        // for(int j = 1 ; j<max-1 ; j++){
        //     for(int i = 1 ; i<max-1 ; i++){
        //         Ix(j,i) = I(j, i+1) - I(j, i);
        //         Iy(j,i) = I(j+1, i) - I(j, i);
        //     }
        // }
    }

// #define debug_flag
// #define debug_flag2

    class MotionCom : public ceres::SizedCostFunction<1, 2> {
    public:
        MotionCom(const Eigen::Ref<const Eigen::VectorXd> &x, const Eigen::Ref<const Eigen::VectorXd> &y, const Eigen::Ref<const Eigen::VectorXd> &t) 
        : x_(x), y_(y), t_(t){};

        virtual ~MotionCom() {}
        virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {

            const double vx = parameters[0][0];
            const double vy = parameters[0][1];

                // std::cout<<std::endl;
                #ifdef debug_flag
                std::cout<<"** [MotionCom]  v = "<<vx<<", "<<vy<<"     ";
                #endif

            // Eigen::VectorXd xp_;
            // Eigen::VectorXd yp_;
            // xp_.resize((x_).size());
            // yp_.resize((y_).size());
            // Eigen::VectorXd tp_;
            // tp_.resize((*t_).size());
            // tp_ = (*t_)/1000;
            uint16_t e_size = 0; // valid size for events
            
            Eigen::MatrixXd Im = Eigen::MatrixXd::Zero(23 *2, 23 *2);
            // cv::Mat image = cv::Mat(23 *2, 23 *2, CV_8UC1, cv::Scalar(0));
            for(int i=0; i< x_.size(); i++ )
            {
                int x_new, y_new;
                x_new = std::floor((x_)(i) - vx * (t_)(i)) ;
                y_new = std::floor((y_)(i) - vy * (t_)(i)) ;
                // std::cout<<"x,y = "<<((x_)(i) - vx * (t_)(i))<<", "<<((y_)(i) - vy * (t_)(i)) <<" x_new = "<<x_new<<", y_new = "<<y_new<<std::endl;
                if(x_new > 0 && x_new < 23 *2 && y_new > 0 && y_new < 23 *2 ){
                    // Eigen::MatrixXd weight = Eigen::MatrixXd::Zero(3, 3);
                    Im(y_new, x_new) += 5.0;
                    // image.at<uchar>(y_new, x_new) += 1;
                    // xp_(e_size) = x_new;
                    // yp_(e_size) = y_new;
                    e_size++;
                }
            }
            double re = Im.squaredNorm();
                #ifdef debug_flag
                std::cout<<" e_size = "<<e_size<<"     ";
                // std::cout<<" Np = "<<cv::countNonZero(image)<<"    ";
                // std::cout<<"Im = "<<std::endl<<Im.block<25,25>(10,10)<<std::endl;
                std::cout<<" residuals = "<<re<< "         ";
                #endif
            if( re <= 0.00000001 ) re = 0.00000001;
            residuals[0] = 1/re;

            // cv::Mat blur_image = cv::Mat(23 *2, 23 *2, CV_8UC1, cv::Scalar(0));
            // // cv::threshold(image, image, 3, 255, 3); 
            // // cv::GaussianBlur(image, blur_image, cv::Size(5, 5), 1);
            // cv::medianBlur(image, blur_image, 3);
            // double re3 = 0;
            // for(int i=0; i<23 *2;i++ )
            //     for(int j=0; j<23 *2;j++ )
            //         re3 += blur_image.at<uchar>(j, i) * blur_image.at<uchar>(j, i) ;
            // std::cout<<" re3 = "<<re3;
                #ifdef debug_flag
                std::cout<<std::endl;
                #endif
            if(!jacobians) return true;
            double* jacobian = jacobians[0];
            if(!jacobian) return true;

            //sobel_gradient
            // cv::Mat Ix = sobel_gradient(image, 1, 23, 23);
            // cv::Mat Iy = sobel_gradient(image, 2, 23, 23);
            
            //diff_Eigen
            Eigen::MatrixXd	Imx = Eigen::MatrixXd::Zero(32*2, 32*2);
            Eigen::MatrixXd	Imy = Eigen::MatrixXd::Zero(32*2, 32*2);
            ::ef_calib::picker::diff_Eigen(Im, Imx, Imy, 23 * 2);
            // std::cout<<"Im = "<<Im<<std::endl;
            // std::cout<<"Imx = "<<Imx<<std::endl;
            // std::cout<<"Im = "<<std::endl<<Im.block<17,17>(18,18)<<" sum = "<<Im.block<17,17>(18,18).sum()<<std::endl;
            // std::cout<<"Imx = "<<std::endl<<Imx.block<25,25>(10,10)<<" sum = "<<Imx.sum()<<std::endl;
            // std::cout<<"Imy = "<<std::endl<<Imy.block<25,25>(10,10)<<" sum = "<<Imy.sum()<<std::endl;
            // std::cout<<"Im * Imx= "<<std::endl<< ( Im.array() * Imx.array() ).block<26,26>(10,10)<<std::endl;
            
            // sobel_Eigen
            // Eigen::MatrixXd	Imxsobel = Eigen::MatrixXd::Zero(32*2, 32*2);
            // Eigen::MatrixXd	Imysobel = Eigen::MatrixXd::Zero(32*2, 32*2);
            // sobel_Eigen(Im, Imxsobel, 2, 23 * 2);
            // sobel_Eigen(Im, Imysobel, 1, 23 * 2);
            // std::cout<<"** [MotionCom] jacobians_sobel = "<<( Im.array() * Imxsobel.array() ).sum()<<", "<<( Im.array() * Imysobel.array() ).sum()<<std::endl;

            double J1 = 0.0;
            double J2 = 0.0;

            // for(int e_itr = 0; e_itr < e_size ; e_itr++ ){
            //     int x_new = std::round(xp_(e_itr)) ;
            //     int y_new = std::round(yp_(e_itr)) ;
            //     J1 -= Im(y_new, x_new) * Imx(y_new, x_new) * (*t_)(e_itr);
            //     J2 -= Im(y_new, x_new) * Imy(y_new, x_new) * (*t_)(e_itr);
            //     // J1 += Imx(y_new, x_new) * (*t_)(e_itr);
            //     // J2 += Imy(y_new, x_new) * (*t_)(e_itr);
            // }
            // Eigen::MatrixXd	ImImx = Eigen::MatrixXd::Zero(32*2, 32*2);
            // Eigen::MatrixXd	ImImy = Eigen::MatrixXd::Zero(32*2, 32*2);
            // ImImx = Im.array() * Imx.array();
            // ImImy = Im.array() * Imx.array();
            // std::cout<<"ImImx = "<<std::endl<<ImImx.block<17,17>(18,18)<<" sum = "<<ImImx.block<17,17>(18,18).sum()<<std::endl;

            // Eigen::MatrixXd	ImImx2 = Eigen::MatrixXd::Zero(32*2, 32*2);
            // Eigen::MatrixXd	ImImy2 = Eigen::MatrixXd::Zero(32*2, 32*2);
            double ImImxSum = 0;
            double ImImySum = 0;
            for(int j = 1 ; j<32*2-1 ; j++){
                for(int i = 1 ; i<32*2-1 ; i++){
                    double valuex = Im(j, i) * Imx(j, i);
                    double valuey = Im(j, i) * Imy(j, i);
                    // std::cout<<" "<<valuex;
                    // ImImx2(j, i) = valuex;
                    // ImImy2(j, i) = valuey;
                    if(!std::isnan(valuex) && !std::isnan(valuey)){
                        ImImxSum += valuex;
                        ImImySum += valuey;
                    } 
                }
                // std::cout<<std::endl;
            }
            // std::cout<<"ImImx2 = "<<std::endl<<ImImx2.block<17,17>(18,18)<<" sum = "<<ImImx2.block<17,17>(18,18).sum()<<std::endl;

            // J1 = ( Im.array() * Imxsobel.array() ).sum();
            // J2 = ( Im.array() * Imysobel.array() ).sum();
            
            J1 = ImImxSum / re;
            // J2 = ImImySum / re;
            J2 = 0.0;
                #ifdef debug_flag
                std::cout<<"** [MotionCom] jacobians = "<<J1<<", "<<J2<<"             ";
                // std::cout<<" Ixsum = "<<ImImxSum<<", Iysum ="<< ImImySum<< ", re = "<<re;
                #endif
            // cv::Mat sx = (cv::Mat_<int>(3,3)<<0, 0, 0, 0, -1, 1, 0, 0, 0);
            // cv::Mat	Ix = cv::Mat(32*2, 32*2, CV_32FC1, cv::Scalar::all(0));
            // //cv::Point(-1, -1) means the Achter is located at the center
            // cv::filter2D(image, Ix, CV_32FC1, sx, cv::Point(-1, -1));
            // cv::Mat sy = (cv::Mat_<int>(3,3)<<0, 0, 0, 0, -1, 0, 0, 1, 0);
            // cv::Mat	Iy = cv::Mat(32*2, 32*2, CV_32FC1, cv::Scalar::all(0));
            // //cv::Point(-1, -1) means the Achter is located at the center
            // cv::filter2D(image, Iy, CV_32FC1, sy, cv::Point(-1, -1)); 
            // double J3, J4;
            // for(int j=0; j<23 *2;j++ ){
            //     for(int i=0; i<23 *2;i++ ){
            //         J3 += Ix.at<uchar>(j, i) * image.at<uchar>(j, i) ;
            //         J4 += Iy.at<uchar>(j, i) * image.at<uchar>(j, i) ;
            //         std::cout<< Ix.at<uchar>(j, i)<<" ";
            //     }
            //     std::cout<<std::endl;
            // }
            // std::cout<<" J3,J4 = "<<J3<<", "<<J4;
                #ifdef debug_flag
                std::cout<<std::endl;
                #endif
            jacobian[0] = - 2*J1  ;
            jacobian[1] = - 2*J2  ;
            
            return true;
        }

    private:
        const Eigen::VectorXd x_;
        const Eigen::VectorXd y_;
        const Eigen::VectorXd t_;
    };


    class MotionComX : public ceres::SizedCostFunction<1, 1> {
    public:
        MotionComX(const Eigen::Ref<const Eigen::VectorXd> &x, const Eigen::Ref<const Eigen::VectorXd> &y, const Eigen::Ref<const Eigen::VectorXd> &t) : x_(x), y_(y), t_(t){};

        virtual ~MotionComX() {}
        virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {

            const double vx = parameters[0][0];

                #ifdef debug_flag
                std::cout<<"** [MotionCom]  vx = "<<vx<<"     ";
                #endif

            uint16_t e_size = 0; // valid size for events
            
            Eigen::MatrixXd Im = Eigen::MatrixXd::Zero(23 *2, 23 *2);
            // cv::Mat image = cv::Mat(23 *2, 23 *2, CV_8UC1, cv::Scalar(0));
            for(int i=0; i< x_.size(); i++ )
            {
                int x_new, y_new;
                x_new = std::floor((x_)(i) - vx * (t_)(i)) ;
                y_new = std::floor((y_)(i) - 0  * (t_)(i)) ;
                // std::cout<<"x,y = "<<((x_)(i) - vx * (t_)(i))<<", "<<((y_)(i) - vy * (t_)(i)) <<" x_new = "<<x_new<<", y_new = "<<y_new<<std::endl;
                if(x_new > 0 && x_new < 23 *2 && y_new > 0 && y_new < 23 *2 ){
                    Im(y_new, x_new) += 5.0;
                    // image.at<uchar>(y_new, x_new) += 1;
                    e_size++;
                }
            }
            double re = Im.squaredNorm();
                #ifdef debug_flag
                std::cout<<" e_size = "<<e_size<<"     ";
                // std::cout<<" Np = "<<cv::countNonZero(image)<<"    ";
                // std::cout<<"Im = "<<std::endl<<Im.block<25,25>(10,10)<<std::endl;
                std::cout<<" residuals = "<<re<< "         ";
                #endif
            if( re <= 0.00000001 ) re = 0.00000001;
            residuals[0] = 1/re;

                #ifdef debug_flag
                std::cout<<std::endl;
                #endif
            if(!jacobians) return true;
            double* jacobian = jacobians[0];
            if(!jacobian) return true;

            
            //diff_Eigen
            Eigen::MatrixXd	Imx = Eigen::MatrixXd::Zero(32*2, 32*2);
            Eigen::MatrixXd	Imy = Eigen::MatrixXd::Zero(32*2, 32*2);
            ::ef_calib::picker::diff_Eigen(Im, Imx, Imy, 23 * 2);
            // std::cout<<"Imx = "<<std::endl<<Imx.block<25,25>(10,10)<<" sum = "<<Imx.sum()<<std::endl;
            // std::cout<<"Imy = "<<std::endl<<Imy.block<25,25>(10,10)<<" sum = "<<Imy.sum()<<std::endl;

            double J1 = 0.0;

            double ImImxSum = 0;
            for(int j = 1 ; j<32*2-1 ; j++){
                for(int i = 1 ; i<32*2-1 ; i++){
                    double valuex = Im(j, i) * Imx(j, i);
                    // std::cout<<" "<<valuex;
                    // ImImx2(j, i) = valuex;
                    if(!std::isnan(valuex)){
                        ImImxSum += valuex;
                    } 
                }
                // std::cout<<std::endl;
            }

            J1 = ImImxSum / re;
                #ifdef debug_flag
                std::cout<<"** [MotionCom] jacobians = "<<J1<<"             ";
                // std::cout<<" Ixsum = "<<ImImxSum<<", Iysum ="<< ImImySum<< ", re = "<<re;
                std::cout<<std::endl;
                #endif
            jacobian[0] = - 2*J1  ;
            
            return true;
        }

    private:
        const Eigen::VectorXd x_;
        const Eigen::VectorXd y_;
        const Eigen::VectorXd t_;
    };

    class MotionComY : public ceres::SizedCostFunction<1, 1> {
    public:
        MotionComY(const Eigen::Ref<const Eigen::VectorXd> &x, const Eigen::Ref<const Eigen::VectorXd> &y, const Eigen::Ref<const Eigen::VectorXd> &t) : x_(x), y_(y), t_(t){};

        virtual ~MotionComY() {}
        virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {

            const double vy = parameters[0][0];

                #ifdef debug_flag
                std::cout<<"** [MotionCom]  vy = "<<vy<<"     ";
                #endif

            uint16_t e_size = 0; // valid size for events
            
            Eigen::MatrixXd Im = Eigen::MatrixXd::Zero(23 *2, 23 *2);
            // cv::Mat image = cv::Mat(23 *2, 23 *2, CV_8UC1, cv::Scalar(0));
            for(int i=0; i< x_.size(); i++ )
            {
                int x_new, y_new;
                x_new = std::floor((x_)(i) -  0 * (t_)(i)) ;
                y_new = std::floor((y_)(i) - vy * (t_)(i)) ;
                // std::cout<<"x,y = "<<((x_)(i) - vx * (t_)(i))<<", "<<((y_)(i) - vy * (t_)(i)) <<" x_new = "<<x_new<<", y_new = "<<y_new<<std::endl;
                if(x_new > 0 && x_new < 23 *2 && y_new > 0 && y_new < 23 *2 ){
                    Im(y_new, x_new) += 0.8;
                    // image.at<uchar>(y_new, x_new) += 1;
                    e_size++;
                }
            }
            double re = Im.squaredNorm();
                #ifdef debug_flag
                std::cout<<" e_size = "<<e_size<<"     ";
                // std::cout<<" Np = "<<cv::countNonZero(image)<<"    ";
                // std::cout<<"Im = "<<std::endl<<Im.block<25,25>(10,10)<<std::endl;
                std::cout<<" residuals = "<<re<< "         ";
                #endif
            if( re <= 0.00000001 ) re = 0.00000001;
            residuals[0] = 1/re;

                #ifdef debug_flag
                std::cout<<std::endl;
                #endif
            if(!jacobians) return true;
            double* jacobian = jacobians[0];
            if(!jacobian) return true;

            
            //diff_Eigen
            Eigen::MatrixXd	Imx = Eigen::MatrixXd::Zero(32*2, 32*2);
            Eigen::MatrixXd	Imy = Eigen::MatrixXd::Zero(32*2, 32*2);
            ::ef_calib::picker::diff_Eigen(Im, Imx, Imy, 23 * 2);
            // std::cout<<"Imx = "<<std::endl<<Imx.block<25,25>(10,10)<<" sum = "<<Imx.sum()<<std::endl;
            // std::cout<<"Imy = "<<std::endl<<Imy.block<25,25>(10,10)<<" sum = "<<Imy.sum()<<std::endl;

            double J2 = 0.0;

            double ImImySum = 0;
            for(int j = 1 ; j<32*2-1 ; j++){
                for(int i = 1 ; i<32*2-1 ; i++){
                    double valuey = Im(j, i) * Imy(j, i);
                    // std::cout<<" "<<valuex;
                    // ImImx2(j, i) = valuex;
                    if(!std::isnan(valuey)){
                        ImImySum += valuey;
                    } 
                }
                // std::cout<<std::endl;
            }

            J2 = ImImySum / re;
                #ifdef debug_flag
                std::cout<<"** [MotionCom] jacobians = "<<J2<<"             ";
                // std::cout<<" Ixsum = "<<ImImxSum<<", Iysum ="<< ImImySum<< ", re = "<<re;
                std::cout<<std::endl;
                #endif
            jacobian[0] = - 2*J2  ;
            
            return true;
        }

    private:
        const Eigen::VectorXd x_;
        const Eigen::VectorXd y_;
        const Eigen::VectorXd t_;
    };

    class MotionComFull : public ceres::SizedCostFunction<1, 1> {
    public:
        MotionComFull(const Eigen::Ref<const Eigen::VectorXd> &x, const Eigen::Ref<const Eigen::VectorXd> &y, const Eigen::Ref<const Eigen::VectorXd> &t) : x_(x), y_(y), t_(t){};

        virtual ~MotionComFull() {}
        virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {

            const double vx = parameters[0][0];
            // const double vy = parameters[0][1];

                #ifdef debug_flag2
                std::cout<<"** [MotionCom2]  vx = "<<vx<<"     ";
                #endif

            uint16_t e_size = 0; // valid size for events
            
            Eigen::MatrixXd Im = Eigen::MatrixXd::Zero(260, 346);
            // cv::Mat image = cv::Mat(23 *2, 23 *2, CV_8UC1, cv::Scalar(0));
            for(int i=0; i< x_.size(); i++ )
            {
                int x_new, y_new;
                x_new = std::floor((x_)(i) - vx * (t_)(i)) ;
                y_new = std::floor((y_)(i) - 0  * (t_)(i)) ;
                // std::cout<<"x,y = "<<((x_)(i) - vx * (t_)(i))<<", "<<((y_)(i) - vy * (t_)(i)) <<" x_new = "<<x_new<<", y_new = "<<y_new<<std::endl;
                if(x_new > 0 && x_new < 346 && y_new > 0 && y_new < 260 ){
                    Im(y_new, x_new) += 1.0;
                    // image.at<uchar>(y_new, x_new) += 1;
                    e_size++;
                }
            }
            double re = Im.squaredNorm();
                #ifdef debug_flag2
                std::cout<<" e_size = "<<e_size<<"     ";
                // std::cout<<" Np = "<<cv::countNonZero(image)<<"    ";
                // std::cout<<"Im = "<<std::endl<<Im.block<25,25>(10,10)<<std::endl;
                std::cout<<" residuals = "<<re<< "         ";
                #endif
            if( re <= 0.00000001 ) re = 0.00000001;
            residuals[0] = 1/re;

                #ifdef debug_flag2
                std::cout<<std::endl;
                #endif
            if(!jacobians) return true;
            double* jacobian = jacobians[0];
            if(!jacobian) return true;

            
            //diff_Eigen
            Eigen::MatrixXd	Imx = Eigen::MatrixXd::Zero(260, 346);
            // Eigen::MatrixXd	Imy = Eigen::MatrixXd::Zero(32*2, 32*2);
            Imx.block(0,0,260,346-1) = Im.block(0,1,260,346-1) - Im.block(0,0,260,346-1);
            // Iy.block(0,0,max-1,max) = I.block(1,0,max-1,max) - I.block(0,0,max-1,max);
            // std::cout<<"Imx = "<<std::endl<<Imx.block<25,25>(10,10)<<" sum = "<<Imx.sum()<<std::endl;
            // std::cout<<"Imy = "<<std::endl<<Imy.block<25,25>(10,10)<<" sum = "<<Imy.sum()<<std::endl;

            double J1 = 0.0;

            double ImImxSum = 0;
            for(int j = 1 ; j<260-1 ; j++){
                for(int i = 1 ; i<346-1 ; i++){
                    double valuex = Im(j, i) * Imx(j, i)*0.1;
                    // std::cout<<" "<<valuex;
                    // ImImx2(j, i) = valuex;
                    if(!std::isnan(valuex)){
                        ImImxSum += valuex;
                    } 
                }
                // std::cout<<std::endl;
            }

            J1 = ImImxSum / re;
                #ifdef debug_flag2
                std::cout<<"** [MotionCom2] jacobians = "<<J1<<"             ";
                // std::cout<<" Ixsum = "<<ImImxSum<<", Iysum ="<< ImImySum<< ", re = "<<re;
                std::cout<<std::endl;
                #endif
            jacobian[0] = - 2*J1  ;
            
            return true;
        }

    private:
        const Eigen::VectorXd x_;
        const Eigen::VectorXd y_;
        const Eigen::VectorXd t_;
    };

    // void drawSubpixel(Eigen::MatrixXd &I, Eigen::MatrixXd &Ix, Eigen::MatrixXd &Iy, int max)
    // {

    // }

    class MotionComSubpixel : public ceres::SizedCostFunction<1, 2> {
    public:
        MotionComSubpixel(const Eigen::Ref<const Eigen::VectorXd> &x, const Eigen::Ref<const Eigen::VectorXd> &y, const Eigen::Ref<const Eigen::VectorXd> &t) : x_(x), y_(y), t_(t){};

        virtual ~MotionComSubpixel() {}
        virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {

            const double vx = parameters[0][0];
            const double vy = parameters[0][1];

                // std::cout<<std::endl;
                #ifdef debug_flag
                std::cout<<"** [MotionComSubpixel]  v = "<<vx<<", "<<vy<<"     ";
                #endif

            uint16_t e_size = 0; // valid size for events
            int range = 23 * 2 ;            
            Eigen::MatrixXd Im = Eigen::MatrixXd::Zero(range *2 +2  , range *2 +2);
            for(int i=0; i< x_.size(); i++ )
            {
                double xn, yn;
                int xn_int, yn_int;
                xn = (x_)(i) - vx * (t_)(i) ;
                yn = (y_)(i) - vy * (t_)(i) ;
                xn_int = std::floor(xn);
                yn_int = std::floor(yn);
                if(xn_int > 0 && xn_int < range && yn_int > 0 && yn_int < range){
                    // vv  y_direction   >> x_direction 
                    //             0   0.25      0.75  1
                    //  -------------------------------------------
                    //        |  *   *  |  *   *  |  *   *  |
                    //        |  *   1  |  2   2  |  3   *  |
                    //  0.25 --------------------------------------
                    //        |  *   4  |  5   5  |  6   *  |
                    //        |  *   4  |  5   5  |  6   *  |  
                    //  0.75 --------------------------------------
                    //        |  *   7  |  8   8  |  9   *  |
                    //        |  *   *  |  *   *  |  *   *  |  
                    //  -------------------------------------------
                    // case 1-9: depending on the decimal value of the input pixel (x,y), 
                    // so one pixel 1 can be divided into 9 pieces.
                    double xn_delta = xn - xn_int;
                    double yn_delta = yn - yn_int;
                    if(xn_delta >= 0.0 && yn_delta >= 0.0){
                        int x_level = std::floor(xn_delta/0.25);
                        int y_level = std::floor(yn_delta/0.25);
                        int x_area = 3;
                        if( x_level== 0 )                      x_area = 1;
                        else if( x_level == 1 || x_level == 2) x_area = 0;
                        else if( x_level == 3)                 x_area = -1;
                        int y_area = 3;
                        if( y_level== 0 )                      y_area = 1;
                        else if( y_level == 1 || y_level == 2) y_area = 0;
                        else if( y_level == 3)                 y_area = -1;
                        // std::cout<<"Data x,y: "<<x_pp<<", "<<y_pp<<"   dx,dy = "<<x_pd<<", "<<y_pd<<" area("<<-x_area+1<<", "<<-y_area+1<<")     ";
                        if(x_area==3 || y_area==3) std::cout<<std::endl<<"something is wrong about area definition!"<<std::endl;
                        
                        Eigen::VectorXd weight = Eigen::VectorXd::Zero(9);
                        double weightSum = 0.0;
                        for(int k=0; k<3; k++){// y
                            for(int l=0; l<3; l++){// x
                                double distance = ( (xn_delta-(0.5*l-0.5*x_area))*(xn_delta-(0.5*l-0.5*x_area)) + 
                                                    (yn_delta-(0.5*k-0.5*y_area))*(yn_delta-(0.5*k-0.5*y_area)) );
                                if( distance == 0.0 )
                                    weight( k*3+l ) = 1000;
                                else
                                    weight( k*3+l ) = 1/ distance;
                                weightSum += weight( k*3+l );
                                // std::cout<<" ("<<0.5*l-0.5*x_area<<", "<<0.5*k-0.5*y_area<<") ";
                                // std::cout<<" ("<<k*3+l<<") "<<weight( k*3+l )<<" ";
                            }
                        }
                        // std::cout<<"   sum = "<<weightSum<<std::endl;
                        
                        int center_x_idx = 2*xn_int + (-x_area+1);
                        int center_y_idx = 2*yn_int + (-y_area+1);
                        for(int k=0; k<3; k++){// y
                            for(int l=0; l<3; l++){// x
                                Im(center_y_idx+k-1 + 1, center_x_idx+l-1 + 1) += weight( k*3+l )/weightSum;
                            }
                        }
                    } else {
                        std::cout<<std::endl<<"something is wrong about std::floor()!"<<std::endl
                                    <<"Data x,y: "<<xn<<", "<<yn<<"      floorData x,y:"<<xn_int<<", "<<yn_int<<"         diffData x,y: "<<xn_delta<<", "<<yn_delta<<std::endl;
                    }

                    // Im(yn, xn) += 1.0;
                    e_size++;
                }
            }
            double re = Im.squaredNorm();
                #ifdef debug_flag
                std::cout<<" e_size = "<<e_size<<"     ";
                // std::cout<<" Np = "<<cv::countNonZero(image)<<"    ";
                // std::cout<<"Im = "<<std::endl<<Im.block<17,17>(18,18)<<std::endl;
                std::cout<<" residuals = "<<re<< "         ";
                #endif
            if( re <= 0.00000001 ) re = 0.00000001;
            residuals[0] = 1/re;

            // cv::Mat blur_image = cv::Mat(23 *2, 23 *2, CV_8UC1, cv::Scalar(0));
            // // cv::threshold(image, image, 3, 255, 3); 
            // // cv::GaussianBlur(image, blur_image, cv::Size(5, 5), 1);
            // cv::medianBlur(image, blur_image, 3);
            // double re3 = 0;
            // for(int i=0; i<23 *2;i++ )
            //     for(int j=0; j<23 *2;j++ )
            //         re3 += blur_image.at<uchar>(j, i) * blur_image.at<uchar>(j, i) ;
            // std::cout<<" re3 = "<<re3;
                #ifdef debug_flag
                std::cout<<std::endl;
                #endif
            if(!jacobians) return true;
            double* jacobian = jacobians[0];
            if(!jacobian) return true;

            // cv::Mat Ix = sobel_gradient(image, 1, 23, 23);
            // cv::Mat Iy = sobel_gradient(image, 2, 23, 23);

            Eigen::MatrixXd	Imx = Eigen::MatrixXd::Zero(range*2+2, range*2+2);
            Eigen::MatrixXd	Imy = Eigen::MatrixXd::Zero(range*2+2, range*2+2);
            ::ef_calib::picker::diff_Eigen(Im, Imx, Imy, range*2+2);
            
            double J1 = 0.0;
            double J2 = 0.0;

            // Eigen::MatrixXd	ImImx = Eigen::MatrixXd::Zero(32*2, 32*2);
            // Eigen::MatrixXd	ImImy = Eigen::MatrixXd::Zero(32*2, 32*2);
            // ImImx = Im.array() * Imx.array();
            // ImImy = Im.array() * Imx.array();
            // std::cout<<"ImImx = "<<std::endl<<ImImx.block<17,17>(18,18)<<" sum = "<<ImImx.block<17,17>(18,18).sum()<<std::endl;

            Eigen::MatrixXd	ImImx2 = Eigen::MatrixXd::Zero(range*2+2, range*2+2);
            Eigen::MatrixXd	ImImy2 = Eigen::MatrixXd::Zero(range*2+2, range*2+2);
            for(int j = 1 ; j<range*2+2-1 ; j++){
                for(int i = 1 ; i<range*2+2-1 ; i++){
                    ImImx2(j, i) = Im(j, i) * Imx(j, i);
                    ImImy2(j, i) = Im(j, i) * Imy(j, i);
                }
            }
            // std::cout<<"ImImx2 = "<<std::endl<<ImImx2.block<17,17>(18,18)<<" sum = "<<ImImx2.block<17,17>(18,18).sum()<<std::endl;

            // J1 = ( Im.array() * Imxsobel.array() ).sum();
            // J2 = ( Im.array() * Imysobel.array() ).sum();
            J1 = ImImx2.sum() / re;
            J2 = ImImy2.sum() / re;
                #ifdef debug_flag
                std::cout<<"** [MotionComSubpixel] jacobians = "<<J1<<", "<<J2<<"             ";
                #endif

                #ifdef debug_flag
                std::cout<<std::endl;
                #endif
            jacobian[0] = - 2*J1  ;
            jacobian[1] = - 2*J2  ;
            
            return true;
        }

    private:
        const Eigen::VectorXd x_;
        const Eigen::VectorXd y_;
        const Eigen::VectorXd t_;
    };

/*
    // struct MotionComAuto {
    //     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    //     MotionComAuto(const std::shared_ptr<std::vector<uint64_t>> x, 
    //                   const std::shared_ptr<std::vector<uint64_t>> y, 
    //                   const std::shared_ptr<std::vector<uint64_t>> t) 
    //                   : x_(x), y_(y), t_(t){};

    //     template<typename T>
    //     bool operator()(const T *const vx, const T *const vy,
    //                           T *residuals) const 
    //     {

    //         const T &vel_x = *vx;
    //         const T &vel_y = *vy;
            
    //         // const T &vel_x = vx[0];
    //         // const T &vel_y = vy[0];

    //         // const Eigen::VectorXd &xp_ = *x_ - vel_x * (*t_);
    //         // const Eigen::VectorXd &yp_ = *y_ - vel_y * (*t_);
    //         // uint16_t e_size = 0; // valid size for events
            
    //         Eigen::MatrixXd Im = Eigen::MatrixXd::Zero(23 *2, 23 *2);
    //         // // cv::Mat image = cv::Mat(23 *2, 23 *2, CV_8UC1, cv::Scalar(0));
    //         for(int i=0; i< x_->size(); i++ )
    //         {
    //             Sophus::Vector2<T> const 
    //             int x_n, y_n;
    //             x_n = x_->at(i) - vel_x * t_->at(i);
    //             y_n = y_->at(i) - vel_y * t_->at(i);

    //             if(x_n > 0 && x_n < 23 *2 && y_n > 0 && y_n < 23 *2 ){
    //                 Im(y_n, x_n) += 1.0;
    //             }
    //         }
    //         // std::cout<<"Im = "<<std::endl<<Im<<std::endl;

    //         // Eigen::MatrixXd Im2 = Im .* Im;
    //         // double re = 1.0;
    //         // // std::cout<<" residuals = "<<re<<std::endl;
    //         residuals[0] = T(-0.1);
            
    //         return true;
    //     }

    //     static ceres::CostFunction *Create
    //             (const std::shared_ptr<std::vector<uint64_t>> x, 
    //              const std::shared_ptr<std::vector<uint64_t>> y, 
    //              const std::shared_ptr<std::vector<uint64_t>> t) 
    //     {
    //         // 1 residuals.size; 2 vx vy
    //         return (new ceres::AutoDiffCostFunction<MotionComAuto, 1, 1, 1>(new MotionComAuto(x, y, t)));
    //     }


    //     const std::shared_ptr<std::vector<uint64_t>> x_;
    //     const std::shared_ptr<std::vector<uint64_t>> y_;
    //     const std::shared_ptr<std::vector<uint64_t>> t_;
    // };
*/
} // calib namespace
}

#endif //ef_calib_MotionCompensate_HPP