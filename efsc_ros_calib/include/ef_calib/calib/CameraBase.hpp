
#ifndef _CALIB_Calib_Camera_HPP_
#define _CALIB_Calib_Camera_HPP_

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace ef_calib { namespace calib{

    class CameraBase {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            typedef std::shared_ptr<CameraBase> Ptr;

            explicit CameraBase(const Eigen::Ref<const Eigen::Vector2i> &size)
            :size_(size){};

            inline Eigen::Vector3d pixel2cam(const cv::Point2f &p){
                //Intrinsic
                double px = (p.x - K_(0, 2)) / K_(0, 0);
                double py = (p.y - K_(1, 2)) / K_(1, 1);

                //distortion [k1, k2, p1, p2, k3]
                double xx = px * px;
			    double yy = py * py;
			    double r2 = xx + yy;
                double r4 = r2 * r2;
                double r6 = r4 * r2;
                double xy = 2.0 * px * py;
                double r_coeff = 1.0 + distCoeffs_[0] * r2 + distCoeffs_[1] * r4 + distCoeffs_[4] * r6 ;

                px *= r_coeff;
                px += distCoeffs_[2] * xy + distCoeffs_[3] * (r2+2.0*xx);
                py *= r_coeff;
                py += distCoeffs_[2] * (r2+2.0*yy) + distCoeffs_[3] * xy;

                return Eigen::Vector3d(px, py, 1.0);
            }

            inline Eigen::Vector2d FromCamToPixel(const Eigen::Vector3d &p_norm){
                // distortion [k1, k2, p1, p2, k3]
                double xx = p_norm[0] * p_norm[0];
			    double yy = p_norm[1] * p_norm[1];
			    double r2 = xx + yy;
                double r4 = r2 * r2;
                double r6 = r4 * r2;
                double xy = 2.0 * p_norm[0] * p_norm[1];
                double r_coeff = 1.0 + distCoeffs_[0] * r2 + distCoeffs_[1] * r4 + distCoeffs_[4] * r6 ;

                double px = p_norm[0] * r_coeff;
                double py = p_norm[1] * r_coeff;
                px += distCoeffs_[2] *           xy  + distCoeffs_[3] * (r2 + 2.0*xx);
                py += distCoeffs_[2] * (r2 + 2.0*yy) + distCoeffs_[3] *           xy ;

                //Intrinsic
                px = px * K_(0, 0) + K_(0, 2);
                py = py * K_(1, 1) + K_(1, 2);

                return Eigen::Vector2d(px, py);
            }

            inline Eigen::Vector2d FromCamNormToCamdis(const Eigen::Vector2d &p_norm){

                // distortion [k1, k2, p1, p2, k3]
                double xx = p_norm[0] * p_norm[0];
			    double yy = p_norm[1] * p_norm[1];
			    double r2 = xx + yy;
                double r4 = r2 * r2;
                double r6 = r4 * r2;
                double xy = 2.0 * p_norm[0] * p_norm[1];
                double r_coeff = 1.0 + distCoeffs_[0] * r2 + distCoeffs_[1] * r4 + distCoeffs_[4] * r6 ;

                double px = p_norm[0] * r_coeff;
                double py = p_norm[1] * r_coeff;
                px += distCoeffs_[2] *           xy  + distCoeffs_[3] * (r2 + 2.0*xx);
                py += distCoeffs_[2] * (r2 + 2.0*yy) + distCoeffs_[3] *           xy ;

                return Eigen::Vector2d(px, py);
            }

            inline Eigen::Vector3d FromPixelToCam(const Eigen::Vector2d &uv){
                double x_dis = (uv[0] - K_(0, 2)) /  K_(0, 0);
                double y_dis = (uv[1] - K_(1, 2)) /  K_(1, 1);
                Eigen::Vector2d x_d(x_dis, y_dis);
                Eigen::Vector2d x_m = x_d;
                bool isFound = false;
                for(int itr=0; itr< 20; itr++){
                    Eigen::Vector2d delta = x_d - FromCamNormToCamdis(x_m);
                    // std::cout<<" itr = "<<itr<<" delta = "<<delta.transpose()<<std::endl;
                    x_m += delta;
                    if(delta[0]< 1e-8 && delta[1]<1e-8){
                        isFound = true;
                        break;
                    }
                }
                if(!isFound) std::cout<<" something is wrong about inverse Distortion. "<<std::endl;

                return Eigen::Vector3d(x_m[0], x_m[1], 1.0);
            }

            inline const Eigen::Vector2i &size() const noexcept {
                return size_;
            }

            inline const Eigen::Matrix3d &K() const noexcept {
                return K_;
            }

            inline const Eigen::Matrix3d &invK() const noexcept {
                return invK_;
            }

            inline Eigen::VectorXd &distCoeffs() noexcept {
                return distCoeffs_;
            }

            inline void setK(cv::Mat &cameraMatrix) {
                cameraMatrix_cv = cameraMatrix;
                cv::cv2eigen(cameraMatrix, K_);
                invK_ = K_.inverse();

                std::cout << K_ << std::endl;
            }

            inline void setdistCoeffs(cv::Mat &distCoeffs) {
                distCoeffs_cv = distCoeffs;
                cv::cv2eigen(distCoeffs, distCoeffs_);
                std::cout << distCoeffs_.transpose() << std::endl;
            }

            inline void setIntrisic(std::vector<double> intri, std::vector<double> distCoeffs) {
                K_<< intri[0],        0,  intri[2], 
                            0, intri[1],  intri[3],
                            0,        0,         1;
                invK_ = K_.inverse();
                cv::eigen2cv(K_, cameraMatrix_cv);
                // std::cout << cameraMatrix_cv << std::endl;

                //distortion [k1, k2, p1, p2, k3]
                distCoeffs_.resize(5);
                distCoeffs_<< distCoeffs[0], distCoeffs[1], distCoeffs[2], distCoeffs[3], distCoeffs[4];
                cv::eigen2cv(distCoeffs_, distCoeffs_cv);
                // std::cout << distCoeffs_cv << std::endl;
                std::cout<<"[calib] Intrinsic Matrix is setted. "<<std::endl;
            }

            // Eigen::VectorXd inverseRadialDistortion(const Eigen::Ref<const Eigen::Vector4d> &radialDistortion) {
            //     const Eigen::Ref<const Eigen::VectorXd> &k = radialDistortion;
            //     Eigen::VectorXd b(5);

            //     double k00 = k[0] * k[0];
            //     double k000 = k[0] * k00;
            //     double k0000 = k[0] * k000;
            //     double k00000 = k[0] * k0000;
            //     double k01 = k[0] * k[1];
            //     double k001 = k[0] * k01;
            //     double k0001 = k[0] * k001;
            //     double k11 = k[1] * k[1];
            //     double k011 = k[0] * k11;
            //     double k02 = k[0] * k[2];
            //     double k002 = k[0] * k02;
            //     double k12 = k[1] * k[2];
            //     double k03 = k[0] * k[3];

            //     b[0] = -k[0];
            //     b[1] = 3 * k00 - k[1];
            //     b[2] = -12 * k000 + 8 * k01 - k[2];
            //     b[3] = 55 * k0000 - 55 * k001 + 5 * k11 + 10 * k02 - k[3];
            //     b[4] = -273 * k00000 + 364 * k0001 - 78 * k011 - 78 * k002 + 12 * k12 + 12 * k03;

            //     return b;
            // }

            cv::Mat cameraMatrix_cv;
            cv::Mat distCoeffs_cv;

        protected:
            Eigen::Vector2i size_;

            Eigen::Matrix3d K_;
            Eigen::Matrix3d invK_;

            // Opencv distortion coefficients (k1,k2,p1,p2[,k3[,k4,k5,k6[,s1,s2,s3,s4[,τx,τy]]]]) of 4, 5, 8, 12 or 14 elements.
            Eigen::VectorXd distCoeffs_;
    };

} //calib namespace
} // end namespace

#endif 
