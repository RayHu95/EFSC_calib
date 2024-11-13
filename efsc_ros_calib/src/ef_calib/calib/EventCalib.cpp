#include <ef_calib/calib/EventCalib.hpp>

using namespace ef_calib::calib;

EventCalib::EventCalib(EventFramePairMap::Ptr map, CameraBase::Ptr camera) : map_(map)
{
    std::cout<<"[calib] <<<<< configure event camera optimization <<<<<<"<<std::endl;
    
    // setup intrinsics_
    const Eigen::Matrix3d &K = camera->K();
    const Eigen::VectorXd &distCoeffs = camera->distCoeffs();
    if (distCoeffs.size() != 5) {
        throw std::logic_error("Sorry! Only radial distortion considered for now.");
    }

    intrinsics_ << K(0, 0), K(1, 1), K(0, 2), K(1, 2),
            distCoeffs(0), distCoeffs(1), distCoeffs(2), distCoeffs(3), distCoeffs(4);
    Rotation_ << 1, 0, 0, 0;
    Translation_ << 0, 0, 0;

    ab_<< 0,0;
}

void EventCalib::setInitialValue(Eigen::Quaterniond Qef, Eigen::Vector3d Tef){// w x y z
    Rotation_ << Qef.w(), Qef.x(), Qef.y(), Qef.z();
    Translation_ << Tef[0], Tef[1], Tef[2];
    std::cout<<" setting Initial optimization parameter: "<<std::endl 
             <<"    R: " << this->Rotation_.transpose() << std::endl
             <<"    t: " << this->Translation_.transpose() << std::endl;
}

void EventCalib::addObervation(const Eigen::Vector3d obs, const Eigen::Vector3d obs2){


        ceres::CostFunction *cost_function = ReprojectionError::Create(
                    obs, obs2);
        // ceres::LossFunction *huber = new ceres::HuberLoss(0.1);
        this->problem.AddResidualBlock(cost_function, nullptr, this->Rotation_.data(), this->Translation_.data());
        
        obs_vec.push_back(obs);
        obs2_vec.push_back(obs2);
}


bool EventCalib::optimize() {

    std::cout<<"[calib] <<<<< start optimization <<<<<<"<<std::endl;
    
    // Set solver options (precision / method)
    ceres::Solver::Options options;
    options.max_num_iterations = 2000; 
    options.gradient_tolerance = Sophus::Constants<double>::epsilon();
    options.function_tolerance = Sophus::Constants<double>::epsilon();
    // options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.linear_solver_type = ceres::DENSE_QR;
    // options.minimizer_progress_to_stdout = true;

    // ** Solve ** //
    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);
    // std::cout << summary.FullReport() << std::endl;

    this->Rotation_.normalize();
    std::cout<<"[calib] <<<<< finish optimization <<<<<<"<<std::endl;
    if(flag_IntrisicEstimate)
        std::cout << "** Intrinsics after optimization:" << this->intrinsics_.transpose() << std::endl;
    std::cout << "** T after optimization: " <<"  R: " << this->Rotation_.transpose() << std::endl
              << "                         " <<"  t: " << this->Translation_.transpose() << std::endl;

    // std::cout << "** ab after optimization: " << this->ab_.transpose() << std::endl;

    Eigen::Quaterniond Rot{this->Rotation_[0], this->Rotation_[1], this->Rotation_[2], this->Rotation_[3]};
    std::cout<<" q(R) = "<<Rot<<std::endl;
    double total_err = 0.0;
    for(int i=0;i<obs_vec.size();i++){
        Eigen::Vector3d dis = Rot.toRotationMatrix() * obs2_vec[i] + this->Translation_ - obs_vec[i];
        double err = dis.norm();
        // std::cout<<" err " << err <<" dis = "<<dis.transpose()<<std::endl;
        total_err += err;
    }
    std::cout<<std::endl<<" total_average_err " << total_err/obs_vec.size()<<std::endl;

    return true;
}
