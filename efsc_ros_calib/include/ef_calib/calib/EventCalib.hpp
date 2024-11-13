#ifndef ef_calib_EventCalib_HPP
#define ef_calib_EventCalib_HPP

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <ef_calib/calib/EventFramePairMap.hpp>
#include <ef_calib/calib/CameraBase.hpp>
#include "sophus/se3.hpp"

namespace ef_calib {
    namespace calib {

    class EventCalib {
    public:
        typedef std::shared_ptr<EventCalib> Ptr;

        template<typename EigenMatrixType>
        using vectorofEigenMatrix = std::vector<EigenMatrixType, Eigen::aligned_allocator<EigenMatrixType>>;

        EventCalib(EventFramePairMap::Ptr map, CameraBase::Ptr camera/*, EventContainer::Ptr eventContainer, bool useSO3, bool reduceMap,
                         double motionTimeStep, double circleRadius*/);
        

        struct ReprojectionErrorLine {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            ReprojectionErrorLine(const Eigen::Vector2d &xy): xy_(xy){}

            template<typename T>
            bool operator()(const T *const ab, T *residuals) const 
            {
                // std::cout<<" data : "<<xy_.transpose()<< " residual = "<<xy_[1] - ( ab[0] * xy_[0]  + ab[1] )<<std::endl;
                residuals[0] = xy_[1] - ( ab[0] * xy_[0]  + ab[1] );
                return true;
            }

            static ceres::CostFunction *Create(const Eigen::Vector2d &xy){
                return (new ceres::AutoDiffCostFunction<ReprojectionErrorLine, 1, 2>(new ReprojectionErrorLine(xy)));
            }

            const Eigen::Vector2d xy_;
        };


        struct ReprojectionError {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            ReprojectionError(const Eigen::Vector3d &obs,
                               const Eigen::Vector3d &obs2)
                : obs_(obs), obs2_(obs2) { }

            template<typename T>
            bool operator()(const T *const Qn, const T *const tn,
                            T *residuals) const 
            {
                // *** Qws: the rotation (unit-Quaterniond-type w x y z) is solved *****//
                Eigen::Map<Sophus::Vector4<T> const> const Qn0(Qn);
                Sophus::Vector4<T> Qef_v = Qn0;
                Qef_v.normalize();
                // Eigen::Quaternion<T> a{b.data()}, in this way 'b' should store value like {x y z w} order. or
                // define Eigen::Quaternion<T> a{w, x, y, z}
                // But Qef_v in order of (w x y z)
                Eigen::Quaternion<T> Qef{Qef_v[0], Qef_v[1], Qef_v[2], Qef_v[3]};
                
                // *** tws: the translation (Vector3d-type) is solved *****//
                Eigen::Map<Sophus::Vector3<T> const> const tn0(tn);
                Sophus::Vector3<T> Tef_v = tn0;

                Sophus::Vector3<T> dis = Qef.toRotationMatrix() * obs2_ + Tef_v - obs_;
                residuals[0] = dis.norm();
                // std::cout<<" re = "<<dis.transpose()<<std::endl;
                return true;
            }

            static ceres::CostFunction *Create(const Eigen::Vector3d &obs, const Eigen::Vector3d &obs2) 
            {
                // 1 residuals.size; 4 rvec; 3 tvec
                return (new ceres::AutoDiffCostFunction<ReprojectionError, 1,
                                                     4, 3>(new ReprojectionError(obs, obs2)));
            }

            const Eigen::Vector3d obs_;
            const Eigen::Vector3d obs2_;
        };
        
        
        
        void setInitialValue(Eigen::Quaterniond Qef, Eigen::Vector3d Tef); // w x y z 
        void addObervation(const Eigen::Vector3d obs, const Eigen::Vector3d obs2);
        bool optimize();



        virtual inline Eigen::Matrix<double, 4, 1> R() const noexcept { return Rotation_;}
        virtual inline Eigen::Matrix<double, 3, 1> T() const noexcept { return Translation_;}

    protected:
        ceres::Problem problem;
        EventFramePairMap::Ptr map_;

        bool flag_IntrisicEstimate = false;

        Eigen::Matrix<double, 9, 1> intrinsics_;

        Eigen::Matrix<double, 4, 1> Rotation_; // w x y z 
        Eigen::Matrix<double, 3, 1> Translation_;
        // double motionTimeStep_;
        // double circleRadius_;
        // std::vector<RelationContainer> relationContainer_; // used for calculating back-projection error
        Eigen::Matrix<double, 2, 1> ab_;

        bool flag_Switch34 = false; //false:3; true:4
        std::vector<Eigen::Vector3d> obs_vec;
        std::vector<Eigen::Vector3d> obs2_vec;
    };
} // calib namespace
}

#endif //ef_calib_EventCalib_HPP
