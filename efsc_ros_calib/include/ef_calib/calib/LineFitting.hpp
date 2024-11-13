#ifndef ef_calib_LineFitting_HPP
#define ef_calib_LineFitting_HPP

#include <ceres/ceres.h>

#include "sophus/se3.hpp"

namespace ef_calib {
    namespace calib {

    struct LineFitting {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        LineFitting(const Eigen::Vector3d &loc) : loc_(loc){}

        template<typename T>
        bool operator()(const T *const LineParam, T *residuals) const 
        {
            // x = k1 * t + b1; y = k2 * t + b2; z = k3 * t + b3;
            const T &k1 = LineParam[0];
            const T &b1 = LineParam[1];
            const T &k2 = LineParam[2];
            const T &b2 = LineParam[3];
            const T &k3 = LineParam[4];
            const T &b3 = LineParam[5];
            // std::cout<<"  k1 = "<<k1<<"  b1 = "<<b1<<"  k2 = "<<k2<<"  b2 = "<<b2<<"  k3 = "<<k3<<"  b3 = "<<b3;

            T t = (loc_(0) - b1) / k1;
            T y = k2 * t + b2;
            T z = k3 * t + b3;

            // // t = 1
            // T x1 = k1 + b1;
            // T y1 = k2 + b2;
            // T z1 = k3 + b3;

            // Sophus::Vector3<T> line1(loc_(0) - x1, loc_(1) - y1, loc_(2) - z1);
            // Sophus::Vector3<T> line2(k1, k2, k3);

            // T S_area = (line2.cross(line1)).norm();
            // std::cout<<"  S_area = "<<S_area;

            // residuals[0] = S_area / line2.norm();

            residuals[0] = (y-loc_(1))*(y-loc_(1)) + (z-loc_(2))*(z-loc_(2));
            // std::cout<<"   residuals = "<<residuals[0]<<std::endl;
            return true;
        }

        static ceres::CostFunction *Create(const Eigen::Vector3d &loc) {
            return (new ceres::AutoDiffCostFunction<LineFitting, 1, 6>(new LineFitting(loc)));
        }

        const Eigen::Vector3d loc_;
    };

} // calib namespace
}

#endif //ef_calib_LineFitting_HPP
