#ifndef ef_calib_EventFramePair_HPP
#define ef_calib_EventFramePair_HPP

#include <shared_mutex>
#include <iterator>
//#include <set>
#include <memory>

#include <ef_calib/picker/ImageSlice.hpp>

namespace ef_calib {
    namespace calib {

    class CalibPair {
    public:
        typedef std::shared_ptr<CalibPair> Ptr;

        CalibPair(uint64_t timestamp, std::vector<cv::Point2f> &EventCenters, std::vector<cv::Point2f> &FrameCenters)
        :time(timestamp), Ecircles(EventCenters), Fcircles(FrameCenters)
        { };

        //** idx **//
        // uint64_t idx;
        uint64_t time;

        //** eventslice part **//
        std::vector<cv::Point2f> Ecircles;
        //** frameslice part **//
        std::vector<cv::Point2f> Fcircles;
        

        inline Eigen::Vector3d twb() const noexcept {
            std::shared_lock lock(poseMutex_);
            return twb_;
        }

        inline Eigen::Quaterniond unitQwb() const noexcept {
            std::shared_lock lock(poseMutex_);
            return unitQwb_;
        }

        inline void setPose(const Eigen::Ref<const Eigen::Vector3d> &twb, const Eigen::Quaterniond &Qwb) {
            std::unique_lock lock(poseMutex_);
            twb_ = twb;
            unitQwb_ = Qwb;
            unitQwb_.normalize();
        }

        inline Eigen::Vector3d twb2() const noexcept {
            std::shared_lock lock(poseMutex2_);
            return twb2_;
        }

        inline Eigen::Quaterniond unitQwb2() const noexcept {
            std::shared_lock lock(poseMutex2_);
            return unitQwb2_;
        }

        inline void setPose2(const Eigen::Ref<const Eigen::Vector3d> &twb2, const Eigen::Quaterniond &Qwb2) {
            std::unique_lock lock(poseMutex2_);
            twb2_ = twb2;
            unitQwb2_ = Qwb2;
            unitQwb2_.normalize();
        }

        void setdepthAndCamNorm(std::vector<Eigen::Vector3d> CamNorm_es, std::vector<double> depth_es,
                                       std::vector<Eigen::Vector3d> CamNorm_fr, std::vector<double> depth_fr) {
            if(IsPoseOptimized)
                return;
            if( CamNorm_es.size() == 0 || CamNorm_es.size() != depth_es.size() || CamNorm_fr.size() != depth_fr.size()){
                std::cout<<" data insert to g2o has problem "<<std::endl;
                return;
            }
            CamNorm_fr_ = CamNorm_fr;
            depth_fr_ = depth_fr;
            CamNorm_es_ = CamNorm_es;
            depth_es_ = depth_es;
            IsPoseOptimized = true;
        }

        inline double getFrameError(int i, Eigen::Vector3d obj) {
            std::shared_lock lock(poseMutex_);
            double error = ( unitQwb_ * obj + twb_  - CamNorm_fr_[i] * depth_fr_[i] ).norm();
            return error;
        }
        inline double getEventError(int i, Eigen::Vector3d obj) { 
            std::shared_lock lock(poseMutex2_);
            double error = ( unitQwb2_* obj + twb2_ - CamNorm_es_[i] * depth_es_[i] ).norm();
            return error;
        }

        inline Eigen::Vector3d getEventCam(int i){
            return CamNorm_es_[i] * depth_es_[i];
        }
        inline Eigen::Vector3d getFrameCam(int i){
            return CamNorm_fr_[i] * depth_fr_[i];
        }

    protected:
        bool IsPoseOptimized = false;

        // ** frame ** //
        Eigen::Vector3d twb_;
        Eigen::Quaterniond unitQwb_;
        mutable std::shared_mutex poseMutex_;
        // norm and depth at the camera Coordinate System
        std::vector<Eigen::Vector3d> CamNorm_fr_;
        std::vector<double> depth_fr_;


        // ** event ** //
        Eigen::Vector3d twb2_;
        Eigen::Quaterniond unitQwb2_;
        mutable std::shared_mutex poseMutex2_;
        // norm and depth at the camera Coordinate System
        std::vector<Eigen::Vector3d> CamNorm_es_;
        std::vector<double> depth_es_;

    };

} // calib namespace
}

#endif //ef_calib_EventFramePairMap_HPP
