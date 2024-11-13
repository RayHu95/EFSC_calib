#ifndef _ef_calib_eventslice_HPP_
#define _ef_calib_eventslice_HPP_

#include <dv-processing/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <unordered_set>
#include <unordered_map>
#include <nanoflann.hpp>
#include <ef_calib/dbscan/KDTreeVectorOfVectorsAdaptor.h>
#include <ef_calib/dbscan/dbscan.h>

#include <ef_calib/picker/Config.hpp>

namespace ef_calib {
    namespace picker {

    template<typename EigenMatrixType>
    using vectorofEigenMatrix = std::vector<EigenMatrixType, Eigen::aligned_allocator<EigenMatrixType>>;

    template<typename T>
    struct EigenMatrixHash : std::unary_function<T, size_t> {
        std::size_t operator()(T const &matrix) const {
            // Note that it is oblivious to the storage order of Eigen matrix (column- or
            // row-major). It will give you the same hash value for two different matrices if they
            // are the transpose of each other in different storage order.
            size_t seed = 0;
            for (size_t i = 0; i < matrix.size(); ++i) {
                auto elem = *(matrix.data() + i);
                seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }
    };//EigenMatrixHash
    template<typename T>
    struct EigenMatrixHash3d : std::unary_function<T, size_t> {
        std::size_t operator()(T const &matrix) const {
            // Note that it is oblivious to the storage order of Eigen matrix (column- or
            // row-major). It will give you the same hash value for two different matrices if they
            // are the transpose of each other in different storage order.
            size_t seed = 0;
            for (size_t i = 0; i < matrix.size()-1; ++i) {
                auto elem = *(matrix.data() + i);
                seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }
    };//EigenMatrixHash

    class CircleFeature{
        public:
            typedef std::shared_ptr<CircleFeature> Ptr;

        CircleFeature(const Eigen::Ref<const Eigen::Vector2d> &loc, double radius)
        : loc_(loc), radius_(radius) {}

        double radius_;
        Eigen::VectorXd loc_;
    }; //CircleFeature

    class EventSlice{
    public:
        typedef std::shared_ptr<EventSlice> Ptr;

        EventSlice(const dv::EventStore &events, Eigen::Vector2f vel = Eigen::Vector2f::Zero(), Eigen::Vector4d bound = Eigen::Vector4d::Zero(), int64_t time_ref = 0);

        bool clusterEvents(uint16_t height, uint16_t width, ::ef_calib::picker::Config &config);
        void centers(vectorofEigenMatrix<Eigen::Vector2d> &pCenters, vectorofEigenMatrix<Eigen::Vector2d> &nCenters);
        double calculatefitErr(Eigen::Vector2d center, double r, size_t p_idx, size_t n_idx );

        inline void pushFeature(Eigen::Vector2d &Center, double &r) {
            features_.push_back(std::make_shared<CircleFeature>(Center, r));
        }

        inline cv::Mat drawEvent() {
            cv::Mat EventImage = cv::Mat(this->height_, this->width_, CV_8UC3, cv::Vec3b(0, 0, 0));
            for (const Eigen::Vector2d &p: positiveEvents_) {
                cv::Point loc(p[0], p[1]);
                EventImage.at<cv::Vec3b>(loc) = cv::Vec3b(0, 0, 200);
            }
            for (const Eigen::Vector2d &p: negativeEvents_) {
                cv::Point loc(p[0], p[1]);
                EventImage.at<cv::Vec3b>(loc) = cv::Vec3b(0, 200, 0);
            }
            return EventImage;
        }

        virtual inline const std::vector<CircleFeature::Ptr> &features() const noexcept {
            return features_;
        }

        inline void releaseEventSet() {
            positiveEvents_.clear();
            negativeEvents_.clear();
        }

        inline int eventsNum() const noexcept {
            return positiveEvents_.size() + negativeEvents_.size();
        }

        virtual inline cv::Mat image() const noexcept { return image_;}

    public:
        /* unique id **/
        uint64_t idx;
        /** Desired Image dimension **/
        // uint16_t height, width;
        /** Time stamps **/
        int64_t first_time, last_time, time, delta_time;
        /** Events Coordinates and normalize coord **/
        std::vector<cv::Point2d> coord, undist_coord;
        /** Events polarities **/
        std::vector<int8_t> pol;
        /** Event Frame pose **/
        // ::base::Affine3d T_w_ef;

        dv::EventStore events_;
        cv::Mat eventImage;
        // the raw clustering result
        std::vector<std::vector<uint>> nClusters_, pClusters_;

    protected:
        uint16_t height_, width_;
        // store events
        vectorofEigenMatrix<Eigen::Vector2d> positiveEvents_, negativeEvents_;

        cv::Mat image_; // Remember to Release if no longer used
        std::vector<CircleFeature::Ptr> features_; // dynamic_cast only work for reference and pointer

        
    };// EventSlice
} //Namespace picker
} //Namespace ef_calib
#endif //_ef_calib_eventframe_HPP_