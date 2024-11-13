#pragma once

#include <tools/camerainfo.hpp>

#include <ef_calib/picker/EventSlice.hpp>
#include <ef_calib/picker/EventSliceMap.hpp>

namespace ef_calib { namespace picker{

    class EventPathes{
        public:
            typedef std::shared_ptr<EventPathes> Ptr;

        EventPathes(const Eigen::Ref<const Eigen::VectorXd> &xp, 
                    const Eigen::Ref<const Eigen::VectorXd> &yp, 
                    const Eigen::Ref<const Eigen::VectorXd> &delta_time,
                    const int &inlier_numbers, const int &biasx, const int &biasy, const int &range_max)
        : xp_(xp), yp_(yp), delta_time_(delta_time), inlier_numbers_(inlier_numbers), bias_x(biasx), bias_y(biasy), maxRange(range_max) {}

        Eigen::VectorXd xp_, yp_, delta_time_;
        
        int inlier_numbers_;
        int bias_x, bias_y; //bias
        double maxRange; //2*rad or max_range
    }; //EventPathes


    class EventPicker{
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            ::ef_calib::picker::Config config;
            ::ef_calib::picker::pattern_Config pattern;

            EventPicker(const ::ef_calib::picker::Config &config, ::tools::CameraInfo &camera, ::ef_calib::picker::pattern_Config &pattern);

            //----- @brief make sure in duration there are enough samples for each half circle.
            bool extractFeatures(std::shared_ptr<ef_calib::picker::EventSlice> es);
            // bool rectifyFeatures(const std::unordered_set<int> &outlierIdxs,
            //                      const Eigen::Ref<const Eigen::Matrix3d> &Rcw, const Eigen::Ref<const Eigen::Vector3d> &tcw);

            // ef_slam::ef_LandmarkBase::Ptr findCenter(const Eigen::Vector2d &p) const;
            
            // divide events based on the features
            void divideEvents(const dv::EventStore &events, std::vector<CircleFeature::Ptr> &features, const size_t &num_results,  
                              std::vector<EventPathes::Ptr> &events_patches, uint64_t time_ref);

            // only for initialization
            // cv::Mat GradientAnalytic(std::vector<int> xp_in, std::vector<int> yp_in, std::vector<int> delta_time_in, int64_t &f_Frobenius, Eigen::Vector2f &Gradient, const Eigen::Vector2f vel, int r);
            cv::Mat GradientAnalytic(Eigen::VectorXd xp_in, Eigen::VectorXd yp_in, Eigen::VectorXd delta_time_in, int64_t &f_Frobenius, Eigen::Vector2f &Gradient, const Eigen::Vector2f vel, int r);
            void MotionCompensate(std::vector<ef_calib::picker::EventPathes::Ptr> &patches, Eigen::Vector2d &vel);
            void MotionCompensateFull(const dv::EventStore &events, Eigen::Vector2d &vel, int &num, uint64_t time_ref, cv::Mat &image_es2);

            void fitCircle        (const std::vector<cv::Point2d> &candidate, Eigen::Ref<Eigen::Vector2d> center, double &radius);
            void fitCircleSubpixel(const std::vector<cv::Point3d> &candidate, Eigen::Ref<Eigen::Vector2d> center, double &radius);

            double EvaluateMC(ef_calib::picker::EventPathes::Ptr &patch, Eigen::Vector2d &vel, cv::Point2f &center, double &radius);

            EventSliceMap::Ptr esMap;
    
            // // DEBUG
            cv::Mat rectifyImage;

            // //std::multimap<double, Event_loc_pol> eventcontainer;
            // std::pair<int64_t, int64_t> duration_;
            // dv::EventStore events_;

        protected:
            // // Minimize geometric distance : \cite{A circle fitting procedure and its error analysis}
            // // TODO: find an eclipse fitting algorithm.
            // void fitCircle(const std::vector<uint> &pSet, const std::vector<uint> &nSet,
            //                Eigen::Ref<Eigen::Vector2d> center, double &radius);

            uint16_t height, width;
            

            // KD-Tree for neighbor searching on features_, used for establish corresponds for given event.
            // std::shared_ptr<KDTreeVectorOfVectorsAdaptor<vectorofEigenMatrix<Eigen::Vector2d>, double, 2>> circleKdTree_;
            // vectorofEigenMatrix<Eigen::Vector2d> circles_;

            
    };// EventSlicer

} //picker namespace
} //namespace ef_calib
