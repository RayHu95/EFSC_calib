#include <ef_calib/picker/EventSlice.hpp>

using namespace ef_calib::picker;
#define picker_debug 1

EventSlice::EventSlice(const dv::EventStore &events, Eigen::Vector2f vel, Eigen::Vector4d bound, int64_t time_ref)
:events_(events)
{
    /** Get the events and polarity **/
    this->first_time = events.getLowestTime();
    this->last_time = events.getHighestTime();
    if (this->first_time > this->last_time)
    {
        std::string error_message = std::string("[EVENT_FRAME] FATAL ERROR Event time[0] > event time [N-1] ");
        throw std::runtime_error(error_message);
    }
    // for (const dv::Event &event : events)
    // {
    //     this->coord.push_back(cv::Point2d(event.x(), event.y()));
    //     this->pol.push_back((event.polarity())?1:-1);
    // }

    /** Frame time as the median event time **/
    this->time = (this->first_time + this->last_time) /2;
    
    /** Delta time of thsi event frame **/
    this->delta_time = (this->last_time - this->first_time);
    
    // std::cout<<"[EVENT_FRAME] Created ID["<<this->idx<<"] with: "<<events.size()<<" events. start time "
    //         <<first_time/1000000<<"."<<first_time%1000000<<" end time "<<last_time/1000000<<"."<<last_time%1000000<<std::endl;
    // std::cout<<"[EVENT_FRAME] event frame ["<<std::addressof(this->event_frame)<<"] size:"<<this->event_frame.size()<<std::endl;
    

    //************** store events **********************
    this->releaseEventSet(); //clear all the positiveEvents_ and negativeEvents_

    // //Eigen::aligned_allocator<Eigen::Vector2d>> positiveEvents, negativeEvents;
    std::unordered_set<Eigen::Vector2d, EigenMatrixHash<Eigen::Vector2d>, std::equal_to<>,
         Eigen::aligned_allocator<Eigen::Vector2d>> positiveEvents, negativeEvents;

    // normal eventslice
    for(const dv::Event &event : events)
    {
        Eigen::Vector2d e_loc(event.x(), event.y());
        if(event.polarity() == 1)
            positiveEvents.insert(e_loc);
        if(event.polarity() == 0)
            negativeEvents.insert(e_loc);
        //eventcontainer.emplace(event.timestamp(), Event_loc_pol(e_loc, event.polarity()));
    }

    
    // delete event, which have + and - event at the same pixel
    for (auto itr = positiveEvents.begin(); itr != positiveEvents.end();) {
        auto found = negativeEvents.find(*itr);
        if (found == negativeEvents.end()) {
            itr++;
        } else {
            negativeEvents.erase(found);
            itr = positiveEvents.erase(itr);
        }
    }
    
    std::move(positiveEvents.begin(), positiveEvents.end(), std::back_inserter(positiveEvents_));
    std::move(negativeEvents.begin(), negativeEvents.end(), std::back_inserter(negativeEvents_));

}


bool EventSlice::clusterEvents(uint16_t height, uint16_t width, ::ef_calib::picker::Config &config){

    this->height_ = height;
    this->width_ = width;

    if (positiveEvents_.size() < 5 || positiveEvents_.size() < 5) {
        return false;
    }

    auto dbscan = DBSCAN<Eigen::Vector2d, double>();
    dbscan.Run(&positiveEvents_, 2, config.dbscan_eps, config.dbscan_startMinSample);
    auto p_noise = std::move(dbscan.Noise);
    auto p_clusters = std::move(dbscan.Clusters);
    dbscan.Run(&negativeEvents_, 2, config.dbscan_eps, config.dbscan_startMinSample);
    auto n_noise = std::move(dbscan.Noise);
    auto n_clusters = std::move(dbscan.Clusters);

    /** Draw the result and filter data **/
    this->image_ = cv::Mat(height, width, CV_8UC1, cv::Scalar(0));
    this->eventImage = cv::Mat(height, width, CV_8UC3, cv::Vec3b(0, 0, 0));

    #ifdef picker_debug
        for (const Eigen::Vector2d &p: positiveEvents_) {
            cv::Point loc(p[0], p[1]);
            this->eventImage.at<cv::Vec3b>(loc) = cv::Vec3b(0, 0, 200);
        }
        for (const Eigen::Vector2d &p: negativeEvents_) {
            cv::Point loc(p[0], p[1]);
            this->eventImage.at<cv::Vec3b>(loc) = cv::Vec3b(0, 0, 100);
        }
    #endif
    
    for (auto itr = p_clusters.begin(); itr != p_clusters.end();) {
        // remove cluster with too few samples
        if (itr->size() < config.clusterMinSample) {
            itr = p_clusters.erase(itr);
        } else {
            #ifdef picker_debug
            unsigned int color = 20 * (itr - p_clusters.begin());
            #endif
            for (unsigned int idx : *itr) {
                cv::Point loc(positiveEvents_[idx][0], positiveEvents_[idx][1]);
                image_.at<uchar>(loc) = 255;
                #ifdef picker_debug
                    this->eventImage.at<cv::Vec3b>(loc) = cv::Vec3b(color / 256, color % 256, 200);
                #endif
            }
            itr++;
        }
    }

    for (auto itr = n_clusters.begin(); itr != n_clusters.end();) {
        // remove cluster with too few samples
        if (itr->size() < config.clusterMinSample) {
            itr = n_clusters.erase(itr);
        } else {
            #ifdef picker_debug
            unsigned int color = 20 * (itr - n_clusters.begin());
            #endif
            for (unsigned int idx : *itr) {
                cv::Point loc(negativeEvents_[idx][0], negativeEvents_[idx][1]);
                image_.at<uchar>(loc) = 255;
                #ifdef picker_debug
                    this->eventImage.at<cv::Vec3b>(loc) = cv::Vec3b(color / 256, color % 256, 100);
                #endif
            }
            itr++;
        }
    }

    // store
    nClusters_ = n_clusters;
    pClusters_ = p_clusters;
    
    return true;
}

void EventSlice::centers(vectorofEigenMatrix<Eigen::Vector2d> &pCenters, vectorofEigenMatrix<Eigen::Vector2d> &nCenters){

    // calculate each cluster center (median)
    std::vector<uint> p_centers, n_centers;

    auto p_compare_fun = [&](uint lhs, uint rhs) { return positiveEvents_[lhs].norm() < positiveEvents_[rhs].norm(); };
    auto n_compare_fun = [&](uint lhs, uint rhs) { return negativeEvents_[lhs].norm() < negativeEvents_[rhs].norm(); };
    for (auto &pCluster: pClusters_) {
        std::nth_element(pCluster.begin(), pCluster.begin() + pCluster.size() / 2, pCluster.end(), p_compare_fun);
        p_centers.push_back(pCluster[pCluster.size() / 2]);
    }
    for (auto &nCluster: nClusters_) {
        std::nth_element(nCluster.begin(), nCluster.begin() + nCluster.size() / 2, nCluster.end(), n_compare_fun);
        n_centers.push_back(nCluster[nCluster.size() / 2]);
    }

    #ifdef picker_debug
        // draw cluster centers
        for (auto center: p_centers) {
            cv::Point loc(positiveEvents_[center][0], positiveEvents_[center][1]);
            cv::circle(this->eventImage, loc, 2, cv::Vec3b(255, 255, 255));
        }
        for (auto center: n_centers) {
            cv::Point loc(negativeEvents_[center][0], negativeEvents_[center][1]);
            cv::circle(this->eventImage, loc, 2, cv::Vec3b(255, 255, 255));
        }
    #endif

    // store pCenters
    pCenters.resize(p_centers.size());
    nCenters.resize(n_centers.size());
    for (int i = 0; i < p_centers.size(); ++i) {
        pCenters[i] = positiveEvents_[p_centers[i]];
    }
    for (int i = 0; i < n_centers.size(); ++i) {
        nCenters[i] = negativeEvents_[n_centers[i]];
    }
}

double EventSlice::calculatefitErr(Eigen::Vector2d center, double r, size_t p_idx, size_t n_idx ){

    double fitErr = 0.0;

    for (auto pEvent: this->pClusters_[p_idx]) {
        fitErr += std::abs((positiveEvents_[pEvent] - center).norm() - r);
    }
    for (auto nEvent: this->nClusters_[n_idx]) {
        fitErr += std::abs((negativeEvents_[nEvent] - center).norm() - r);
    }
    fitErr /= (this->pClusters_[p_idx].size() + this->nClusters_[n_idx].size()) * r;

    return fitErr;
}

