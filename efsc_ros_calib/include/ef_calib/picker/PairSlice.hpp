#include <ef_calib/picker/EventSlice.hpp>
#include <ef_calib/picker/ImageSlice.hpp>

namespace ef_calib { namespace picker{ 

    class PairSlice {
    public:
        typedef std::shared_ptr<PairSlice> Ptr;

        PairSlice(EventSlice::Ptr &es, cv::Mat &image, uint64_t timestamp, std::vector<cv::Point2f> &outCenters, dv::EventStore &events)
        :es_(es), time(timestamp), events_(events)
        {
            fr_ = std::make_shared<ef_calib::picker::ImageSlice>(image, timestamp, outCenters);
        };

        //** reference time **//
        uint64_t time;

        //** eventslice part **//
        EventSlice::Ptr es_;
        dv::EventStore events_;

        //** frameslice part **//
        ImageSlice::Ptr fr_;
        // std::vector<cv::Point2f> centers;
    };

} //picker namespace
} //namespace ef_calib