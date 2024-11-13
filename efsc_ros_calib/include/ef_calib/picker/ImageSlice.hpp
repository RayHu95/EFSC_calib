#ifndef _ef_calib_imageslice_HPP_
#define _ef_calib_imageslice_HPP_


#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

#include <ef_calib/picker/Config.hpp>

namespace ef_calib {
    namespace picker {

    struct ImageCenter
	{
		ImageCenter(cv::Mat &image, std::vector<cv::Point2f> &outCenters) : image_(image.clone()), outCenters_(outCenters) {}
		typedef std::shared_ptr<ImageCenter> Ptr;
        // image_(image.clone()) means copyto image_ and something happen to image will not happen on image_
        // image_(image) means the pointer of image is copyto image_ and something happen to image will happen on image_ too

		cv::Mat image_;
		std::vector<cv::Point2f> outCenters_;
	};

    class ImageSlice{
    public:
        typedef std::shared_ptr<ImageSlice> Ptr;

        ImageSlice(cv::Mat &image, uint64_t timestamp, std::vector<cv::Point2f> &outCenters)
        :image_(image.clone()), time_(timestamp), centers(outCenters)
        {};

        virtual inline cv::Mat image() const noexcept { return image_;}
        virtual inline uint64_t time() const noexcept { return time_;}
        virtual inline std::vector<cv::Point2f> OriginalCircles() const noexcept { return centers;}
        virtual inline std::vector<cv::Point2f> Circles() const noexcept { return edgeCenter;}
        // virtual inline std::vector<double> Radiuss() const noexcept { return edgeRadius;}

        bool checkParallel();
        bool checkPattern(cv::Mat &img, ::ef_calib::picker::pattern_Config &pattern);

        int findCirclesBesedOnEdge(::ef_calib::picker::pattern_Config &pattern, cv::Mat &imagePattern, cv::Mat &image_temp, cv::Mat &image_temp2);
        int biasx(){return this->bias_x;}
        int biasy(){return this->bias_y;}

    protected:
        cv::Mat image_;
        uint64_t time_;
        
        int radius, bias_x, bias_y;
        std::vector<cv::Point2f> centers;
        std::vector<cv::Point2f> edgeCenter;
        std::vector<double> edgeRadius;

        Eigen::VectorXd edgeGood;

    };// ImageSlice
} //Namespace picker
} //Namespace ef_calib
#endif //_ef_calib_imageslice_HPP_