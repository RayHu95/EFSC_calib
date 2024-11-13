#include <tools/tools.hpp>


namespace tools {

    std::string type2str(int type){
        std::string r;

        uchar depth = type & CV_MAT_DEPTH_MASK;
        uchar chans = 1 + (type >> CV_CN_SHIFT);

        switch ( depth ) {
            case CV_8U:  r = "8U"; break;
            case CV_8S:  r = "8S"; break;
            case CV_16U: r = "16U"; break;
            case CV_16S: r = "16S"; break;
            case CV_32S: r = "32S"; break;
            case CV_32F: r = "32F"; break;
            case CV_64F: r = "64F"; break;
            default:     r = "User"; break;
        }

        r += "C";
        r += (chans+'0');

        return r;
    }


    double medianMat(cv::Mat &_in)
    {
        cv::Mat in = _in.clone();
        in = in.reshape(0,1);
        std::vector<double> vec_from_mat;
        in.copyTo(vec_from_mat);
        std::nth_element(vec_from_mat.begin(), vec_from_mat.begin() + vec_from_mat.size() / 2, vec_from_mat.end());
        return vec_from_mat[vec_from_mat.size() / 2];
    }
}// namespace tools
