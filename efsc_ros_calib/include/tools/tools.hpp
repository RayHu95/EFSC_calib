#ifndef _TOOLS_HPP_
#define _TOOLS_HPP_

#include <opencv2/opencv.hpp>
#include <numeric>
#include <tools/camerainfo.hpp>

/** Base types **/

namespace tools {
        
    std::string type2str(int type);

    double medianMat(cv::Mat &in);

    
    inline double expWeight(const double &idx, const double &window_size)
    {
        double value = (idx - (window_size/2)) / (window_size/6.0);
        return std::exp(-0.5*value*value);
    };

    template<typename Iter_T>
    double vectorNorm(Iter_T first, Iter_T last)
    {
        return sqrt(inner_product(first, last, first, 0.0L));
    };
    template<typename T>
    T n_quantile_vector(std::vector<T> &vec, const int n)
    {
        std::nth_element(vec.begin(), vec.begin() + n, vec.end());
        return vec[n];
    };


    template<typename T>    
    void mean_std_vector(const std::vector<T> &vec, T &mu, T &std_dev)
    {
        const size_t sz = vec.size();
        if (sz == 1) {
            mu = vec[0]; std_dev=0.0;
            return;
        }

        // Calculate the mean
        mu = std::accumulate(vec.begin(), vec.end(), 0.0) / sz;

        // Now calculate the variance
        auto variance_func = [&mu, &sz](T accumulator, const T& val) {
            return accumulator + ((val - mu)*(val - mu) / (sz - 1));
        };

        std_dev = std::accumulate(vec.begin(), vec.end(), 0.0, variance_func);
    };

    template<class bidiiter>
    bidiiter random_unique(bidiiter begin, bidiiter end, size_t num_random)
    {
        size_t left = std::distance(begin, end);
        while (num_random--) {
            bidiiter r = begin;
            std::advance(r, rand()%left);
            std::swap(*begin, *r);
            ++begin;
            --left;
        }
        return begin;
    };

}// namespace tools
#endif // _TOOLS_HPP_
