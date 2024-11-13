#include <ef_calib/picker/ImageSlice.hpp>

using namespace ef_calib::picker;

bool ImageSlice::checkParallel(){
    if(centers.size() < 10 )
        return false;

    Eigen::Vector2d lineBase = Eigen::Vector2d(centers[0].x - centers[1].x, centers[0].y - centers[1].y);
    double lineBaseLength = lineBase.norm();
    for(int i = 2; i<centers.size(); i+=2){
        Eigen::Vector2d line = Eigen::Vector2d(centers[i].x - centers[i+1].x, centers[i].y - centers[i+1].y);
        double angle = std::acos( (line(0) * lineBase(0) + line(1) * lineBase(1) ) / (line.norm()*lineBaseLength) );
        // std::cout<<" angle between two lines: "<<angle<<" lineBase = "<<lineBaseLength<<" line = "<<line.transpose()<<std::endl;
        if(angle > 0.1)
            return false;
    }

    return true;
}

bool isEdge(int i, ::ef_calib::picker::pattern_Config &pattern){
    if(i>pattern.rows*pattern.cols - 1 - pattern.cols || i<pattern.cols)
        return true;
    if(i%(2*pattern.cols) == 0 || i%(2*pattern.cols) == 3)
        return true;      
    return false;
}

bool ImageSlice::checkPattern(cv::Mat &img, ::ef_calib::picker::pattern_Config &pattern){
    //only happen when 2*7
    // edgeGood = Eigen::VectorXd::Ones(edgeCenter.size());
    for(int i = 0; i<edgeCenter.size(); i+=1){
        if(edgeRadius[i] < this->radius * 0.85 || edgeRadius[i] > this->radius * 1.3){
            std::cout<<"** [Frame] checking: i = "<<i<<", radius "<< edgeRadius[i]<< std::endl;
            // cv::circle(img, cv::Point2f(edgeCenter[i].x - this->bias_x, edgeCenter[i].y - this->bias_y), this->radius, 100, cv::FILLED);
            // edgeGood[i] = 0;
            // if(isEdge(i, pattern))
            return false; 
        }
        
    }
     // std::cout<<"** [Frame] checking: edgeGood.sum() = "<<edgeGood.sum()<<std::endl;
    
   
    // if(edgeGood.sum()<7+2*2-2)
    //     return false;

    // for(int i = 0; i<edgeCenter.size(); i+=1){
    //     if(!isEdge(i, pattern)){
    //         if( i%(2*pattern.cols) == 2 ){            
    //             Eigen::Vector2d line1 = Eigen::Vector2d(edgeCenter[i].x - edgeCenter[i-2].x, edgeCenter[i].y - edgeCenter[i-2].y);
    //             Eigen::Vector2d line2 = Eigen::Vector2d(edgeCenter[i].x - edgeCenter[i-1].x, edgeCenter[i].y - edgeCenter[i-1].y);
    //             Eigen::Vector2d line3 = Eigen::Vector2d(edgeCenter[i].x - edgeCenter[i+3].x, edgeCenter[i].y - edgeCenter[i+3].y);
    //             Eigen::Vector2d line4 = Eigen::Vector2d(edgeCenter[i].x - edgeCenter[i+2].x, edgeCenter[i].y - edgeCenter[i+2].y);

    //             double angle13 = std::acos( (line1(0) * line3(0) + line1(1) * line3(1) ) / (line1.norm()*line3.norm()) ) * 180.0 / 3.1415926;
    //             double angle24 = std::acos( (line2(0) * line4(0) + line2(1) * line4(1) ) / (line2.norm()*line4.norm()) ) * 180.0 / 3.1415926;
    //             std::cout<<" "<<i<<", angle13 = "<<angle13<<", angle24 = "<<angle24<<std::endl;
    //             if( angle13<178.0 || angle24<178.0){
    //                 cv::circle(img, cv::Point2f(edgeCenter[i].x - this->bias_x, edgeCenter[i].y - this->bias_y), this->radius, 100, cv::FILLED);
    //                 edgeGood[i] = 0;
    //             }
    //         }
    //         if( i%(2*pattern.cols) == 1 ){
    //             Eigen::Vector2d line1 = Eigen::Vector2d(edgeCenter[i].x - edgeCenter[i-3].x, edgeCenter[i].y - edgeCenter[i-3].y);
    //             Eigen::Vector2d line2 = Eigen::Vector2d(edgeCenter[i].x - edgeCenter[i-2].x, edgeCenter[i].y - edgeCenter[i-2].y);
    //             Eigen::Vector2d line3 = Eigen::Vector2d(edgeCenter[i].x - edgeCenter[i+2].x, edgeCenter[i].y - edgeCenter[i+2].y);
    //             Eigen::Vector2d line4 = Eigen::Vector2d(edgeCenter[i].x - edgeCenter[i+1].x, edgeCenter[i].y - edgeCenter[i+1].y);

    //             double angle13 = std::acos( (line1(0) * line3(0) + line1(1) * line3(1) ) / (line1.norm()*line3.norm()) ) * 180.0 / 3.1415926;
    //             double angle24 = std::acos( (line2(0) * line4(0) + line2(1) * line4(1) ) / (line2.norm()*line4.norm()) ) * 180.0 / 3.1415926;
    //             std::cout<<" "<<i<<", angle13 = "<<angle13<<", angle24 = "<<angle24<<std::endl;
    //             if( angle13<178.0 || angle24<178.0){
    //                 cv::circle(img, cv::Point2f(edgeCenter[i].x - this->bias_x, edgeCenter[i].y - this->bias_y), this->radius, 100, cv::FILLED);
    //                 edgeGood[i] = 0;
    //             }
    //         }
    //     }
    // }

    //     if(angle > 0.05){
    //         cv::circle(img, cv::Point2f(edgeCenter[i].x - this->bias_x, edgeCenter[i].y - this->bias_y), 1, 0, cv::FILLED);
    //         cv::circle(img, cv::Point2f(edgeCenter[i+1].x - this->bias_x, edgeCenter[i+1].y - this->bias_y), 1, 0, cv::FILLED);


    return true;
}

void fitCircle2(const std::vector<Eigen::Vector2d> &data, 
                    Eigen::Ref<Eigen::Vector2d> center, double &rad) {
    double sum_x = 0, sum_y = 0;
    double sum_xx = 0, sum_yy = 0, sum_xy = 0;
    double sum_xxx = 0, sum_yyy = 0, sum_xyy = 0, sum_xxy = 0;
    for (int p_idx = 0; p_idx < data.size(); p_idx++) {
        const Eigen::Vector2d &sample = data[p_idx];

        sum_x += sample[0];
        sum_y += sample[1];

        double xx = sample[0] * sample[0];
        double yy = sample[1] * sample[1];
        double xy = sample[0] * sample[1];

        sum_xx += xx;
        sum_yy += yy;
        sum_xy += xy;

        sum_xxx += xx * sample[0];
        sum_yyy += yy * sample[1];
        sum_xyy += xy * sample[1];
        sum_xxy += sample[0] * xy;
    }

    Eigen::Matrix3d A;
    A << 2 * sum_x, 2 * sum_y, data.size(),
        2 * sum_xx, 2 * sum_xy, sum_x,
        2 * sum_xy, 2 * sum_yy, sum_y;
    Eigen::Vector3d b(sum_xx + sum_yy, sum_xxx + sum_xyy, sum_xxy + sum_yyy);

    Eigen::Vector3d x = A.lu().solve(b);
    center = x.block<2, 1>(0, 0);
    rad = std::sqrt(x[0] * x[0] + x[1] * x[1] + x[2]);
}


int ImageSlice::findCirclesBesedOnEdge(::ef_calib::picker::pattern_Config &pattern, cv::Mat &imagePattern, cv::Mat &image_temp, cv::Mat &image_temp2){

    Eigen::Vector2d line = Eigen::Vector2d(centers[0].x - centers[1].x, centers[0].y - centers[1].y);
    this->radius = pattern.circleRadius/(pattern.squareSize*2.0) * std::abs(line.norm());
    // std::cout<<"** [Frame] radius_assuming = circleRadius/squareSize*line.norm() = "<<radius<<std::endl;

    int min_x_idx = centers[12].x<centers[13].x ? 12 : 13 ;
    int min_y_idx = centers[12].y<centers[ 4].y ? 12 :  4 ;
    this->bias_x = centers[min_x_idx].x - 4*radius;
    this->bias_y = centers[min_y_idx].y - 4*radius;
    int bias_xlength = std::abs(centers[13-min_x_idx].x - centers[min_x_idx].x) + 8*radius;
    int bias_ylength = std::abs(centers[15-min_y_idx].y - centers[min_y_idx].y) + 8*radius;
    if( bias_x>0 && bias_x<848-1 && bias_y>0 && bias_y<480-1 &&
        bias_x+bias_xlength>0 && bias_x+bias_xlength<848-1 && bias_y+bias_ylength>0 && bias_y+bias_ylength<480-1	)
    {
        cv::Rect roi(bias_x, bias_y, bias_xlength, bias_ylength );
        imagePattern = image_(roi).clone();
        image_temp2 = image_(roi).clone();
        image_temp = cv::Mat(bias_ylength, bias_xlength, CV_8UC1, cv::Scalar(0));

        // ** Get max elem for each patch ** //
        std::vector< std::vector<Eigen::Vector2d> > edges;
        edges.resize(centers.size());
        edgeCenter.resize(centers.size());
        edgeRadius.resize(centers.size());
        int patchNumber = 400/centers.size();
        int PatchError = 0;
        for(int patch_idx = 0; patch_idx < edges.size(); patch_idx++){
            int PatchSize = 1.5 * radius;
            int Patch_x = centers[patch_idx].x - bias_x - PatchSize;
            int Patch_y = centers[patch_idx].y - bias_y - PatchSize;
            // std::cout<<" ** patch No."<<patch_idx<<" at ["<<Patch_x<<", "<<Patch_y<<"] with range ["<<2*PatchSize<<", "<<2*PatchSize<<"]. "<<std::endl;
            if( Patch_x>0 && Patch_x<bias_xlength-1 && Patch_y>0 && Patch_y<bias_ylength-1 &&
                Patch_x+ 2*PatchSize>0 && Patch_x+ 2*PatchSize<bias_xlength-1 && Patch_y+ 2*PatchSize>0 && Patch_y+ 2*PatchSize<bias_ylength-1	)
            {
                cv::Rect roiPatch(Patch_x, Patch_y, 2*PatchSize, 2*PatchSize);
                cv::Mat patch = imagePattern(roiPatch).clone();

                // ** Sobel detection for edges of frame image. ** //
                cv::Mat grad_x, grad_y, abs_grad_x, abs_grad_y, grad;
                cv::Sobel(patch, grad_x, CV_16S, 1, 0, 3);
                cv::convertScaleAbs(grad_x, abs_grad_x);
                cv::Sobel(patch, grad_y, CV_16S, 0, 1, 3);
                cv::convertScaleAbs(grad_y, abs_grad_y);
                cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);

                cv::circle(image_temp2, cv::Point(centers[patch_idx].x - bias_x, centers[patch_idx].y - bias_y), 1, 255, cv::FILLED);
                // image_temp = grad.clone();
                for(int y=0;y<PatchSize*2;y++)
                for(int x=0;x<PatchSize*2;x++){
                    image_temp.at<uchar>(Patch_y+y,Patch_x+x) = grad.at<uchar>(y,x);
                }

                for (int i=0; i<patchNumber; ++i)
                {
                    double min, max;
                    cv::Point min_loc, max_loc;
                    cv::minMaxLoc(grad, &min, &max, &min_loc, &max_loc);
                    // std::cout<<"max mag: "<<max<<", pos = "<<max_loc.x<<", "<<max_loc.y<<std::endl;
                    if (max == min || max <= 30.0 ) break;
                    edges[patch_idx].emplace_back(Eigen::Vector2d(max_loc.x, max_loc.y));
                    grad.at<uchar>(max_loc) = 0;
                }
                // std::cout<<", EdgeSize = "<<edges[patch_idx].size();
                // for(int i=0; i<edges[patch_idx].size(); i++){
                // 	cv::circle(image2, cv::Point(edges[patch_idx][i](0) + Patch_x, edges[patch_idx][i](1) + Patch_y), 1, cv::Scalar(0), cv::FILLED);
                // }
                if(edges[patch_idx].size() >= 10){
                    Eigen::Vector2d center;
                    fitCircle2(edges[patch_idx], center, edgeRadius[patch_idx]);
                    
                    // cv::circle(grad, cv::Point2f(center(0), center(1)), 1, cv::Scalar(255), cv::FILLED);
                    // image3 = grad.clone();
                    // std::cout<<"** [Frame] NO."<<patch_idx<<" raidus = "<< edgeRadius[patch_idx]<<std::endl;
                    cv::circle(imagePattern, cv::Point2f(center(0) + Patch_x, center(1) + Patch_y), edgeRadius[patch_idx], cv::Scalar(255), 1.5, cv::LINE_AA);
                    cv::circle(imagePattern, cv::Point2f(center(0) + Patch_x, center(1) + Patch_y), 1, cv::Scalar(255), cv::FILLED);

                    edgeCenter[patch_idx] = cv::Point2f(center(0) + Patch_x + bias_x, center(1) + Patch_y + bias_y);
                    // std::cout<<" center = "<<edgeCenter[patch_idx];
                } else {
                    std::cout<<"** [Frame] the point from sobel edge is not enough. NO."<<patch_idx<<std::endl;
                    PatchError++;
                    return PatchError;
                    // edgeCenter[patch_idx] = centers[patch_idx];
                }

            } else {
                std::cout<<"** [Frame] the patch is out of range. The number is "<<patch_idx<<std::endl;
                return PatchError;
                PatchError++;
            }
        }
        // cv::circle(imagePattern, cv::Point2f(centers[min_x_idx].x-this->bias_x, centers[min_x_idx].y-this->bias_y), 1, cv::Scalar(0), cv::FILLED);
        // cv::circle(imagePattern, cv::Point2f(centers[min_y_idx].x-this->bias_x, centers[min_y_idx].y-this->bias_y), 1, cv::Scalar(0), cv::FILLED);
        // cv::circle(imagePattern, cv::Point2f(centers[13-min_x_idx].x-this->bias_x, centers[13-min_x_idx].y-this->bias_y), 3, cv::Scalar(0), cv::FILLED);
        // cv::circle(imagePattern, cv::Point2f(centers[15-min_y_idx].x-this->bias_x, centers[15-min_y_idx].y-this->bias_y), 3, cv::Scalar(0), cv::FILLED);
        return PatchError;
    }
    return centers.size();
}