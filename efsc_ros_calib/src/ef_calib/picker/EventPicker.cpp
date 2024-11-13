#include <ef_calib/picker/EventPicker.hpp>
#include <ef_calib/picker/MotionCompensate.hpp>

using namespace ef_calib::picker;

EventPicker::EventPicker(const ::ef_calib::picker::Config &config, ::tools::CameraInfo &camera, ::ef_calib::picker::pattern_Config &pattern)
{
    this->config = config;
    this->height = camera.height;    this->width = camera.width;
    // std::cout<<"[Picker] EventNumThreshold["<<this->config.frameEventNumThreshold<<"] "<<std::endl;
    if(config.fitCircle) {std::cout<<"[Picker] Attetion Hard detection. fitCircle["<< config.fitCircle <<"]"<<std::endl;}

    this->pattern = pattern;
    
    double circleRadiusThreshold; //Calculate 'circleRadiusThreshold_' 
    if (pattern.isAsymmetric) {
        circleRadiusThreshold = std::min(
            std::max(width, height) / std::max(pattern.rows, 2 * pattern.cols),
            std::min(width, height) / std::min(pattern.rows, 2 * pattern.cols)) 
            / pattern.squareSize * pattern.circleRadius * 1.5;
    } else {
        circleRadiusThreshold = std::min(
            std::max(width, height) / std::max(pattern.rows, pattern.cols),
            std::min(width, height) / std::min(pattern.rows, pattern.cols)) 
            / pattern.squareSize * pattern.circleRadius * 1.5;
    }
    this->pattern.RThreshold = circleRadiusThreshold;
    pattern.RThreshold = circleRadiusThreshold;
    std::cout<<"[Picker] circleRadiusThreshold[" << this->pattern.RThreshold << "]"<<std::endl; 
}

bool EventPicker::extractFeatures(std::shared_ptr<ef_calib::picker::EventSlice> es) {
    //find clusters
    if(!es->clusterEvents(this->height, this->width, this->config)){
        return false;
    }

    // too few clusters of event.
    if (es->pClusters_.size() < pattern.rows * pattern.cols || es->nClusters_.size() < pattern.rows * pattern.cols) {
        return false;
    }
    //std::cout<<"[Picker] p_clusters[" << es->pClusters_.size() << "], n_clusters[" << es->nClusters_.size() << "], rows*cols["<<pattern.rows * pattern.cols<<"]"<<std::endl; 

    //find the centers of each clusters
    vectorofEigenMatrix<Eigen::Vector2d> pCenters, nCenters;
    es->centers(pCenters, nCenters);
    // building k-d tree
    KDTreeVectorOfVectorsAdaptor<vectorofEigenMatrix<Eigen::Vector2d>, double, 2, nanoflann::metric_L2_Simple>
            p_kdTree(2, pCenters, 10), n_kdTree(2, nCenters, 10);


    /** find candidate circles **/
    std::vector<std::pair<size_t, size_t>> candidates;
    vectorofEigenMatrix<Eigen::Vector2d> candidateCenters;
    std::vector<double> candidatesRadius;

    // const size_t num_results = this->config.fitCircle ? config.knn_num : 1;
    std::vector<size_t> n_idx(1);
    std::vector<size_t> p_idx(1);
    std::vector<double> out_dists_sqr(1);


    for (size_t pi = 0; pi < pCenters.size(); ++pi) {
        n_kdTree.query(pCenters[pi].data(), 1, n_idx.data(), out_dists_sqr.data());
        // remove cluster too far away
        if (out_dists_sqr[0] > 4 * this->pattern.RThreshold * this->pattern.RThreshold) {
            continue;
        }
        p_kdTree.query(nCenters[n_idx[0]].data(), 1, p_idx.data(),
                        out_dists_sqr.data());
        if (p_idx[0] == pi) {
            // check circle
            Eigen::Vector2d center =
                    (pCenters[p_idx[0]] + nCenters[n_idx[0]]) / 2;
            double r = (pCenters[p_idx[0]] - nCenters[n_idx[0]]).norm() / 2;

            double fitErr = es->calculatefitErr(center, r, p_idx[0], n_idx[0]);                

            if (fitErr < 10 / r) {
                candidates.emplace_back(pi, n_idx[0]);
                candidateCenters.push_back(center);
                candidatesRadius.push_back(r);
            }
        }
    }

    // std::cout<<"[Picker] candidates.size["<<candidates.size()<<"]"<<std::endl;
    // draw candidate circle
    for (int i = 0; i < candidates.size(); ++i) {
        cv::Point loc(candidateCenters[i][0], candidateCenters[i][1]);
        cv::circle(es->eventImage, loc, candidatesRadius[i], cv::Vec3b(0, 255, 0));
    }
    
    // use prior pattern to match the candidate 
    std::vector<cv::Point2f> outCenters;
    cv::Mat points(height, width, CV_8UC1, cv::Scalar(255));
    for (const Eigen::Vector2d &center: candidateCenters) {
        cv::Point loc(center[0], center[1]);
        cv::circle(points, loc, 1, 0, -1);
        //points.at<uchar>(center[1], center[0]) = 1; //The order the x,y may be incorrect
    }

    //ROS_INFO("candidatesRadius.size = %d",candidatesRadius.size());
    cv::SimpleBlobDetector::Params dParams;
    //dParams.filterByColor = false; //no influence
    dParams.filterByArea = false;   //great influence, The area of these points
    //dParams.filterByInertia = false;  //without this setting, even better.
    //dParams.filterByCircularity = false; //without this setting, even better.
    //dParams.filterByConvexity = false; //no influence
    bool isFound = cv::findCirclesGrid(points, cv::Size(this->pattern.cols, this->pattern.rows), outCenters,
                                      cv::CALIB_CB_ASYMMETRIC_GRID, cv::SimpleBlobDetector::create(dParams));
    if (!isFound)
    isFound = cv::findCirclesGrid(points, cv::Size(this->pattern.cols, this->pattern.rows), outCenters,
                                      cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, cv::SimpleBlobDetector::create(dParams));
   
    // eventImage = points.clone();
    // cv::drawChessboardCorners(es->eventImage, cv::Size(this->pattern.cols, this->pattern.rows), cv::Mat(outCenters), isFound);

    if (isFound) {
        // cv::circle(es->eventImage, cv::Point(outCenters[0].x, outCenters[0].y), 10, cv::Vec3b(255, 255, 255));
        KDTreeVectorOfVectorsAdaptor<vectorofEigenMatrix<Eigen::Vector2d>, double, 2, nanoflann::metric_L2_Simple>
                kdTree(2, candidateCenters, 10);
        std::vector<size_t> orderIdxs(this->pattern.rows * this->pattern.cols);
        std::vector<double> out_dist_sqr(1);
        for (int i = 0; i < outCenters.size(); ++i) {
            Eigen::Vector2d c(outCenters[i].x, outCenters[i].y);
            kdTree.query(c.data(), 1, orderIdxs.data() + i, out_dist_sqr.data());
        }
        // features_
        for (auto idx: orderIdxs) {
            es->pushFeature(candidateCenters[idx], candidatesRadius[idx]);
        }
    }

    return isFound;
}

cv::Mat sobel_gradient(cv::Mat image_temp, int flag, int width, int height)
{
    if(flag == 1){
        cv::Mat sx = (cv::Mat_<int>(3,3)<<-1, 0, 1, -2, 0, 2, -1, 0, 1);
        cv::Mat	Ix = cv::Mat(height, width, CV_32FC1, cv::Scalar::all(0));
        //cv::Point(-1, -1) means the Achter is located at the center
        cv::filter2D(image_temp, Ix, CV_32FC1, sx, cv::Point(-1, -1));
        return Ix;
    }
    else{
        cv::Mat sy = (cv::Mat_<int>(3,3)<<-1, -2, -1, 0, 0, 0, 1, 2, 1);
        cv::Mat	Iy = cv::Mat(height, width, CV_32FC1, cv::Scalar::all(0));
        //cv::Point(-1, -1) means the Achter is located at the center
        cv::filter2D(image_temp, Iy, CV_32FC1, sy, cv::Point(-1, -1)); 
        return Iy;
    }
}

cv::Mat EventPicker::GradientAnalytic(Eigen::VectorXd xp_in, Eigen::VectorXd yp_in, Eigen::VectorXd delta_time_in, int64_t &f_Frobenius, Eigen::Vector2f &Gradient, const Eigen::Vector2f vel, int r)
{
    float vel_norm = vel.norm();
    Eigen::VectorXd xp, yp, delta_time;
    int size_in = xp_in.size();
    xp.resize(size_in);
    yp.resize(size_in);
    delta_time.resize(size_in);
    uint16_t e_size = 0; // valid size for events
    int range = 2*r;
    
    cv::Mat image = cv::Mat( range, range, CV_8UC1, cv::Scalar(0));

    int x_new, y_new;
    for(int i = 0; i < size_in; i++){
        if(vel_norm < 1e-15){
            x_new = xp_in(i);
            y_new = yp_in(i);
        } else {
            x_new = std::round( xp_in(i) - vel(0) *  delta_time_in(i) );
            y_new = std::round( yp_in(i) - vel(1) *  delta_time_in(i) );
        }
        

        if( x_new > 0 && x_new < range && y_new > 0 && y_new < range){
            xp(e_size) = x_new;
			yp(e_size) = y_new;
			delta_time(e_size) = delta_time_in(i);
            // std::cout<<" "<< delta_time(e_size);
            image.at<uchar>(y_new, x_new) += 1;
            e_size++;
        }
    }//end for

    cv::medianBlur(image, image, 3);
    //cv::GaussianBlur(image, image, cv::Size(5, 5), 1);
    
    cv::Mat Ix = sobel_gradient(image, 1, range, range);
    cv::Mat Iy = sobel_gradient(image, 2, range, range);
    int Np = cv::countNonZero(image);

    f_Frobenius = 0;
    for(int e_itr = 0; e_itr < e_size ; e_itr++ ){

        f_Frobenius += image.at<uchar>(yp(e_itr), xp(e_itr)) * image.at<uchar>(yp(e_itr), xp(e_itr));

        Gradient(0) += Ix.at<float>(yp(e_itr), xp(e_itr)) * image.at<uchar>(yp(e_itr), xp(e_itr)) * delta_time(e_itr) * 0.0000001f;
        Gradient(1) += Iy.at<float>(yp(e_itr), xp(e_itr)) * image.at<uchar>(yp(e_itr), xp(e_itr)) * delta_time(e_itr) * 0.0000001f;   
    }
    
    // std::cout<<"NonZero = "<<Np<<", inside events = "<<e_size<<", f_Frobenius = "<<std::setprecision(10)<<f_Frobenius<<", Gradient = "<<Gradient(0)<<", "<<Gradient(1)<<std::endl;
    return image;
}

void EventPicker::divideEvents(const dv::EventStore &events, std::vector<CircleFeature::Ptr> &features, const size_t &num_results, 
                               std::vector<EventPathes::Ptr> &events_patches, uint64_t time_ref){
                                
    // forming the kd-tree of events
    std::unordered_set<Eigen::Vector3d, EigenMatrixHash3d<Eigen::Vector3d>, std::equal_to<>,
        Eigen::aligned_allocator<Eigen::Vector3d>> pnEvents;
    for(const dv::Event &event : events)
    {
        Eigen::Vector3d e_loc_time(event.x(), event.y(), event.timestamp());
        pnEvents.insert(e_loc_time);
    }
    vectorofEigenMatrix<Eigen::Vector3d> Events_;
    std::move(pnEvents.begin(), pnEvents.end(), std::back_inserter(Events_));
    KDTreeVectorOfVectorsAdaptor3d<vectorofEigenMatrix<Eigen::Vector3d>, double, 2, nanoflann::metric_L2_Simple>
        events_kdTree(2, Events_, 50);

    // Extract events within a certain circle from a large number of events
    for(auto itr = 0; itr < features.size(); itr++){
        Eigen::Vector2d loc = features[itr]->loc_;
        double rad = features[itr]->radius_;
        std::vector<size_t> events_idx(num_results);
        std::vector<double> out_dists_sqr(num_results);
        events_kdTree.query(loc.data(), num_results, events_idx.data(), out_dists_sqr.data());

        Eigen::VectorXd xp, yp, delta_time;
        xp.resize(num_results);
        yp.resize(num_results);
        delta_time.resize(num_results);
        
        double range =  this->pattern.RThreshold > 2*rad ? 2*rad : this->pattern.RThreshold;
        int range_max = std::ceil(range);
        int numbers = 0;
        int biasx = std::floor(loc(0) - range_max);
        int biasy = std::floor(loc(1) - range_max);
        for (int idx = 0; idx < num_results; idx++ ) {
            if(out_dists_sqr[idx] < range * range ){
                xp(idx) = Events_[events_idx[idx]][0] - biasx;
                yp(idx) = Events_[events_idx[idx]][1] - biasy;
                delta_time(idx) = (Events_[events_idx[idx]][2] - time_ref);
                numbers ++ ;
            }
            else{
                xp(idx) = 0.0;
                yp(idx) = 0.0;
            }
        }
        // std::cout<<"** [Event] [Picker] "<<itr<<" rad = "<<rad<<", MaxRange = "<<range_max<<" with "<<numbers<<" events"<<std::endl;
        events_patches.push_back(std::make_shared<EventPathes>(xp ,yp, delta_time, numbers, biasx, biasy, range_max));
    }

    // Events_.clear();
}


void EventPicker::MotionCompensate(std::vector<ef_calib::picker::EventPathes::Ptr> &patches, Eigen::Vector2d &vel){

    // std::cout<<"** [MotionCom] range = "<<range<<std::endl;
    
    Eigen::VectorXd vel_x, vel_y;
    vel_x.resize(1);
    vel_y.resize(1);
    vel_x(0) = vel[0];     
    vel_y(0) = vel[1];
    // ** generate problem **//
    ceres::Problem problem;
    for(int idx=2; idx< 5; idx++){
    // 	cv::Mat image = cv::Mat( range *2, range*2, CV_8UC1, cv::Scalar(0));
    	::ef_calib::picker::EventPathes::Ptr patch = patches[idx];

    	Eigen::VectorXd xp = patch->xp_;
    	Eigen::VectorXd yp = patch->yp_;
    	Eigen::VectorXd tp = patch->delta_time_;
    	int e_data = 0;
        Eigen::VectorXd xp_data, yp_data, tp_data;
    	xp_data.resize(patch->inlier_numbers_);
    	yp_data.resize(patch->inlier_numbers_);
    	tp_data.resize(patch->inlier_numbers_);
    	for(int i = 0; i < xp.size(); i++){
    		if( xp(i) > 0.0 && yp(i) > 0.0 ){
    			xp_data(e_data) = xp(i);
    			yp_data(e_data) = yp(i);
    			tp_data(e_data) = tp(i);
     			// std::cout<<" "<<xp_data(e_data)<<" "<< yp_data(e_data)<<" "<< tp_data(e_data)<<std::endl;
    			e_data ++;
    		}
     	}
    	// std::cout<<"** [MotionCom] "<<"No."<<idx<<", inliers_number = "<<e_data<<std::endl;

    	ceres::CostFunction* cost_function_X = new ::ef_calib::picker::MotionComX(xp_data, yp_data, tp_data);
    	problem.AddResidualBlock(cost_function_X, nullptr, vel_x.data());
        // ceres::CostFunction* cost_function_Y = new ::ef_calib::picker::MotionComY(xp_data, yp_data, tp_data);
    	// problem.AddResidualBlock(cost_function_Y, nullptr, vel_y.data());
        // ceres::CostFunction* cost_function = new ::ef_calib::picker::MotionCom(xp_data, yp_data, tp_data);
    	// problem.AddResidualBlock(cost_function, nullptr, vel.data());
    }

    // Set solver options (precision / method)
    ceres::Solver::Options options;
    options.max_num_iterations = 80; 
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    // options.linear_solver_type = ceres::DENSE_QR;
    // options.minimizer_progress_to_stdout = true;

    // ** Solve ** //
    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);
    // std::cout << summary.FullReport() << std::endl;


    vel[0] = vel_x(0);
    vel[1] = vel_y(0);
    std::cout<<"** [MotionCom] vx & vy = "<<vel.transpose()<<std::endl;

}

void EventPicker::MotionCompensateFull(const dv::EventStore &events, Eigen::Vector2d &vel, int &num, uint64_t time_ref, cv::Mat &image_es2){
    image_es2 = cv::Mat(260, 346, CV_8UC1, cv::Scalar(255));
    num=0;
    for(const dv::Event &event : events)
    {
        num++;
        image_es2.at<uchar>(event.y(), event.x()) -= 8.0;
    }
    Eigen::VectorXd xp_data, yp_data, tp_data;
    xp_data.resize(num);
    yp_data.resize(num);
    tp_data.resize(num);
    int idx = 0;
    for(const dv::Event &event : events)
    {
        xp_data(idx) = event.x();
        yp_data(idx) = event.y();
        int64_t delta = event.timestamp() - time_ref;
        tp_data(idx) = delta;
        // if(event.timestamp() < time_ref)
            // std::cout<<"No. "<<idx<<", ("<<xp_data(idx)<<", "<<yp_data(idx)<<") at "<<tp_data(idx)<<std::endl;
        idx++;
    }
    // std::cout<<"idx = "<<idx<<", num = "<<num<<std::endl;

    ceres::Problem problem;
    Eigen::VectorXd vel_x;
    vel_x.resize(1);
    vel_x(0) = vel[0];

    ceres::CostFunction* cost_function = new ::ef_calib::picker::MotionComFull(xp_data, yp_data, tp_data);
    problem.AddResidualBlock(cost_function, nullptr, vel_x.data());

    // std::cout<<" start optimization"<<std::endl;
    // Set solver options (precision / method)
    ceres::Solver::Options options;
    options.max_num_iterations = 80; 
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    // options.linear_solver_type = ceres::DENSE_QR;
    // options.minimizer_progress_to_stdout = true;

    // ** Solve ** //
    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);
    // std::cout << summary.FullReport() << std::endl;

    vel[0] = vel_x(0);
    std::cout<<"** [MotionCom2] vx & vy = "<<vel.transpose()<<std::endl;
}

void EventPicker::fitCircle(const std::vector<cv::Point2d> &candidate,
                            Eigen::Ref<Eigen::Vector2d> center, double &radius) {
    double sum_x = 0, sum_y = 0;
    double sum_xx = 0, sum_yy = 0, sum_xy = 0;
    double sum_xxx = 0, sum_yyy = 0, sum_xyy = 0, sum_xxy = 0;
    for (auto it=candidate.begin(); it!=candidate.end(); ++it){
        sum_x += it->x;
        sum_y += it->y;

        double xx = it->x * it->x;
        double yy = it->y * it->y;
        double xy = it->x * it->y;

        sum_xx += xx;
        sum_yy += yy;
        sum_xy += xy;

        sum_xxx += xx * it->x;
        sum_yyy += yy * it->y;
        sum_xyy += xy * it->y;
        sum_xxy += it->x * xy;
    }

    Eigen::Matrix3d A;
    A << 2 * sum_x, 2 * sum_y, candidate.size(),
            2 * sum_xx, 2 * sum_xy, sum_x,
            2 * sum_xy, 2 * sum_yy, sum_y;
    Eigen::Vector3d b(sum_xx + sum_yy, sum_xxx + sum_xyy, sum_xxy + sum_yyy);

    Eigen::Vector3d x = A.lu().solve(b);
    center = x.block<2, 1>(0, 0);
    radius = std::sqrt(x[0] * x[0] + x[1] * x[1] + x[2]);
}

void EventPicker::fitCircleSubpixel(const std::vector<cv::Point3d> &candidate,
                                    Eigen::Ref<Eigen::Vector2d> center, double &radius) {
    double sum_wx = 0, sum_wy = 0, sum_w = 0;
    double sum_wxx = 0, sum_wyy = 0, sum_wxy = 0;
    double sum_wxxx = 0, sum_wyyy = 0, sum_wxyy = 0, sum_wxxy = 0;
    for (auto it=candidate.begin(); it!=candidate.end(); ++it){
        double weight = it->z;
        sum_wx += weight * it->x;
        sum_wy += weight * it->y;
        sum_w += weight;

        double wxx = weight * it->x * it->x;
        double wyy = weight * it->y * it->y;
        double wxy = weight * it->x * it->y;

        sum_wxx += wxx;
        sum_wyy += wyy;
        sum_wxy += wxy;

        sum_wxxx += wxx * it->x;
        sum_wyyy += wyy * it->y;
        sum_wxyy += wxy * it->y;
        sum_wxxy += wxy * it->x;
    }

    Eigen::Matrix3d A;
    A <<    2 * sum_wx,  2 * sum_wy,  sum_w,
            2 * sum_wxx, 2 * sum_wxy, sum_wx,
            2 * sum_wxy, 2 * sum_wyy, sum_wy;
    Eigen::Vector3d b(sum_wxx + sum_wyy, sum_wxxx + sum_wxyy, sum_wxxy + sum_wyyy);

    Eigen::Vector3d x = A.lu().solve(b);
    center = x.block<2, 1>(0, 0);
    radius = std::sqrt(x[0] * x[0] + x[1] * x[1] + x[2]);
}

double EventPicker::EvaluateMC(ef_calib::picker::EventPathes::Ptr &patch, Eigen::Vector2d &vel, cv::Point2f &center, double &radius ){
    Eigen::VectorXd &xp = patch->xp_;
    Eigen::VectorXd &yp = patch->yp_;
    Eigen::VectorXd &tp = patch->delta_time_;
    int e_data = 0;
    double error = 0.0;
     
    // double center_x = center.x - patch->bias_x;
    // double center_y = center.y - patch->bias_y;
    Eigen::Vector2d cen = Eigen::Vector2d(center.x - patch->bias_x, center.y - patch->bias_y);
    double r_real = radius*0.5;
    // std::cout<<" center = "<<cen.transpose()<<", radius = "<<r_real<<std::endl;
    for(int i = 0; i < xp.size(); i++){
        if( xp(i) > 0.0 && yp(i) > 0.0 ){
            e_data ++;
            Eigen::Vector2d radius_vec = Eigen::Vector2d(xp(i), yp(i)) - vel*tp(i) - cen;
            double r_error = radius_vec.norm() - r_real;
            // double x_new = xp(i) - vel(0) * tp(i) - center_x ;
            // double y_new = yp(i) - vel(1) * tp(i) - center_y ;
            
            // std::cout<<" r = "<<radius_vec.transpose()<<", r_error = "<<r_error<<std::endl;
            if(std::abs(r_error) > r_real*0.5)
                error += 10 * r_error*r_error;
            else
                error += 0.5*r_error*r_error;
            // std::cout<<" x_new = "<<xp(i) - vel(0) * tp(i)<<" y_new = "<<yp(i) - vel(1) * tp(i)<<" ["<<std::sqrt(x_new*x_new + y_new*y_new)<<"]"<<std::endl;
        }
    }
    std::cout<<"** [Event] id=5 num = "<<e_data<<", error = "<<error<<", avg = "<<error/(e_data*1.0)<<std::endl;
    return error/(e_data*1.0);
}
