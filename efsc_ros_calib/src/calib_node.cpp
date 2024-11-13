#include <calib_node.hpp>

#include <dynamic_reconfigure/server.h>
#include <efsc_ros_calib/CalibConfig.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// #include <ef_calib/calib/LineFitting.hpp>

using namespace std;
using namespace cv;
using namespace std::chrono_literals;

int main(int argc, char **argv) {

	// Initialize the node
	ros::init(argc, argv, "calib_node");
	ros::NodeHandle nh("~");

	auto Pub1 = nh.advertise<dv_ros_msgs::ImageMessage>("/ef_calib/FrameImage", 10);
	auto Pub2 = nh.advertise<dv_ros_msgs::ImageMessage>("/ef_calib/EventSlice", 10);
	auto Pub3 = nh.advertise<dv_ros_msgs::ImageMessage>("/ef_calib/Sebel", 10);
	auto Pub4 = nh.advertise<dv_ros_msgs::ImageMessage>("/ef_calib/MC", 10);
	auto Pub5 = nh.advertise<dv_ros_msgs::ImageMessage>("/ef_calib/debug", 10); // it is for debug if needed

	ros::Publisher pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/pose_visual/pose_pub", 1);
	ros::Publisher pose_pub2_ = nh.advertise<geometry_msgs::PoseStamped>("/pose_visual/pose_pub2", 1);
	ros::Publisher pose_pub3_ = nh.advertise<geometry_msgs::PoseStamped>("/pose_visual/pose_pub3", 1);

	boost::lockfree::spsc_queue<dv::EventStore> eventQueue(200);
	boost::lockfree::spsc_queue<ef_calib::picker::PairSlice::Ptr> efPairQueue(200);
	

	/********************** Load Parameters ********************************/	
	ROS_INFO_STREAM("[calib_node] >>>>>>>>>>>>>>>>>>>> Load params from Yaml  <<<<<<<<<<<<<<<<<<<<");
	ef_calib_node::Task task;

	// load param from Yaml-ros
	task.config.picker  = ::ef_calib::picker::readPickerConfig(nh);
	task.config.pattern = ::ef_calib::picker::readPatternConfig(nh);
	task.config.calib   = ::ef_calib::paramcalib::readCalibConfig(nh);

	bool continueThread = false; // The flag of determining the thread running.
	bool isInitilized = ::tools::param(nh, "IsInitialed", false);
	int initialNum = ::tools::param(nh, "InitialNum", 15); // the number of event-frame pair for the intialization
	int OptimizeNum = ::tools::param(nh, "OptimizeNum", 10); // the number of event-frame pair for the Optimization

	// read the camera	
	task.eventcam = ::tools::readCameraInfo(nh, "cam_es");
	if(isInitilized){
		task.framecam = ::tools::readCameraInfo(nh, "cam_fr");
		initialNum = 0;
	}
	ROS_INFO_STREAM("[calib_node] cameras are "<<(isInitilized?"already":"[ not not not ]")<<" calibrated. ");
	ROS_INFO_STREAM("[calib_node] >>>>>>>>>>>>>>>>>>>> finish parameter configuration  <<<<<<<<<<<<<<<<<<<<");

	// online change param
	std::cout<<"[calib_node] dynamic_reconfigure registering ....  "<<std::endl;
	const auto slicerCallback = [&eventQueue](const dv::EventStore &events) {
		eventQueue.push(events);
	};
	std::optional<int> jobId;
	dv::EventStreamSlicer slicer;
	dynamic_reconfigure::Server<efsc_ros_calib::CalibConfig> server;
	const auto reconfigureCallback = [&task, &jobId, &slicer, &slicerCallback](const efsc_ros_calib::CalibConfig &config, uint32_t level) {
		
		if (jobId.has_value()) { slicer.removeJob(*jobId); }
		jobId = slicer.doEveryTimeInterval(dv::Duration(static_cast<int>(config.dealing_time) * 1000LL), slicerCallback);
		std::cout<<std::endl<<" dealing_time = "<<static_cast<int>(config.dealing_time)<<std::endl<<std::endl;
		
		task.data_loader.num_events = static_cast<size_t>(config.num_events);
		task.data_loader.overlap = static_cast<double>(config.overlap);
		if (task.data_loader.overlap < 0.0) task.data_loader.overlap = 0.0;
		task.data_loader.overlap /= 100.0;

	};
	server.setCallback(reconfigureCallback);

	/********************** Intial configuration **************************/  
	std::cout<<"[calib_node] Intial configuration. "<<std::endl;
    // // Reset counters //
	task.es_idx = task.fr_idx = task.kf_idx = 0;
	task.event_num_k = 0;

	// // Create the event-frame picker //
	task.picker = std::make_shared<ef_calib::picker::EventPicker>(task.config.picker, task.eventcam, task.config.pattern);
	task.picker->esMap = std::make_shared<ef_calib::picker::EventSliceMap>();
	if( task.config.pattern.RThreshold < 23 && task.config.pattern.RThreshold > 0)
		continueThread = true;

	// // Create the event-frame CalibPair for calibration //
	task.pairMap = std::make_shared<ef_calib::calib::EventFramePairMap>();

// 	/*********************** Subscribe events data from DVS ********************************/
	auto eventSubscriber = nh.subscribe<dv_ros_msgs::EventArrayMessage>("events", 200, [&task, &slicer, &isInitilized, &continueThread](const auto &events) {
		if(task.camera_es == nullptr)
		{
			Eigen::Vector2i camerasize = Eigen::Vector2i(346, 260);
			std::cout<<"[calib] eventslice width = "<<camerasize[0]<<", height = "<<camerasize[1]<<" hand-input"<<std::endl;
			task.camera_es = std::make_shared<::ef_calib::calib::CameraBase>(camerasize);
			if(isInitilized){
				if(camerasize[0] == task.eventcam.width && camerasize[1] == task.eventcam.height ){
					task.camera_es->setIntrisic(task.eventcam.intrinsics, task.eventcam.D);
				} else {
					ROS_ERROR(" camerainfo.yaml has wrong input size for event camera. ");
					continueThread = false;
				}
			}
		}
		
		try { 
			// picker thread
			slicer.accept(dv_ros_msgs::toEventStore(*events)); 
			// event thread
		}
		catch (std::out_of_range &exception) {
			ROS_WARN("%s", exception.what());
		}
	});
	
// 	/*********************** Create Thread for Events ********************************/
	ROS_INFO_STREAM("[calib_node] Create Thread for slicing events into event-slices. ");
	std::thread eventsPickerThread([&task, &Pub2, &continueThread, &eventQueue] {
		// int64_t last_detection_timestamp;
		while (continueThread) {
		eventQueue.consume_all([&task, &Pub2/*, &Pub3*/](const dv::EventStore &events) {

				task.event_num_k += events.getTotalLength();

				std::shared_ptr<ef_calib::picker::EventSlice> es = std::make_shared<ef_calib::picker::EventSlice>(events);
				
				// Avoid the number of events is too large.
				if( es->eventsNum() < task.config.picker.frameEventNumThreshold ){
					cv::Mat image2, image_temp;
					// std::cout<<"es->eventsNum() = "<<es->eventsNum()<<std::endl;
					if( task.picker->extractFeatures(es) ) {
						// /** Set to initializing if this is the first eventframe **/
						if (task.es_idx == 0){
							std::cout<<"{es} First timestamp = "<<events.getLowestTime()%10000000<<", the number of events = "<<task.event_num_k<<std::endl; 
							task.event_num_k = 0;
							task.stateRunning();
						}

						/** Increment EF INDEX **/
						task.es_idx++;
						/** Add the Event Slice **/
						task.picker->esMap->addEventSlice(es);	
						// std::cout<<"Insert es idx ["<<es->idx<<"], timstamp = "<<es->time%10000000<<", esMap size = "<<task.picker->esMap->SliceNum()<<std::endl;

						// cv::Mat image = (es->image()).clone();
						// image_temp = es->drawEvent();					
						image2 = es->eventImage;
						// std::cout<<"{es} No."<<task.es_idx<<" detectable timestamp = "<< events.getHighestTime()%10000000
						//  		 <<", past number of events = "<<task.event_num_k<<std::endl;
					} //extract pattern information into event-frames.
					
					if( !image2.empty()){
						dv_ros_msgs::ImageMessage msg2 = dv_ros_msgs::toRosImageMessage(image2);
						msg2.header.stamp              = ros::Time::now();
						Pub2.publish(msg2);
					} //publish image
					
				} //if too many events included, ingnore it.
			
			task.events_buff += events;
			if( task.events_buff.size() >= task.data_loader.num_events * 2 ){

				// task.picker->esMap->cleanMap();

				int next_element = (1.0 - task.data_loader.overlap)* task.data_loader.num_events;
				task.events_buff = task.events_buff.slice(next_element);
				// std::cout<<"[EVENTS] Reduce events size ["<<next_element+task.events_buff.size()<<"] to ["<<task.events_buff.size()<<"] after inserting eventRefineQueue"<<std::endl;
			}

		}); // eventQueue.consume_all

		if(task.state() == ef_calib_node::State::OPTIMIZE)
			std::this_thread::sleep_for(30ms);
		else
			std::this_thread::sleep_for(std::chrono::nanoseconds(100));
		}//continueThread
	});

	//*********** Reading image data through ROS ***********************
	std::map<uint64_t, ::ef_calib::picker::ImageCenter::Ptr> frame_vector;
	ROS_INFO_STREAM("[calib_node] Create Thread for frame cameras and select the matched eventslice. ");
	auto imageSubscriber = nh.subscribe<dv_ros_msgs::ImageMessage>("/camera/color/image_raw", 10, [&task, &Pub1, &frame_vector, &efPairQueue, &isInitilized, &continueThread ](const auto &msg){
		if(task.camera_fr == nullptr)
		{
			std::cout<<"[calib] frameimage width = "<<static_cast<int>(msg->width)<<", height = "<<static_cast<int>(msg->height)<<std::endl;
			Eigen::Vector2i camerasize = Eigen::Vector2i(static_cast<int>(msg->width), static_cast<int>(msg->height));
			task.camera_fr = std::make_shared<::ef_calib::calib::CameraBase>(camerasize);
			if(isInitilized){
				if(camerasize[0] == task.framecam.width && camerasize[1] == task.framecam.height ){
					task.camera_fr->setIntrisic(task.framecam.intrinsics, task.framecam.D);
				} else {
					ROS_ERROR(" camerainfo.yaml has wrong input size for event camera. ");
					continueThread = false;
				}
			}
		}
		
		cv::Mat image;
		int64_t timestamp;
		bool isFound = false;
		std::vector<cv::Point2f> outCenters;
		
		try {
			image = cv::Mat(static_cast<int32_t>(msg->height), static_cast<int32_t>(msg->width), CV_MAKETYPE(CV_8U, 3),
									 const_cast<uchar *>(&msg->data[0]), msg->step);
			timestamp = dv_ros_msgs::toDvTime(msg->header.stamp);

			/** Convert the grayscale in case of color and normalize the image **/
    		if (image.channels() > 1) cv::cvtColor(image, image, cv::COLOR_RGB2GRAY);
			image.convertTo(image, CV_8UC1);

			//** Founding the circle pattern board. **//			
			cv::SimpleBlobDetector::Params dParams;
			dParams.filterByArea = false;
			isFound = cv::findCirclesGrid(image, cv::Size(task.config.pattern.cols, task.config.pattern.rows), outCenters,
											cv::CALIB_CB_ASYMMETRIC_GRID, cv::SimpleBlobDetector::create(dParams));
			if (!isFound)
			isFound = cv::findCirclesGrid(image, cv::Size(task.config.pattern.cols, task.config.pattern.rows), outCenters,
											cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, cv::SimpleBlobDetector::create(dParams));
			
		}
		catch (std::out_of_range &exception) {
			std::cout<<" something is wrong about the imageSubscriber. "<<std::endl;
			ROS_WARN("%s", exception.what());
		}

		if(isFound && outCenters.size()==14 ){
			::ef_calib::picker::ImageCenter::Ptr ImageCenter_ptr = std::make_shared<::ef_calib::picker::ImageCenter>(image, outCenters);
			frame_vector[timestamp] = ImageCenter_ptr;

			// cornerSubPix(image, outCenters, cv::Size(3, 3), cv::Size(-1, -1), TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 0.1));
			drawChessboardCorners(image, cv::Size(task.config.pattern.cols, task.config.pattern.rows), cv::Mat(outCenters), isFound);
			// cv::circle(image, cv::Point(outCenters[0].x, outCenters[0].y), 10, 255);

			/** Increment EF INDEX **/
			task.fr_idx++;
			// std::cout<<"{fr} No."<<task.fr_idx<<" current detectable timestamp = "<<timestamp%10000000<<", current number of events = "<<task.event_num_k<<std::endl;

			if( frame_vector.size() >= 5 && continueThread){
				// ** query the event-slice map and request the remove before the first frame in frame_vector**//
				std::map<uint64_t, ef_calib::picker::EventSlice::Ptr> es_buffer;
				task.picker->esMap->cleanMap(); // clear the map first
				task.picker->esMap->EventSliceLockShared();
				// std::cout<<" the difference is "<< (timestamp - frame_vector.cbegin()->first)/1000<< " ms. "<<std::endl;
				es_buffer = task.picker->esMap->EventSlicebefore(timestamp, frame_vector.begin()->first);
				task.picker->esMap->EventSliceUnlockShared();
								
				//***** time matching *****//
				if( es_buffer.size() > 1 ){
					// ** choose the shortest time differnce between frame and event-slice **//
					int64_t t_es, t_fr;
					int64_t tdmin = std::numeric_limits<int64_t>::max();
					for(auto f_it = frame_vector.begin(); f_it!=frame_vector.end(); f_it++){
						// std::cout<<" -"<< (timestamp - f_it->first)/1000<<"ms";

						int64_t time_diff_back, time_diff_front;
						auto e_it = es_buffer.lower_bound(f_it->first);
						if(e_it == es_buffer.end()){
							e_it--;
							time_diff_front = f_it->first - e_it->first;
							time_diff_back = std::numeric_limits<int64_t>::max();
							// std::cout<<"Already touch the end, d = "<<f_it->first - e_it->first<<" frame = "<<f_it->first%10000000<<std::endl;
						}
						else{
							time_diff_back = e_it->first - f_it->first;
							// std::cout<<e_it->first%10000000<<" > "<<f_it->first%10000000;
							time_diff_front = std::numeric_limits<int64_t>::max();
							if( e_it!=es_buffer.begin() ){
								e_it--;
								time_diff_front = f_it->first - e_it->first;
								// std::cout<<" > "<<e_it->first%10000000<<std::endl;
							}
						}

						//** determine the value the min time-difference tdmin. **//
						// std::cout<<" time_diff_back = "<<time_diff_back/1000<<"ms, time_diff_front = "<<time_diff_front/1000<<"ms"<<std::endl;
						if(tdmin > time_diff_front || tdmin > time_diff_back){
							tdmin = (time_diff_front < time_diff_back) ? time_diff_front : time_diff_back;
							t_fr = f_it->first;
							if(time_diff_front < time_diff_back)
								t_es = e_it->first;
							else
								t_es = (++e_it)->first;
						}
					}
					

					//** store the matched pair into map if tdmin < 1/(30Hz) = 33ms, /2 = 15ms. **//
					//** if the eventslice are newly formed (<5ms), the bag of events are not aligned with frame. Then restart the process ane let the events accumulate for another round**//
					if( tdmin <= 15000 && task.events_buff.getHighestTime() - t_es > 3000 && t_fr - task.events_buff.getLowestTime() > 0 ) //100ms
					{			
						task.kf_idx ++;
						// std::cout<<"No."<<task.kf_idx;
						std::cout<<"[Image] Feed Matched es["<<t_es%100000000<<"] and fr["<<t_fr%100000000
									<<"] with [td="<<tdmin/1000<<"ms] and events to ef_pair_Thread. " <<std::endl;

						dv::EventStore ef_events = task.events_buff;
						std::shared_ptr<ef_calib::picker::PairSlice> fr_matched = std::make_shared<ef_calib::picker::PairSlice>
							(es_buffer[t_es], frame_vector[t_fr]->image_, t_fr, frame_vector[t_fr]->outCenters_, ef_events);
						efPairQueue.push(fr_matched);

						frame_vector.erase(t_fr); // in case of macthing the repeated es-frame pairs

					} else
						frame_vector.erase(frame_vector.begin()->first);

				} //endif es_buffer is not empty 
				else
					frame_vector.erase(frame_vector.begin()->first);

			}//endif frame_vector.size() >= 5

		}//endif isFound
		
		if( !image.empty() ){
			dv_ros_msgs::ImageMessage msg1 = dv_ros_msgs::toRosImageMessage(image);
			msg1.header.stamp              = ros::Time::now();
			Pub1.publish(msg1);
		} //publish image



		if(task.state() == ef_calib_node::State::OPTIMIZE)
			std::this_thread::sleep_for(30ms);
		else
		std::this_thread::sleep_for(std::chrono::nanoseconds(100));
	});

// 	/*********************** Create Thread for picker ********************************/
	ROS_INFO_STREAM("[calib_node] Create Thread for Event-frame Matched Pair. ");
	std::thread efPairThread([&task, &Pub1, &Pub2, &Pub3, &Pub4, &continueThread, &isInitilized, &efPairQueue, &pose_pub_, &pose_pub2_] {
		int PairNumber_idx = 0;
		std::vector<cv::Point3f> obj;
		task.calcBoardCornerPositions(obj);

		while (continueThread) {
		efPairQueue.consume_all([&task, &Pub1, &Pub2, &Pub3, &Pub4, &PairNumber_idx, &isInitilized, &obj, &pose_pub_, &pose_pub2_, &continueThread]
			(const ef_calib::picker::PairSlice::Ptr &pairslice) {
			// std::cout<<" pair time are consuming ( "<<pairslice->time/1000<<" ms)"<<std::endl;
			PairNumber_idx ++;
			std::cout<<"[Pair] No."<<PairNumber_idx<<": "<<std::endl;

			// std::vector<cv::Point2f> imageP_es;
			cv::Mat	image3, image_MC, image_show_es, image_temp, image_temp2;
			cv::Mat image_show_fr;//****************************move back to the original code position
			cv::Mat image_show_1, image_show_2;
			std::vector<cv::Point2f> center;
			std::vector<double> radius;
			bool EventsIsFound = false;
			Eigen::Vector2d param;
			std::vector<ef_calib::picker::EventPathes::Ptr> esPatches;
			if( task.state() == ef_calib_node::State::RUNNING){

				
				// Get raw Events windows.
				uint64_t time_ref = pairslice->time;
				uint64_t events_gap = 100000;//100ms
				uint64_t events_front, events_back;
				if( pairslice->es_->time < time_ref )
				{
					uint64_t time1 = pairslice->es_->first_time - events_gap;
					uint64_t time2 = time_ref + events_gap;
					events_front = time1 > pairslice->events_.getLowestTime() ? time1 : pairslice->events_.getLowestTime();//max
					events_back = time2 > pairslice->events_.getHighestTime() ? pairslice->events_.getHighestTime() : time2;//min
				} else {
					uint64_t time1 = time_ref - events_gap;
					uint64_t time2 = pairslice->es_->last_time + events_gap;
					events_front = time1 > pairslice->events_.getLowestTime() ? time1 : pairslice->events_.getLowestTime();//max
					events_back = time2 > pairslice->events_.getHighestTime() ? pairslice->events_.getHighestTime() : time2;//min
				}
				dv::EventStore events = pairslice->events_.sliceTime(events_front, events_back);
				// std::cout<<"** [Event] events_front = "<<events_front%100000000<<" events_back = "<<events_back%100000000<<std::endl;
				
				// divide events into patches
				const size_t num_results = 2000;
				std::vector<ef_calib::picker::CircleFeature::Ptr> es_features = pairslice->es_->features();
				task.picker->divideEvents(events, es_features, num_results, esPatches, time_ref);
				// std::cout<<"** [Event] divide events ["<<events.size()<<", "<< (events.getHighestTime()-events.getLowestTime())/1000<<"ms]"<<
				//                               " into ["<<num_results<<", "<<esPatches.size()<<" patches] based on features"
				// 							  <<", time duration ["<<events.getLowestTime()%100000000<<", "<<events.getHighestTime()%100000000<<"] "
				// 							  <<std::endl;

				// *** Event-based Motion Compensation method *** //
				param = Eigen::Vector2d(0.0, 0.0);
				task.picker->MotionCompensate(esPatches, param);
				
				image3   = cv::Mat(task.eventcam.height, task.eventcam.width, CV_8UC1, cv::Scalar(255));
				image_MC = cv::Mat(task.eventcam.height*2, task.eventcam.width*2, CV_8UC3, cv::Vec3b(0, 0, 0));

				std::vector<std::vector<cv::Point3d>> candidate;
				candidate.resize(esPatches.size());
				center.resize(esPatches.size());
				radius.resize(esPatches.size());
				int numNotFit = 0;
				for(int idx = 0; idx < esPatches.size(); idx++){
					Eigen::Vector2d centerPatch;
					bool CircleIsfit = task.findcircle(esPatches[idx], param, image_MC,
									candidate[idx], centerPatch, radius[idx]);
					// std::cout<<"** [Event] finish to findcircle....... No."<<idx<<" CircleIsfit = "<<(CircleIsfit?"true":"false")<<std::endl;
				
					if(CircleIsfit){
						//recover the original pixel position
						double center_x = centerPatch(0);
						double center_y = centerPatch(1);
						center[idx] = cv::Point2f( center_x*0.5+esPatches[idx]->bias_x, center_y*0.5+esPatches[idx]->bias_y);

						cv::circle(image3, center[idx], 1, 0, cv::FILLED);
						// std::cout<<"** [Event] findcircle: "<<idx<<", center = "<<center[idx].x<<", "<<center[idx].y<<", radius = "<<radius[idx]<<std::endl;
					} else {
						std::cout<<"** [Event]-MotionComError happens "
									<<" No."<<idx<<", candidate.size() = "<<candidate[idx].size()<<std::endl;
						numNotFit++;
						break;
					}
				}
				if( numNotFit == 0){
					double error = task.picker->EvaluateMC(esPatches[5], param, center[5], radius[5]);
					if(error < 5.0){
						std::vector<cv::Point2f> outCenters;
						cv::SimpleBlobDetector::Params dParams;
						// dParams.filterByColor = false; //no influence
						dParams.filterByArea = false;   //great influence, The area of these points
						// dParams.filterByInertia = false;  //without this setting, even better.
						// dParams.filterByCircularity = false; //without this setting, even better.
						// dParams.filterByConvexity = false; //no influence
						EventsIsFound = cv::findCirclesGrid(image3, cv::Size(task.config.pattern.cols, task.config.pattern.rows), outCenters,
														cv::CALIB_CB_ASYMMETRIC_GRID, cv::SimpleBlobDetector::create(dParams));
						if (!EventsIsFound)
						EventsIsFound = cv::findCirclesGrid(image3, cv::Size(task.config.pattern.cols, task.config.pattern.rows), outCenters,
														cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, cv::SimpleBlobDetector::create(dParams));
						
						std::cout<<"** [Event] Is the CirclesGrid found again? "<< (EventsIsFound? " Yes. ":" None. ")<<std::endl;
						

					} else {
						// ***** Motion Compensation Optimized failed
						std::cout<<"** [Event]-MotionComError fitting circle error is big."<<std::endl;
					}
				}
					
			}



			cv::Mat image5, image_P;
			std::vector<cv::Point2f> frCenter;
			bool FrameIsFound = false;
			if( task.state() == ef_calib_node::State::RUNNING ){
				//***** Frame: Extract more accurate circle base on Sobel-egde *****//
		
				int PatchErrorNum = 14;
				cv::Mat ImagePatch;
				if(pairslice->fr_->checkParallel())
					PatchErrorNum = pairslice->fr_->findCirclesBesedOnEdge(task.config.pattern, ImagePatch, image_temp, image_temp2);
				
				if( PatchErrorNum == 0 ){
					// std::cout<<"** [Frame] finish finding the circles based the Edege-based method. PatchErrorNum = "<<PatchErrorNum<<std::endl;
					image_P = ImagePatch.clone();

					Eigen::Vector2i frSize = task.camera_fr->size();
					image5 = cv::Mat(frSize(1), frSize(0), CV_8UC1, cv::Scalar(255));
					frCenter = pairslice->fr_->Circles();   
					// frCenter = pairslice->fr_->OriginalCircles(); //ablation study
					// std::vector<double> R_temp =  pairslice->fr_->Radiuss();
					for(int j=0; j<frCenter.size(); j++){
						// std::cout<<" "<<frCenter[j]<<", "<< R_temp[j]<<std::endl;
						cv::circle(image5, frCenter[j], 1, 0, cv::FILLED);
						
					}

					std::vector<cv::Point2f> outCenters2;
					cv::SimpleBlobDetector::Params dParams2;
					// dParams.filterByColor = false; //no influence
					dParams2.filterByArea = false;   //great influence, The area of these points
					// dParams.filterByInertia = false;  //without this setting, even better.
					// dParams.filterByCircularity = false; //without this setting, even better.
					// dParams.filterByConvexity = false; //no influence
					FrameIsFound = cv::findCirclesGrid(image5, cv::Size(task.config.pattern.cols, task.config.pattern.rows), outCenters2,
													cv::CALIB_CB_ASYMMETRIC_GRID, cv::SimpleBlobDetector::create(dParams2));
					if (!FrameIsFound)
					FrameIsFound = cv::findCirclesGrid(image5, cv::Size(task.config.pattern.cols, task.config.pattern.rows), outCenters2,
													cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, cv::SimpleBlobDetector::create(dParams2));
					
					std::cout<<"** [Frame] Is the CirclesGrid found again? "<< (FrameIsFound? " Yes. ":" None. ")<<std::endl;
					// if(FrameIsFound){
					// 	// drawChessboardCorners(image3, cv::Size(task.config.pattern.cols, task.config.pattern.rows), cv::Mat(outCenters), FrameIsFound);
					// }
				}
			}


			if( EventsIsFound && FrameIsFound ){
					
				if(!pairslice->fr_->checkPattern(image_P, task.config.pattern) /*|| error >= 20.0*/ )
				{
					std::cout<<"[Pair] No."<<PairNumber_idx<<": not Successfully"<<std::endl
							 <<"[Pair] event Motion-compensation not good. or radius of frame-circles not good.  "<<std::endl;
					
					// std::cout<<" system is empty. "<<std::endl;
					// task.stateNull();
					// continueThread = false;
				} else {

					std::cout<<"[Pair] No."<<PairNumber_idx<<": Successfully dealing with the data from two cameras. "<<std::endl;
					std::shared_ptr<ef_calib::calib::CalibPair> calibpair = std::make_shared<ef_calib::calib::CalibPair>
										(pairslice->time, center, frCenter);
					
					if(isInitilized){
						image_show_es = image_MC.clone();
						image_show_fr = image_P.clone();						

						//***** Frames *****//
						cv::Mat rvec, tvec;
						std::vector<cv::Point2f> center_fr = calibpair->Fcircles;
						std::vector<int> inliers;
						cv::solvePnPRansac(obj, center_fr, task.camera_fr->cameraMatrix_cv, task.camera_fr->distCoeffs_cv,
										rvec, tvec, false, 100, 4.0, 0.99, inliers, cv::SOLVEPNP_IPPE);
						// publish pose
						Eigen::Vector3d        twb;
						Eigen::Quaterniond unitQwb;
						task.publishpose(pose_pub_, rvec, tvec, twb, unitQwb);
						std::cout<<"** [Frame] Confidence of the current pose (inliers/14) = "<<inliers.size()/14.0 *100<<"%. "<<std::endl;
						std::vector<cv::Point2f> center_fr_obj;
						projectPoints(obj, rvec, tvec, task.camera_fr->cameraMatrix_cv, task.camera_fr->distCoeffs_cv, center_fr_obj);
						double err_fr = norm(center_fr, center_fr_obj, cv::NORM_L2);
						std::cout<<"** [Frame] ProjectError of the current pose = "<<err_fr<<std::endl;
						// task.PrintdepthOfImage(center_fr, rvec, tvec, task.camera_fr);
						std::cout<<" Before: rotation_cv = "<<rvec.t()<<", translation_cv = "<<tvec.t()<<std::endl
								 <<" After : "<<unitQwb<<", "<<twb.transpose()<<std::endl;

						//***** Events *****//
						cv::Mat rvec2, tvec2;
						std::vector<cv::Point2f> center_es = calibpair->Ecircles;
						std::vector<int> inliers2;
						cv::solvePnPRansac(obj, center_es, task.camera_es->cameraMatrix_cv, task.camera_es->distCoeffs_cv,
										rvec2, tvec2, false, 100, 4.0, 0.99, inliers2, cv::SOLVEPNP_IPPE);
						// publish pose
						Eigen::Vector3d        twb2;
						Eigen::Quaterniond unitQwb2;
						task.publishpose(pose_pub2_, rvec2, tvec2, twb2, unitQwb2);
						std::cout<<"** [Event] Confidence of the current pose (inliers2/14) = "<<inliers2.size()/14.0 *100<<"%. "<<std::endl;
						std::vector<cv::Point2f> center_es_obj;
						projectPoints(obj, rvec2, tvec2, task.camera_es->cameraMatrix_cv, task.camera_es->distCoeffs_cv, center_es_obj);
						double err2 = norm(center_es, center_es_obj, cv::NORM_L2);
						std::cout<<"** [Event] ProjectError of the current pose = "<<err2<<std::endl;
						
						// task.PrintdepthOfImage(center_es, rvec2, tvec2, task.camera_es);


						calibpair->setPose(twb, unitQwb);
						calibpair->setPose2(twb2, unitQwb2);
						std::cout<<"** [Frame] twb = "<<twb.transpose()<<std::endl;
						std::cout<<"** [Event] twb = "<<twb2.transpose()<<std::endl;
						task.pairMap->addPair(calibpair);
						
					} else {
						image_show_es = image_MC.clone();
						image_show_fr = image_P.clone();
						task.pairMap->addPair(calibpair);
					}
				}
			}


			if( !image_show_fr.empty()){ // frame
				dv_ros_msgs::ImageMessage msg3 = dv_ros_msgs::toRosImageMessage(image_show_fr);
				msg3.header.stamp              = ros::Time::now();
				Pub3.publish(msg3);
			} //publish image
			if( !image_show_es.empty()){ // event
				dv_ros_msgs::ImageMessage msg4 = dv_ros_msgs::toRosImageMessage(image_show_es);
				msg4.header.stamp              = ros::Time::now();
				Pub4.publish(msg4);
			} //publish image
			std::cout<<std::endl;

		
		}); // efPairQueue.consume_all


		if(task.state() == ef_calib_node::State::CHECK || task.state() == ef_calib_node::State::INITIALIZING)
			std::this_thread::sleep_for(30ms);
		else
			std::this_thread::sleep_for(1ms);
		}//continueThread
	});

	// 	/*********************** Create Thread for calibration ********************************/
	ROS_INFO_STREAM("[calib_node] Create Thread for Calibration ");
	std::thread CalibThread([&task, &Pub3, &continueThread, &isInitilized, &pose_pub3_, &Pub5, &initialNum, &OptimizeNum] {
		int Pair_idx = 0;
		int64_t last_timestamp = 0;
		std::vector<std::vector<cv::Point2f>> imagePoints;
		std::vector<std::vector<cv::Point2f>> imageP_fr(initialNum);
		std::vector<std::vector<cv::Point2f>> imageP_es(initialNum);

		// ***** objectPoints *****//
		std::vector<std::vector<cv::Point3f> > objectPoints(1);
		task.calcBoardCornerPositions(objectPoints[0]);
		// for(int i=0; i<objectPoints[0].size(); i++)
		// 	std::cout<<" pattern corners["<<i<<"] = "<<objectPoints[0][i]<<std::endl;
		if(initialNum != 0)
			objectPoints.resize( initialNum , objectPoints[0]);

		// ** Main loop of calibration **//
		while (continueThread && task.config.calib.calib_flag_) {
		

		// Main loop, 
		if(task.state() == ef_calib_node::State::RUNNING){
			task.pairMap->PairLockShared();
			int PairNumber_inMap = task.pairMap->PairNum();
			task.pairMap->PairUnLockShared();

			if( PairNumber_inMap > Pair_idx ){
				Pair_idx++;
				if( Pair_idx<=initialNum && !isInitilized ){
					
					//** Get the next pair in Map **//
					ef_calib::calib::CalibPair::Ptr pair_next;
					task.pairMap->PairLockShared();
					auto itr = task.pairMap->EventFramePairs().lower_bound(last_timestamp+1);
					if (itr != task.pairMap->EventFramePairs().end())
						pair_next = itr->second;
					else {
						pair_next = task.pairMap->EventFramePairs().rbegin()->second;
					}
					task.pairMap->PairUnLockShared();
					last_timestamp = pair_next->time;

					// for initialization
					imageP_fr[Pair_idx-1] = pair_next->Fcircles;
					imageP_es[Pair_idx-1] = pair_next->Ecircles;
					std::cout<<"[CALIB] collected one event-frame pair for Initialization. No."<<Pair_idx<<" at "<<last_timestamp%100000000<<std::endl<<std::endl;
				
				}
				
				// *****  Initialization  ****** //
				if( Pair_idx == initialNum && !isInitilized ){
					// mono calibration from OpenCV
					task.stateInitial();
					continueThread = false;

					// for(int i=0; i<imageP_fr.size(); i++){
					// 	for(int j=0; j<imageP_fr[i].size(); j++){
					// 		std::cout<<"   ("<<j<<"):"<<imageP_fr[i][j];
					// 		if(j%4==3)
					// 			std::cout<<std::endl;
					// 	}
					// 	std::cout<<std::endl<<std::endl;
					// }

					//***** frame *****//
					cv::Mat cameraMatrix_fr = cv::Mat::eye(3, 3, CV_64F); //[fx 0 cx; 0 fy cy; 0 0 1]
					cv::Mat distCoeffs_fr = cv::Mat::zeros(5, 1, CV_64F);// [k1, k2, p1, p2, k3]
					
					// Find intrinsic and extrinsic camera parameters
					Eigen::Vector2i Size1 = task.camera_fr->size();
					std::vector<cv::Mat> rvecs1, tvecs1; //rotation vector, translation vector			
					//use LU instead of SVD decomposition for solving. much faster but potentially less precise
					double rms1 = cv::calibrateCamera(objectPoints, imageP_fr, cv::Size(Size1(0), Size1(1)), cameraMatrix_fr, distCoeffs_fr,
													rvecs1, tvecs1, task.config.calib.flag_ | cv::CALIB_USE_LU);
					std::cout << "** [Image] Re-projection error reported by calibrateCamera: " << rms1 << std::endl;
					bool ok_fr = cv::checkRange(cameraMatrix_fr) && cv::checkRange(distCoeffs_fr);
					std::vector<float> reprojErrs1;
					double totalAvgErr1 = task.computeReprojectionErrors(objectPoints, imageP_fr, rvecs1, tvecs1, 
																		cameraMatrix_fr, distCoeffs_fr, reprojErrs1);
					std::cout <<"** [Image]" << (ok_fr ? " Calibration succeeded" : "Calibration failed")<< ". avg re projection error = " << totalAvgErr1 << std::endl;

					//***** Events *****//
					cv::Mat cameraMatrix_es = cv::Mat::eye(3, 3, CV_64F); //[fx 0 cx; 0 fy cy; 0 0 1]
					cv::Mat distCoeffs_es = cv::Mat::zeros(8, 1, CV_64F);// [k1, k2, p1, p2, k3, k4, k5, k6]

					// Find intrinsic and extrinsic camera parameters
					Eigen::Vector2i Size2 = task.camera_es->size();
					std::vector<cv::Mat> rvecs2, tvecs2; //rotation vector, translation vector
					//use LU instead of SVD decomposition for solving. much faster but potentially less precise
					double rms2 = cv::calibrateCamera(objectPoints, imageP_es, cv::Size(Size2(0), Size2(1)), cameraMatrix_es, distCoeffs_es,
												rvecs2, tvecs2, task.config.calib.flag_ | cv::CALIB_USE_LU);
					std::cout << "** [Event] Re-projection error reported by calibrateCamera: " << rms2 << std::endl;
					bool ok_es = cv::checkRange(cameraMatrix_es) && cv::checkRange(distCoeffs_es);
					std::vector<float> reprojErrs2;
					double totalAvgErr2 = task.computeReprojectionErrors(objectPoints, imageP_es, rvecs2, tvecs2, 
																			cameraMatrix_es, distCoeffs_es, reprojErrs2);
					std::cout <<"** [Event]" << (ok_es ? " Calibration succeeded" : "Calibration failed")<< ". avg re projection error = " << totalAvgErr2 << std::endl;
					
					std::cout<<std::endl;
					if(ok_es && ok_fr){
						// ***** frame *****//
						std::cout<<"** [FrameImage] K: "<<std::endl;
						task.camera_fr->setK(cameraMatrix_fr);
						task.camera_fr->setdistCoeffs(distCoeffs_fr);
						std::cout<<std::endl;
						// ***** events *****//
						std::cout<<"** [EventSlice] K: "<<std::endl;
						task.camera_es->setK(cameraMatrix_es);
						task.camera_es->setdistCoeffs(distCoeffs_es);
						std::cout<<std::endl;
						// isInitilized = true;
						std::cout<<" *******************  "<<std::endl
							     <<"[Calib] Finished Initialization ! ! ! "<<std::endl
								 <<" *******************  "<<std::endl;
						// task.stateRunning();
						
						imageP_fr.clear();
						imageP_es.clear();

					} else {
						std::cout<<"[Calib] Failed to Initialization ! ! ! "<<std::endl
								 <<"[Calib] Please check the First "<<initialNum<<" pairs of data. "<<std::endl;
					}
				
					// stop at Initial
					task.stateNull();
				
				}
							
				// *****  evaluate the distance against the wall ***** //
				if(  Pair_idx == OptimizeNum && isInitilized ){
					task.stateOptimize();

					task.pairMap->PairLockShared();
					std::map<uint64_t, ::ef_calib::calib::CalibPair::Ptr> MapTemp = task.pairMap->copyPairs();
					task.pairMap->PairUnLockShared();
					
					// delete first 15 frames which is for Initialization.
					// int countToRemove = initialNum;
					// while (!MapTemp.empty() && countToRemove>0 ) {
					// 	countToRemove--;
					// 	MapTemp.erase(MapTemp.begin());
					// }


					// 	*************  pose PnP Pose Estimation amd RT between two cameras, with know camera Intrinsic ***************** //
					int counter_pair = 0;
					::ef_calib::calib::SolveCamToCam::Ptr PoseAmongTwo = std::make_shared<::ef_calib::calib::SolveCamToCam>
							(task.camera_es->K(), task.camera_es->distCoeffs(), task.camera_fr->K(), task.camera_fr->distCoeffs());
					Sophus::SE3d pose_CamToCam = Sophus::SE3d(Eigen::Quaterniond(1,0,0,0), Eigen::Vector3d(16.5,0.5,3.0));
					PoseAmongTwo->addVertexAmongTwo(pose_CamToCam);
					Eigen::Vector4d param_g2o_huber = Eigen::Vector4d(task.config.calib.huber_es_,
																	  task.config.calib.huber_fr_,
																	  task.config.calib.huber_esTofr_,
																	  task.config.calib.huber_frToes_);
					std::cout<<" g2o huber "<<param_g2o_huber.transpose()<<std::endl<<std::endl;
					
					for (auto &itr : MapTemp) {
						::ef_calib::calib::CalibPair::Ptr bf = itr.second;
						counter_pair++;
						Eigen::Quaterniond q_camtocam = bf->unitQwb() * (bf->unitQwb2().conjugate());
						q_camtocam.normalize();
						// Eigen::Vector3d t_camtocam = - q_camtocam.toRotationMatrix()*bf->twb2() + bf->twb();
						std::cout<<"** NO."<<counter_pair<<" t = "<<(-bf->twb2() + bf->twb()).transpose()<<", q = "<<q_camtocam<<std::endl;            		

						std::vector<cv::Point2f> epoints = bf->Ecircles;
						std::vector<cv::Point2f> fpoints = bf->Fcircles;
						::ef_calib::calib::SolveCamToCam::VecEigen3d pts_3d_eigen;
						::ef_calib::calib::SolveCamToCam::VecEigen2d pts_2d_eigen_es;
						::ef_calib::calib::SolveCamToCam::VecEigen2d pts_2d_eigen_fr;
						for (size_t i = 0; i < objectPoints[0].size(); ++i) {
							pts_3d_eigen.push_back(Eigen::Vector3d(objectPoints[0][i].x, objectPoints[0][i].y, objectPoints[0][i].z));
							pts_2d_eigen_es.push_back(Eigen::Vector2d(epoints[i].x, epoints[i].y));
							pts_2d_eigen_fr.push_back(Eigen::Vector2d(fpoints[i].x, fpoints[i].y));
						}
						Sophus::SE3d pose_es = Sophus::SE3d(bf->unitQwb2(), bf->twb2());
						Sophus::SE3d pose_fr = Sophus::SE3d(bf->unitQwb(), bf->twb());
						
						PoseAmongTwo->addVertex(pose_es, pose_fr);
						
						bool isEdgeOk = PoseAmongTwo->addEdge(pts_3d_eigen, pts_2d_eigen_es, pts_2d_eigen_fr, param_g2o_huber); //add edge for the latest vertex
						std::cout<<"     isEdgeOk = "<<(isEdgeOk?"yes":"no")<<std::endl;
						std::cout<<std::endl;
					}
					
					PoseAmongTwo->optimize();
					std::cout<<" PnP Pose Estimation amd RT between two cameras, with know camera Intrinsic "<<std::endl
							 <<"done & exit "<<std::endl<<std::endl;


					// evaluation
					double err0_total_es = 0.0,  err0_total_fr = 0.0;//before
					double err_total_es = 0.0,  err_total_fr = 0.0;//after
					double err_total = 0.0;//camtocam
					size_t bf_count = 0;
					Sophus::SE3d CamToCam = PoseAmongTwo->CamToCam();
					Eigen::Matrix4d es_To_fr = CamToCam.matrix();
					Eigen::Matrix3d v0_R = es_To_fr.block<3, 3>(0, 0);
					Eigen::Vector3d v0_t = es_To_fr.block<3, 1>(0, 3);
					Eigen::Quaterniond v0_q(v0_R);
					v0_q.normalize();
					std::vector<uint64_t> PairsToBeErased_;
					for (auto &itr : MapTemp) {
						::ef_calib::calib::CalibPair::Ptr bf = itr.second;

						// ***** after *****
						std::vector<cv::Point2f> epoints = bf->Ecircles; // Event
						Sophus::SE3d bf_pose_es = PoseAmongTwo->pose_es(bf_count);
						Eigen::Matrix4d T_Rt_es = bf_pose_es.matrix();
						// Eigen::Vector3d T_t_es{T_Rt_es(0,3), T_Rt_es(1,3), T_Rt_es(2,3)};
						Eigen::Vector3d T_R2_es{T_Rt_es(2,0), T_Rt_es(2,1), T_Rt_es(2,2)};
						std::vector<cv::Point2f> fpoints = bf->Fcircles; //frame
						Sophus::SE3d bf_pose_fr = PoseAmongTwo->pose_fr(bf_count);
						Eigen::Matrix4d T_Rt_fr = bf_pose_fr.matrix();
						// Eigen::Vector3d T_t_fr{T_Rt_fr(0,3), T_Rt_fr(1,3), T_Rt_fr(2,3)};
						Eigen::Vector3d T_R2_fr{T_Rt_fr(2,0), T_Rt_fr(2,1), T_Rt_fr(2,2)};

						// *****  before   *****
						Sophus::SE3d bf0_pose_es = Sophus::SE3d(bf->unitQwb2(), bf->twb2());// Event
						Eigen::Matrix4d T0_Rt_es = bf0_pose_es.matrix();
						Eigen::Vector3d T0_R2_es{T0_Rt_es(2,0), T0_Rt_es(2,1), T0_Rt_es(2,2)};
						Sophus::SE3d bf0_pose_fr = Sophus::SE3d(bf->unitQwb(), bf->twb()); //frame
						Eigen::Matrix4d T0_Rt_fr = bf0_pose_fr.matrix();
						Eigen::Vector3d T0_R2_fr{T0_Rt_fr(2,0), T0_Rt_fr(2,1), T0_Rt_fr(2,2)};
						
						double err_total_pair = 0.0;
						for(size_t i = 0; i < objectPoints[0].size(); ++i){
							Eigen::Vector3d obj{objectPoints[0][i].x, objectPoints[0][i].y, objectPoints[0][i].z};
							Eigen::Vector2d xd_es(epoints[i].x, epoints[i].y);
							Eigen::Vector3d pc_es = task.camera_es->FromPixelToCam(xd_es);
							Eigen::Vector2d xd_fr(fpoints[i].x, fpoints[i].y);
							Eigen::Vector3d pc_fr = task.camera_fr->FromPixelToCam(xd_fr);

							// ***** after *****
							// Event
							double depth_es = - T_Rt_es(2,3) / (T_R2_es.dot(pc_es));
							Eigen::Vector3d _error_es = bf_pose_es * obj - depth_es * pc_es;
							err_total_es += _error_es.norm();
							//frame
							double depth_fr = - T_Rt_fr(2,3) / (T_R2_fr.dot(pc_fr));
							Eigen::Vector3d _error_fr = bf_pose_fr * obj - depth_fr * pc_fr;
							err_total_fr += _error_fr.norm();

							// *****  before   *****
							// Event
							double depth0_es = - T0_Rt_es(2,3) / (T0_R2_es.dot(pc_es));
							Eigen::Vector3d _error0_es = bf0_pose_es * obj - depth0_es * pc_es;
							err0_total_es += _error0_es.norm();
							// frame
							double depth0_fr = - T0_Rt_fr(2,3) / (T0_R2_fr.dot(pc_fr));
							Eigen::Vector3d _error0_fr = bf0_pose_fr * obj - depth0_fr * pc_fr;
							err0_total_fr += _error0_fr.norm();
							// std::cout<<"[es] p3d "/*<<(pc_es*depth_es).transpose()*/<<"("<<_error0_es.norm()<<"), --> "/*<<(depth0_es * pc_es).transpose()*/<<"("<<_error_es.norm()<<")     "
							//   		 <<"[fr] p3d "/*<<(pc_fr*depth_fr).transpose()*/<<"("<<_error0_fr.norm()<<"), --> "/*<<(depth0_fr * pc_fr).transpose()*/<<"("<<_error_fr.norm()<<")     ";
							
							
							// ** from event to frame ** 
							Eigen::Vector3d pc_fr_estimation = CamToCam * (pc_es*depth_es);
							pc_fr_estimation /= pc_fr_estimation[2];
							Eigen::Vector2d pp_fr_estimation = task.camera_fr->FromCamToPixel(pc_fr_estimation);
							double error = (pp_fr_estimation - xd_fr).norm();
							err_total += error; 
							// std::cout<<" esTofr = "<<error<<std::endl;
							err_total_pair+=error;
						}

						if(err_total_pair>=0.7*14)
						{
							uint64_t timestamp = bf->time;
							if (MapTemp.find(timestamp) != MapTemp.end()) {
								PairsToBeErased_.push_back(timestamp);
							}
							else{
								std::cout<<std::endl<<"erase happen wrong."<<std::endl<<std::endl;
							}
						}
						// std::cout<<" esTofr = "<<err_total_pair<<std::endl;
						bf_count++;
						// std::cout<<std::endl;
					}
					std::cout<<" es_average_error = "<<err0_total_es/(bf_count*objectPoints[0].size())<<", "
							 <<" fr_average_error = "<<err0_total_fr/(bf_count*objectPoints[0].size())<<" (Before g2o) "<<std::endl;
					std::cout<<" es_average_error = "<<err_total_es/(bf_count*objectPoints[0].size())<<", "
							 <<" fr_average_error = "<<err_total_fr/(bf_count*objectPoints[0].size())<<" (After g2o) "<<std::endl;
					std::cout << "[CamToCam] v0_t = " << v0_t.transpose()<<" v0_q = "<< v0_q << std::endl;
					std::cout<<" CamtoCam_error = "<<err_total/(bf_count*objectPoints[0].size())<<std::endl;

					std::cout<<std::endl<<"done & exit "<<std::endl<<std::endl;

				}
	
			}

			if( Pair_idx == OptimizeNum && task.state() == ef_calib_node::State::OPTIMIZE){
				std::cout<<std::endl
							<<"*************************************************************"<<std::endl
							<<"[CALIB] collected "<<OptimizeNum-initialNum<<" event-frame pairs"<<std::endl
							<<"Finish to calibrate, the system come into the [Null] state. "<<std::endl
							<<"**************************************************************"<<std::endl;
				std::cout<<std::endl;
				task.stateNull(); // The stateOptimize state is blocked
				continueThread = false;
			}

		}


		// /empty function now  // for ablation study
		if(task.state() == ef_calib_node::State::OPTIMIZE){
			
			task.pairMap->PairLockShared();
			std::map<uint64_t, ::ef_calib::calib::CalibPair::Ptr> MapTemp = task.pairMap->copyPairs();
			task.pairMap->PairUnLockShared();

			std::vector<std::vector<cv::Point3f> > objectPoints(1);
			task.calcBoardCornerPositions(objectPoints[0]);

			// ablation study 

			task.stateNull();
			continueThread = false;
		}

		std::this_thread::sleep_for(5ms);
		}//continueThread
	});



	ROS_INFO_STREAM("[calib_node] >>>>>>>>>>>>>>> Finished All the Configuretions <<<<<<<<<<<<<<<");
	// Sping ros
	while (ros::ok()) {
		ros::spinOnce();
		std::this_thread::sleep_for(1ms);
	}

	continueThread = false;
	eventsPickerThread.join();
	efPairThread.join();
	CalibThread.join();
	return 0;
}
