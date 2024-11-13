#include <ef_calib/calib/PoseCalib_g2o.hpp>
#include <g2o/core/robust_kernel_impl.h>

using namespace ef_calib::calib;

CurveFitting::CurveFitting(){
    auto solver = new g2o::OptimizationAlgorithmGaussNewton(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

    optimizer.setAlgorithm(solver); //set solver.
    optimizer.setVerbose(false); // close debug
    if(!optimizer.solver()){
        std::cout<<" G2O constructed failed. "<<std::endl;
    }

    vertex_idx = 0;
}

void CurveFitting::addVertex(Eigen::Vector3d abc){
    CurveFittingVertex *v = new CurveFittingVertex();
    v->setEstimate(abc);
    v->setId(vertex_idx);
    optimizer.addVertex(v);
    std::cout<<" insert vertex "<<vertex_idx<< ", its initial param = "<<abc.transpose()<<std::endl;
    v_vec.push_back(v);
    
    vertex_idx++;    
}

bool CurveFitting::addEdge(int id, std::vector<double> x_data, std::vector<double> y_data){
    if( id >= v_vec.size() || x_data.size() != y_data.size())
        return false;
        
    for(int i=0; i<x_data.size(); i++){
        CurveFittingEdge *edge = new CurveFittingEdge(x_data[i]);
        edge->setId(i);
        edge->setVertex(id, v_vec[id]);
        edge->setMeasurement(y_data[i]);
        // double w_sigma = 1.0;
        // edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity()*1/(w_sigma*w_sigma));
        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
        optimizer.addEdge(edge);
        // std::cout<<" for vertex "<<id<< ", data["<<i<<"], x_ = "<<x_data[i]<<", y_ = "<<y_data[i]<<std::endl;
    }
    return true;
}

void CurveFitting::optimize(){
    // 执行优化
    std::cout << "start optimization" << std::endl;
    // chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(20);
    // chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    // chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    // cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

    // 输出优化值
    for(int i=0; i<v_vec.size(); i++){
        Eigen::Vector3d abc_estimate = v_vec[i]->estimate();
        std::cout << "No. "<<i << " vertex, estimated model: " << abc_estimate.transpose() << std::endl;
    }
}


// ****  pose PnP Estimation with know camera Intrinsic *** //

SolvePnPG2O::SolvePnPG2O(Eigen::Matrix3d K, Eigen::MatrixXd Distortion){
    auto solver = new g2o::OptimizationAlgorithmGaussNewton(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

    optimizer.setAlgorithm(solver); //set solver.
    optimizer.setVerbose(true); // open debug
    if(!optimizer.solver()){
        std::cout<<" G2O constructed failed. "<<std::endl;
    }

    vertex_idx = 0;

    K_eigen = K;
    std::cout<<" K_eigen :"<<std::endl<< K_eigen<<std::endl;

    Distortion_eigen = Distortion;
    std::cout<<" Distortion_eigen "<< Distortion_eigen.transpose()<<std::endl;
}

void SolvePnPG2O::addVertex(Sophus::SE3d ini){
    PoseVertex *v = new PoseVertex();
    v->setEstimate(ini);
    v->setId(vertex_idx);
    optimizer.addVertex(v);
    // std::cout<<" insert vertex "<<vertex_idx<< ", its param = "<<abc.transpose()<<std::endl;
    v_vec.push_back(v);
    vertex_idx++;    
}

bool SolvePnPG2O::addEdge(VecEigen3d points_3d, VecEigen2d points_2d){
    if( v_vec.size() == 0 || points_3d.size() != points_2d.size())
        return false;

    for (size_t i = 0; i < points_2d.size(); ++i) {
        Eigen::Vector2d p2d = points_2d[i];
        Eigen::Vector3d p3d = points_3d[i];
        PnPEdge *edge = new PnPEdge(p2d, K_eigen, Distortion_eigen);
        edge->setId(i);
        edge->setVertex(0, v_vec[vertex_idx-1]); // for BaseUnaryEdge(), the Vertex_id = 0; for BaseBinaryEdge(), the Vertex_id = 0 or 1;
        edge->setMeasurement(p3d);
        edge->setInformation(Eigen::Matrix3d::Identity()*1.0);
        optimizer.addEdge(edge);
        // std::cout<<" id = "<<(vertex_idx-1)*points_2d.size() + i<<" p2d "<< p2d.transpose()<<", p3d "<< p3d.transpose()<<std::endl;
    }
    return true;
}

void SolvePnPG2O::optimize(){
    // 执行优化
    std::cout << "start optimization" << std::endl;
    // chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    // chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    // chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    // cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

    // 输出优化值
    for(int i=0; i<v_vec.size(); i++){
        Sophus::SE3d esti = v_vec[i]->estimate();
        Eigen::Matrix4d pose_RT = esti.matrix();
        Eigen::Matrix3d pose_R;
        pose_R << pose_RT(0, 0), pose_RT(0, 1), pose_RT(0, 2),
                  pose_RT(1, 0), pose_RT(1, 1), pose_RT(1, 2),
                  pose_RT(2, 0), pose_RT(2, 1), pose_RT(2, 2);
        Eigen::Quaterniond pose_q(pose_R);
        Eigen::Vector3d pose_t;
        pose_t << pose_RT(0, 3), pose_RT(1, 3), pose_RT(2, 3);
        // std::cout << "No. "<<i << " vertex, estimated model: "<< " t = "<< pose_t.transpose()<< " R = " << pose_q  << std::endl;
    }
}


// ****  pose and camera Intrinsic Estimation *** //

/*
SolvePnPIntrinsicG2O::SolvePnPIntrinsicG2O(){
    auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

    optimizer.setAlgorithm(solver); //set solver.
    optimizer.setVerbose(true); // open debug
    if(!optimizer.solver()){
        std::cout<<" G2O constructed failed. "<<std::endl;
    }

    vertex_idx = 0;
}

void SolvePnPIntrinsicG2O::addVertex(Sophus::SE3d T, Eigen::Matrix3d K, Eigen::VectorXd dis){
    PoseIntrinsicVertex *v = new PoseIntrinsicVertex();
    v->setEstimate(PoseAndIntrinsics(T, K, dis));
    v->setId(vertex_idx);
    // g2o在BA中需要手动设置待Marg的顶点
    // v->setMarginalized(true);
    optimizer.addVertex(v);
    // std::cout<<" insert vertex "<<vertex_idx<< ", its param = "<<abc.transpose()<<std::endl;
    v_vec.push_back(v);
    vertex_idx++;    
}

bool SolvePnPIntrinsicG2O::addEdge(VecEigen3d points_3d, VecEigen2d points_2d){
    if( v_vec.size() == 0 || points_3d.size() != points_2d.size())
        return false;

    for (size_t i = 0; i < points_2d.size(); ++i) {
        Eigen::Vector2d p2d = points_2d[i];
        Eigen::Vector3d p3d = points_3d[i];
        ProjectionEdge *edge = new ProjectionEdge(p3d);
        edge->setId(i);
        edge->setVertex(0, v_vec[vertex_idx-1]); // for BaseUnaryEdge(), the Vertex_id = 0; for BaseBinaryEdge(), the Vertex_id = 0 or 1;
        edge->setMeasurement(p2d);
        edge->setInformation(Eigen::Matrix2d::Identity());
        edge->setRobustKernel(new g2o::RobustKernelHuber());
        optimizer.addEdge(edge);
        // std::cout<<" p2d "<< p2d.transpose()<<", p3d "<< p3d.transpose()<<std::endl;
    }
    return true;
}

void SolvePnPIntrinsicG2O::optimize(){
    // 执行优化
    std::cout << "start optimization" << std::endl;
    // chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(40);
    // chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    // chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    // cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

    // 输出优化值
    for(int i=0; i<v_vec.size(); i++){
        PoseAndIntrinsics esti = v_vec[i]->estimate();
        std::cout << "No. "<<i << " vertex, estimated model: " << std::endl
                  << " RotationAndTranslation = " <<std::endl << esti.RT.matrix() <<std::endl
                  <<"  focal = " << esti.focal << "  cx = " << esti.cx <<"  cy = " << esti.cy << std::endl
                  <<"  k1 = " << esti.k1 << "  k2 = " << esti.k2 <<"  p1 = " << esti.p1 <<"  p2 = " << esti.p2 << "  k3 = " << esti.k3 << std::endl;
    }
}

*/


// ****  pose PnP Pose Estimation amd RT between two cameras, with know camera Intrinsic *** //


SolveCamToCam::SolveCamToCam(Eigen::Matrix3d K_es, Eigen::MatrixXd Distortion_es, Eigen::Matrix3d K_fr, Eigen::MatrixXd Distortion_fr){
    auto solver = new g2o::OptimizationAlgorithmGaussNewton(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

    optimizer.setAlgorithm(solver); //set solver.
    optimizer.setVerbose(true); // open debug
    if(!optimizer.solver()){
        std::cout<<" G2O constructed failed. "<<std::endl;
    }

    v0_idx = 0;

    K_es_eigen = K_es;
    Distortion_es_eigen = Distortion_es;
    std::cout<<"** [Event] Intrinsic "<<std::endl<< K_es_eigen<<std::endl
             << Distortion_es_eigen.transpose()<<std::endl;

    K_fr_eigen = K_fr;
    Distortion_fr_eigen = Distortion_fr;
    std::cout<<"** [Frame] Intrinsic "<<std::endl<< K_fr_eigen<<std::endl
             << Distortion_fr_eigen.transpose()<<std::endl;
}



void SolveCamToCam::addVertex(Sophus::SE3d ini_es, Sophus::SE3d ini_fr){
    v0_idx++;

    PoseVertex *v_es = new PoseVertex();
    v_es->setEstimate(ini_es);
    v_es->setId(v0_idx*2-1);
    optimizer.addVertex(v_es);
    // std::cout<<" insert vertex "<<vertex_idx<< ", its param = "<<abc.transpose()<<std::endl;
    v0_es_vec.push_back(v_es);

    PoseVertex *v_fr = new PoseVertex();
    v_fr->setEstimate(ini_fr);
    v_fr->setId(v0_idx*2);
    optimizer.addVertex(v_fr);
    // std::cout<<" insert vertex "<<vertex_idx<< ", its param = "<<abc.transpose()<<std::endl;
    v0_fr_vec.push_back(v_fr);
    std::cout<<"**[vertex] add the vertex : id es "<<v0_idx*2-1<< ", id fr "<< v0_idx*2<<std::endl;
}

// excute only once
void SolveCamToCam::addVertexAmongTwo(Sophus::SE3d ini){
    if(v1_exist)
        return;
    v1 = new PoseVertex();
    v1->setEstimate(ini);
    v1->setId(0);
    optimizer.addVertex(v1);
    // std::cout<<" insert vertex "<<vertex_idx<< ", its param = "<<abc.transpose()<<std::endl; 
    v1_exist = true;  
}


Eigen::Vector2d FromCamNormToCamdis(Eigen::Vector2d p_norm, Eigen::VectorXd distCoeffs_){
    // distortion [k1, k2, p1, p2, k3]
    double xx = p_norm[0] * p_norm[0];
    double yy = p_norm[1] * p_norm[1];
    double r2 = xx + yy;
    double r4 = r2 * r2;
    double r6 = r4 * r2;
    double xy = 2 * p_norm[0] * p_norm[1];
    double r_coeff = 1.0 + distCoeffs_[0] * r2 + distCoeffs_[1] * r4 + distCoeffs_[4] * r6 ;

    double px = p_norm[0] * r_coeff;
    double py = p_norm[1] * r_coeff;
    px += distCoeffs_[2] *         xy  + distCoeffs_[3] * (r2 + 2*xx);
    py += distCoeffs_[2] * (r2 + 2*yy) + distCoeffs_[3] *         xy ;

    return Eigen::Vector2d(px, py);
}
Eigen::Vector3d FromPixelToCam(Eigen::Vector2d uv, Eigen::Matrix3d K_, Eigen::VectorXd distCoeffs_){
    double x_dis = (uv[0] - K_(0, 2)) /  K_(0, 0);
    double y_dis = (uv[1] - K_(1, 2)) /  K_(1, 1);
    Eigen::Vector2d x_d(x_dis, y_dis);
    Eigen::Vector2d x_m = x_d;
    bool isFound = false;
    for(int itr=0; itr< 20; itr++){
        Eigen::Vector2d delta = x_d - FromCamNormToCamdis(x_m, distCoeffs_);
        // std::cout<<" itr = "<<itr<<" delta = "<<delta.transpose()<<std::endl;
        x_m += delta;
        if(delta[0]< 1e-8 && delta[1]<1e-8){
            isFound = true;
            break;
        }
    }
    if(!isFound) std::cout<<" something is wrong about inverse Distortion. "<<std::endl;

    return Eigen::Vector3d(x_m[0], x_m[1], 1.0);
}


//g2o::RobustKernelHuber has the power of limiting the edge error
// add edge for latest vertex
bool SolveCamToCam::addEdge(VecEigen3d points_3d, VecEigen2d points_2d_es, VecEigen2d points_2d_fr, Eigen::Vector4d param){
    std::cout<<"**[Edge] add the edges : ";
    if( v0_es_vec.size() == 0 || v0_fr_vec.size() == 0 || !v1_exist || points_3d.size() != points_2d_es.size() || points_3d.size() != points_2d_fr.size() )
        return false;

    double weight = 5.0; //5.0
    for (size_t i = 0; i < points_3d.size(); ++i) {
        Eigen::Vector2d p2d_es = points_2d_es[i];
        Eigen::Vector2d p2d_fr = points_2d_fr[i];
        Eigen::Vector3d p3d = points_3d[i];

        PnPEdge *edge0_es = new PnPEdge(p2d_es, K_es_eigen, Distortion_es_eigen);
        edge0_es->setId( ((v0_idx-1)*4 + 2) * points_3d.size() + i );
        edge0_es->setVertex(0, v0_es_vec[v0_idx-1]); // for BaseUnaryEdge(), the Vertex_id = 0; for BaseBinaryEdge(), the Vertex_id = 0 or 1;
        edge0_es->setMeasurement(p3d);
        edge0_es->setInformation(Eigen::Matrix3d::Identity());
        g2o::RobustKernelHuber *rkhuber_es = new g2o::RobustKernelHuber;
        rkhuber_es->setDelta(param(0));
        edge0_es->setRobustKernel(rkhuber_es);
        optimizer.addEdge(edge0_es);
        // std::cout<<" p2d "<< p2d.transpose()<<", p3d "<< p3d.transpose()<<std::endl;

        PnPEdge *edge0_fr = new PnPEdge(p2d_fr, K_fr_eigen, Distortion_fr_eigen);
        edge0_fr->setId( ((v0_idx-1)*4 + 3) * points_3d.size() + i );
        edge0_fr->setVertex(0, v0_fr_vec[v0_idx-1]); // for BaseUnaryEdge(), the Vertex_id = 0; for BaseBinaryEdge(), the Vertex_id = 0 or 1;
        edge0_fr->setMeasurement(p3d);
        edge0_fr->setInformation(Eigen::Matrix3d::Identity());
        g2o::RobustKernelHuber *rkhuber_fr = new g2o::RobustKernelHuber;
        rkhuber_fr->setDelta(param(1));
        edge0_fr->setRobustKernel(rkhuber_fr);
        optimizer.addEdge(edge0_fr);
        // std::cout<<" p2d "<< p2d.transpose()<<", p3d "<< p3d.transpose()<<std::endl;

        CamToCamEdge *edge1_es = new CamToCamEdge(p2d_es, K_es_eigen, Distortion_es_eigen, K_fr_eigen, Distortion_fr_eigen, true);
        edge1_es->setId( ((v0_idx-1)*4 + 0) * points_3d.size() + i );
        edge1_es->setVertex(0, v0_es_vec[v0_idx-1]); // for BaseBinaryEdge(), the Vertex_id = 0 or 1;
        edge1_es->setVertex(1, v1); //  for BaseBinaryEdge(), the Vertex_id = 0 or 1;
        edge1_es->setMeasurement(p2d_fr);
        edge1_es->setInformation(Eigen::Matrix2d::Identity()*weight);
        g2o::RobustKernelHuber *rkhuber_es2 = new g2o::RobustKernelHuber;
        rkhuber_es2->setDelta(param(2));
        edge1_es->setRobustKernel(rkhuber_es2);
        optimizer.addEdge(edge1_es);

        CamToCamEdge *edge1_fr = new CamToCamEdge(p2d_fr, K_es_eigen, Distortion_es_eigen, K_fr_eigen, Distortion_fr_eigen, false);
        edge1_fr->setId( ((v0_idx-1)*4 + 1) * points_3d.size() + i );
        edge1_fr->setVertex(0, v0_fr_vec[v0_idx-1]); // for BaseUnaryEdge(), the Vertex_id = 0; for BaseBinaryEdge(), the Vertex_id = 0 or 1;
        edge1_fr->setVertex(1, v1); // for BaseUnaryEdge(), the Vertex_id = 0; for BaseBinaryEdge(), the Vertex_id = 0 or 1;
        edge1_fr->setMeasurement(p2d_es);
        edge1_fr->setInformation(Eigen::Matrix2d::Identity()*weight);
        g2o::RobustKernelHuber *rkhuber_fr2 = new g2o::RobustKernelHuber;
        rkhuber_fr2->setDelta(param(3));
        edge1_fr->setRobustKernel(rkhuber_fr2);
        optimizer.addEdge(edge1_fr);
    }
    std::cout<<"   esTofr("<< ((v0_idx-1)*4 + 0) * points_3d.size() <<"-"<< ((v0_idx-1)*4 + 0) * points_3d.size() + points_3d.size()-1
             <<"), frToes("<< ((v0_idx-1)*4 + 1) * points_3d.size() <<"-"<< ((v0_idx-1)*4 + 1) * points_3d.size() + points_3d.size()-1
             <<"), es("    << ((v0_idx-1)*4 + 2) * points_3d.size() <<"-"<< ((v0_idx-1)*4 + 2) * points_3d.size() + points_3d.size()-1 
             <<"), fr("    << ((v0_idx-1)*4 + 3) * points_3d.size() <<"-"<< ((v0_idx-1)*4 + 3) * points_3d.size() + points_3d.size()-1<<");  ";
    return true;
}


void SolveCamToCam::optimize(){
    // 执行优化
    std::cout << "start optimization" << std::endl;
    // chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(50);
    // chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    // chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    // cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

    // 输出优化值
    for(int i=0; i<v0_es_vec.size(); i++){
        std::cout<<"No."<<i<<std::endl;
        Sophus::SE3d v0_estimate_es = v0_es_vec[i]->estimate();
        Eigen::Matrix4d v0_Rt_es = v0_estimate_es.matrix();  
        Eigen::Matrix3d v0_R_es = v0_Rt_es.block<3, 3>(0, 0);
        Eigen::Vector3d v0_t_es = v0_Rt_es.block<3, 1>(0, 3);
        Eigen::Quaterniond v0_q_es(v0_R_es);
        v0_q_es.normalize();
        std::cout << "[es] v0_t = " << v0_t_es.transpose()<<" v0_q = "<< v0_q_es << std::endl;

        Sophus::SE3d v0_estimate_fr = v0_fr_vec[i]->estimate();        
        Eigen::Matrix4d v0_Rt_fr = v0_estimate_fr.matrix();  
        Eigen::Matrix3d v0_R_fr = v0_Rt_fr.block<3, 3>(0, 0);
        Eigen::Vector3d v0_t_fr = v0_Rt_fr.block<3, 1>(0, 3);
        Eigen::Quaterniond v0_q_fr(v0_R_fr);
        v0_q_fr.normalize();
        std::cout << "[fr] v0_t = " << v0_t_fr.transpose()<<" v0_q = "<< v0_q_fr << std::endl;
    }
    Sophus::SE3d v1_estimate = v1->estimate();
    std::cout << std::endl << " Among two, estimated model: esTofr = " << std::endl 
              << v1_estimate.matrix() << std::endl;
}


// ****  pose and camera Intrinsic Estimation *** //

/*
SolvePnPIntrinsicG2O::SolvePnPIntrinsicG2O(){
    auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

    optimizer.setAlgorithm(solver); //set solver.
    optimizer.setVerbose(true); // open debug
    if(!optimizer.solver()){
        std::cout<<" G2O constructed failed. "<<std::endl;
    }

    v0_idx = 0;
}

// excute only once
void SolvePnPIntrinsicG2O::addIntrinsicVertex(Eigen::Matrix<double, 4, 1> ini){
    if(this->v1_exist)
        return;
    v1 = new IntrinsicVertex();
    v1->setEstimate(ini);
    v1->setId(0);
    // v1->setMarginalized(true);
    optimizer.addVertex(v1);
    std::cout<<" insert vertex "<<0<< ", its param = "<<ini.transpose()<<std::endl; 
    this->v1_exist = true;  
}

void SolvePnPIntrinsicG2O::addVertex(Sophus::SE3d T){
    v0_idx++;

    PoseVertex *v = new PoseVertex();
    v->setEstimate(T);
    v->setId(v0_idx);
    // // g2o在BA中需要手动设置待Marg的顶点
    // // v->setMarginalized(true);
    optimizer.addVertex(v);
    // std::cout<<" insert vertex "<<v0_idx<< ", its param = "<<T.matrix()<<std::endl;
    v0_vec.push_back(v);
       
}

bool SolvePnPIntrinsicG2O::addEdge(VecEigen3d points_3d, VecEigen2d points_2d, Eigen::Matrix3d K){
    if( v0_vec.size() == 0 || !v1_exist || points_3d.size() != points_2d.size())
        return false;

    for (size_t i = 0; i < points_2d.size(); ++i) {
        Eigen::Vector2d p2d = points_2d[i];
        Eigen::Vector3d p3d = points_3d[i];
        PoseIntrinsicEdge *edge = new PoseIntrinsicEdge(p3d, K);
        edge->setVertex(0, v0_vec[v0_idx-1]); // for BaseUnaryEdge(), the Vertex_id = 0; for BaseBinaryEdge(), the Vertex_id = 0 or 1;
        edge->setVertex(1, v1);
        edge->setMeasurement(p2d);
        edge->setInformation(Eigen::Matrix2d::Identity());
        // edge->setRobustKernel(new g2o::RobustKernelHuber());
        optimizer.addEdge(edge);
        // std::cout<<" p2d "<< p2d.transpose()<<", p3d "<< p3d.transpose()<<std::endl;
    }
    return true;
}

void SolvePnPIntrinsicG2O::optimize(){
    // 执行优化
    std::cout << "start optimization" << std::endl;
    // chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(100);
    // chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    // chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    // cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

    // // 输出优化值
    for(int i=0; i<v0_vec.size(); i++){
        Sophus::SE3d esti = v0_vec[i]->estimate();
        std::cout << "No. "<<i << " vertex, estimated model: " << std::endl
                  << " RotationAndTranslation = " <<std::endl << esti.matrix() <<std::endl;
    }

    Eigen::Matrix<double, 4, 1> Intrin = v1->estimate();
    std::cout << " Intrinsic vertex, estimated model: "<< Intrin.transpose() << std::endl;
}
*/

// ****  pose PnP Pose Estimation and 3D points, with know camera Intrinsic *** //
/*
PosePoint3DG2O::PosePoint3DG2O(){
    auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

    optimizer.setAlgorithm(solver); //set solver.
    optimizer.setVerbose(true); // open debug
    if(!optimizer.solver()){
        std::cout<<" G2O constructed failed. "<<std::endl;
    }

    v0_idx = 0;
    v1_idx = 0;
}

void PosePoint3DG2O::addPoseVertex(Sophus::SE3d T){
    PoseVertex *v = new PoseVertex();
    v->setEstimate(T);
    v->setId(v0_idx);
    // // g2o在BA中需要手动设置待Marg的顶点
    // // v->setMarginalized(true);
    optimizer.addVertex(v);
    // std::cout<<" insert vertex "<<v0_idx<< ", its param = "<<T.matrix()<<std::endl;
    v0_vec.push_back(v);
    v0_idx++; 
}

void PosePoint3DG2O::addPoint3DVertex(Eigen::Vector3d ini){
    Point3DVertex *v = new Point3DVertex();
    v->setEstimate(ini);
    v->setId(v1_idx);
    // v1->setMarginalized(true);
    optimizer.addVertex(v);
    // std::cout<<" insert vertex "<<0<< ", its param = "<<ini.transpose()<<std::endl; 
    v1_vec.push_back(v); 
    v1_idx++;
}

bool PosePoint3DG2O::addEdge(VecEigen3d pw_3d, VecEigen3d pc_3d){
    if( v0_vec.size() == 0 || v1_vec.size() == 0 || pw_3d.size() != pc_3d.size())
        return false;

    for (size_t i = 0; i < pw_3d.size(); ++i) {
        Eigen::Vector3d pw = pw_3d[i];
        Eigen::Vector3d pc = pc_3d[i];

        PosePoint3DEdge *edge = new PosePoint3DEdge();
        edge->setVertex(0, v0_vec[v0_idx-1]); // for BaseUnaryEdge(), the Vertex_id = 0; for BaseBinaryEdge(), the Vertex_id = 0 or 1;
        edge->setVertex(1, v1_vec[i]);
        edge->setMeasurement(pc);
        edge->setInformation(Eigen::Matrix3d::Identity());
        // edge->setRobustKernel(new g2o::RobustKernelHuber());
        optimizer.addEdge(edge);
        // std::cout<<" p2d "<< p2d.transpose()<<", p3d "<< p3d.transpose()<<std::endl;


        Point3DEdge *edge2 = new Point3DEdge();
        edge2->setVertex(0, v1_vec[i]);
        edge2->setMeasurement(pw);
        edge2->setInformation(Eigen::Matrix3d::Identity());
        // edge->setRobustKernel(new g2o::RobustKernelHuber());
        optimizer.addEdge(edge2);
    }
    return true;
}

void PosePoint3DG2O::optimize(){
    // 执行优化
    std::cout << "start optimization" << std::endl;
    // chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(100);
    // chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    // chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    // cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

    // // 输出优化值
    for(int i=0; i<v0_vec.size(); i++){
        Sophus::SE3d esti = v0_vec[i]->estimate();
        std::cout << "No. "<<i << " vertex, estimated model: " << std::endl
                  << " RotationAndTranslation = " <<std::endl << esti.matrix() <<std::endl;
    }
    for(int i=0; i<v1_vec.size(); i++){
        Eigen::Vector3d esti = v1_vec[i]->estimate();
        std::cout << "No. "<<i << " vertex, estimated model: " << std::endl
                  << " 3D_Points = " <<std::endl << esti.transpose() <<std::endl;
    }
}

*/

