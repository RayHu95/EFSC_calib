#ifndef ef_calib_PoseCalib_G2O_HPP
#define ef_calib_PoseCalib_G2O_HPP

#include<iostream>
#include<g2o/core/g2o_core_api.h>
#include<g2o/core/base_vertex.h>
#include<g2o/core/base_unary_edge.h>
#include<g2o/core/base_binary_edge.h>
#include<g2o/core/block_solver.h>
#include<g2o/core/optimization_algorithm_levenberg.h>
#include<g2o/core/optimization_algorithm_gauss_newton.h>
// #include<g2o/core/optimization_algorithm_dogleg.h>
#include<g2o/solvers/dense/linear_solver_dense.h>
#include<Eigen/Core>

#include <sophus/se3.hpp>

using namespace std;

namespace ef_calib {
    namespace calib {

    class CurveFittingVertex: public g2o::BaseVertex<3, Eigen::Vector3d>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        CurveFittingVertex() = default;

        virtual void setToOriginImpl() override { // reset
            _estimate << 0, 0, 0;
        }

        virtual void oplusImpl( const double *update) override { // update
            _estimate +=Eigen::Vector3d(update);
        }
        virtual bool read( istream &in) {return false;}
        virtual bool write( ostream &out) const {return false;}
    };

    class CurveFittingEdge: public g2o::BaseUnaryEdge<1, double, CurveFittingVertex>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        CurveFittingEdge(double x ): BaseUnaryEdge(), _x(x) {}

        virtual void computeError() override { 
            const CurveFittingVertex *v = static_cast<const CurveFittingVertex *> (_vertices[0]);
            const Eigen::Vector3d abc = v->estimate();
            _error(0, 0) = _measurement - std::exp(abc(0,0)*_x*_x + abc(1,0)*_x + abc(2,0));
        }

        virtual bool read( istream &in) {return false;}
        virtual bool write( ostream &out) const {return false;}

    public:
        double _x;
    };

    class CurveFitting{
    public:
        typedef std::shared_ptr<CurveFitting> Ptr;

        typedef g2o::BlockSolver<g2o::BlockSolverTraits<3,1>> BlockSolverType;
		typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
        
        CurveFitting();

        void addVertex(Eigen::Vector3d abc);

        bool addEdge(int id, std::vector<double> x_data, std::vector<double> y_data);

        void optimize();


    protected:
        int vertex_idx;
        std::vector<CurveFittingVertex*> v_vec;
        g2o::SparseOptimizer optimizer;
    }; // CurveFitting class

// ****  pose PnP Pose Estimation, with know camera Intrinsic *** //

    class PoseVertex: public g2o::BaseVertex<6, Sophus::SE3d>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        PoseVertex() = default;

        virtual void setToOriginImpl() override { // reset
            _estimate = Sophus::SE3d();
        }

        virtual void oplusImpl( const double *update) override { // update
            Eigen::Matrix<double, 6, 1> update_eigen;
            update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
            _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
        }

        virtual bool read( istream &in) {return false;}
        virtual bool write( ostream &out) const {return false;}
    };

    class PnPEdge: public g2o::BaseUnaryEdge<3, Eigen::Vector3d, PoseVertex>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        PnPEdge(const Eigen::Vector2d &pos, const Eigen::Matrix3d &K, Eigen::VectorXd &distortion)
                : BaseUnaryEdge(), _pp2d(pos), _K(K), _dist(distortion){}

        virtual void computeError() override { 
            const PoseVertex *v = static_cast<PoseVertex *> (_vertices[0]);
            Sophus::SE3d T = v->estimate();

            double x_dis = (_pp2d[0] - _K(0, 2)) /  _K(0, 0);
            double y_dis = (_pp2d[1] - _K(1, 2)) /  _K(1, 1);
            Eigen::Vector2d x_d(x_dis, y_dis);
            Eigen::Vector2d x_m = x_d;
            bool isFound = false;
            for(int itr=0; itr< 20; itr++){
                Eigen::Vector2d pc{0.0, 0.0};
                double xx = x_m[0] * x_m[0];
                double yy = x_m[1] * x_m[1];
                double r2 = xx + yy;
                double xy = 2 * x_m[0] * x_m[1];
                double distortion = 1.0 + r2 * (_dist[0] + _dist[1] * r2 + _dist[4] * r2 * r2);
                pc[0] = x_m[0] * distortion + _dist[2] *         xy  + _dist[3] * (r2 + 2*xx);
                pc[1] = x_m[1] * distortion + _dist[2] * (r2 + 2*yy) + _dist[3] *         xy ;
                Eigen::Vector2d delta = x_d - pc;
                // std::cout<<" itr = "<<itr<<" delta = "<<delta.transpose()<<std::endl;
                x_m += delta;
                if(delta[0]< 1e-8 && delta[1]<1e-8){
                    isFound = true;
                    break;
                }
            }
            if(!isFound) std::cout<<" something is wrong about inverse Distortion. "<<std::endl;
            Eigen::Vector3d pc{x_m[0], x_m[1], 1.0};

            Eigen::Matrix4d T_Rt = T.matrix();
            Eigen::Vector3d T_Rota2{T_Rt(2,0), T_Rt(2,1), T_Rt(2,2)};
            double depth = - T_Rt(2,3) / (T_Rota2.dot(pc));

            // std::cout<<" pc "<<pc.transpose() <<",  depth = "<< depth<<std::endl;
            pc *= depth;
            // Eigen::Vector3d pw = T.inverse() * pc;
            _error = T * _measurement - pc;
            // std::cout<<"  depth = "<< depth <<", error = "<<_error.transpose()<< " norm = "<<_error.norm()<<std::endl;
        }

        virtual bool read( istream &in) {return false;}
        virtual bool write( ostream &out) const {return false;}

    private:
        Eigen::Vector2d _pp2d;
        Eigen::Matrix3d _K;
        Eigen::VectorXd _dist;
    };

    class SolvePnPG2O{
    public:
        typedef std::shared_ptr<SolvePnPG2O> Ptr;

        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType; // pose is 6, landmark is 3
		typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

        // BA by g2o
        typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecEigen2d;
        typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecEigen3d;
        
        SolvePnPG2O(Eigen::Matrix3d K, Eigen::MatrixXd Distortion);

        void addVertex(Sophus::SE3d ini);

        bool addEdge(VecEigen3d points_3d, VecEigen2d points_2d);

        void optimize();

        int idx(){return vertex_idx;};
        Sophus::SE3d pose(int i){return v_vec[i]->estimate();};

    protected:
        int vertex_idx;
        std::vector<PoseVertex*> v_vec;
        g2o::SparseOptimizer optimizer;
        Eigen::Matrix3d K_eigen;
        Eigen::VectorXd Distortion_eigen;
    }; // SolvePnPG2O class

    class PnPBetweenEdge: public g2o::BaseUnaryEdge<3, Eigen::Vector3d, PoseVertex>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        PnPBetweenEdge(const Eigen::Vector3d &pos_es, const Eigen::Vector3d &pos_fr, bool isForward)
                       : BaseUnaryEdge(), _pp3d_es(pos_es), _pp3d_fr(pos_fr), _isForward(isForward){}

        virtual void computeError() override { 
            const PoseVertex *v = static_cast<PoseVertex *> (_vertices[0]);
            Sophus::SE3d T = v->estimate();
            
            Eigen::Vector3d pc;
            if(_isForward){ 
                pc = T * _pp3d_es;
                // std::cout<<"  depth2 = "<< pc_out[2];
            } else {
                Eigen::Matrix4d T_Rt = T.matrix();
                Eigen::Matrix3d T_R = T_Rt.block<3, 3>(0, 0);
                Eigen::Vector3d T_t = T_Rt.block<3, 1>(0, 3);
                Eigen::Quaterniond T1_q(T_R);
                T1_q.normalize();
                pc = T1_q.conjugate() * (_pp3d_fr - T_t);   
            }
            
            _error = _measurement - pc;
            // std::cout<< "  error = "<<_error.norm()<<std::endl;
        }

        virtual bool read( istream &in) {return false;}
        virtual bool write( ostream &out) const {return false;}

    private:
        Eigen::Vector3d _pp3d_es;
        Eigen::Vector3d _pp3d_fr;
        bool _isForward = true;
    };

    class SolveBetweenG2O{
    public:
        typedef std::shared_ptr<SolveBetweenG2O> Ptr;

        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType; // pose is 6, landmark is 3
		typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

        // BA by g2o
        typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecEigen3d;
        
        SolveBetweenG2O(){
            auto solver = new g2o::OptimizationAlgorithmGaussNewton(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

            optimizer.setAlgorithm(solver); //set solver.
            optimizer.setVerbose(true); // open debug
            if(!optimizer.solver()){
                std::cout<<" G2O constructed failed. "<<std::endl;
            }
        
            // the pose between two cameras. 
            v = new PoseVertex();
            v->setEstimate(Sophus::SE3d(Eigen::Quaterniond(1,0,0,0), Eigen::Vector3d(10.0, 0.0, 0.0)));
            v->setId(0);
            optimizer.addVertex(v);
        }

        void addEdge(Eigen::Vector3d p3d_es, Eigen::Vector3d p3d_fr, double w_sigma_es, double w_sigma_fr ){
            pose_in.push_back(p3d_es);
            pose_out.push_back(p3d_fr);

            PnPBetweenEdge *edge_between_es = new PnPBetweenEdge(p3d_es, p3d_fr, true);
            edge_between_es->setId(pose_in.size()-1);
            edge_between_es->setVertex(0, v); // for BaseUnaryEdge(), the Vertex_id = 0; for BaseBinaryEdge(), the Vertex_id = 0 or 1;
            edge_between_es->setMeasurement(p3d_fr);
            edge_between_es->setInformation(Eigen::Matrix3d::Identity()*1.0/w_sigma_es);
            optimizer.addEdge(edge_between_es);

            PnPBetweenEdge *edge_between_fr = new PnPBetweenEdge(p3d_es, p3d_fr, false);
            edge_between_fr->setId(pose_out.size()-1);
            edge_between_fr->setVertex(0, v); // for BaseUnaryEdge(), the Vertex_id = 0; for BaseBinaryEdge(), the Vertex_id = 0 or 1;
            edge_between_fr->setMeasurement(p3d_es);
            edge_between_fr->setInformation(Eigen::Matrix3d::Identity()*1.0/w_sigma_fr);
            optimizer.addEdge(edge_between_fr);

        };

        void optimize(){
            std::cout << "start optimization" << std::endl;
            // chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
            optimizer.initializeOptimization();
            optimizer.optimize(10);
            // chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
            // chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
            // cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

            // 输出优化值
            Sophus::SE3d esti = v->estimate();
            Eigen::Matrix4d pose_ef = esti.matrix();
            Eigen::Matrix3d pose_R = pose_ef.block<3, 3>(0, 0);
            Eigen::Quaterniond pose_q(pose_R);
            Eigen::Vector3d pose_t = pose_ef.block<3, 1>(0, 3);
            std::cout << " pose between two cameras : "<< " t = "<< pose_t.transpose()<< "  R = " << pose_q  << std::endl;

            double error = 0.0;
            for(size_t i = 0; i < pose_in.size(); ++i){
                error += (esti * pose_in[i] -  pose_out[i]).norm();
            }
            std::cout << std::endl << " error : "<< error/pose_in.size()  << std::endl;
        };

        // Sophus::SE3d pose(int i){return v_vec[i]->estimate();};

    protected:
        PoseVertex *v;
        g2o::SparseOptimizer optimizer;
        VecEigen3d pose_in;
        VecEigen3d pose_out;

    }; // SolveBetweenG2O class

// ****  pose and camera Intrinsic Estimation *** //
/*
    struct PoseAndIntrinsics {
        PoseAndIntrinsics() {}

        /// set from given data address
        explicit PoseAndIntrinsics(Sophus::SE3d T, Eigen::Matrix3d K, Eigen::VectorXd dis) {
            // rotation = Sophus::SO3d::exp(Eigen::Vector3d(data_addr[0], data_addr[1], data_addr[2]));
            // translation = Eigen::Vector3d(data_addr[3], data_addr[4], data_addr[5]);
            RT = T;
            focal = K(0, 0);
            cx = K(0, 2);
            cy = K(1, 2);
            k1 = dis[0];
            k2 = dis[1];
            p1 = dis[2];
            p2 = dis[3];
            k3 = dis[4];
        }

        // Sophus::SO3d rotation;
        // Eigen::Vector3d translation = Eigen::Vector3d::Zero();
        Sophus::SE3d RT;
        double focal = 0, cx = 0, cy = 0;
        double k1 = 0, k2 = 0, p1 = 0, p2 = 0, k3 = 0;
    };

    class PoseIntrinsicVertex: public g2o::BaseVertex<14, PoseAndIntrinsics>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        PoseIntrinsicVertex() = default;

        virtual void setToOriginImpl() override { // reset
            _estimate = PoseAndIntrinsics();
        }

        virtual void oplusImpl( const double *update) override { // update
            Eigen::Matrix<double, 6, 1> update_eigen;
            update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
            _estimate.RT = Sophus::SE3d::exp(update_eigen) * _estimate.RT;
            _estimate.focal += update[6];
            _estimate.cx += update[7];
            _estimate.cy += update[8];
            _estimate.k1 += update[9];
            _estimate.k2 += update[10];
            _estimate.p1 += update[11];
            _estimate.p2 += update[12];
            _estimate.k3 += update[13];
        }

        /// 根据估计值投影一个点
        Eigen::Vector2d project(const Eigen::Vector3d &point) {
            Eigen::Vector3d pc = _estimate.RT * point;
            pc = pc / pc[2];
            // double r2 = pc.squaredNorm();
            double xx = pc[0] * pc[0];
            double yy = pc[1] * pc[1];
            double r2 = xx + yy;
            double xy = 2 * pc[0] * pc[1];
            double distortion = 1.0 + r2 * (_estimate.k1 + _estimate.k2 * r2 + _estimate.k3 * r2 * r2);
            pc[0] = pc[0] * distortion + _estimate.p1 *         xy  + _estimate.p2 * (r2 + 2*xx);
            pc[1] = pc[1] * distortion + _estimate.p1 * (r2 + 2*yy) + _estimate.p2 *         xy ;

            pc[0] = pc[0] * _estimate.focal + _estimate.cx;
            pc[1] = pc[1] * _estimate.focal + _estimate.cy;
            return Eigen::Vector2d(pc[0], pc[1]);
        }

        virtual bool read( istream &in) {return false;}
        virtual bool write( ostream &out) const {return false;}
    };

    class ProjectionEdge: public g2o::BaseUnaryEdge<2, Eigen::Vector2d, PoseIntrinsicVertex>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ProjectionEdge(const Eigen::Vector3d &pos): BaseUnaryEdge(), _pos3d(pos) {}

        virtual void computeError() override {
            auto v = (PoseIntrinsicVertex *) _vertices[0];
            Eigen::Vector2d proj = v->project(_pos3d);
            _error = proj - _measurement;
        }

        virtual bool read( istream &in) {return false;}
        virtual bool write( ostream &out) const {return false;}

    private:
        Eigen::Vector3d _pos3d;
    };

    class SolvePnPIntrinsicG2O{
    public:
        typedef std::shared_ptr<SolvePnPIntrinsicG2O> Ptr;

        typedef g2o::BlockSolver<g2o::BlockSolverTraits<14, 3>> BlockSolverType; // pose is 6, K is 3, Distortion is 5; landmark is 3
		typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

        // BA by g2o
        typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecEigen2d;
        typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecEigen3d;
        
        SolvePnPIntrinsicG2O();

        void addVertex(Sophus::SE3d T, Eigen::Matrix3d K, Eigen::VectorXd dis);

        bool addEdge(VecEigen3d points_3d, VecEigen2d points_2d);

        void optimize();

        int idx(){return vertex_idx;};

    protected:
        int vertex_idx;
        std::vector<PoseIntrinsicVertex *> v_vec;
        g2o::SparseOptimizer optimizer;
    }; // SolvePnPIntrinsicG2O class
*/

// ****  pose PnP Pose Estimation amd RT between two cameras, with known camera Intrinsic *** //

    // class PoseVertex;

    // class PoseSingleEdge: public g2o::BaseUnaryEdge<2, Eigen::Vector2d, PoseVertex>
    // {
    // public:
    //     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //     PoseSingleEdge(const Eigen::Vector3d &pos, const Eigen::Matrix3d &K, Eigen::VectorXd &distortion)
    //                      : BaseUnaryEdge(), _pos3d(pos), _K(K), _dist(distortion){}
    //     virtual void computeError() override { 
    //         const PoseVertex *v = static_cast<PoseVertex *> (_vertices[0]);
    //         Sophus::SE3d T = v->estimate();
    //         Eigen::Vector3d pc = T * _pos3d;
    //         pc = pc / pc[2];
    //         // double r2 = pc.squaredNorm();
    //         double xx = pc[0] * pc[0];
    //         double yy = pc[1] * pc[1];
    //         double r2 = xx + yy;
    //         double xy = 2 * pc[0] * pc[1];
    //         double distortion = 1.0 + r2 * (_dist[0] + _dist[1] * r2 + _dist[4] * r2 * r2);
    //         pc[0] = pc[0] * distortion + _dist[2] *         xy  + _dist[3] * (r2 + 2*xx);
    //         pc[1] = pc[1] * distortion + _dist[2] * (r2 + 2*yy) + _dist[3] *         xy ;
    //         pc = _K * pc;
    //         _error = _measurement - pc.head<2>();
    //     }
    //     virtual bool read( istream &in) {return false;}
    //     virtual bool write( ostream &out) const {return false;}
    // private:
    //     Eigen::Vector3d _pos3d;
    //     Eigen::Matrix3d _K;
    //     Eigen::VectorXd _dist;
    // };

    class CamToCamEdge: public g2o::BaseBinaryEdge<2, Eigen::Vector2d, PoseVertex, PoseVertex>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        CamToCamEdge(const Eigen::Vector2d &pos, 
                     const Eigen::Matrix3d &K1, Eigen::VectorXd &distortion1,//event
                     const Eigen::Matrix3d &K2, Eigen::VectorXd &distortion2,//frame
                     bool isFromEventToFrame): BaseBinaryEdge(),
                        _pos2d(pos), _isForward(isFromEventToFrame)
                        , _K1(K1), _dist1(distortion1)
                        , _K2(K2), _dist2(distortion2){}

        virtual void computeError() override { 
            const PoseVertex *v0 = static_cast<PoseVertex *> (_vertices[0]); // pose_self
            const PoseVertex *v1 = static_cast<PoseVertex *> (_vertices[1]); // pose_CamToCam
            Sophus::SE3d T0 = v0->estimate();
            Sophus::SE3d T1 = v1->estimate();

            if(_isForward){ 
                // obtain the Normalized _pos3d 
                // from _pos2d at the pixel plane of the event camera 
                double x_dis = (_pos2d[0] - _K1(0, 2)) /  _K1(0, 0);
                double y_dis = (_pos2d[1] - _K1(1, 2)) /  _K1(1, 1);
                Eigen::Vector2d x_d(x_dis, y_dis);
                Eigen::Vector2d x_m = x_d;
                bool isFound = false;
                for(int itr=0; itr< 20; itr++){
                    Eigen::Vector2d pc{0.0, 0.0};
                    double xx = x_m[0] * x_m[0];
                    double yy = x_m[1] * x_m[1];
                    double r2 = xx + yy;
                    double xy = 2 * x_m[0] * x_m[1];
                    double distortion = 1.0 + r2 * (_dist1[0] + _dist1[1] * r2 + _dist1[4] * r2 * r2);
                    pc[0] = x_m[0] * distortion + _dist1[2] *         xy  + _dist1[3] * (r2 + 2*xx);
                    pc[1] = x_m[1] * distortion + _dist1[2] * (r2 + 2*yy) + _dist1[3] *         xy ;
                    Eigen::Vector2d delta = x_d - pc;
                    // std::cout<<" itr = "<<itr<<" delta = "<<delta.transpose()<<std::endl;
                    x_m += delta;
                    if(delta[0]< 1e-8 && delta[1]<1e-8){
                        isFound = true;
                        break;
                    }
                }
                if(!isFound) std::cout<<" something is wrong about inverse Distortion. "<<std::endl;
                Eigen::Vector3d pc{x_m[0], x_m[1], 1.0};
                
                // from event camera axis to frame camera axis
                Eigen::Matrix4d T_es = T0.matrix();
                Eigen::Vector3d T_R2_es{T_es(2,0), T_es(2,1), T_es(2,2)};
                double depth_es = - T_es(2,3) / (T_R2_es.dot(pc));

                Eigen::Vector3d pc2 = T1 * (depth_es * pc);
                
                // from frame camera axis to frame pixel plane
                pc2 /= pc2[2];
                Eigen::Vector3d pc_fr = pc2;
                double xx_fr = pc_fr[0] * pc_fr[0];
                double yy_fr = pc_fr[1] * pc_fr[1];
                double r2_fr = xx_fr + yy_fr;
                double xy_fr = 2 * pc_fr[0] * pc_fr[1];
                double distortion2 = 1.0 + r2_fr * (_dist2[0] + _dist2[1] * r2_fr + _dist2[4] * r2_fr * r2_fr);
                pc_fr[0] = pc_fr[0] * distortion2 + _dist2[2] *            xy_fr  + _dist2[3] * (r2_fr + 2*xx_fr);
                pc_fr[1] = pc_fr[1] * distortion2 + _dist2[2] * (r2_fr + 2*yy_fr) + _dist2[3] *            xy_fr ;
                pc_fr = _K2 * pc_fr;

                _error = _measurement - pc_fr.head<2>();

            } else {
                // obtain the Normalized _pos3d 
                // from _pos2d at the pixel plane of the frame camera 
                double x_dis = (_pos2d[0] - _K2(0, 2)) /  _K2(0, 0);
                double y_dis = (_pos2d[1] - _K2(1, 2)) /  _K2(1, 1);
                Eigen::Vector2d x_d(x_dis, y_dis);
                Eigen::Vector2d x_m = x_d;
                bool isFound = false;
                for(int itr=0; itr< 20; itr++){
                    Eigen::Vector2d pc{0.0, 0.0};
                    double xx = x_m[0] * x_m[0];
                    double yy = x_m[1] * x_m[1];
                    double r2 = xx + yy;
                    double xy = 2 * x_m[0] * x_m[1];
                    double distortion = 1.0 + r2 * (_dist2[0] + _dist2[1] * r2 + _dist2[4] * r2 * r2);
                    pc[0] = x_m[0] * distortion + _dist2[2] *         xy  + _dist2[3] * (r2 + 2*xx);
                    pc[1] = x_m[1] * distortion + _dist2[2] * (r2 + 2*yy) + _dist2[3] *         xy ;
                    Eigen::Vector2d delta = x_d - pc;
                    // std::cout<<" itr = "<<itr<<" delta = "<<delta.transpose()<<std::endl;
                    x_m += delta;
                    if(delta[0]< 1e-8 && delta[1]<1e-8){
                        isFound = true;
                        break;
                    }
                }
                if(!isFound) std::cout<<" something is wrong about inverse Distortion. "<<std::endl;
                Eigen::Vector3d pc{x_m[0], x_m[1], 1.0};
                
                // from frame camera axis to event camera axis
                Eigen::Matrix4d T_fr = T0.matrix();
                Eigen::Vector3d T_R2_fr{T_fr(2,0), T_fr(2,1), T_fr(2,2)};
                double depth_fr = - T_fr(2,3) / (T_R2_fr.dot(pc));

                Eigen::Matrix4d T1_Rt = T1.matrix();
                Eigen::Matrix3d T1_R = T1_Rt.block<3, 3>(0, 0);
                Eigen::Vector3d T1_t = T1_Rt.block<3, 1>(0, 3);
                Eigen::Quaterniond T1_q(T1_R);
                T1_q.normalize();
                Eigen::Vector3d pc2 = T1_q.conjugate() * (depth_fr * pc - T1_t); 

                // from frame camera axis to event pixel plane
                pc2 /= pc2[2];
                Eigen::Vector3d pc_es = pc2;
                double xx_es = pc_es[0] * pc_es[0];
                double yy_es = pc_es[1] * pc_es[1];
                double r2_es = xx_es + yy_es;
                double xy_es = 2 * pc_es[0] * pc_es[1];
                double distortion1 = 1.0 + r2_es * (_dist1[0] + _dist1[1] * r2_es + _dist1[4] * r2_es * r2_es);
                pc_es[0] = pc_es[0] * distortion1 + _dist1[2] *            xy_es  + _dist1[3] * (r2_es + 2*xx_es);
                pc_es[1] = pc_es[1] * distortion1 + _dist1[2] * (r2_es + 2*yy_es) + _dist1[3] *            xy_es ;
                pc_es = _K1 * pc_es;

                _error = _measurement - pc_es.head<2>();
            }
        }

        virtual bool read( istream &in) {return false;}
        virtual bool write( ostream &out) const {return false;}

    private:
        Eigen::Vector2d _pos2d;
        Eigen::Matrix3d _K1;    //event
        Eigen::VectorXd _dist1; //event
        Eigen::Matrix3d _K2;    //frame
        Eigen::VectorXd _dist2; //frame 
        bool _isForward;
    };

    class SolveCamToCam{
    public:
        typedef std::shared_ptr<SolveCamToCam> Ptr;

        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType; // pose is 6, landmark is 3
		typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

        // BA by g2o
        typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecEigen2d;
        typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecEigen3d;
        
        SolveCamToCam(Eigen::Matrix3d K_es, Eigen::MatrixXd Distortion_es, Eigen::Matrix3d K_fr, Eigen::MatrixXd Distortion_fr);

        void addVertex(Sophus::SE3d ini_es, Sophus::SE3d ini_fr);
        void addVertexAmongTwo(Sophus::SE3d ini);

        bool addEdge(VecEigen3d points_3d, VecEigen2d points_2d_es, VecEigen2d points_2d_fr, Eigen::Vector4d param);

        void optimize();

        int idx(){return v0_idx;};
        Sophus::SE3d pose_es(int i){return v0_es_vec[i]->estimate();};
        Sophus::SE3d pose_fr(int i){return v0_fr_vec[i]->estimate();};
        Sophus::SE3d CamToCam(){return v1->estimate();};

    protected:
        int v0_idx; // v0_es_vec.size()-1
        std::vector<PoseVertex*> v0_es_vec; // Vertex 1, 3, 5, 7, ......
        std::vector<PoseVertex*> v0_fr_vec; // Vertex 2, 4, 6, 8, ......
        bool v1_exist = false;
        PoseVertex* v1; // Vertex 0
        g2o::SparseOptimizer optimizer;

        Eigen::Matrix3d K_es_eigen;
        Eigen::VectorXd Distortion_es_eigen;
        Eigen::Matrix3d K_fr_eigen;
        Eigen::VectorXd Distortion_fr_eigen;
    }; // SolveCamToCam class


// ****  pose and camera Intrinsic Estimation *** //

/*
    // class PoseVertex: public g2o::BaseVertex<6, Sophus::SE3d>{};

    class IntrinsicVertex: public g2o::BaseVertex<4, Eigen::Matrix<double, 4, 1>>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        IntrinsicVertex() = default;

        virtual void setToOriginImpl() override { // reset
            _estimate << 100, 0, 0, 0;
        }

        virtual void oplusImpl( const double *update) override { // update
            Eigen::Matrix<double, 4, 1> update_eigen;
            update_eigen << update[0], update[1], update[2], update[3];
            _estimate += update_eigen;
        }

        virtual bool read( istream &in) {return false;}
        virtual bool write( ostream &out) const {return false;}
    };

    class PoseIntrinsicEdge: public g2o::BaseBinaryEdge<2, Eigen::Vector2d, PoseVertex, IntrinsicVertex>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        PoseIntrinsicEdge(const Eigen::Vector3d &pos, Eigen::Matrix3d &K)
                        : BaseBinaryEdge(), _pos3d(pos), _K(K) {}

        virtual void computeError() override {
            const PoseVertex *v0 = static_cast<PoseVertex *> (_vertices[0]);
            // auto v1 = (IntrinsicVertex *) _vertices[1];
            const IntrinsicVertex *v1 = static_cast<IntrinsicVertex *> (_vertices[1]);
            Sophus::SE3d T = v0->estimate();
            Eigen::Matrix<double, 4, 1> _Intrinsic = v1->estimate();

            Eigen::Vector3d pc = T * _pos3d;
            pc = pc / pc[2];
            // double r2 = pc.squaredNorm();
            double xx = pc[0] * pc[0];
            double yy = pc[1] * pc[1];
            double r2 = xx + yy;
            double xy = 2 * pc[0] * pc[1];
            double distortion = 1.0 + r2 * (_Intrinsic[1] + _Intrinsic[2] * r2 );
            pc[0] = pc[0] * distortion + _Intrinsic[3] *         xy  + 0 * (r2 + 2*xx);
            pc[1] = pc[1] * distortion + _Intrinsic[3] * (r2 + 2*yy) + 0 *         xy ;

            pc[0] = pc[0] * _Intrinsic[0] + _K(0, 2);
            pc[1] = pc[1] * _Intrinsic[0] + _K(1, 2);
            _error = _measurement - pc.head<2>();
        }

        virtual bool read( istream &in) {return false;}
        virtual bool write( ostream &out) const {return false;}

    private:
        Eigen::Vector3d _pos3d;
        Eigen::Matrix3d _K;
    };

    class SolvePnPIntrinsicG2O{
    public:
        typedef std::shared_ptr<SolvePnPIntrinsicG2O> Ptr;

        typedef g2o::BlockSolver<g2o::BlockSolverTraits<14, 3>> BlockSolverType; // pose is 6, K is 3, Distortion is 5; landmark is 3
		typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

        // BA by g2o
        typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecEigen2d;
        typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecEigen3d;
        
        SolvePnPIntrinsicG2O();

        void addVertex(Sophus::SE3d T);
        void addIntrinsicVertex(Eigen::Matrix<double, 4, 1> ini);

        bool addEdge(VecEigen3d points_3d, VecEigen2d points_2d, Eigen::Matrix3d K);

        void optimize();

        int idx(){return v0_idx;};

    protected:
        int v0_idx;
        std::vector<PoseVertex *> v0_vec;
        IntrinsicVertex *v1;
        bool v1_exist = false;
        g2o::SparseOptimizer optimizer;
    }; // SolvePnPIntrinsicG2O class

*/

// ****  pose PnP Pose Estimation and 3D points, with known camera Intrinsic *** //

/*
    class Point3DVertex: public g2o::BaseVertex<3, Eigen::Vector3d>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Point3DVertex(){};

        virtual void setToOriginImpl() override {
            _estimate = Eigen::Vector3d(0, 0, 0);
        }

        virtual void oplusImpl( const double *update) override {
            _estimate += Eigen::Vector3d(update[0], update[1], update[2]);
        }
        virtual bool read( istream &in) {return false;}
        virtual bool write( ostream &out) const {return false;}
    };

    class PosePoint3DEdge: public g2o::BaseBinaryEdge<3, Eigen::Vector3d, PoseVertex, Point3DVertex>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        virtual void computeError() override { 
            const PoseVertex *v0 = static_cast<PoseVertex *> (_vertices[0]);
            const Point3DVertex *v1 = static_cast<Point3DVertex *> (_vertices[1]);
            Sophus::SE3d T = v0->estimate();
            Eigen::Vector3d pw = v1->estimate();
            Eigen::Vector3d pc = T * pw;
            Eigen::Matrix4d T_Rt = T.matrix();
            Eigen::Vector3d T_Rota2{T_Rt(2,0), T_Rt(2,1), T_Rt(2,2)};
            double depth = - T_Rt(2,3) / (T_Rota2.dot(_measurement));
            std::cout<<T_Rt<<" "<<T_Rota2.transpose()<<" depth = "<<depth<<std::endl;
            _error = _measurement * depth - pc;
        }

        virtual bool read( istream &in) {return false;}
        virtual bool write( ostream &out) const {return false;}

    };

    class Point3DEdge: public g2o::BaseUnaryEdge<3, Eigen::Vector3d, Point3DVertex>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        virtual void computeError() override { 
            const Point3DVertex *v = static_cast<Point3DVertex *> (_vertices[0]);
            Eigen::Vector3d pw = v->estimate();
            _error = _measurement - pw;
        }
        virtual bool read( istream &in) {}
        virtual bool write( ostream &out) const {}
    };

    class PosePoint3DG2O{
    public:
        typedef std::shared_ptr<PosePoint3DG2O> Ptr;

        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType; // pose is 6; landmark is 3
		typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

        // BA by g2o
        typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecEigen3d;
        
        PosePoint3DG2O();

        void addPoseVertex(Sophus::SE3d T);
        void addPoint3DVertex(Eigen::Vector3d ini);

        bool addEdge(VecEigen3d pw_3d, VecEigen3d pc_3d);

        void optimize();

        int idx(){return v0_idx;};

    protected:
        int v0_idx;
        std::vector<PoseVertex *> v0_vec;
        int v1_idx;
        std::vector<Point3DVertex *> v1_vec;
        g2o::SparseOptimizer optimizer;
    }; // SolvePnPIntrinsicG2O class

*/


} // calib namespace
}

#endif //ef_calib_PoseCalib_G2O_HPP
