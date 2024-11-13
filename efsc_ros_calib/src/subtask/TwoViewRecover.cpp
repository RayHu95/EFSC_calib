#include <calib_node.hpp>

// #include <sophus/se3.hpp>

// #include <Eigen/Core>

namespace ef_calib_node {

	void Task::TranslationBetween(cv::Mat &rvec, cv::Mat &tvec, cv::Mat &rvec2, cv::Mat &tvec2, Eigen::Vector3d &tef, Eigen::Matrix3d &Ref, Eigen::Quaterniond &Qef){
		// frame R
		cv::Mat cvRfw;
		cv::Rodrigues(rvec, cvRfw);
		Eigen::Matrix3d Rfw;
		cv::cv2eigen(cvRfw, Rfw);
		// Eigen::Quaterniond Qfw(Rfw);
		// Qfw.normalize();
		// frame t
		Eigen::Vector3d tfw;
		cv::cv2eigen(tvec, tfw);

		// event R
		cv::Mat cvRew;
		cv::Rodrigues(rvec2, cvRew);
		Eigen::Matrix3d Rew;
		cv::cv2eigen(cvRew, Rew);
		// Eigen::Quaterniond Qew(Rew);
		// Qew.normalize();
		// event t
		Eigen::Vector3d tew;
		cv::cv2eigen(tvec, tew);

		// ef
		// Eigen::Matrix3d Ref;
		Ref = Rfw*Rew; //Rfw.dot(Rew.transpose());
		Eigen::Quaterniond Qef2(Ref);
		Qef2.normalize();
		Qef = Qef2;
		
		Eigen::Vector3d tef2 = Ref*tew;//Ref.dot(tew);
		tef = tfw - tef2;
	}

    void DecomposeE(const Eigen::Matrix3f &E, Eigen::Matrix3f &R1, Eigen::Matrix3f &R2, Eigen::Vector3f &t)
    {

        Eigen::JacobiSVD<Eigen::Matrix3f> svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);

        Eigen::Matrix3f U = svd.matrixU();
        Eigen::Matrix3f Vt = svd.matrixV().transpose();

        t = U.col(2);
        t = t / t.norm();

        Eigen::Matrix3f W;
        W.setZero();
        W(0,1) = -1;
        W(1,0) = 1;
        W(2,2) = 1;

        R1 = U * W * Vt;
        if(R1.determinant() < 0)
            R1 = -R1;

        R2 = U * W.transpose() * Vt;
        if(R2.determinant() < 0)
            R2 = -R2;
    }

    bool Triangulate(Eigen::Vector3f &x_c1, Eigen::Vector3f &x_c2,Eigen::Matrix<float,3,4> &Tc1w ,Eigen::Matrix<float,3,4> &Tc2w , Eigen::Vector3f &x3D)
    {
        Eigen::Matrix4f A;
        A.block<1,4>(0,0) = x_c1(0) * Tc1w.block<1,4>(2,0) - Tc1w.block<1,4>(0,0);
        A.block<1,4>(1,0) = x_c1(1) * Tc1w.block<1,4>(2,0) - Tc1w.block<1,4>(1,0);
        A.block<1,4>(2,0) = x_c2(0) * Tc2w.block<1,4>(2,0) - Tc2w.block<1,4>(0,0);
        A.block<1,4>(3,0) = x_c2(1) * Tc2w.block<1,4>(2,0) - Tc2w.block<1,4>(1,0);

        Eigen::JacobiSVD<Eigen::Matrix4f> svd(A, Eigen::ComputeFullV);

        Eigen::Vector4f x3Dh = svd.matrixV().col(3);

        if(x3Dh(3)==0)
            return false;

        // Euclidean coordinates
        x3D = x3Dh.head(3)/x3Dh(3);

        return true;
    }

    int CheckRT(const Eigen::Ref<Eigen::Matrix3f> &R, const Eigen::Ref<Eigen::Vector3f> &t, const std::vector<cv::Point2f> &vP1,const std::vector<cv::Point2f> &vP2,
                      std::vector<cv::Point3f> &vP3D, float th2, std::vector<bool> &vbGood, float &parallax)
    {

        vbGood = std::vector<bool>(vP1.size(),false);
        vP3D.resize(vP1.size());

        std::vector<float> vCosParallax;
        vCosParallax.reserve(vP1.size());

        // Camera 1 Projection Matrix K[I|0]
        Eigen::Matrix<float,3,4> P1;
        P1.setZero();
        P1.block<3,3>(0,0) = Eigen::MatrixXf::Identity(3, 3);
        // Camera 2 Projection Matrix K[R|t]
        Eigen::Matrix<float,3,4> P2;
        P2.block<3,3>(0,0) = R;
        P2.block<3,1>(0,3) = t;

        Eigen::Vector3f O1;
        O1.setZero();
        Eigen::Vector3f O2 = -R.transpose() * t;

        // std::cout<<"** ** O2 = -R.transpose() * t  " <<O2.transpose() <<std::endl<<std::endl;

        int nGood=0;

        for(size_t i=0, iend=vP1.size();i<iend;i++)
        {
            Eigen::Vector3f p3dC1;
            Eigen::Vector3f x_p1(vP1[i].x, vP1[i].y, 1);
            Eigen::Vector3f x_p2(vP2[i].x, vP2[i].y, 1);

        // std::cout<<"** ** Point: ("<<x_p1.transpose()<<"),  ("<<x_p2.transpose()<<").  "<<std::endl;
            Triangulate(x_p1, x_p2, P1, P2, p3dC1);
        // std::cout<<"** ** After Triangulate,  p3dC1 {"<<p3dC1.transpose()<<" } "<<std::endl;

            if(!isfinite(p3dC1(0)) || !isfinite(p3dC1(1)) || !isfinite(p3dC1(2)))
            {
                vbGood[i]=false;
                continue;
            }

            // Check parallax
            Eigen::Vector3f normal1 = p3dC1 - O1;
            float dist1 = normal1.norm();

            Eigen::Vector3f normal2 = p3dC1 - O2;
            float dist2 = normal2.norm();

            float cosParallax = normal1.dot(normal2) / (dist1*dist2);

        // std::cout<<"** ** dist "<<dist1<<" "<<dist2<<" cosParallax = "<<cosParallax<<std::endl;

            // Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)
            if(p3dC1(2)<=0.0 && cosParallax<0.99998)
                continue;

            // Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
            Eigen::Vector3f p3dC2 = R * p3dC1 + t;
        // std::cout<<"** ** p3dC2 = R * p3dC1 + t; {"<<p3dC2.transpose()<<"} "<<std::endl;    

            if(p3dC2(2)<=0.0 && cosParallax<0.99998)
                continue;

        // std::cout<<"** ** only depth > 0 || cosParallax > 0.99998, and depth = "<<p3dC1(2)<<" "<<p3dC2(2)<<std::endl;

            // Check reprojection error in first image
            float im1x, im1y;
            float invZ1 = 1.0/p3dC1(2);
            im1x = p3dC1(0)*invZ1; //fx*p3dC1(0)*invZ1+cx;
            im1y = p3dC1(1)*invZ1; //fy*p3dC1(1)*invZ1+cy;
            float squareError1 = (im1x-vP1[i].x)*(im1x-vP1[i].x)+(im1y-vP1[i].y)*(im1y-vP1[i].y);
            if(squareError1>th2)
                continue;

            // Check reprojection error in second image
            float im2x, im2y;
            float invZ2 = 1.0/p3dC2(2);
            im2x = p3dC2(0)*invZ2;//fx*p3dC2(0)*invZ2+cx;
            im2y = p3dC2(1)*invZ2;//fy*p3dC2(1)*invZ2+cy;
            float squareError2 = (im2x-vP2[i].x)*(im2x-vP2[i].x)+(im2y-vP2[i].y)*(im2y-vP2[i].y);
            if(squareError2>th2)
                continue;
           
        // std::cout<<"** ** squareError < th2 : "<<squareError1<<", "<<squareError2<<" < "<<th2<<std::endl;
            
            vCosParallax.push_back(cosParallax);
            vP3D[i] = cv::Point3f(p3dC1(0), p3dC1(1), p3dC1(2));
            nGood++;

            if(cosParallax<0.99998)
                vbGood[i]=true;

        // std::cout<<std::endl;
        }

        if(nGood>0)
        {
            sort(vCosParallax.begin(),vCosParallax.end());

            size_t idx = std::min(50,int(vCosParallax.size()-1));
            parallax = acos(vCosParallax[idx])*180/CV_PI;
        }
        else
            parallax=0;

        return nGood;
    }
    int CheckFundamental(const Eigen::Matrix3f &F21, std::vector<cv::Point2f> &p1, std::vector<cv::Point2f> &p2)
    {
        const float f11 = F21(0,0);
        const float f12 = F21(0,1);
        const float f13 = F21(0,2);
        const float f21 = F21(1,0);
        const float f22 = F21(1,1);
        const float f23 = F21(1,2);
        const float f31 = F21(2,0);
        const float f32 = F21(2,1);
        const float f33 = F21(2,2);
        float num_sum = 0.0;
        for(int i=0; i<p1.size(); i++)
        {
            const float u1 = p1[i].x;
            const float v1 = p1[i].y;
            const float u2 = p2[i].x;
            const float v2 = p2[i].y;
            // Reprojection error in second image
            // l2=F21x1=(a2,b2,c2)
            float a2 = f11*u1+f12*v1+f13;
            float b2 = f21*u1+f22*v1+f23;
            float c2 = f31*u1+f32*v1+f33;

            float num2 = a2*u2+b2*v2+c2;
            num_sum += std::abs(num2);
            // std::cout<<" **  num = "<<num2<<std::endl;
        }
        // std::cout<<" **  num_sum = "<<num_sum / p1.size()<<std::endl;
        return num_sum/p1.size();
    }
    bool Task::FindFundamental(const std::vector<cv::Point2f> &vPn1i,const std::vector<cv::Point2f> &vPn2i, 
                                ::ef_calib::calib::CameraBase::Ptr camera1, ::ef_calib::calib::CameraBase::Ptr camera2, 
                                Eigen::Matrix3f &F21, Eigen::Matrix3f &R21, Eigen::Vector3f &t21){

        const int N = vPn1i.size();

        std::vector<cv::Point2f> imageP_pp1;
        std::vector<cv::Point2f> imageP_pp2;
        Eigen::MatrixXf A = Eigen::MatrixXf::Zero(N, 9);
        for(int i=0; i<N; i++ ){
            cv::Point2f p1 = vPn1i[i];
            Eigen::Vector3d pp1 = camera1->pixel2cam(p1);
            cv::Point2f p2 = vPn2i[i];
            Eigen::Vector3d pp2 = camera2->pixel2cam(p2);
            imageP_pp1.push_back(cv::Point2f(pp1(0), pp1(1)));
            imageP_pp2.push_back(cv::Point2f(pp2(0), pp2(1)));
            std::cout<<"** data pp1: ( "<<pp1.transpose()<<" ).     pp2: ( "<<pp2.transpose()<<" )"<<std::endl;

            A(i, 0) = pp2(0) * pp1(0);
            A(i, 1) = pp2(0) * pp1(1);
            A(i, 2) = pp2(0);
            A(i, 3) = pp2(1) * pp1(0);
            A(i, 4) = pp2(1) * pp1(1);
            A(i, 5) = pp2(1);
            A(i, 6) =         pp1(0);
            A(i, 7) =         pp1(1);
            A(i, 8) =            1.0;
        }

        Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix<float,3,3,Eigen::RowMajor> Fpre(svd.matrixV().col(8).data());
        Eigen::JacobiSVD<Eigen::Matrix3f> svd2(Fpre, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Vector3f w = svd2.singularValues();
        w(2) = 0;
        F21 = svd2.matrixU() * Eigen::DiagonalMatrix<float,3>(w) * svd2.matrixV().transpose();

        float Avg_err = CheckFundamental(F21, imageP_pp1, imageP_pp2);
        if( Avg_err  > 0.0005)
            return false;

        Eigen::Matrix3f R1, R2;
        Eigen::Vector3f t;
        // Recover the 4 motion hypotheses
        DecomposeE(F21,R1,R2,t);
        Eigen::Vector3f t1 = t;
        Eigen::Vector3f t2 = -t;

        std::cout<<std::endl<<"** RECOVER THE POSE	: "<<std::endl
                            <<" t = "<<std::endl<<t<<std::endl
                            <<" R1 = "<<std::endl<<R1<<std::endl
                            <<" R2 = "<<std::endl<<R2<<std::endl;


        // Reconstruct with the 4 hyphoteses and check
        std::vector<cv::Point3f> vP3D1, vP3D2, vP3D3, vP3D4;
        std::vector<bool> vbTriangulated1,vbTriangulated2,vbTriangulated3, vbTriangulated4;
        float parallax1,parallax2, parallax3, parallax4;

        int nGood1 = CheckRT(R1,t1, imageP_pp1, imageP_pp2, vP3D1, 4.0*0.00001, vbTriangulated1, parallax1);
        std::cout<<"** R1,t1 nGood1 = "<<nGood1<<", parallax1 = "<<parallax1<<std::endl;
        int nGood2 = CheckRT(R2,t1, imageP_pp1, imageP_pp2, vP3D2, 4.0*0.00001, vbTriangulated2, parallax2);
        std::cout<<"** R2,t1 nGood2 = "<<nGood2<<", parallax2 = "<<parallax2<<std::endl;
        int nGood3 = CheckRT(R1,t2, imageP_pp1, imageP_pp2, vP3D3, 4.0*0.00001, vbTriangulated3, parallax3);
        std::cout<<"** R1,t2 nGood3 = "<<nGood3<<", parallax3 = "<<parallax3<<std::endl;
        int nGood4 = CheckRT(R2,t2, imageP_pp1, imageP_pp2, vP3D4, 4.0*0.00001, vbTriangulated4, parallax4);
        std::cout<<"** R2,t2 nGood4 = "<<nGood4<<", parallax4 = "<<parallax4<<std::endl;

        std::cout<<"** nGood = "<<nGood1<<" "<<nGood2<<" "<<nGood3<<" "<<nGood4<<" "<<std::endl;

        int maxGood = std::max(nGood1,std::max(nGood2,std::max(nGood3,nGood4)));
        int nMinGood = std::max(static_cast<int>(0.9*N), 10 );//minTriangulated = 50

        // std::cout<<" maxGood = "<<maxGood<<", nMinGood = "<<nMinGood<<std::endl;

        int nsimilar = 0;
        if(nGood1>0.7*maxGood)
            nsimilar++;
        if(nGood2>0.7*maxGood)
            nsimilar++;
        if(nGood3>0.7*maxGood)
            nsimilar++;
        if(nGood4>0.7*maxGood)
            nsimilar++;
        // If there is not a clear winner or not enough triangulated points reject initialization
        if(maxGood<nMinGood || nsimilar>1)
        {
            return false;
        }

        float minParallax = 0.01;
        // std::vector<cv::Point3f> vP3D;
        // std::vector<bool> vbTriangulated;
        // Sophus::SE3f T21;
        // If best reconstruction has enough parallax initialize
        if(maxGood==nGood1)
        {
            if(parallax1>minParallax)
            {
                // vP3D = vP3D1;
                // vbTriangulated = vbTriangulated1;

                // T21 = Sophus::SE3f(R1, t1);
                R21 = R1;
                t21 = t1;
                return true;
            }
        }else if(maxGood==nGood2)
        {
            if(parallax2>minParallax)
            {
                // vP3D = vP3D2;
                // vbTriangulated = vbTriangulated2;

                // T21 = Sophus::SE3f(R2, t1);
                R21 = R2;
                t21 = t1;
                return true;
            }
        }else if(maxGood==nGood3)
        {
            if(parallax3>minParallax)
            {
                // vP3D = vP3D3;
                // vbTriangulated = vbTriangulated3;

                // T21 = Sophus::SE3f(R1, t2);
                R21 = R1;
                t21 = t2;
                return true;
            }
        }else if(maxGood==nGood4)
        {
            if(parallax4>minParallax)
            {
                // vP3D = vP3D4;
                // vbTriangulated = vbTriangulated4;

                // T21 = Sophus::SE3f(R2, t2);
                R21 = R2;
                t21 = t2;
                return true;
            }
        }

        return false;
    }
    
} //Namespace ef_calib_node
