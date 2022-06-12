#include "Register.h"

using namespace std;
using namespace Eigen;

Register::Register(const std::string& GMM_path,const std::string& PlaneFlag_path,const std::string& Planes_path){
    
    mGMM = loadGMMModel(GMM_path);
    readplaneparam(Planes_path,mvPlanesPiror,mMF0);
    readNodesFlag(PlaneFlag_path,mvNodesFlag);

    n_total = N_NODE * (1 - std::pow(N_NODE, max_tree_level)) / (1 - N_NODE);
    if(mGMM.size()!=n_total){
        std::cerr << "WRONG : num of GMM is different from n_total!"<< std::endl;
    }

    plane_num = mvPlanesPiror.size();

    mPose = Eigen::Matrix4f::Identity();
}

void Register::accumulate(Eigen::Vector4f &moments, float gamma, const Eigen::Vector3f& z){
    moments[0] += gamma;
    moments.tail(3) += gamma * z;
    // std::get<2>(moments) += gamma * z * z.transpose();
}

void Register::SetInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud){
    mSource_cloud = source_cloud;
}

void Register::run(){

    pcl::PointCloud<pcl::PointXYZ>::Ptr clouds(new pcl::PointCloud<pcl::PointXYZ>());
    
    pcl::transformPointCloud(*mSource_cloud,*clouds,mPose);

    for(int iter = 0; iter < maxiter; iter++){
        
        vMoments curMoment = RegEstep(clouds);
        Eigen::Matrix4f curPose = RegMstep(curMoment);
        mPose = curPose * mPose;
        pcl::PointCloud<pcl::PointXYZ>::Ptr tf_clouds(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*clouds,*tf_clouds,curPose);
        clouds = tf_clouds;
        // cout<<curPose<<endl;
    }
    cout<<mPose<<endl;

}

vMoments Register::RegEstep(pcl::PointCloud<pcl::PointXYZ>::Ptr tf_cloud){

    vMoments moments(n_total,Eigen::Vector4f::Zero());

    #pragma omp parallel for
    for (size_t i = 0; i < tf_cloud->points.size(); ++i) {
        
        pcl::PointXYZ &pt3d = (tf_cloud->points)[i];
        Eigen::Vector3f pt(pt3d.x,pt3d.y,pt3d.z);

        int search_id = -1;
        Eigen::VectorXf gamma = Eigen::VectorXf::Zero(N_NODE);
        int j0 = 0;
        for (int l = 0; l < max_tree_level; ++l) {
            
            // 计算子节点开始的序号 （search_id+1）*8
            j0 = (search_id+1)*8;

            for (int j = j0; j < j0 + N_NODE; ++j) {
                gamma[j - j0] = mGMM[j]->mWeight * mGMM[j]->gaussian3d(pt);
            }

            const float den = gamma.sum();
            if (den > eps) {
                gamma /= den;   //归一化
            } else {
                gamma.fill(0.0);
            }
            gamma.maxCoeff(&search_id);
            search_id += j0;

            if(mGMM[search_id]->mbDegenerated)     break;
        }
        // 指定一块同一时间只能被一条线程执行的代码区域，作用是为了避免多线程的数据冲突？
        #pragma omp critical
        {
            // 依据当前观测点的响应，计算 序号为 search_id 的 moment，在函数里面会累加
            accumulate(moments[search_id], gamma[search_id - j0], pt);
        }
    }

    return moments;
}

vMoments Register::RegEstepStruct(pcl::PointCloud<pcl::PointXYZ>::Ptr tf_cloud){

}

Eigen::Matrix4f Register::RegMstep(vMoments &moments){
    Eigen::Matrix4f Pos;
    Pos.setIdentity();
    Eigen::MatrixXf Amat(Eigen::MatrixXf::Zero(3*n_total,6));
    Eigen::VectorXf bmat(Eigen::VectorXf::Zero(3*n_total));
    
    for(int i=0; i<n_total; i++){
        GaussianComponent* pGMM = mGMM[i];
        
        float m0 = moments[i][0];
        if(m0 < eps)
            continue;
        
        Eigen::Vector3f s = moments[i].tail(3) / m0;

        Eigen::Matrix3f nn;
        nn.col(0) = pGMM->mVectors.col(0) * sqrt(m0 / pGMM->mValues[0]);
        nn.col(1) = pGMM->mVectors.col(1) * sqrt(m0 / pGMM->mValues[1]);
        nn.col(2) = pGMM->mVectors.col(2) * sqrt(m0 / pGMM->mValues[2]);

        Eigen::Matrix3f A0;
        A0.col(0) = s.cross(nn.col(0));
        A0.col(1) = s.cross(nn.col(1));
        A0.col(2) = s.cross(nn.col(2));

        Eigen::Vector3f bi = nn.transpose() * (pGMM->mMean-s);
        
        Amat.block(3*i,0,3,3) = A0.transpose();
        Amat.block(3*i,3,3,3) = nn.transpose();
        bmat.segment(3*i,3) = bi;
    }

    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> x_jacobiSvd;
    x_jacobiSvd = Amat.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(bmat);
    float phi_x,phi_y,phi_z,tx,ty,tz;
    phi_x = x_jacobiSvd(0,0); phi_y = x_jacobiSvd(1,0); phi_z = x_jacobiSvd(2,0);
    tx = x_jacobiSvd(3,0); ty = x_jacobiSvd(4,0); tz = x_jacobiSvd(5,0);

    Pos << 1.0, -phi_z, phi_y,tx,
            phi_z,1.0,-phi_x,ty,
            -phi_y,phi_x,1.0,tz,
            0.0,0.0,0.0,1.0;

    //TODO 把Pos给正交化
    return Pos;
}