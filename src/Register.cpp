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
    float last_res = 0;

    // This is the traditional EM method 

    // for(int iter = 0; iter < maxiter; iter++){
    //     vMoments curMoment = RegEstep(clouds);
    //     float res = 0;
    //     Eigen::Matrix4f curPose = RegMstep(curMoment,res);
    //     mPose = curPose * mPose;

    //     if(res > 0 && abs(res-last_res) < log_th){
    //         break;
    //     }

    //     last_res = res;
    //     pcl::transformPointCloud(*clouds,*clouds,curPose);
    // }

    Eigen::Matrix4f T_st;
    bool initFlag = false;

    for(int iter = 0; iter < maxiter; iter++){
        vMoments curMoment;
        float res = 0;
        if(!initFlag){
            T_st.setIdentity();
            curMoment = StructAlignment(clouds,T_st);
            Eigen::Matrix3f R = T_st.block(0,0,3,3);
            Eigen::Quaternionf q(R);
            if(abs(q.w()) > 0.9995)
                initFlag = true;
        }else{
            curMoment = RegEstep(clouds);
        }

        if(!initFlag){
            mPose = T_st * mPose;
            pcl::transformPointCloud(*clouds,*clouds,T_st);
            curMoment = RegEstep(clouds);
        }
        
        Eigen::Matrix4f curPose = RegMstep(curMoment,res);
        mPose = curPose * mPose;

        if(res > 0 && abs(res-last_res) < log_th)
            break;

        last_res = res;

        pcl::transformPointCloud(*clouds,*clouds,curPose);
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
        #pragma omp critical
        {
            accumulate(moments[search_id], gamma[search_id - j0], pt);
        }
    }

    return moments;
}

vMoments Register::StructAlignment(pcl::PointCloud<pcl::PointXYZ>::Ptr tf_cloud,Eigen::Matrix4f &T_st){
    
    vMoments moments(n_total,Eigen::Vector4f::Zero());
    
    // TODO using std<set> to deal with "omp parallel for?"
    std::vector<std::vector<int> > vPlanePoints(plane_num);
    std::vector<Vector4f, Eigen::aligned_allocator<Vector4f> > vPlaneParams(plane_num, Vector4f(0.0,0.0,0.0,0.0));
    vector<int> vInlierNum(plane_num,0);
    Eigen::Vector4f centroid;			
    pcl::compute3DCentroid(*tf_cloud, centroid);	
    Eigen::Vector3f center = centroid.head(3);


    std::mutex mtx;
    #pragma omp parallel for
    for (size_t i = 0; i < tf_cloud->points.size(); ++i) {
        
        pcl::PointXYZ &pt3d = (tf_cloud->points)[i];
        Eigen::Vector3f pt(pt3d.x,pt3d.y,pt3d.z);

        int search_id = -1;
        Eigen::VectorXf gamma = Eigen::VectorXf::Zero(N_NODE);
        int j0 = 0;
        for (int l = 0; l < max_tree_level; ++l) {
            
            j0 = (search_id+1)*8;

            for (int j = j0; j < j0 + N_NODE; ++j) {
                gamma[j - j0] = mGMM[j]->mWeight * mGMM[j]->gaussian3d(pt);
            }

            const float den = gamma.sum();
            if (den > eps) {
                gamma /= den;   
            } else {
                gamma.fill(0.0);
            }
            gamma.maxCoeff(&search_id);
            search_id += j0;

            if(mGMM[search_id]->mbDegenerated){
                int plane_id = mvNodesFlag[search_id];
                if(plane_id<0)
                    break;
                
                std::lock_guard<std::mutex> guard(mtx); 
                vPlanePoints[plane_id].push_back(i);

                break;
            }     
        }
        #pragma omp critical
        {
            accumulate(moments[search_id], gamma[search_id - j0], pt);
        }
    }
    
    for(size_t i =0; i <vPlanePoints.size(); ++i){
        int n = vPlanePoints[i].size();
        if(n<100)    continue;
        
        std::vector<int> & ids = vPlanePoints[i];
        std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > vPointclouds;
        vPointclouds.reserve(n);

        for(size_t j = 0; j<n; j++){
            int k = ids[j];
            
            pcl::PointXYZ &pt3d = (tf_cloud->points)[k];
            Eigen::Vector3f pt(pt3d.x,pt3d.y,pt3d.z);

            vPointclouds.push_back(pt);
        }

        float cur_dist;
        Eigen::Vector3f cur_nor;
        int inlier_num = fit_plane(vPointclouds,cur_dist,cur_nor);

        vPlaneParams[i] << cur_nor, cur_dist;
        vInlierNum[i] = inlier_num;
    }

    Eigen::Matrix3f MF_c;
    MF_c.setZero();
    vector<int> vFlag_axis(3,1);
    int inlier_th = 500;    
    float th_parall = cos(40.0 * M_PI/180.0);
    float est_t[] = {0.0,0.0,0.0};

    int idInterval[] = {0,1,2,3,4,5}; // euroc

    for(size_t i =0; i<3; ++i){

        int start = idInterval[2*i];
        int end = idInterval[2*i+1];
        
        bool bOK = false;
        int max_id = start, max_inlier(0);

        for(int j = start; j <= end ; ++j){
            Vector4f p1 = vPlaneParams[j];
            if(vInlierNum[j] > inlier_th && abs(p1.head(3).dot(mMF0.col(i))) > th_parall){
                
                // std::cout<<"plane id is "<<j<<" estimated normal is "<<p1.head(3).transpose()<<
                // " and dist is "<<p1[3]<<" inliers are "<<vInlierNum[j]<<std::endl;

                if(vInlierNum[j] >= max_inlier){
                    max_inlier = vInlierNum[j];
                    max_id = j;
                }

                bOK = true;
            }            
        }

        if(!bOK) {
            vFlag_axis[i] = -1;
            continue;
        }   
        
        Vector4f & p1 = vPlaneParams[max_id];
        Vector4f & p1_ = mvPlanesPiror[max_id];

        if( p1.head(3).dot(p1_.head(3)) >  -p1.head(3).dot(p1_.head(3)) ){
            MF_c.col(i) = p1.head(3);
        }
        else{
            MF_c.col(i) = - p1.head(3);
        }  
    }

    Eigen::Matrix3f R_st;
    Eigen::Vector3f t_st;
    R_st.setIdentity();
    t_st.setZero();

    T_st.block(0,0,3,3) = R_st;
    T_st.block(0,3,3,1) = t_st;

    // check if manhattan frame is build
    int cur_sum = vFlag_axis[0] + vFlag_axis[1] + vFlag_axis[2];

    if(cur_sum < 0){
        return moments;
    }

    else if(cur_sum == 1){
        
        int j = 0;
        for(size_t i=0;i<3;++i){
            if(vFlag_axis[i]<0){
                j = i;
                break;
            }
        }

        if(j==0)    MF_c.col(j) = MF_c.col(1).cross(MF_c.col(2));
        else if(j==1)    MF_c.col(j) = MF_c.col(2).cross(MF_c.col(0));
        else if(j==2)    MF_c.col(j) = MF_c.col(0).cross(MF_c.col(1));

        t_st = center - mMF0 * MF_c.transpose() * center;
        T_st.block(0,3,3,1) = t_st;
    }
    
    else{
        
        vector<int> cnts;

        for(size_t i =0; i< vPlaneParams.size(); ++i){
            if(vInlierNum[i]>=inlier_th)
                cnts.push_back(i);
        }
        int inliner_size = cnts.size();

        Eigen::MatrixXf A(Eigen::MatrixXf::Zero(inliner_size, 3));
        Eigen::VectorXf b(Eigen::VectorXf::Ones(inliner_size));

        for(size_t i =0; i < inliner_size; ++i){
            Vector4f & p1 = vPlaneParams[cnts[i]];
            Vector4f & p1_ = mvPlanesPiror[cnts[i]];

            A.row(i) << p1_(0), p1_(1), p1_(2);
            if( p1.head(3).dot(p1_.head(3)) > -p1.head(3).dot(p1_.head(3)) )
                b[i] = p1(3) - p1_(3);
            else
                b[i] = -p1(3) - p1_(3);
        }

        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> x_jacobiSvd;
        x_jacobiSvd = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
        float tx,ty,tz;
        tx = x_jacobiSvd(0,0); ty = x_jacobiSvd(1,0); tz = x_jacobiSvd(2,0);

        t_st << tx,ty,tz;
        // cout<<"estimated translation is "<<t_st.transpose()<<endl;
        T_st.block(0,3,3,1) = t_st;
    }
    
    Eigen::JacobiSVD<Matrix3f> svd(MF_c, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Matrix3f U = svd.matrixU() , V = svd.matrixV();
    MF_c = U * V.transpose();
    // cout<<"estimated rotation is \n"<<MF_c<<endl;

    R_st = mMF0 * MF_c.transpose();
    T_st.block(0,0,3,3) = R_st;
    
    return moments;
}

Eigen::Matrix4f Register::RegMstep(vMoments &moments, float& res_){
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


    Eigen::VectorXf r = Amat*x_jacobiSvd.block(0,0,6,1)-bmat;
    res_ = r.dot(r);
    
    Eigen::Matrix3f R = Pos.block(0,0,3,3);
    JacobiSVD<Matrix3f> svd(R, ComputeFullU | ComputeFullV);
    Matrix3f U = svd.matrixU() , V = svd.matrixV();
    R = U * V.transpose();
    Pos.block(0,0,3,3) = R;

    return Pos;
}