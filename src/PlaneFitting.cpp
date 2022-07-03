#include "utils.h"
#include "PlaneFitting.h"

bool getSample(vector<int> set, vector<int> &sset)
{
    int i[3];
    if (set.size() > 3)
    {
        do
        {
            for (int n = 0; n < 3; n++)
                i[n] = int(uniformRandom() * (set.size() - 1));
        } while ( (i[1] == i[0]) || (i[1] == i[2]) || (i[2] == i[0]));
        for (int n = 0; n < 3; n++)
        {
            sset.push_back(i[n]);
        }
    }
    else
    {
        return false;
    }
    return true;
}

bool verify(std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > &pts){
    
    Eigen::Vector3f pt1 = pts[0];
    Eigen::Vector3f pt2 = pts[1];
    Eigen::Vector3f pt3 = pts[2];
    float angle_th = 0.85;
    Eigen::Vector3f p12, p13;
    p12 = pt1-pt2;
    p13 = pt1-pt3;
    
    p12.normalize();
    p13.normalize();
    float angle_err = p13.transpose() * p12; 
    if(angle_err > angle_th || angle_err < -angle_th)
        return false ;

    else 
        return true;
}

int fit_plane(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > &mvPointcloud,
                        float &dist_, Eigen::Vector3f &normal_ , bool do_leastsquare)
{

    double residual_error = 0.1;
    bool stop_loop = false;
    int maximum = 0; 

    std::vector<bool> inlierFlag(mvPointcloud.size(), false);
    std::vector<double> resids_(mvPointcloud.size(), 10);
    int sample_count = 0;
    int N = 500;

    srand((unsigned int)time(NULL)); 
    std::vector<int> ptsID;

    for (unsigned int i = 0; i < mvPointcloud.size(); i++)
    {ptsID.push_back(i);}

    while (N > sample_count && !stop_loop)
    {
        std::vector<bool> inlierstemp;
        std::vector<double> residualstemp;
        std::vector<int> ptss; 
        int inlier_count = 0;
        if (!getSample(ptsID, ptss)) 
        {
            stop_loop = true;
            continue;
        }

        std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > pt_sam;
        pt_sam.clear();

        pt_sam.push_back(mvPointcloud[ptss[0]]);
        pt_sam.push_back(mvPointcloud[ptss[1]]);
        pt_sam.push_back(mvPointcloud[ptss[2]]);

        if(!verify(pt_sam))
        {
            ++sample_count;
            continue;
        }

        // plane param is : ax + by + cz = -1;
        Eigen::Matrix3f A;
        Eigen::Vector3f b;
        b << -1,-1,-1;
        for(int i = 0 ; i<3; i++)
        {   A.row(i) = pt_sam[i].transpose();}

        normal_ = A.colPivHouseholderQr().solve(b);

        dist_ = 1.0 / normal_.norm();
        normal_.normalize();

        for(size_t i=0 ; i<mvPointcloud.size() ; i++)
        {
            float resid_ = abs( normal_.transpose() * mvPointcloud[i] + dist_ );
            
            residualstemp.push_back(resid_);
            inlierstemp.push_back(false);
            
            if(resid_ < residual_error)
            {
                ++inlier_count;
                inlierstemp[i] = true;
            }
        }

        if (inlier_count >= maximum)
        {
            maximum = inlier_count;
            resids_ = residualstemp;
            inlierFlag = inlierstemp;
        }        

        if (inlier_count == 0)
        {
            N = 500;
        }
        else
        {
            double epsilon = double(inlier_count) / (double)mvPointcloud.size(); 
            double p = 0.99; 
            double s = 3.0;
            N = int(log(1.0 - p) / log(1.0 - pow( epsilon , s) ) );
        }        
        ++sample_count;
    }

    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > Pts_inlier;
    int inliner_size = 0;
    for(size_t i=0 ; i<inlierFlag.size() ; i++)
    {
        if(inlierFlag[i])
        {
            inliner_size++;
            Pts_inlier.push_back(mvPointcloud[i]); 
        }    
    }

    if(do_leastsquare)
    {
        Eigen::MatrixXf A(Eigen::MatrixXf::Zero(inliner_size, 3));
        Eigen::VectorXf b(Eigen::VectorXf::Ones(inliner_size));

        b = -b;
        for(size_t i = 0 ; i<inliner_size; i++)
        {
            A.row(i) = Pts_inlier[i].transpose();
        }
        
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> x_jacobiSvd;
        x_jacobiSvd = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
        float tx,ty,tz;
        tx = x_jacobiSvd(0,0); ty = x_jacobiSvd(1,0); tz = x_jacobiSvd(2,0);
        normal_  << tx,ty,tz;
        dist_ = 1.0 / normal_.norm();
        normal_.normalize();
    }

    return inliner_size;
}