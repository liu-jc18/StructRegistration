#include "utils.h"

vector<GaussianComponent*> loadGMMModel(const std::string &filename){

    std::vector<GaussianComponent*> vpComps;
    
    std::ifstream file_gmm;
    file_gmm.open(filename.c_str());
    if(!file_gmm.is_open())
    {
        cerr<<"Could not open gmm model \n";
        return vpComps;
    }

    while(!file_gmm.eof())
    {
        std::string s;
        std::getline(file_gmm,s);
        
        float w;
        Vector3f mean;
        Matrix3f cov;

        if(!s.empty())
        {
            std::stringstream ss;
            ss << s;
            
            ss >> w;
            
            float tx,ty,tz;
            ss >> tx;
            ss >> ty;
            ss >> tz;
            mean<<tx,ty,tz;

            float c1,c2,c3,c4,c5,c6,c7,c8,c9;
            ss >> c1;
            ss >> c2;
            ss >> c3;
            ss >> c4;
            ss >> c5;
            ss >> c6;
            ss >> c7;
            ss >> c8;
            ss >> c9;
            cov<<c1,c2,c3,c4,c5,c6,c7,c8,c9;

            vpComps.push_back(new GaussianComponent(w,mean,cov));
        }
    }

    cout<<"load gmm model is done,  load  "<< vpComps.size()<<" model in total ! \n";

    return vpComps;
}

void readNodesFlag(const std::string& filename, std::vector<int>& vflags){
    std::ifstream file;
    file.open(filename.c_str());
    if(!file.is_open()){
        std::cerr<<"could not open file ! \n";
    }

    while(!file.eof()){
        std::string s;
        std::getline(file,s);

        if(!s.empty()){
            
            int id;
            
            std::stringstream ss;
            ss << s;
            ss >> id;
            vflags.push_back(id);
            
        }
    }

    std::cout << "load GMM nodes flag is done , load "<<vflags.size()<<" list in total \n";
}

void readplaneparam(const std::string& filename, 
                        std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > &vPlanes
                        , Eigen::Matrix3f& MF0)
{
    std::ifstream file;
    file.open(filename.c_str());
    if(!file.is_open()){
        std::cerr<<"could not open file ! \n";
    }

    // the first row is the Quaternionf of MF0
    bool readMF0 = false;

    while(!file.eof()){
        std::string s;
        std::getline(file,s);

        if(!s.empty()){
            
            float x,y,z,d,w;
            
            std::stringstream ss;
            ss << s;

            if(!readMF0){
                ss >> x;
                ss >> y;
                ss >> z;
                ss >> w;
                Eigen::Quaternionf q;
                q.x() = x;
                q.y() = y;
                q.z() = z;
                q.w() = w;
                MF0 = q.toRotationMatrix();
                readMF0 = true;
            }else{
                ss >> x;
                ss >> y;
                ss >> z;
                ss >> d;
                Eigen::Vector4f pl;
                pl << x,y,z,d;

                vPlanes.push_back(pl);
            }

        }
    }

    std::cout << "load piror plane param is done , load "<<vPlanes.size()<<" planes in total \n";
}


void LoadPoseGroundtruth(string str_pose, map<int,vector<float> > & pose ){
    ifstream file;
    file.open(str_pose.c_str());
    int row = 0;

    while(!file.eof())
    {   
        string s;
        getline(file,s);


        if( !s.empty() )
        {
            stringstream ss;
            ss << s;
            float t,tx,ty,tz,qx,qy,qz,qw;
            ss >> t >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
            float cur_p[] = {tx,ty,tz,qx,qy,qz,qw};
            vector<float> v_p;
            for(int i=0;i<7;i++){
                v_p.push_back(cur_p[i]);
            }
            pose.insert(make_pair(row,v_p));
        }
        row ++;
    }
}

void LoadPoseGroundtruth(string str_pose, Eigen::Matrix4f& pose ){
    ifstream file;
    file.open(str_pose.c_str());

    while(!file.eof())
    {   
        string s;
        getline(file,s);

        if( !s.empty() )
        {
            stringstream ss;
            ss << s;
            float r1,r2,r3,r4,r5,r6,r7,r8,r9,t1,t2,t3;
            ss >> r1>>r2>>r3>>r4>>r5>>r6>>r7>>r8>>r9>>t1>>t2>>t3;
            pose << r1,r2,r3,t1,
                    r4,r5,r6,t2,
                    r7,r8,r9,t3,
                    0.0,0.0,0.0,1.0;
        }
    }
}