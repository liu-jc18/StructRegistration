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
            
            int id;
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
                ss >> id;

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
