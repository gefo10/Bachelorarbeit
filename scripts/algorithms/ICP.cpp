#include "ICP.h"
#include <random>
#include <cmath>
#include "KdTree.h"
#include "Eigen/SVD"
#include "Eigen/Core"

void icp(std::vector<Point*>& sourceCloud,std::vector<Point*>& targetCloud,const int maxIterations)
{
    sourceCloud.erase(std::remove_if(sourceCloud.begin(),sourceCloud.end(), [](auto& x) { return x->x == -0 || x->y == -0 || x->z == -0 || (x->x == 0 && x->y == 0 && x->z ==0); }));
    targetCloud.erase(std::remove_if(targetCloud.begin(),targetCloud.end(), [](auto& x) { return x->x == -0 || x->y == -0 || x->z == -0 || (x->x == 0 && x->y == 0 && x->z ==0); }));
    //taArgetCloud.erase(std::remove_if(targetCloud.begin(),targetCloud.end(), [](auto& x) { return x.x == -0 || x.y == -0 || x.z == -0 || (x.x == 0 && x.y == 0 && x.z ==0); }));

    using namespace Eigen;
    Matrix3f rotation = Matrix3f::Identity();
    Vector3f translation = Vector3f::Zero();
    
    Point sourceMean = computeCloudMean(sourceCloud);
    Point targetMean = computeCloudMean(targetCloud);
   
    Vector3f sourceMean_eigen(sourceMean.x,sourceMean.y,sourceMean.z);
    Vector3f targetMean_eigen(targetMean.x,targetMean.y,targetMean.z);
     
    auto old_distance = (targetMean_eigen - sourceMean_eigen).norm();

    std::cout << "Source size: "<<sourceCloud.size() << " target:" <<targetCloud.size();
    std::cout << "sourceMean: " << sourceMean_eigen(0) << " " << sourceMean_eigen(1) << " " << sourceMean_eigen(2) << std::endl;
    std::cout << "targetMean: " << targetMean_eigen(0) << " " << targetMean_eigen(1) << " " << targetMean_eigen(2) << std::endl;
    std::cout << "START distance: " << old_distance << std::endl;

    KdTree* tree = new KdTree(targetCloud);
    
    //std::cout << "AFTER"  <<std::endl <<std::flush;
    size_t numPointsSource = sourceCloud.size();
    size_t numPointsTarget = targetCloud.size();

    const int numRandomSamples = 1000 % sourceCloud.size();

   Point p{0.f,0.f,0.f},x{0.f,0.f,0.f};
    
   // float cost = 1.0;
    std::srand(time(0));

    Point qs{0.f,0.f,0.f},qt{0.f,0.f,0.f};

    Matrix3f H = Matrix3f::Zero();
    //Matrix3f temp_H = Matrix3f::Zero();


    for(int iter =0; iter < max_iterations && abs(error) > eps; iter ++)
    {
        //reset parameters
        error =0.0;
        H = Matrix3f::Zero();

        sourceMean = computeCloudMean(sourceCloud);

        for (int i = 0; i < numRandomSamples; i++)
        {

            int randSample = std::rand() % sourceCloud.size();
            // sample the dynamic point cloud
            p = *sourceCloud[randSample];

            // get the closest point in the static point cloud
       //     std::cout << "BEFORE2"  <<std::endl <<std::flush;
            tree->search(p, x);
       //     std::cout << "AFTER2"  <<std::endl <<std::flush;
            
            qs = p - sourceMean;
            qt = x - targetMean;

            H += Vector3f(qt.x,qt.y,qt.z) * Vector3f(qs.x,qs.y,qs.z).transpose();


            error += compute_error(x, p, rotation, translation);
            
        }

        JacobiSVD<Matrix3f> svd(H,ComputeFullV | ComputeFullU);
        Matrix3f U = svd.matrixU();
        Matrix3f V = svd.matrixV();
        Matrix3f rotation = U * V.transpose();

        Vector3f sourceMean_eigen(sourceMean.x,sourceMean.y,sourceMean.z);
        Vector3f targetMean_eigen(targetMean.x,targetMean.y,targetMean.z);
        
        bool err =  abs(error) > eps ? true :false; 
        std::cout << "Distance between means: " << (targetMean_eigen - sourceMean_eigen).norm() << " iter:" << iter << "  error:" << error << std::endl << std::flush;

        translation = targetMean_eigen - (rotation*sourceMean_eigen);
        
        for(int i = 0; i < sourceCloud.size(); i++)
        {
            rotate(*sourceCloud[i],rotation);
            translate(*sourceCloud[i],translation);
        }
        sourceMean = computeCloudMean(sourceCloud);
        sourceMean_eigen = Vector3f(sourceMean.x,sourceMean.y,sourceMean.z);
        auto new_distance = (targetMean_eigen - sourceMean_eigen).norm();
        if(new_distance >= old_distance){
            std::cout << "ICP converged ..." << std::endl << std::flush;
            break;
        }

    }

    delete tree;
    
      
}

Point computeCloudMean(std::vector<Point>& cloud)
{
     Point mean{0.f,0.f,0.f};
     for (int i = 0; i < cloud.size(); i++)
     {
           mean.x += cloud[i].x;
           mean.y += cloud[i].y;
           mean.z += cloud[i].z;
     }
     mean.x = mean.x / cloud.size();//static_cast<float>(cloud.size());
     mean.y = mean.y / cloud.size();//static_cast<float>(cloud.size());
     mean.z = mean.z / cloud.size();//static_cast<float>(cloud.size());
   
     return mean;
}


Point computeCloudMean(std::vector<Point*>& cloud)
{
     Point mean{0.f,0.f,0.f};
     for (int i = 0; i < cloud.size(); i++)
     {
           mean.x += cloud[i]->x;
           mean.y += cloud[i]->y;
           mean.z += cloud[i]->z;
     }
     mean.x = mean.x / cloud.size();//static_cast<float>(cloud.size());
     mean.y = mean.y / cloud.size();//static_cast<float>(cloud.size());
     mean.z = mean.z / cloud.size();//static_cast<float>(cloud.size());
   
     return mean;
}

double compute_error(Point& targetPoint, Point& sourcePoint, Eigen::Matrix3f rotationMatrix, Eigen::Vector3f translationVector)
{
    Eigen::Vector3f res {sourcePoint.x,sourcePoint.y,sourcePoint.z};

    res = rotationMatrix * res;
    double err = pow(targetPoint.x - res(0) - translationVector(0),2);
    err +=  pow(targetPoint.y - res(1) - translationVector(1),2);
    err +=  pow(targetPoint.z - res(2) - translationVector(2),2);
    return err;
}


void rotate(Point& p, Eigen::Matrix3f rotationMatrix)
{
    Eigen::Vector3f temp{p.x,p.y,p.z};

    temp = rotationMatrix * temp;
    p.x = temp(0);
    p.y = temp(1);
    p.z = temp(2);

}
void translate(Point& p, Eigen::Vector3f translationVector)
{
    p.x += translationVector(0);
    p.y += translationVector(1);
    p.z += translationVector(2);
}
void icp(std::vector<Point>& sourceCloud,std::vector<Point>& targetCloud,const int maxIterations)
{
   // sourceCloud.erase(std::remove_if(sourceCloud.begin(),sourceCloud.end(), [](auto& x) { return x.z < 1e-8;}),sourceCloud.end());//x.x == -0 || x.y == -0 || x.z == -0 || (x.x == 0 && x.y == 0 && x.z ==0); }),sourceCloud.end());
  //  targetCloud.erase(std::remove_if(targetCloud.begin(),targetCloud.end(), [](auto& x) { return x.z < 1e-8;}),targetCloud.end()); //|| (x.x == 0 && x.y == 0 && x.z ==0);  }),targetCloud.end());
    //taArgetCloud.erase(std::remove_if(targetCloud.begin(),targetCloud.end(), [](auto& x) { return x.x == -0 || x.y == -0 || x.z == -0 || (x.x == 0 && x.y == 0 && x.z ==0); }));
    //std::erase_if(sourceCloud,[](auto& x) { return x.x == -0 || x.y == -0 || x.z == -0 || (x.x == 0 && x.y == 0 && x.z ==0) || x.z > 1.0f; });
    //std::erase_if(targetCloud,[](auto& x) { return x.x == -0 || x.y == -0 || x.z == -0 || (x.x == 0 && x.y == 0 && x.z ==0) || x.z > 1.0f; });
    using namespace Eigen;
    Matrix3f rotation = Matrix3f::Identity();
    Vector3f translation = Vector3f::Zero();
    
    Point sourceMean = computeCloudMean(sourceCloud);
    Point targetMean = computeCloudMean(targetCloud);
   
    Vector3f sourceMean_eigen(sourceMean.x,sourceMean.y,sourceMean.z);
    Vector3f targetMean_eigen(targetMean.x,targetMean.y,targetMean.z);
     
    auto old_distance = (targetMean_eigen - sourceMean_eigen).norm();

   // std::cout << "Source size: "<<sourceCloud.size() << " target:" <<targetCloud.size();
   // std::cout << "sourceMean: " << sourceMean_eigen(0) << " " << sourceMean_eigen(1) << " " << sourceMean_eigen(2) << std::endl;
   // std::cout << "targetMean: " << targetMean_eigen(0) << " " << targetMean_eigen(1) << " " << targetMean_eigen(2) << std::endl;
   // std::cout << "START distance: " << old_distance << std::endl;
   std::cout << "init tree" << std::endl << std::flush;
    KdTree* tree = new KdTree(targetCloud);
    
   std::cout << "tree done." << std::endl << std::flush;
    //std::cout << "AFTER"  <<std::endl <<std::flush;
    size_t numPointsSource = sourceCloud.size();
    size_t numPointsTarget = targetCloud.size();

    const int numRandomSamples = 10000 % sourceCloud.size();
   std::cout << "samples: " << numRandomSamples << std::endl;
   Point p{0.f,0.f,0.f},x{0.f,0.f,0.f};
    
   // float cost = 1.0;
    std::srand(time(0));

    Point qs{0.f,0.f,0.f},qt{0.f,0.f,0.f};

    Matrix3f H = Matrix3f::Zero();
    //Matrix3f temp_H = Matrix3f::Zero();


    for(int iter =0; iter < max_iterations && abs(error) > eps; iter ++)
    {
        //reset parameters
        error =0.0;
        H = Matrix3f::Zero();

        sourceMean = computeCloudMean(sourceCloud);

        for (int i = 0; i < numRandomSamples; i++)
        {

            if(i%5000 == 0) std::cout << i << "/" << sourceCloud.size() <<std::endl;


            int randSample = std::rand() % sourceCloud.size();
            // sample the dynamic point cloud
            p = sourceCloud[randSample];

   // std::cout << "p: " << p.x << " " << p.y << " " << p.z << std::endl << std::flush;
            // get the closest point in the static point cloud
           // std::cout << "BEFORE2"  <<std::endl <<std::flush;
            tree->search(p, x);
           // std::cout << "AFTER2"  <<std::endl <<std::flush;
            
            qs = p - sourceMean;
            qt = x - targetMean;

            H += Vector3f(qt.x,qt.y,qt.z) * Vector3f(qs.x,qs.y,qs.z).transpose();


            error += compute_error(x, p, rotation, translation);
            
        }

        JacobiSVD<Matrix3f> svd(H,ComputeFullV | ComputeFullU);
        Matrix3f U = svd.matrixU();
        Matrix3f V = svd.matrixV();
        Matrix3f rotation = U * V.transpose();

        Vector3f sourceMean_eigen(sourceMean.x,sourceMean.y,sourceMean.z);
        Vector3f targetMean_eigen(targetMean.x,targetMean.y,targetMean.z);
        
        bool err =  abs(error) > eps ? true :false; 
        std::cout << "Distance between means: " << (targetMean_eigen - sourceMean_eigen).norm() << " iter:" << iter << "  error:" << error << std::endl << std::flush;

        translation = targetMean_eigen - (rotation*sourceMean_eigen);
        
        for(int i = 0; i < sourceCloud.size(); i++)
        {
            rotate(sourceCloud[i],rotation);
            translate(sourceCloud[i],translation);
        }
        sourceMean = computeCloudMean(sourceCloud);
        sourceMean_eigen = Vector3f(sourceMean.x,sourceMean.y,sourceMean.z);
        auto new_distance = (targetMean_eigen - sourceMean_eigen).norm();
        if(new_distance >= old_distance){
            std::cout << "ICP converged ..." << std::endl << std::flush;
            break;
        }
        old_distance = new_distance;

    }

    delete tree;
    
      
}


