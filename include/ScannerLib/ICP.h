#pragma once 

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Sparse>
#include <ScannerLib/Point.h>
#include <ScannerLib/pcl_helper.h>
#include <Eigen/Core>

extern const double eps;
extern double error;
extern int max_iterations;


void icp(std::vector<Point*>& sourceCloud,std::vector<Point*>& targetCloud,const int maxIterations=max_iterations);
void icp(std::vector<Point>& sourceCloud,std::vector<Point>& targetCloud,const int maxIterations=max_iterations);
Point computeCloudMean(std::vector<Point*>& cloud);
Point computeCloudMean(std::vector<Point>& cloud);
double compute_error(Point& targetPoint, Point& sourcePoint, Eigen::Matrix3f rotationMatrix, Eigen::Vector3f translationVector);
void rotate(Point& p, Eigen::Matrix3f rotationMatrix);
void translate(Point& p, Eigen::Vector3f translationVector);
void test_cloud_random_shift(std::vector<Point>& cloud);


template<typename PointT>
PointT computeCloudMean(typename pcl::PointCloud<PointT>::Ptr cloud)
{
     PointT mean;
     mean.x = 0.f;
     mean.y = 0.f;
     mean.z = 0.f;
     for (int i = 0; i < cloud.size(); i++)
     {
           mean.x += cloud->points[i].x;
           mean.y += cloud->points[i].y;
           mean.z += cloud->points[i].z;
     }
     mean.x = mean.x / cloud->points.size();//static_cast<float>(cloud.size());
     mean.y = mean.y / cloud->points.size();//static_cast<float>(cloud.size());
     mean.z = mean.z / cloud->points.size();//static_cast<float>(cloud.size());
   
     return mean;

}

std::vector<Point>& icp(std::vector<std::vector<Point>>& frames, const int maxIterations=max_iterations);
//template<PointT>
//void icp(typename pcl::PointCloud<PointT>::Ptr sourceCloud, typename pcl::PointCloud<PointT>::Ptr targetCloud)
//{
//    sourceCloud->points.erase(std::remove_if(sourceCloud->points.begin(),sourceCloud->points.end(), [](auto& x) { return x.z > 1.0f || x.z <1e-8 || (x.x == 0 && x.y == 0 && x.z ==0); }),sourceCloud->points.end());
//    targetCloud->points.erase(std::remove_if(targetCloud->points.begin(),targetCloud->points.end(), [](auto& x) { return x.z > 1.0f || x.z <1e-8 || (x.x == 0 && x.y == 0 && x.z ==0);  }),targetCloud->points.end());
//    using namespace Eigen;
//    Matrix3f rotation = Matrix3f::Identity();
//    Vector3f translation = Vector3f::Zero();
//    
//    Point sourceMean = computeCloudMean(sourceCloud);
//    Point targetMean = computeCloudMean(targetCloud);
//   
//    Vector3f sourceMean_eigen(sourceMean.x,sourceMean.y,sourceMean.z);
//    Vector3f targetMean_eigen(targetMean.x,targetMean.y,targetMean.z);
//     
//    auto old_distance = (targetMean_eigen - sourceMean_eigen).norm();
//
//   // std::cout << "Source size: "<<sourceCloud.size() << " target:" <<targetCloud.size();
//   // std::cout << "sourceMean: " << sourceMean_eigen(0) << " " << sourceMean_eigen(1) << " " << sourceMean_eigen(2) << std::endl;
//   // std::cout << "targetMean: " << targetMean_eigen(0) << " " << targetMean_eigen(1) << " " << targetMean_eigen(2) << std::endl;
//   // std::cout << "START distance: " << old_distance << std::endl;
//   std::cout << "init tree" << std::endl << std::flush;
//    KdTree* tree = new KdTree(targetCloud);
//    
//   std::cout << "tree done." << std::endl << std::flush;
//    //std::cout << "AFTER"  <<std::endl <<std::flush;
//    size_t numPointsSource = sourceCloud.size();
//    size_t numPointsTarget = targetCloud.size();
//
//    const int numRandomSamples = 1000 % sourceCloud.size();
//   //std::cout << "samples: " << numRandomSamples << std::endl;
//   Point p{0.f,0.f,0.f},x{0.f,0.f,0.f};
//    
//   // float cost = 1.0;
//    std::srand(time(0));
//
//    Point qs{0.f,0.f,0.f},qt{0.f,0.f,0.f};
//
//    Matrix3f H = Matrix3f::Zero();
//    //Matrix3f temp_H = Matrix3f::Zero();
//
//
//    for(int iter =0; iter < max_iterations && abs(error) > eps; iter ++)
//    {
//
//        //reset parameters
//        error =0.0;
//        H = Matrix3f::Zero();
//
//        sourceMean = computeCloudMean(sourceCloud);
//
//        for (int i = 0; i < sourceCloud.size(); i++)
//        {
//
//            //int randSample = std::rand() % sourceCloud.size();
//            // sample the dynamic point cloud
//            p = sourceCloud[i];
//
//   // std::cout << "p: " << p.x << " " << p.y << " " << p.z << std::endl << std::flush;
//            // get the closest point in the static point cloud
//           // std::cout << "BEFORE2"  <<std::endl <<std::flush;
//            tree->search(p, x);
//           // std::cout << "AFTER2"  <<std::endl <<std::flush;
//            
//            qs = p - sourceMean;
//            qt = x - targetMean;
//
//            H += Vector3f(qt.x,qt.y,qt.z) * Vector3f(qs.x,qs.y,qs.z).transpose();
//
//
//            error += compute_error(qt, p, rotation, translation);
//            
//        }
//
//        JacobiSVD<Matrix3f> svd(H,ComputeFullV | ComputeFullU);
//        Matrix3f U = svd.matrixU();
//        Matrix3f V = svd.matrixV();
//        Matrix3f rotation = U * V.transpose();
//
//        Vector3f sourceMean_eigen(sourceMean.x,sourceMean.y,sourceMean.z);
//        Vector3f targetMean_eigen(targetMean.x,targetMean.y,targetMean.z);
//        
//        bool err =  abs(error) > eps ? true :false; 
//        std::cout << "Distance between means: " << (targetMean_eigen - sourceMean_eigen).norm() << " iter:" << iter << "  error:" << error << std::endl << std::flush;
//
//        translation = targetMean_eigen - (rotation*sourceMean_eigen);
//        
//        for(int i = 0; i < sourceCloud.size(); i++)
//        {
//            rotate(sourceCloud[i],rotation);
//            translate(sourceCloud[i],translation);
//        }
////        sourceMean = computeCloudMean(sourceCloud);
////        sourceMean_eigen = Vector3f(sourceMean.x,sourceMean.y,sourceMean.z);
////        auto new_distance = (targetMean_eigen - sourceMean_eigen).norm();
////        if(new_distance >= old_distance){
////            std::cout << "ICP converged ..." << std::endl << std::flush;
////            break;
////        }
////        old_distance = new_distance;
////
//    }
//
//    delete tree;
//    
//   
//}
