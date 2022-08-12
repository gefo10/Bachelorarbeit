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
extern int maxPoints;

//void icp(std::vector<Point*>& sourceCloud,std::vector<Point*>& targetCloud,const int maxIterations=max_iterations);
void icp(std::vector<Point>& sourceCloud,std::vector<Point>& targetCloud,const int maxIterations=max_iterations,int maxPoints=maxPoints);
Point computeCloudMean(std::vector<Point*>& cloud);
Point computeCloudMean(std::vector<Point>& cloud);
double compute_error(Point& targetPoint, Point& sourcePoint, Eigen::Matrix4f rotationMatrix, Eigen::Vector3f translationVector,Eigen::Vector3f targetMean_eigen);
void rotate(Point& p, Eigen::Matrix3f rotationMatrix);
void translate(Point& p, Eigen::Vector3f translationVector);
void RotateAndTranslateX(std::vector<Point>& cloud,float rotationAngle=0.0, Eigen::Vector3f translation = Eigen::Vector3f::Zero());
void RotateAndTranslateY(std::vector<Point>& cloud,float rotationAngle=0.0, Eigen::Vector3f translation = Eigen::Vector3f::Zero());
void RotateAndTranslateZ(std::vector<Point>& cloud,float rotationAngle=0.0, Eigen::Vector3f translation = Eigen::Vector3f::Zero());


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

