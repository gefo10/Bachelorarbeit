#pragma once 

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Sparse>
#include "Point.h"
#include "../pcl_helper.h"
#include "Eigen/Core"
const double eps = 1e-0;
double error = 100.0f;
int max_iterations = 4;


void icp(std::vector<Point*>& sourceCloud,std::vector<Point*>& targetCloud,const int maxIterations=max_iterations);
void icp(std::vector<Point>& sourceCloud,std::vector<Point>& targetCloud,const int maxIterations=max_iterations);
Point computeCloudMean(std::vector<Point*>& cloud);
Point computeCloudMean(std::vector<Point>& cloud);
double compute_error(Point& targetPoint, Point& sourcePoint, Eigen::Matrix3f rotationMatrix, Eigen::Vector3f translationVector);
void rotate(Point& p, Eigen::Matrix3f rotationMatrix);
void translate(Point& p, Eigen::Vector3f translationVector);

void icp_demo_pcl(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& sourceCloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& targetCloud,const int maxIterations=max_iterations)
{
    sourceCloud->points.erase(std::remove_if(sourceCloud->points.begin(),sourceCloud->points.end(), [](auto& x) { return x.z > 1.0f || x.z <1e-8 || (x.x == 0 && x.y == 0 && x.z ==0); }),sourceCloud->points.end());
    targetCloud->points.erase(std::remove_if(targetCloud->points.begin(),targetCloud->points.end(), [](auto& x) { return x.z > 1.0f || x.z <1e-8 || (x.x == 0 && x.y == 0 && x.z ==0);  }),targetCloud->points.end());
    
      Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
      pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
      icp.setMaximumIterations (maxIterations);
      icp.setInputSource (sourceCloud);
      icp.setInputTarget (targetCloud);
      icp.align (*sourceCloud);
      icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function

      if (icp.hasConverged ())
      {
          return;
      //  std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
      //  transformation_matrix = icp.getFinalTransformation().cast<double>();
      //  for(int i = 0; i < sourceCloud->points.size(); i++)
      //  {
      //      //rotate
      //      Eigen::Vector3f temp{sourceCloud->points[i].x,sourceCloud->points[i].y,sourceCloud->points[i].z};

      //      Eigen::Matrix3f rotationMatrix = Eigen::Matrix3f::Identity();
      //      rotationMatrix.block<3,3>(0,0) = transformation_matrix.block<3,3>(0,0).cast<float>();
      //      temp = rotationMatrix * temp;

      //      sourceCloud->points[i].x = temp(0);
      //      sourceCloud->points[i].y = temp(1);
      //      sourceCloud->points[i].z = temp(2);

      //      //translate
      //      sourceCloud->points[i].x += transformation_matrix(0,3); 
      //      sourceCloud->points[i].y += transformation_matrix(1,3); 
      //      sourceCloud->points[i].z += transformation_matrix(2,3); 

      //  }

      }
      else
      {
        PCL_ERROR ("\nICP has not converged.\n");
      }
}

void test_cloud_random_shift(std::vector<Point>& cloud);
