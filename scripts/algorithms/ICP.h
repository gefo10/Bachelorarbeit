#pragma once 

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Sparse>
#include "Point.h"
const double eps = 1e-8;
double error = 1.0f;
int max_iterations = 500;


void icp(std::vector<Point>& sourceCloud,std::vector<Point>& targetCloud,const int maxIterations=max_iterations);
Point computeCloudMean(std::vector<Point>& cloud);
double compute_error(Point& targetPoint, Point& sourcePoint, Eigen::Matrix3f rotationMatrix, Eigen::Vector3f translationVector);
void rotate(Point& p, Eigen::Matrix3f rotationMatrix);
void translate(Point& p, Eigen::Vector3f translationVector);
