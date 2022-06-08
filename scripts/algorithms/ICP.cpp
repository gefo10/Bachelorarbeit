#include "ICP.h"
#include <random>
#include <math.h>
#include "KdTree.h"
void icp(std::vector<Point>& sourceCloud,std::vector<Point>& targetCloud,const int maxIterations)
{
    using namespace Eigen;
    Matrix3f rotation = Matrix3f::Identity();
    Vector3f translation = Vector3f::Zero();
    
    Point sourceMean = computeCloudMean(sourceCloud);
    Point targetMean = computeCloudMean(targetCloud);
    KdTree tree(targetCloud);

    size_t numPointsSource = sourceCloud.size();
    size_t numPointsTarget = targetCloud.size();

    const int numRandomSamples = sourceCloud.size()/2 + targetCloud.size()/2;

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

        Point sourceMean = computeCloudMean(sourceCloud);
        for (int i = 0; i < numRandomSamples; i++)
        {
            int randSample = std::rand() % sourceCloud.size();
            // sample the dynamic point cloud
            p = sourceCloud[randSample];

            // get the closest point in the static point cloud
            tree.search(p, x);
            
            qs = p - sourceMean;
            qt = x - targetMean;

            H += Vector3f(qt.x,qt.y,qt.z) * Vector3f(qs.x,qs.y,qs.z).transpose();


            error += compute_error(x, p, rotation, translation);
            
        }


        JacobiSVD<Matrix3f> svd(H,ComputeFullV | ComputeFullU);
        Matrix3f U = svd.matrixU();
        Matrix3f V = svd.matrixV();
        //float d = (U * V.transpose()).determinant();
        //Matrix3f D = Matrix3f::Identity();
        //D(2,2) = d;
        Matrix3f rotation = U * V.transpose();

        Vector3f sourceMean_eigen(sourceMean.x,sourceMean.y,sourceMean.z);
        Vector3f targetMean_eigen(targetMean.x,targetMean.y,targetMean.z);
        
        bool err =  abs(error) > eps ? true :false; 
        std::cout << "Distance between means: " << (targetMean_eigen - sourceMean_eigen).norm() << " iter:" << iter << "  error:" << error << std::endl << std::flush;

        translation = targetMean_eigen - rotation*sourceMean_eigen;
        
        for(int i = 0; i < sourceCloud.size(); i++)
        {
            rotate(sourceCloud[i],rotation);
            translate(sourceCloud[i],translation);
        }
    }
    
      
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
     mean.x = mean.x / static_cast<float>(cloud.size());
     mean.y = mean.y / static_cast<float>(cloud.size());
     mean.z = mean.z/ static_cast<float>(cloud.size());
   
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
