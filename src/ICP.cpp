#include <ScannerLib/ICP.h>
#include <random>
#include <cmath>
#include <ScannerLib/KdTree.h>
#include <Eigen/SVD>
#include <Eigen/Core>

const double eps = 1e-6;
double error = 1.0f;
int max_iterations = 100;

//void icp(std::vector<Point*>& sourceCloud,std::vector<Point*>& targetCloud,const int maxIterations)
//{
//    sourceCloud.erase(std::remove_if(sourceCloud.begin(),sourceCloud.end(), [](auto& x) { return x->x == -0 || x->y == -0 || x->z == -0 || (x->x == 0 && x->y == 0 && x->z ==0); }));
//    targetCloud.erase(std::remove_if(targetCloud.begin(),targetCloud.end(), [](auto& x) { return x->x == -0 || x->y == -0 || x->z == -0 || (x->x == 0 && x->y == 0 && x->z ==0); }));
//    //taArgetCloud.erase(std::remove_if(targetCloud.begin(),targetCloud.end(), [](auto& x) { return x.x == -0 || x.y == -0 || x.z == -0 || (x.x == 0 && x.y == 0 && x.z ==0); }));
//
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
//    std::cout << "Source size: "<<sourceCloud.size() << " target:" <<targetCloud.size();
//    std::cout << "sourceMean: " << sourceMean_eigen(0) << " " << sourceMean_eigen(1) << " " << sourceMean_eigen(2) << std::endl;
//    std::cout << "targetMean: " << targetMean_eigen(0) << " " << targetMean_eigen(1) << " " << targetMean_eigen(2) << std::endl;
//    std::cout << "START distance: " << old_distance << std::endl;
//
//    KdTree* tree = new KdTree(targetCloud);
//    
//    //std::cout << "AFTER"  <<std::endl <<std::flush;
//    size_t numPointsSource = sourceCloud.size();
//    size_t numPointsTarget = targetCloud.size();
//
//    const int numRandomSamples = 1000 % sourceCloud.size();
//
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
//        //reset parameters
//        error =0.0;
//        H = Matrix3f::Zero();
//
//        sourceMean = computeCloudMean(sourceCloud);
//
//        for (int i = 0; i < numRandomSamples; i++)
//        {
//
//            int randSample = std::rand() % sourceCloud.size();
//            // sample the dynamic point cloud
//            p = *sourceCloud[randSample];
//
//            // get the closest point in the static point cloud
//       //     std::cout << "BEFORE2"  <<std::endl <<std::flush;
//            tree->search(p, x);
//       //     std::cout << "AFTER2"  <<std::endl <<std::flush;
//            
//            qs = p - sourceMean;
//            qt = x - targetMean;
//
//            H += Vector3f(qt.x,qt.y,qt.z) * Vector3f(qs.x,qs.y,qs.z).transpose();
//
//
//            error += compute_error(qt, p, rotation, translation,targetMean_eigen);
//            
//        }
//
//        JacobiSVD<Matrix3f> svd(H,ComputeFullV | ComputeFullU);
//        Matrix3f U = svd.matrixU();
//        Matrix3f V = svd.matrixV();
//        Matrix3f rotation = V * U.transpose();
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
//            rotate(*sourceCloud[i],rotation);
//            translate(*sourceCloud[i],translation);
//        }
//        sourceMean = computeCloudMean(sourceCloud);
//        sourceMean_eigen = Vector3f(sourceMean.x,sourceMean.y,sourceMean.z);
//        auto new_distance = (targetMean_eigen - sourceMean_eigen).norm();
//        if(new_distance >= old_distance){
//            std::cout << "ICP converged ..." << std::endl << std::flush;
//            break;
//        }
//
//    }
//
//    delete tree;
//    
//      
//}
//
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

double compute_error(Point& targetPoint, Point& sourcePoint, Eigen::Matrix3f rotationMatrix, Eigen::Vector3f translationVector,Eigen::Vector3f targetMean_eigen)
{
    Eigen::Vector3f res {sourcePoint.x,sourcePoint.y,sourcePoint.z};
    Eigen::Vector3f target_eigen{targetPoint.x,targetPoint.y,targetPoint.z};

    res = rotationMatrix * res;
    //double err = pow(targetPoint.x - res(0) - translationVector(0),2);
    //err +=  pow(targetPoint.y - res(1) - translationVector(1),2);
    //err +=  pow(targetPoint.z - res(2) - translationVector(2),2);
    //double err = pow((target_eigen+targetMean_eigen - res - translationVector).norm(),2);
    double err = pow((target_eigen - (res + translationVector) ).norm(),2);
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
    //sourceCloud.erase(std::remove_if(sourceCloud.begin(),sourceCloud.end(), [](auto& x) { return x.z > 1.0f || x.z <1e-8 || (x.x == 0 && x.y == 0 && x.z ==0); }),sourceCloud.end());
    //targetCloud.erase(std::remove_if(targetCloud.begin(),targetCloud.end(), [](auto& x) { return x.z > 1.0f || x.z <1e-8 || (x.x == 0 && x.y == 0 && x.z ==0);  }),targetCloud.end());
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

    int numRandomSamples = 1000 % sourceCloud.size();
   //std::cout << "samples: " << numRandomSamples << std::endl;
   Point p{0.f,0.f,0.f},x{0.f,0.f,0.f};
    
   // float cost = 1.0;
    std::srand(time(0));

    Point qs{0.f,0.f,0.f},qt{0.f,0.f,0.f};

    Matrix3f H = Matrix3f::Zero();
    //Matrix3f temp_H = Matrix3f::Zero();


    for(int iter =0; iter < max_iterations && abs(error) > eps; iter ++)
    {

        std::cout << "Iteration: " << iter << std::endl;
        //reset parameters
        error =0.0;
        H = Matrix3f::Zero();

        int countSkips=0;
        sourceMean = computeCloudMean(sourceCloud);


        for (int i = 0; i <  numRandomSamples; i++)//sourceCloud.size(); i++)
        {
            //std::cout << "point num:" << i << "/" << sourceCloud.size() << std::flush << std::endl;
            int randSample = std::rand() % sourceCloud.size();
            // sample the dynamic point cloud
            p = sourceCloud[randSample];

            bool found = tree->search(p, x);
            if(!found) {
                countSkips++;
                continue;
            }
           //// std::cout << "AFTER2"  <<std::endl <<std::flush;
            
            qs = p - sourceMean;
            qt = x - targetMean;

            H += Vector3f(qs.x,qs.y,qs.z) * Vector3f(qt.x,qt.y,qt.z).transpose();


            error += compute_error(x, p, rotation, translation,targetMean_eigen);
            
        }
        if(countSkips == numRandomSamples) error =1.0f;

        JacobiSVD<Matrix3f> svd(H,ComputeFullU | ComputeFullV);
        Matrix3f U = svd.matrixU();
        Matrix3f V = svd.matrixV();
        Matrix3f D = Matrix3f::Identity();
        D(2,2) = (V*U.transpose()).determinant();
        std::cout <<"D:" <<D(2,2) << std::endl;
        Matrix3f rotation =V *D * U.transpose();//.transpose();

        
        std::cout << "Det(0):" << rotation.determinant() << std::endl;
      //  if(rotation.determinant() < 0)
      //  {
      //      //rotation matrix is numerically correct but 'reflected' -> multiply 3rd column of V by -1
      //      //JacobiSVD<Matrix3f> svd_new(rotation,ComputeFullU | ComputeFullV);
      //      //U = svd_new.matrixU();
      //      //V = svd_new.matrixV().transpose();
      //      V.col(2) *= -1;
      //      rotation = U* V.transpose();
      //      std::cout << "Det(1):" << rotation.determinant() << std::endl;
      //  }
   //std::cout << rotation(0,0) << " " << rotation(0,1) << " "  << rotation(0,2) << " " << std:: endl; 
   //std::cout << rotation(1,0) << " " << rotation(1,1) << " "  << rotation(1,2) << " " << std:: endl; 
   //std::cout << rotation(2,0) << " " << rotation(2,1) << " "  << rotation(2,2) << " " << std:: endl; 

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
    }

    delete tree;
    
      
}


void test_cloud_random_shift(std::vector<Point>& cloud)
{
    // Defining a rotation matrix and translation vector
  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

  // A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
 // double theta = 45;  // The angle of rotation in radians
 // transformation_matrix (0, 0) = std::cos (theta);
 // transformation_matrix (0, 1) = -sin (theta);
 // transformation_matrix (1, 0) = sin (theta);
 // transformation_matrix (1, 1) = std::cos (theta);
  double theta = 180;  // The angle of rotation in radians
  transformation_matrix (0, 0) = std::cos (theta);
  transformation_matrix (0, 2) = sin (theta);
  transformation_matrix (2, 0) = -sin (theta);
  transformation_matrix (2, 2) = std::cos (theta);


  theta = -90;
  auto t2 = transformation_matrix;
  t2(1,1) = std::cos(theta);
  t2(2,1) = -std::sin(theta);
  t2(1,2) = std::sin(theta);
  t2(2,2) = std::cos(theta);
  // A translation on Z axis (0.4 meters)
  //transformation_matrix (2, 3) = 0.0;

   Eigen::Matrix3f rotMatrix = Eigen::Matrix3f::Identity();
     rotMatrix.block<3,3>(0,0) = transformation_matrix.block<3,3>(0,0).cast<float>();


   auto rot2 = t2.block<3,3>(0,0).cast<float>();
   std::cout << transformation_matrix(0,0) << " " << transformation_matrix(0,1) << " "  << transformation_matrix(0,2) << " " << transformation_matrix(0,3); 
   std::cout << transformation_matrix(1,0) << " " << transformation_matrix(1,1) << " "  << transformation_matrix(1,2) << " " << transformation_matrix(1,3); 
   std::cout << transformation_matrix(2,0) << " " << transformation_matrix(2,1) << " "  << transformation_matrix(2,2) << " " << transformation_matrix(2,3); 
   std::cout << transformation_matrix(3,0) << " " << transformation_matrix(3,1) << " "  << transformation_matrix(3,2) << " " << transformation_matrix(3,3); 
   for(auto i = 0 ; i < cloud.size(); i++)
   {
       //rotate
            Eigen::Vector3f temp{cloud[i].x,cloud[i].y,cloud[i].z};
            temp = rotMatrix * temp;

            cloud[i].x = temp(0);
            cloud[i].y = temp(1);
            cloud[i].z = temp(2);

            //translate
           // cloud[i].x += transformation_matrix(0,3); 
           // cloud[i].y += transformation_matrix(1,3); 
           // cloud[i].z += transformation_matrix(2,3); 

   }

}

std::vector<Point>& icp(std::vector<std::vector<Point>>& frames, const int maxIterations)
{
    if(frames.size() == 0)
        throw std::invalid_argument("No frames");

   
    int i = 1;
    while(i != frames.size())
    {
        icp(frames[0],frames[i],maxIterations);
        frames[0].insert(frames[0].end(),frames[i].begin(),frames[i].end());
    }

    return frames[0];
    

}
