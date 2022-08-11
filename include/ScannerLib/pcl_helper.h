#pragma once 
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <librealsense2/rs.hpp>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <stdlib.h>
#include <unordered_set>
#include <boost/thread/thread.hpp>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <thread>
#include <chrono>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <fstream>
#include <pcl/common/common_headers.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/SVD>
#include <typeinfo>
#include <pcl/io/ply_io.h>
#include <ScannerLib/Point.h>
#include <ScannerLib/KdTree.h>
#include <cmath>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
//#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
//#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
//#include <pcl/surface/on_nurbs/triangulation.h>
using Color = Eigen::Vector4d;
using Cloud_simplePtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
using Cloud_rgbPtr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr;
using Cloud_simple = pcl::PointCloud<pcl::PointXYZ>;
using Cloud_rgb = pcl::PointCloud<pcl::PointXYZRGB>;

std::tuple<int,int,int> RGB_Texture(rs2::video_frame,rs2::texture_coordinate);

namespace pcl_helpers {

    Cloud_simplePtr load_PLY(const std::string &file_name,Cloud_simplePtr cloud_ptr);
    Cloud_rgbPtr load_PLY(const std::string &file_name, Cloud_rgbPtr cloud_ptr);
    

    void view(Cloud_simplePtr ,float = 0.05f,float = 0.05f,float = 0.05f,float = 0.0f);  
    void view(Cloud_rgbPtr,float = 0.05f,float = 0.05f,float = 0.05f,float = 0.0f); 
    void view(pcl::PolygonMesh ,float = 0.05f,float = 0.05f,float = 0.05f,float = 0.0f);
    void view(const std::string& ,float = 0.05f,float = 0.05f,float= 0.05f,float = 0.0f); 
    
    void fastTriangulation(const std::string &file_name,pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals);
    void fastTriangulation(Cloud_simplePtr& cloud,pcl::PolygonMesh& mesh);
    
    void testPrint();
    
    pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(Cloud_simplePtr& cloud);
    pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(Cloud_rgbPtr& cloud);
    
    void filterZ(Cloud_simplePtr& cloud,float min, float max);
    void filterZ(Cloud_rgbPtr& cloud,float min, float max);
    void filterY(Cloud_simplePtr& cloud,float min, float max);
    void filterY(Cloud_rgbPtr& cloud,float min, float max);
    void filterX(Cloud_simplePtr& cloud,float min, float max);
    void filterX(Cloud_rgbPtr& cloud,float min, float max);
    
    //apply transformation on pointcloud (rotation is in radians)
    void transform(Cloud_simplePtr& cloud, float x, float y, float z,float rotation);
    void transform(Cloud_rgbPtr& cloud, float x, float y, float z,float rotation);
    
    pcl::PointCloud<pcl::PointNormal>::Ptr smooth_mls(Cloud_simplePtr& cloud);

    Cloud_simple::Ptr pc_toPCL(const rs2::points&);
    Cloud_rgb::Ptr pcRGB_toPCL(const rs2::points&,const rs2::video_frame&);
    template<typename PointT>
    PointT computeCloudMean(typename pcl::PointCloud<PointT>::Ptr cloud)
    {
         PointT mean;
         mean.x = 0;
         mean.y = 0;
         mean.z = 0;
         for (int i = 0; i < cloud->points.size(); i++)
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

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> RANSAC_SegmentPlane(
	      typename pcl::PointCloud<PointT>::Ptr cloud,
	      int maxIterations,
	      float distanceThreshold)
{

	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	PointT point1;
	PointT point2;
	PointT point3;
	int idx1;
	int idx2;
	int idx3;
	float a,b,c,d,dis,len;
    
    int indx1,indx2,indx3;
	for(int it=0;it<maxIterations;it++)
	{
		std::unordered_set<int> temp_Indices;
		while(temp_Indices.size()<3)
			temp_Indices.insert((rand() % cloud->points.size()));
		auto iter = temp_Indices.begin();
		idx1 = *iter;
		++iter;
		idx2 = *iter;
		++iter;
		idx3 = *iter;
		point1 = cloud->points[idx1];
		point2 = cloud->points[idx2];
		point3 = cloud->points[idx3];

		a = (((point2.y-point1.y)*(point3.z-point1.z))-((point2.z-point1.z)*(point3.y-point1.y)));
		b = (((point2.z-point1.z)*(point3.x-point1.x))-((point2.x-point1.x)*(point3.z-point1.z)));
		c = (((point2.x-point1.x)*(point3.y-point1.y))-((point2.y-point1.y)*(point3.x-point1.x)));


		d = -(a*point1.x+b*point1.y+c*point1.z);
		len = std::sqrt(a*a+b*b+c*c);

        //std::cout << "d original: " << d << std::endl;
        //std::cout << "len original: " << d << std::endl;

		for(int i=0; i < cloud->points.size(); i++)
		{
			if(i!=idx1 && i !=idx2 && i!=idx3)
			{
				dis = (std::abs(a * cloud->points[i].x + b * cloud->points[i].y + c * cloud->points[i].z + d) / len);
				if(dis <= distanceThreshold)
				{
					temp_Indices.insert(i);
				}
			}
		}
		if(temp_Indices.size()>inliersResult.size())
		{
			inliersResult.clear();
			inliersResult = temp_Indices;
            indx1=idx1;
            indx2=idx2;
            indx3=idx3;

		}
	}

	if (inliersResult.size () == 0)
	{
	  std::cerr << "No Inliers found." << std::endl;
	}
	typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
        {
            if(index == indx1 || index == indx2 || index == indx3)
            {
                point.r = 255;
                point.g = 0;
                point.b = 0;
            }else{
                point.r = 0;
                point.b = 255;
                point.g = 0;
                
            }
			cloudInliers->points.push_back(point);
        }
		else
			cloudOutliers->points.push_back(point);
	}

	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers, cloudInliers);


    return segResult;
}



//returns std::pair(pointcloud, isEmpty) 
// 1.arg:  detected plane (inliers)
// 2.arg:  bool == True if pointcloud is empty
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> plane_detection(
        typename pcl::PointCloud<PointT>::Ptr plane_scan,float distanceThreshold,int max_iterations)
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr      inliers (new pcl::PointIndices);

    //create segm object
    typename pcl::SACSegmentation<PointT> seg;

    //Optional, but no harm
    seg.setOptimizeCoefficients(true);

    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_MSAC);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setMaxIterations (max_iterations);
    seg.setInputCloud(plane_scan);
    seg.segment(*inliers,*coefficients);


    if(inliers->indices.size() == 0)
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
        typename pcl::PointCloud<PointT>::Ptr cloud(new typename pcl::PointCloud<PointT>);

        return std::make_pair(cloud,cloud);
    }

    typename pcl::PointCloud<PointT>::Ptr result(new typename pcl::PointCloud<PointT>);
    for(const auto& indx: inliers->indices)
        result->points.push_back(plane_scan->points[indx]);

    int count = 0;
    for(const auto& indx: inliers->indices)
    {
	plane_scan->points.erase(plane_scan->points.begin() + indx + count);
	count--;
    }


    return {result,plane_scan};
}

template<typename PointT>
void ransac_pcl(typename pcl::PointCloud<PointT>::Ptr cloud,float distanceThreshold)
{
    std::vector<int> inliers;

    // created RandomSampleConsensus object and compute the appropriated model
    typename pcl::SampleConsensusModelPlane<PointT>::Ptr model_p (new typename pcl::SampleConsensusModelPlane<PointT> (cloud));
 
    typename pcl::RandomSampleConsensus<PointT> ransac (model_p);
    ransac.setDistanceThreshold (distanceThreshold);
    ransac.computeModel();
    ransac.getInliers(inliers);

  
    // copies all inliers of the model computed to another PointCloud
    typename pcl::PointCloud<PointT>::Ptr new_cloud ( new typename pcl::PointCloud<PointT>);

    pcl::copyPointCloud (*cloud, inliers, *new_cloud);
    *cloud = *new_cloud;
}
template<typename PointT>
void RANSAC_SVD(typename pcl::PointCloud<PointT>::Ptr cloud,typename pcl::PointCloud<PointT>::Ptr plane, int numPoints, float distanceThreshold)
{
    using namespace Eigen;
    plane->width = plane->points.size();
    plane->height = 1;

    plane->points.erase(std::remove_if(plane->points.begin(),plane->points.end(), [&plane](auto& x) 
			    { //return x.z > 2.0f or x.y < 1.0f or x.x < 3.0f or x.x > -3.0f; 
			      //bool val = x.z > 1.5f or x.y < 0.8f or x.y > 3.f or x.x < -1.f or x.x > 1.f; 
                  bool val = x.z > 1.5f;
			      if(val) plane->width--;
			      return val;
			      //float distance = sqrt(pow(node.current->x - cu.x, 2.0) + pow(node.current->y - p.y, 2.0) + pow(node.current->z - p.z, 2.0));

			    }),plane->points.end());


    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->points.erase(std::remove_if(cloud->points.begin(),cloud->points.end(), [&cloud](auto& x) { 
			    bool val = x.z > 1.5f;; //or x.y < -7.0f ; 
			    if(val) cloud->width--;
			    return val;

			    }),cloud->points.end());
    //cloud->points.erase(std::remove_if(cloud->points.begin(),cloud->points.end(), [](auto& x) { return x.z > 1.0f; }),cloud->points.end());

    auto segmentation_pair = plane_detection<PointT>(plane,0.01f,5000);


    *plane = *segmentation_pair.first;
    MatrixXf m (plane->points.size(),3);
    //Matrix<float,numPoints,3> m;
	srand(time(NULL));

    std::cout << m.size() << std::endl << std::flush;
    
    for(int i = 0; i < plane->points.size(); i++)
    {
        //unsigned int index = std::rand() % plane->points.size();
        auto& p = plane->points[i];
        //insert into matrix
        m(i,0) = p.x;
        m(i,1) = p.y;
        m(i,2) = p.z;
    }

    JacobiSVD<MatrixXf> svd(m,ComputeThinU | ComputeFullV);
    auto v= svd.matrixV(); 
    Vector3f x = v.col(0);
    Vector3f y = v.col(1);
    Vector3f z = x.cross(y);
    auto sig_values = svd.singularValues();

    PointT mean = computeCloudMean<PointT>(plane);

	
    Vector3f mean_eigen(mean.x,mean.y,mean.z);
    // Vector3f x(sig_values(0,0)+mean.x,mean.y,mean.z);
    // Vector3f y(mean.x,sig_values(1,0)+mean.y,mean.z);
    x += mean_eigen;
    y += mean_eigen;
   // Vector3f y(mean.x,sig_values(1,0)+mean.y,mean.z);

    //Vector3f n = (x-mean_eigen).cross((y-mean_eigen));
    Vector3f n = -z;
    n.normalize();

    

    PointT normal;
    normal.x = n(0);
    normal.y = n(1);
    normal.z = n(2);
    normal.r = 0;
    normal.g = 255;
    normal.b = 0;
    
    using namespace std;


    //chose a random point from the plane to filter the source point cloud
    //unsigned int index = std::rand() % plane->points.size();
    //auto& p_plane = plane->points[index];
    //
    float max_y = -1000.0f;
    PointT p_plane;
    for (int i=0 ; i< plane->points.size(); i++)
    {
        auto& tmp = plane->points[i];
        if(max_y < tmp.y)
        {
            max_y = tmp.y;
            p_plane = tmp;
        }
        
    }
    p_plane.g = 255;
    p_plane.r = 0;
    p_plane.b = 0;

    std::cout << "p_plane : " << p_plane.x << "  " << p_plane.y << "  " << p_plane.z << std::endl;


    Vector3f p_plane_eigen(p_plane.x, p_plane.y, p_plane.z);
    cloud->height = 1;
    cloud->width = cloud->points.size();
    distanceThreshold = 1e-2;
    cloud->points.erase(std::remove_if(cloud->points.begin(),cloud->points.end(),       
          [&n,&distanceThreshold,&mean_eigen,&p_plane_eigen,&cloud](auto& x) { 
             
                Vector3f p_cloud_eigen(x.x, x.y, x.z);
                float distance = (p_cloud_eigen-mean_eigen).dot(n-mean_eigen);
                if (distance <= distanceThreshold )
                    cloud->width -= 1;

                return distance <= distanceThreshold;
                
                }),cloud->points.end());

    PointT origin;
    origin.x = 0.f;
    origin.y =0.f;
    origin.z =0.f;
    origin.r =0;
    origin.b =0;
    origin.g =0;
    //cloud->points.push_back(p_plane);

    //for(unsigned int i = 0; i< cloud->points.size(); i++)
    //{
    //    auto& p_cloud = cloud->points[i];
    //    Vector3f p_cloud_eigen(p_cloud.x, p_cloud.y, p_cloud.z);

    //    float distance = (p_cloud_eigen - p_plane_eigen).dot(n);

    //    if(distance <= distanceThreshold) { 
    //        cloud->points.erase( cloud->points.begin() + i);
    //        i--;
    //    }

    //}

  //  cloud->points.push_back(p_plane);

   mean.r = 0; mean.g = 255; mean.b = 0;
   cloud->points.push_back(mean);
   cloud->points.push_back(normal);
   //for(auto& p : plane->points){
   //    p.r = 0; p.g = 0; p.b = 255;
   //    cloud->points.push_back(p);
   //}





    
}

    void poisson_reconstruction(pcl::PointCloud<pcl::PointNormal>::Ptr&, pcl::PolygonMesh&,int= 8,int=8,int=8,int=4.0f);
   // void poisson_reconstruction2(Cloud_simplePtr&, pcl::PolygonMesh&);

    void poisson_gp3_reconstruction(std::vector<Point> cloud, int surface_mode,int normal_method,pcl::PolygonMesh& mesh);

    template <typename PointT,typename PointTNormal>
    void poisson_gp3_reconstruction(typename pcl::PointCloud<PointT>::Ptr& cloud, int surface_mode,int normal_method,pcl::PolygonMesh& mesh)
    {
       /* ****Translated point cloud to origin**** */
         Eigen::Vector4f centroid;
         pcl::compute3DCentroid(*cloud, centroid);

         Eigen::Affine3f transform = Eigen::Affine3f::Identity();
         transform.translation() << -centroid[0], -centroid[1], -centroid[2];

         typename pcl::PointCloud<PointT>::Ptr cloudTranslated(new typename pcl::PointCloud<PointT>());
         pcl::transformPointCloud(*cloud, *cloudTranslated, transform);

         /* ****kdtree search and msl object**** */
         typename pcl::search::KdTree<PointT>::Ptr kdtree_for_points (new typename pcl::search::KdTree<PointT>);
         kdtree_for_points->setInputCloud(cloudTranslated);
         typename pcl::PointCloud<PointTNormal>::Ptr cloud_with_normals (new typename pcl::PointCloud<PointTNormal> ());

         bool mls_mode = false;
         bool normal_mode = false;

         if(normal_method == 1){
            normal_mode = true;
         }else if(normal_method == 2){
            mls_mode = true;
         }else{
            std::cout << "Select:\n '1' for normal estimation \n '2' for mls normal estimation " << std::endl;
            std::exit(-1);
         }

         bool gp3_mode = false;
         bool poisson_mode = false;

         if(surface_mode == 1){
            poisson_mode = true;
         }else if(surface_mode == 2){
            gp3_mode = true;
         }else{
            std::cout << "Select: \n'1' for surface poisson method \n '2' for surface gp3 method " << std::endl;
            std::exit(-1);
         }

         if(mls_mode){

           std::cout << "Using mls method estimation...";

           typename pcl::PointCloud<PointTNormal>::Ptr mls_points (new typename pcl::PointCloud<PointTNormal>());
           typename pcl::MovingLeastSquares<PointT, PointTNormal> mls;
           //Set parameters
           mls.setComputeNormals(true);
           mls.setInputCloud(cloudTranslated);
          // mls.setDilationIterations(10);
           //mls.setDilationVoxelSize(0.5);
           //mls.setSqrGaussParam(2.0);
           //mls.setUpsamplingRadius(5);
           //mls.setPolynomialOrder (2);
           //mls.setPointDensity(30);
           mls.setSearchMethod(kdtree_for_points);
           mls.setSearchRadius(0.03);
           mls.process(*mls_points);

           typename pcl::PointCloud<PointT>::Ptr temp(new typename pcl::PointCloud<PointT>());

           for(int i = 0; i < mls_points->points.size(); i++) {

                  PointT pt;
                  pt = cloud->points[i];

                  temp->points.push_back(pt);
            }


           pcl::concatenateFields (*temp, *mls_points, *cloud_with_normals);
           std::cout << "[OK]" << std::endl;

         }else if(normal_mode){

           std::cout << "Using normal method estimation...";

           typename pcl::NormalEstimationOMP<PointT, pcl::Normal> n;
           pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

           n.setInputCloud(cloudTranslated);
           n.setSearchMethod(kdtree_for_points);
           //n.setKSearch(15); //It was 20
           n.setRadiusSearch(0.1);
           n.compute(*normals);//Normals are estimated using standard method.

           //pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal> ());
           pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

           std::cout << "[OK]" << std::endl;

         }else{
            std::cout << "Select: '1' for normal method estimation \n '2' for mls normal estimation " << std::endl;
            std::exit(-1);
         }

         // Create search tree*
         typename pcl::search::KdTree<PointTNormal>::Ptr kdtree_for_normals (new typename pcl::search::KdTree<PointTNormal>);
         kdtree_for_normals->setInputCloud(cloud_with_normals);

         std::cout << "Applying surface meshing...";

         if(gp3_mode){

           std::cout << "Using surface method: gp3 ..." << std::endl;

           int searchK = 100; //was 100
           int search_radius = 10; //was 10
           int setMU = 5; //was 5
           int maxiNearestNeighbors = 100; //was 100
           bool normalConsistency = false;

           typename pcl::GreedyProjectionTriangulation<PointTNormal> gp3;

           gp3.setSearchRadius(search_radius);//It was 0.025
           gp3.setMu(setMU); //It was 2.5
           gp3.setMaximumNearestNeighbors(maxiNearestNeighbors);    //It was 100
           //gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees    //it was 4
           //gp3.setMinimumAngle(M_PI/18); // 10 degrees //It was 18
           //gp3.setMaximumAngle(M_PI/1.5); // 120 degrees        //it was 1.5
           gp3.setNormalConsistency(normalConsistency); //It was false
           gp3.setInputCloud(cloud_with_normals);
           gp3.setSearchMethod(kdtree_for_normals);
           gp3.reconstruct(mesh);

           //vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
           //pcl::PolygonMesh mesh_pcl;
           //pcl::VTKUtils::convertToVTK(triangles,polydata);
           //pcl::VTKUtils::convertToPCL(polydata,mesh_pcl);

           //pcl::io::savePolygonFilePLY("mesh.ply", mesh_pcl);

           std::cout << "[OK]" << std::endl;

         }else if(poisson_mode){

            std::cout << "Using surface method: poisson ..." << std::endl;

            int nThreads=8;
            int setKsearch=10; //was 10
            int depth=10; // was 7
            float pointWeight=2.0; //was 2.0
            float samplePNode=20;//1.5
            float scale=1.0; //was 1.1 
            int isoDivide=8;
            bool confidence=true;
            bool outputPolygons=true;
            bool manifold=true;//was true
            int solverDivide=8;

            typename pcl::Poisson<PointTNormal> poisson;

            poisson.setDepth(depth);//9
            poisson.setInputCloud(cloud_with_normals);
            poisson.setPointWeight(pointWeight);//4
            poisson.setDegree(2);
            poisson.setSamplesPerNode(samplePNode);//1.5
            poisson.setScale(scale);//1.1
            poisson.setIsoDivide(isoDivide);//8
            poisson.setConfidence(confidence);
            poisson.setOutputPolygons(outputPolygons);
            poisson.setManifold(manifold);
            poisson.setSolverDivide(solverDivide);//8
            poisson.reconstruct(mesh);

            //pcl::PolygonMesh mesh2;
            //poisson.reconstruct(mesh2);
            //pcl::surface::SimplificationRemoveUnusedVertices rem;
            //rem.simplify(mesh2,triangles);

            std::cout << "[OK]" << std::endl;

         }else{
            std::cout << "Select: \n'1' for surface poisson method \n '2' for surface gp3 method " << std::endl;
            std::exit(-1);
         }
         pcl::io::saveOBJFile("poisson_test.obj",mesh);
         //pcl::io::savePLYFile("poisson_test.ply",mesh);

     //      pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>) ;

     //   typename pcl::NormalEstimation<PointT , pcl::Normal> n ;//Normal estimation object
     //   pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>) ;//Store the estimated normal
     //   typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>) ;
     //   tree->setInputCloud(cloud) ;
     //   n.setInputCloud(cloud) ;
     //   n.setSearchMethod(tree) ;
     //   n.setKSearch(20) ;
     //   n.compute(*normals) ;

     //   pcl::concatenateFields(*cloud , *normals , *cloud_with_normals) ;

     //   pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>) ;
     //   tree2->setInputCloud(cloud_with_normals) ;
     //   pcl::Poisson<pcl::PointNormal> pn ;
     //   pn.setSearchMethod(tree2) ;
     //   pn.setInputCloud(cloud_with_normals) ;
     //   pn.setConfidence(false) ;//Set the confidence flag. When it is true, use the length of the normal vector as the confidence information. If false, normalize the normal
     //   pn.setManifold(false) ;//Set the fashion flag , If set to true, add the center of gravity when subdividing the polygon, set false to not add
     //   pn.setOutputPolygons(false) ;//Set whether the output is a polygon
     //   pn.setIsoDivide(8) ;
     //   pn.setSamplesPerNode(3) ;//Set the minimum number of sampling points on each octree node

     //   //pcl::PolygonMesh mesh ;

     //   pn.performReconstruction(mesh) ;
     //   pcl::io::saveOBJFile("poisson_test.obj",mesh);
     //   //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer")) ;
     //   //viewer->setBackgroundColor(0 , 0 , 0) ;
     //   //viewer->addPolygonMesh(mesh , "my") ;
     //   //viewer->initCameraParameters() ;

     //   //while (!viewer->wasStopped())
     //   //{
     //   //    viewer->spinOnce(100) ;
     //   //  std::this_thread::sleep_for(std::chrono::milliseconds(100000));
     //   //}
     //   //return ;

    }

   // void surface_reconstructionCGAL(std::vector<std::pair<CGAL::Exact_predicates_inexact_constructions_kernel::Point_3,CGAL::Exact_predicates_inexact_constructions_kernel::Vector_3> points, CGAL::Polyhedron_3<CGAL::Exact_predicates_inexact_constructions_kernel> mesh,std::string& filename_output);

    
   // void gp3Normal_reconstruction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
   // void gp3Normal_reconstruction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
   // void gp3Normal_reconstruction(std::vector<Point> cloud);
   // void gp3Mls_reconstruction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
   // void gp3Mls_reconstruction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
   // void gp3Mls_reconstruction(std::vector<Point> cloud);
   // 
   // void poissonNormal_reconstruction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
   // void poissonNormal_reconstruction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
   // void poissonNormal_reconstruction(std::vector<Point> cloud);
   // void poissonMls_reconstruction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
   // void poissonMls_reconstruction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
   // void poissonMls_reconstruction(std::vector<Point> cloud);


    void statistical_removal(Cloud_simplePtr& input,int meanK,float threshold);
    void statistical_removal(Cloud_rgbPtr& input,int meanK, float threshold);
    

    void downsample(Cloud_simplePtr& input);

}
