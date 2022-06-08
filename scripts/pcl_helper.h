#pragma once 
#include <librealsense2/rs.hpp>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <stdlib.h>
#include <unordered_set>
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
    
    void transform(Cloud_simplePtr& cloud, float x, float y, float z);
    void transform(Cloud_rgbPtr& cloud, float x, float y, float z);
    
    pcl::PointCloud<pcl::PointNormal>::Ptr smooth_mls(Cloud_simplePtr& cloud);

    Cloud_simple::Ptr pc_toPCL(const rs2::points&);
    Cloud_rgb::Ptr pcRGB_toPCL(const rs2::points&,const rs2::video_frame&);
    
    template<typename PointT>
    std::pair<typename pcl::PointCloud<PointT>::Ptr,typename pcl::PointCloud<PointT>::Ptr> ransac_impl(typename pcl::PointCloud<PointT>::Ptr& cloud,int maxIterations,float distanceThreshold)
    {
        std::unordered_set<int> inliersResult;
        srand(time(NULL));

        while(maxIterations--)
        {
            std::unordered_set<int> inliers;
            while (inliers.size() < 3)
                inliers.insert( rand() % cloud->points.size() );

            float x1,y1,z1,x2,y2,z2,x3,y3,z3;
            auto indx_ptr = inliers.begin();

            x1 = cloud->points[*indx_ptr].x;
            y1 = cloud->points[*indx_ptr].y;
            z1 = cloud->points[*indx_ptr].z;

            ++indx_ptr;

            x2 = cloud->points[*indx_ptr].x;
            y2 = cloud->points[*indx_ptr].y;
            z2 = cloud->points[*indx_ptr].z;

            ++indx_ptr;

            x3 = cloud->points[*indx_ptr].x;
            y3 = cloud->points[*indx_ptr].y;
            z3 = cloud->points[*indx_ptr].z;

            float a = (y2 - y1) * (z3-z1) - (z2-z1) * (y3-y1);
            float b = (z2-z1) * (x3-x1) - (x2-x1) * (z3-z1);
            float c = (x2-x1) * (y3-y1) - (y2-y1) * (x3-x1);
            float d = - (a * x1 + b * y1 + c * z1);

            for(int i = 0; i < cloud->points.size(); i++) {
                if (inliers.count(i) > 0)
                    continue;

                PointT point = cloud->points[i];
                float x4 = point.x;
                float y4 = point.y;
                float z4 = point.z;

                float dis = fabs(a*x4 + b*y4 + c*z4 + d)/sqrt(a*a + b*b + c*c);

                if (dis <= distanceThreshold)
                    inliers.insert(i);
            }

            if (inliers.size() > inliersResult.size())
                inliersResult = inliers;

        }

        typename pcl::PointCloud<PointT>::Ptr cloudInliers (new pcl::PointCloud<PointT>());
        typename pcl::PointCloud<PointT>::Ptr cloudOutliers (new pcl::PointCloud<PointT>());

        for(int i = 0 ; i < cloud->points.size(); i++){

            PointT point = cloud->points[i];
            if (inliersResult.count(i))
                cloudInliers->points.push_back( point );
            else
                cloudOutliers->points.push_back( point );
        }

        std::pair<typename pcl::PointCloud<PointT>::Ptr,typename pcl::PointCloud<PointT>::Ptr> segResult = std::make_pair(cloudInliers,cloudOutliers);

        return segResult;
    }

    
    void poisson_reconstruction(pcl::PointCloud<pcl::PointNormal>::Ptr&, pcl::PolygonMesh&,int= 8,int=8,int=8,int=4.0f);
   // void poisson_reconstruction2(Cloud_simplePtr&, pcl::PolygonMesh&);
    void poisson_reconstruction3(Cloud_simplePtr& input, pcl::PolygonMesh& mesh);
   // void surface_reconstructionCGAL(std::vector<std::pair<CGAL::Exact_predicates_inexact_constructions_kernel::Point_3,CGAL::Exact_predicates_inexact_constructions_kernel::Vector_3> points, CGAL::Polyhedron_3<CGAL::Exact_predicates_inexact_constructions_kernel> mesh,std::string& filename_output);


    void statistical_removal(Cloud_simplePtr& input);

    void downsample(Cloud_simplePtr& input);
}
