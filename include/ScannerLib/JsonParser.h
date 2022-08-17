#pragma once
#include <string>
//#include <nlohmann/json.hpp>
#include <AndreiUtils/utilsJson.h>
using Json = nlohmann::json;

class JsonParser {
    public:

        //delete default constructor -> construct only with given json file name
        JsonParser() = delete; 
        JsonParser(std::string file_name);

        std::string GetFileName() { return filename; };
        std::string GetWindowName() { return window_name; };
        float GetWindowWidth() const { return window_width; };
        float GetWindowHeight() const { return window_height; };
        float GetRedColor() {return r;};
        float GetGreenColor() {return g;};
        float GetBlueColor() {return b;};
        int GetMaxIterationsICP() const { return max_iterations_icp; };
        int GetMaxPointsICP() { return max_points_icp; };
      //  unsigned int GetNumberOfPointsRansac() const { return numPointsRansac; };
        float GetDistanceThresholdRansac() const { return distanceThresholdRansac; };
        int GetMaxIterationsRansac() {return maxIterationsRansac;};
        int GetStatisticalRemovalKPoints() {return statisticalRemoval_KPoints;};
        float GetStatisticalRemovalTheshold() {return statisticalRemoval_threshold;};
        float GetDistanceThresholdPlaneRansac() {return distanceThresholdPlane;};
        
        const char* GetSavePath() const { return save_path.c_str(); };

        float GetMLSRadiusSearch() {return mls_radius_search;};
        int GetNEKSearch() {return normal_estim_KSearch;};

        int GetGP3KSearch() { return gp3_KSearch;};
        int GetGP3MU() {return gp3_MU;};
        int GetGP3SearchRadius() {return gp3_searchRadius;};
        int GetGP3MaxNearestNeighbors() {return gp3_maxNearestNeighbors;};
        bool GetGP3NormalConsistency() {return gp3_NormalConsistency;};
     
        int GetPoissonNThreads() {return poisson_nThreads;};
        int GetPoissonKSearch() {return poisson_KSearch;};
        int GetPoissonDepth() {return poisson_depth;};
        float GetPoissonPointWeight() {return poisson_PointWeight;};
        float GetPoissonSamplePNode() {return poisson_samplePNode;};
        float GetPoissonScale() {return poisson_scale;};
        int GetPoissonIsoDivide(){return poisson_isoDevide;};
        bool GetPoissonConfidence(){return poisson_confidence;};
        bool GetPoissonOutputPolygons() {return poisson_outputPolygons;};
        bool GetPoissonManifold() {return poisson_manifold;};
        int GetPoissonSolverDivide() {return poisson_solverDivide;};
        int GetPoissonDegree() {return poisson_degree;};
    private:
        std::string filename;
        std::string window_name;
        float window_width;
        float window_height;
        int max_iterations_icp;
        int max_points_icp;
        float r;
        float g;
        float b;

        //RANSAC
        float distanceThresholdRansac;
        int maxIterationsRansac;
        float distanceThresholdPlane;
        int statisticalRemoval_KPoints;
        float statisticalRemoval_threshold;
        //unsigned int numPointsRansac;

        //save path
        std::string save_path;
        

        //mls NE
        float mls_radius_search; //default 0.03
        //NE
        int normal_estim_KSearch; // default 20

        //gp3 recon
        int gp3_KSearch;         //default 100
        int gp3_MU;              //default 5
        int gp3_searchRadius;    //default 10
        int gp3_maxNearestNeighbors;//default 100
        bool gp3_NormalConsistency;//default false

        //poisson recon
        int poisson_nThreads; //default 8
        int poisson_KSearch; //default 10
        int poisson_depth;   //default 7
        float poisson_PointWeight;//default 2.0
        float poisson_samplePNode;//default 1.5
        float poisson_scale;      //default 1.1
        int poisson_isoDevide;    //default 8
        bool poisson_confidence;  //default true
        bool poisson_outputPolygons; //default true
        bool poisson_manifold;      //default true
        int poisson_solverDivide;   //default 8

        int poisson_degree;       //default 2

};
