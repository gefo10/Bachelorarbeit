#include <ScannerLib/JsonParser.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

JsonParser::JsonParser(std::string filename)
    :window_width(1280),
    window_height(720),
    window_name("Pointcloud window"),
    max_iterations_icp(30),
    distanceThresholdRansac(1e-08),
    maxIterationsRansac(5000),
    distanceThresholdPlane(0.1),
    statisticalRemoval_KPoints(100),
    statisticalRemoval_threshold(0.01f),
    r(0.20f),
    g(0.30f),
    b(0.40f),
  //  numPointsRansac(100),
    save_path("./"),
    mls_radius_search(0.03),
    normal_estim_KSearch(20), // default 20
    gp3_KSearch(100),         //default 100
    gp3_MU(5),              //default 5
    gp3_searchRadius(10),    //default 10
    gp3_maxNearestNeighbors(100),//default 100
    gp3_NormalConsistency(false),//default false
    poisson_nThreads(8), //default 8
    poisson_KSearch(10), //default 10
    poisson_depth(7),   //default 7
    poisson_PointWeight(2.0),//default 2.0
    poisson_samplePNode(1.1),//default 1.5
    poisson_scale(1.1),      //default 1.1
    poisson_isoDevide(8),    //default 8
    poisson_confidence(true),  //default true
    poisson_outputPolygons(true), //default true
    poisson_manifold(true),      //default true
    poisson_solverDivide(8),   //default 8
    poisson_degree(2)       //default 2

{
    using namespace std;

    cout << "Configuration json used: " << filename << endl;

    //Json content = readJsonFile(filename.c_str());
    std::ifstream input(filename);
    std::stringstream ss;
    if(input)
    {
        //feed stringstream with file content 
        ss << input.rdbuf();
        input.close();
    } else {
        throw std::invalid_argument("File couldn't open");
    }
    Json content = Json::parse(ss.str());
    if(content.contains("window"))
    {
        cout << "window config found:" << endl;
        //check width
        if(content.at("window").contains("width"))
        {
            this->window_width = content.at("window").at("width").get<float>();
            cout << "  width: " << this->window_width << endl;
        }
        else {
            cout << "  width not found, using default width:" << this->window_width << endl;
        }
        if(content.at("window").contains("background"))
        {
            if(content.at("window").at("background").contains("r"))
            {
                this->r= content.at("window").at("background").at("r").get<float>();
                if(this->r > 1.0f || this->r < 0)
                {
                    this->r = 1.0f;
                }
            }
            if(content.at("window").at("background").contains("g"))
            {
                this->g= content.at("window").at("background").at("g").get<float>();
                if(this->g > 1.0f || this->g < 0)
                {
                    this->g  = 1.0f;
                }
            }
            if(content.at("window").at("background").contains("b"))
            {
                this->b= content.at("window").at("background").at("b").get<float>();
                if(this->b > 1.0f || this->b < 0)
                {
                    this->b  = 1.0f;
                }
            }


            cout << "  r: " << this->r << " g:" << this->g << " b:" << this->b << endl;
        }
        else {
            cout << "  background not found, using default background:";
            cout << "  r: " << this->r << " g:" << this->g << " b:" << this->b << endl;
        }


        //check height
        if(content.at("window").contains("height"))
        {
            this->window_height = content.at("window").at("height").get<float>();
            cout << "  height: " << this->window_height << endl;
        } else {
            cout << "  height not found, using default height:" << this->window_height << endl;
        
        }

        //check name 
        if(content.at("window").contains("name"))
        {
            this->window_name = content.at("window").at("name").get<string>();
            cout << "  name: " << this->window_name << endl;
        } else {
            cout << "  name not found, using default name:" << this->window_name << endl;
        }

    }

    if(content.contains("icp"))
    {
        cout << "ICP config found:" << endl;
        if(content.at("icp").contains("max_iterations"))
        {
            this->max_iterations_icp = content.at("icp").at("max_iterations").get<int>();
            cout << "  max_iterations:" << this->max_iterations_icp << endl;
        } else {
            cout << "  max_iterations_icp not found, using default max_iterations_icp:" << this->max_iterations_icp << endl;
        }

    }

    //RANSAC settings
    if(content.contains("ransac"))
    {
        cout << "Ransac config found:" << endl;
        if(content.at("ransac").contains("distanceThreshold"))
        {
            this->distanceThresholdRansac = content.at("ransac").at("distanceThreshold").get<float>();
            cout << "  distanceThreshold:" << this->distanceThresholdRansac  << endl;
        } else {
            cout << "  distanceThreshold not found, using default distanceThreshold:" << this->distanceThresholdRansac << endl;
        }
        if(content.at("ransac").contains("max_iterations"))
        {
            this->maxIterationsRansac = content.at("ransac").at("max_iterations").get<int>();
            cout << "  max_iterations:" << this->maxIterationsRansac  << endl;
        } else {
            cout << "  max_iterations not found, using default max_iterations:" << this->maxIterationsRansac << endl;
        }   
        if(content.at("ransac").contains("distanceThreshold_plane"))
        {
            this->distanceThresholdPlane = content.at("ransac").at("distanceThreshold_plane").get<float>();
            cout << "  distanceThreshold_plane:" << this->distanceThresholdPlane  << endl;
        } else {
            cout << "  distanceThreshold_plane not found, using default distanceThreshold_plane:" << this->distanceThresholdPlane<< endl;
        }

        if(content.at("ransac").contains("statistical_removal"))
        {
            cout << "  statistical removal settings for RANSAC found:" << endl;
            if(content.at("ransac").at("statistical_removal").contains("KPoints"))
            {
                this->statisticalRemoval_KPoints = content.at("ransac").at("statistical_removal").at("KPoints").get<int>();
                cout << "    KPoints:" << this->statisticalRemoval_KPoints  << endl;
            } else {
                cout << "    KPoints not found, using default KPoints:" << this->statisticalRemoval_KPoints<< endl;
            }
            if(content.at("ransac").at("statistical_removal").contains("threshold"))
            {
                this->statisticalRemoval_threshold= content.at("ransac").at("statistical_removal").at("threshold").get<int>();
                cout << "    threshold:" << this->statisticalRemoval_threshold  << endl;
            } else {
                cout << "    threshold not found, using default threshold:" << this->statisticalRemoval_threshold<< endl;
            }
        }
//       if(content.at("ransac").contains("numberOfPoints"))
//        {
//            this->numPointsRansac = content.at("ransac").at("numberOfPoints").get<unsigned int>();
//            cout << "  numberOfPoints:" << this->numPointsRansac  << endl;
//        } else {
//            cout << "  numberOfPoints not found, using default numberOfPoints:" << this->numPointsRansac << endl;
//        }
//
    }

    //recon settings
    if(content.contains("reconstruction"))
    {
        cout <<"Reconstruction config found:" << endl;
        if(content.at("reconstruction").contains("mls_radius_search"))
        {
            this->mls_radius_search = content.at("reconstruction").at("mls_radius_search").get<float>();
            cout << "  mls_radius_search:" << this->mls_radius_search  << endl;
        } else {
            cout << "  mls_radius_search not found, using default mls_radius_search:" << this->mls_radius_search << endl;
        }
    
       if(content.at("reconstruction").contains("normal_estimation_KSearch"))
        {
            this->normal_estim_KSearch  = content.at("reconstruction").at("normal_estimation_KSearch").get<int>();
            cout << "  normal_estimation_KSearch" << this->normal_estim_KSearch  << endl;
        } else {
            cout << "  normal_estimation_KSearch not found, using default normal_estimation_KSearch:" << this->normal_estim_KSearch << endl;
        }

        if(content.at("reconstruction").contains("greedy_recon"))
        {
                
            cout << "  Greedy triangulation settings found:" << endl;
            if(content.at("reconstruction").at("greedy_recon").contains("KSearch"))
            {
                this->gp3_KSearch = content.at("reconstruction").at("greedy_recon").at("KSearch").get<int>();
                cout << "    KSearch:" << this->gp3_KSearch  << endl;
            } else {
                cout << "    KSearch not found, using default KSearch:" << this->gp3_KSearch << endl;
            }
    
            if(content.at("reconstruction").at("greedy_recon").contains("MU"))
            {
                this->gp3_MU  = content.at("reconstruction").at("greedy_recon").at("MU").get<int>();
                cout << "    MU" << this->gp3_MU << endl;
            } else {
                cout << "    MU not found, using default MU:" << this->gp3_MU << endl;
            }
            if(content.at("reconstruction").at("greedy_recon").contains("search_radius"))
            {
                this->gp3_searchRadius  = content.at("reconstruction").at("greedy_recon").at("search_radius").get<int>();
                cout << "    search_radius" << this->gp3_searchRadius << endl;
            } else {
                cout << "    search_radius not found, using default search_radius:" << this->gp3_searchRadius << endl;
            }
            if(content.at("reconstruction").at("greedy_recon").contains("max_nearest_neighbors"))
            {
                this->gp3_maxNearestNeighbors = content.at("reconstruction").at("greedy_recon").at("max_nearest_neighbors").get<int>();
                cout << "    max_nearest_neighbors:" << this->mls_radius_search  << endl;
            } else {
                cout << "    max_nearest_neighbors not found, using default max_nearest_neighbors:" << this->gp3_maxNearestNeighbors << endl;
            }
    
            if(content.at("reconstruction").at("greedy_recon").contains("normal_consistency"))
            {
                this->gp3_NormalConsistency  = content.at("reconstruction").at("greedy_recon").at("normal_consistency").get<bool>();
                cout << "    normal_consistency" << this->gp3_NormalConsistency  << endl;
            } else {
                cout << "    normal_consistency not found, using default normal_consistency:" << this->gp3_NormalConsistency << endl;
            }
        }


        if(content.at("reconstruction").contains("poisson"))
        {
            cout << "  Poisson settings found:" << endl;
            if(content.at("reconstruction").at("poisson").contains("nThreads"))
            {
                this->poisson_nThreads = content.at("reconstruction").at("poisson").at("nThreads").get<int>();
                cout << "    nThreads:" << this->poisson_nThreads  << endl;
            } else {
                cout << "    nThreads not found, using default nThreads:" << this->poisson_nThreads << endl;
            }
    
            if(content.at("reconstruction").at("poisson").contains("KSearch"))
            {
                this->poisson_KSearch  = content.at("reconstruction").at("poisson").at("KSearch").get<int>();
                cout << "    KSearch" << this->poisson_KSearch<< endl;
            } else {
                cout << "    KSearch not found, using default KSearch:" << this->poisson_KSearch << endl;
            }
            if(content.at("reconstruction").at("poisson").contains("depth"))
            {
                this->poisson_depth = content.at("reconstruction").at("poisson").at("depth").get<int>();
                cout << "    depth:" << this->poisson_depth << endl;
            } else {
                cout << "    depth not found, using default depth:" << this->poisson_depth << endl;
            }
    
            if(content.at("reconstruction").at("poisson").contains("point_weight"))
            {
                this->poisson_PointWeight  = content.at("reconstruction").at("poisson").at("point_weight").get<float>();
                cout << "    point_weight" << this->poisson_PointWeight << endl;
            } else {
                cout << "    point_weight not found, using default point_weight:" << this->poisson_PointWeight << endl;
            }


            if(content.at("reconstruction").at("poisson").contains("sample_per_node"))
            {
                this->poisson_samplePNode = content.at("reconstruction").at("poisson").at("sample_per_node").get<float>();
                cout << "    sample_per_node:" << this->poisson_samplePNode  << endl;
            } else {
                cout << "    sample_per_node not found, using default sample_per_node:" << this->poisson_samplePNode << endl;
            }
    
            if(content.at("reconstruction").at("poisson").contains("scale"))
            {
                this->poisson_scale  = content.at("reconstruction").at("poisson").at("scale").get<float>();
                cout << "    scale" << this->poisson_scale<< endl;
            } else {
                cout << "    scale not found, using default scale:" << this->poisson_scale << endl;
            }
            if(content.at("reconstruction").at("poisson").contains("isoDivide"))
            {
                this->poisson_isoDevide = content.at("reconstruction").at("poisson").at("isoDivide").get<int>();
                cout << "    isoDivide:" << this->poisson_isoDevide << endl;
            } else {
                cout << "    isoDivide not found, using default isoDivide:" << this->poisson_isoDevide << endl;
            }
    
            if(content.at("reconstruction").at("poisson").contains("confidence"))
            {
                this->poisson_confidence = content.at("reconstruction").at("poisson").at("confidence").get<bool>();
                cout << "    confidence" << this->poisson_confidence << endl;
            } else {
                cout << "    confidence not found, using default confidence:" << this->poisson_confidence << endl;
            }
            if(content.at("reconstruction").at("poisson").contains("output_polygons"))
            {
                this->poisson_outputPolygons= content.at("reconstruction").at("poisson").at("output_polygons").get<bool>();
                cout << "    output_polygons:" << this->poisson_outputPolygons  << endl;
            } else {
                cout << "    output_polygons not found, using default output_polygons:" << this->poisson_outputPolygons << endl;
            }
    
            if(content.at("reconstruction").at("poisson").contains("manifold"))
            {
                this->poisson_manifold  = content.at("reconstruction").at("poisson").at("manifold").get<bool>();
                cout << "    manifold" << this->poisson_manifold<< endl;
            } else {
                cout << "    manifold not found, using default manifold:" << this->poisson_manifold << endl;
            }
            if(content.at("reconstruction").at("poisson").contains("solver_divide"))
            {
                this->poisson_solverDivide = content.at("reconstruction").at("poisson").at("solver_divide").get<int>();
                cout << "    solver_divide:" << this->poisson_solverDivide << endl;
            } else {
                cout << "    solver_divide not found, using default solver_divide:" << this->poisson_solverDivide << endl;
            }
    
            if(content.at("reconstruction").at("poisson").contains("degree"))
            {
                this->poisson_degree = content.at("reconstruction").at("poisson").at("degree").get<int>();
                cout << "    degree" << this->poisson_degree << endl;
            } else {
                cout << "    degree not found, using default degree:" << this->poisson_degree << endl;
            }
        }

    }
    


    //save settings
    if(content.contains("save_folder"))
    {
        cout << "save_folder config found:" << endl;
        if(content.at("save_folder").contains("path"))
        {
            this->save_path = content.at("save_folder").at("path").get<string>();
            cout << "  path:" << this->save_path << endl;
        } else {
            cout << "  path not found, using default path:" << this->save_path << endl;
        }

    }
    


}
