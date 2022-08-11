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
    numPointsRansac(100),
    save_path("./")
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
    
       if(content.at("ransac").contains("numberOfPoints"))
        {
            this->numPointsRansac = content.at("ransac").at("numberOfPoints").get<unsigned int>();
            cout << "  numberOfPoints:" << this->numPointsRansac  << endl;
        } else {
            cout << "  numberOfPoints not found, using default numberOfPoints:" << this->numPointsRansac << endl;
        }

    }
    
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
