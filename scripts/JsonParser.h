#include <string>
#include <nlohmann/json.hpp>
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
        int GetMaxIterationsICP() const { return max_iterations_icp; };
        unsigned int GetNumberOfPointsRansac() const { return numPointsRansac; };
        float GetDistanceThresholdRansac() const { return distanceThresholdRansac; };
        
        const char* GetSavePath() const { return save_path.c_str(); };
    private:
        std::string filename;
        std::string window_name;
        float window_width;
        float window_height;
        int max_iterations_icp;

        float distanceThresholdRansac;
        unsigned int numPointsRansac;


        std::string save_path;
};
