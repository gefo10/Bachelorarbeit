#include<AndreiUtils/utilsJson.h>
#include<iostream>

using namespace std;

int main () {
    cout << "Hello World!" << endl;

    nlohmann::json content = AndreiUtils::readJsonFile("../config/testJson.json");
    cout << content.dump() << endl;

    if (content.contains("inputModality")) {
        cout << "input modality is " << content.at("inputModality").get<string>() << endl;
    }

    return 0;
}
