#include<AndreiUtils/utilsJson.h>
#include<iostream>

using namespace std;

int main () {
    cout << "Hello World!" << endl;

    nlohmann::json content = AndreiUtils::readJsonFile("../config/testJson.json");
    //cout << content.dump() << endl;

    if (content.at("new").contains("key")) {
        cout << "input answer is " << content.at("new").at("key").at("value").at(0).get<string>() << endl;
    }
    return 0;
}
