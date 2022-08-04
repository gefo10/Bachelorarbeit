#pragma once 
#include <vector>
#include <string>
#include <GL/glew.h>
#include <glm/glm.hpp>
#include <ScannerLib/VertexBuffer.h>
#include <ScannerLib/Shader.h>
#include <ScannerLib/VertexArray.h>

struct Vertex {
    // position
    glm::vec3 Position;

    glm::vec3 Color= glm::vec3(1.0f,1.0f,1.0f);
    // normal
    //glm::vec3 Normal;
    // texCoords
    //glm::vec2 TexCoords;
    // tangent
    //glm::vec3 Tangent;
    // bitangent
    //glm::vec3 Bitangent;
    //bone indexes which will influence this vertex
    //int m_BoneIDs[MAX_BONE_INFLUENCE];
    //weights from each bone
    //float m_Weights[MAX_BONE_INFLUENCE];
};


class Mesh {
public:
    // mesh Data
    Mesh();
    
    // render the mesh
    void Draw();//Shader& shader);


    bool loadOBJ(const std::string& filename);
private:
    // render data 
    void initBuffers();

    bool mLoaded;
    std::vector<Vertex> mVertices;

    //VertexBuffer mVBO;
    VertexArray  mVAO;
};

