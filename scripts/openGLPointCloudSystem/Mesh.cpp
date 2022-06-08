#include "Mesh.h"
#include <iostream>
#include <glm/gtc/matrix_transform.hpp>
#include <sstream>
#include <fstream>

Mesh::Mesh()
    : mLoaded(false)
{}

bool Mesh::loadOBJ(const std::string& filename)
{
    std::vector<unsigned int> vertexIndices,uvIndices;
    std::vector<glm::vec3> tempVertices;
    std::vector<glm::vec2> tempUVs;

    if(filename.find(".obj") != std::string::npos)
    {
        std::ifstream fin(filename,std::ios::in);
        if (!fin)
        {
            std::cerr << "Cannot open" << filename << std::endl;
            return false;
        }
        std::cout << "Loading OBJ file" << filename << " ..." << std::endl;

        std::string lineBuffer;
        while(std::getline(fin,lineBuffer))
        {
            if(lineBuffer.substr(0,2) == "v ")
            {
                std::istringstream v(lineBuffer.substr(2));

                glm::vec3 vertex;
                v >> vertex.x; 
                v >> vertex.y;
                v >> vertex.z;
                std::cout << vertex.x << " " << vertex.y << " " << vertex.z << " " << std::endl << std::flush;
                tempVertices.push_back(vertex);
            } else if (lineBuffer.substr(0,2) == "vt")
            {
                std::istringstream vt(lineBuffer.substr(3));
                glm::vec2 uv;
                vt >> uv.x;
                vt >> uv.y;
                tempUVs.push_back(uv);
            } else if (lineBuffer.substr(0,2) == "f ")
            {
                int p1,p2,p3; //store mesh index
                int t1,t2,t3; //store uv(texture) index
                int n1,n2,n3; //normal index
                const char* face = lineBuffer.c_str();
                int match = sscanf(face, "f %i/%i/%i %i/%i/%i %i/%i/%i",
                        &p1,&t1,&n1,
                        &p2,&t2,&n2,
                        &p3,&t3,&n3);
                if (match != 9) std::cout << "Simple parser not compatible !!" <<std::endl;
                //if (match != 9){
                //     match = sscanf(face, "f %i//%i %i//%i %i//%i",
                //        &p1,&n1,&p2,
                //        //&t1,&t2,&t3,
                //        &n2,&p3,&n3);
                //     if(match != 6) 
                //        std::cout << "Failed to parse OBJ file" << std::endl;
                //}                 
                
                //ignoring normals for now

                vertexIndices.push_back(p1);
                vertexIndices.push_back(p2);
                vertexIndices.push_back(p3);

                uvIndices.push_back(t1);
                uvIndices.push_back(t2);
                uvIndices.push_back(t3);
                //-------------
                //TODO: normals
                //---------------
            }
        }

        fin.close();

        //for each vertex of each triangle
        for(unsigned int i = 0; i < vertexIndices.size(); i++)
        {
            glm::vec3 vertex = tempVertices[vertexIndices[i] -1];
            //glm::vec2 uv = tempUVs[uvIndices[i] -1];

            Vertex meshVertex;
            meshVertex.Position = vertex;
            //meshVertex.TexCoords = uv;


            mVertices.push_back(meshVertex);
        }

        //Create and initialize buffers
        initBuffers();
        mLoaded = true;

        return mLoaded;
    }


    return false;
}

void Mesh::Draw()//Shader& shader)
{
    if(!mLoaded) return;

    mVAO.Bind();
    glDrawArrays(GL_TRIANGLES,0,mVertices.size());
    mVAO.Unbind();
}

void Mesh::initBuffers()
{
    VertexBuffer mVBO(&mVertices[0],mVertices.size() * sizeof(Vertex));
    VertexBufferLayout layout;

    //position
    layout.Push(3,false,VertexBufferLayout::FLOAT);
    mVAO.AddBuffer(mVBO,layout);
    //tex coord
    //layout.Push(2,false,VertexBufferLayout::FLOAT);

    mVAO.Unbind();

}
