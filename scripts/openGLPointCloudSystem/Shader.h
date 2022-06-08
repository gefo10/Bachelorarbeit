#pragma once 
#include <string>
#include "glm/glm.hpp"

struct ShaderProgramSource
{
    std::string VertexSource;
    std::string FragmentSource;
};


class Shader
{
public:
    Shader(const std::string& filename);
    ~Shader();

    void Bind() const;
    void Unbind() const;


    //set uniforms
    void SetUniform4f(const std::string& name,float v0, float v1, float v2, float v3);
    void SetUniform4Mat(const std::string& name,glm::mat4 m);
    void SetUniformFloat(const std::string& name,float size);
private:
    unsigned int m_RendererID;
    std::string m_FilePath;

    unsigned int GetUniformLocation(const std::string& name);

    unsigned int CompileShader(unsigned int type, const std::string& source);
    ShaderProgramSource ParseShader(const std::string& filepath);
    unsigned int CreateShader(const std::string& vertexShader, const std::string& fragmentShader);
};
