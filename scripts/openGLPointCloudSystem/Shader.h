#pragma once 
#include <string>
#include "glm/glm.hpp"

const char* M_Basic_Vertex = 
"version 330 core\n"
"layout(location = 0) in vec3 position;\n"
"layout(location = 1) in vec3 color;\n"
"out vec3 color_from_vb;\n"
"uniform mat4 MVP;\n"
"void main()\n"
"{\n"
"    gl_Position = MVP * vec4(position,1.0);\n"
"    color_from_vb = color;\n"
"}\0";

const char* M_Basic_Fragment =
"#version 330 core\n"
"in vec3 color_from_vb;\n"
"out vec4 color_fs;\n"
"void main()\n"
"{\n"
"    color_fs = vec4(color_from_vb,1.0f);\n"
"}\0";

const char* Point_Vertex = 
"#version 330 core\n"
"layout(location = 0) in vec4 position;\n"
"layout(location = 1) in vec3 color;\n"
"out vec3 color_from_vshader;\n"
"uniform mat4 MVP;\n"
"void main()\n"
"{\n"
"    gl_Position = MVP * position;\n"
"    color_from_vshader = color;\n"
"}\0";

const char* Point_Fragment =
"#version 330 core\n"
"in vec3 color_from_vshader;\n"
"out vec4 color;\n"
"void main()\n"
"{\n"
"    color = vec4(color_from_vshader,1.0f);\n"
"}\0"

struct ShaderProgramSource
{
    const char* VertexSource;
    const char* FragmentSource;
};


class Shader
{
public:
    Shader(const char* v_shader, const char* f_shader);
    ~Shader();

    void Bind() const;
    void Unbind() const;


    //set uniforms
    void SetUniform4f(const std::string& name,float v0, float v1, float v2, float v3);
    void SetUniform4Mat(const std::string& name,glm::mat4 m);
    void SetUniformFloat(const std::string& name,float size);
private:
    unsigned int m_RendererID;

    unsigned int GetUniformLocation(const std::string& name);

    unsigned int CompileShader(unsigned int type, const std::string& source);
    //ShaderProgramSource ParseShader(const std::string& filepath);
    unsigned int CreateShader(const std::string& vertexShader, const std::string& fragmentShader);
};
