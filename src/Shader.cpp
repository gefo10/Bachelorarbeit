#include <ScannerLib/Shader.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include "GL/glew.h"
#include "glm/gtc/type_ptr.hpp"




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
"}\0";

Shader::Shader(const char* v_shader,const char* f_shader)
    :m_RendererID(0)
{
    //ShaderProgramSource source = ParseShader(filename);
    m_RendererID = CreateShader(v_shader, f_shader);

}

Shader::~Shader()
{
    glDeleteProgram(m_RendererID);
}

unsigned int Shader::CompileShader(unsigned int type, const char* source)
{
   unsigned int id = glCreateShader(type);
   glShaderSource(id,1,&source,nullptr);
   glCompileShader(id);

   //TODO: error handling
   int result;
   glGetShaderiv(id, GL_COMPILE_STATUS, &result);
   if(!result)
   {
        int length;
        glGetShaderiv(id, GL_INFO_LOG_LENGTH,&length);
        char* message = (char*) alloca(length * sizeof(char));

        glGetShaderInfoLog(id,length,&length,message);

        std::cout << "Failed to compile shader!" << std::endl;
        std::cout << message << std::endl;
        glDeleteShader(id);
        return 0;
   }
   return id;

}

unsigned int Shader::CreateShader(const char* vertexShader, const char* fragmentShader)
{
    unsigned int program = glCreateProgram();
    unsigned int vs = CompileShader(GL_VERTEX_SHADER,vertexShader);
    unsigned int fs = CompileShader(GL_FRAGMENT_SHADER,fragmentShader);

    glAttachShader(program,vs);
    glAttachShader(program,fs);
    glLinkProgram(program);
    glValidateProgram(program);

    glDeleteShader(vs);
    glDeleteShader(fs);

    return program;
}

//ShaderProgramSource Shader::ParseShader(const std::string& filepath)
//{
//    std::ifstream stream(filepath);
//
//    enum class ShaderType
//    {
//        NONE = -1, VERTEX = 0, FRAGMENT = 1
//    };
//
//    std::stringstream ss[2];
//    std::string line;
//    ShaderType type;
//    while (getline(stream,line))
//    {
//        if (line.find("#shader") != std::string::npos)
//        {
//            if (line.find("vertex") != std::string::npos)
//                //set mode to vertex
//                type = ShaderType::VERTEX;
//            else if (line.find("fragment") != std::string::npos)
//                //set mode to fragment
//                type = ShaderType::FRAGMENT;
//        }
//        else
//        {
//            ss[(int)type]  << line << '\n';
//        }
//
//    }
//
//   return {ss[0].str(), ss[1].str() };
//}
//
unsigned int Shader::GetUniformLocation(const std::string& name)
{
    int location = glGetUniformLocation(m_RendererID,name.c_str());
    if(location == -1)
        std::cout << "Warning: uniform '" << name << "' doesn't exist!" << std::endl;
    return location; 
}


void Shader::SetUniformFloat(const std::string& name, float size)
{
 unsigned int location = GetUniformLocation(name);
    if(location != -1)
        glUniform1f(location,size);

}
void Shader::SetUniform4f(const std::string& name, float v0, float v1, float v2, float v3)
{
    unsigned int location = GetUniformLocation(name);
    if(location != -1)
        glUniform4f(location,v0,v1,v2,v3);
}

void Shader::Bind() const
{
    glUseProgram(m_RendererID);
}

void Shader::Unbind() const
{
    glUseProgram(0);
}

void Shader::SetUniform4Mat(const std::string& name,glm::mat4 m)
{
    unsigned int location = GetUniformLocation(name);
    if (location != -1)
        glUniformMatrix4fv(location,1,GL_FALSE,glm::value_ptr(m));
}

