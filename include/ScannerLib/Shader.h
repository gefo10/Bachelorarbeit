#pragma once 
#include <string>
#include <glm/glm.hpp>

extern const char* M_Basic_Vertex;
extern const char* M_Basic_Fragment;
extern const char* Point_Vertex;
extern const char* Point_Fragment;
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

    unsigned int CompileShader(unsigned int type, const char* source);
    //ShaderProgramSource ParseShader(const std::string& filepath);
    unsigned int CreateShader(const char* vertexShader, const char* fragmentShader);
};
