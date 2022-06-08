#pragma once
#include <vector>
#include <GL/glew.h>


struct VertexBufferElement
{
    unsigned int type;
    unsigned int count;
    bool normalized;

    static unsigned int GetSizeOfType(unsigned int type)
    {
        switch(type)
        {
        case GL_FLOAT:         return 4;
        case GL_UNSIGNED_INT:  return 4;
        case GL_UNSIGNED_BYTE: return 1;
        }

        return 0;
    }
};

class VertexBufferLayout
{
public:
    VertexBufferLayout()
        :m_Stride(0){};

    VertexBufferLayout(unsigned int stride)
        :m_Stride(stride){};
    
    enum TYPE{ FLOAT, UNSIGNED_INT, UNSIGNED_CHAR };
    static unsigned int convertType(TYPE type)
    {
        switch(type)
        {
            case FLOAT: return GL_FLOAT;
            case UNSIGNED_INT: return GL_UNSIGNED_INT;
            case UNSIGNED_CHAR: return GL_UNSIGNED_BYTE;
        }
        return 0;
    }
    void Push(unsigned int count,bool normalized,TYPE type)
    {
        unsigned int typeGL = convertType(type);
        m_Elements.push_back({typeGL,count,normalized});
        m_Stride += VertexBufferElement::GetSizeOfType(typeGL) * count;
    }
   
    inline const std::vector<VertexBufferElement> GetElements() const {return m_Elements;}
    inline unsigned int GetStride() const {return m_Stride;}
private:
    std::vector<VertexBufferElement> m_Elements;
    unsigned int m_Stride;

};
