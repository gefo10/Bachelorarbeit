#pragma once
#include <ScannerLib/VertexBuffer.h>
#include <ScannerLib/VertexBufferLayout.h>

class VertexArray
{
private:
   unsigned int m_RendererID;

public:
    VertexArray();
    ~VertexArray();

    void AddBuffer(const VertexBuffer& vb, const VertexBufferLayout& layout);
    void Bind() const;
    void Unbind() const;
};
