#pragma once
class VertexBuffer
{
private:
    unsigned int m_RendererID;
    bool m_Empty;

public:
    VertexBuffer(const void* data, unsigned int size);
    VertexBuffer();
    ~VertexBuffer();

    bool isEmpty(){ return m_Empty;};
    void loadData(const void* data, unsigned int size);
    void Bind() const;
    void Unbind() const;
};
