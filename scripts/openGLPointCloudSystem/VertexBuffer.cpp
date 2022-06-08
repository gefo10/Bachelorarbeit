#include "VertexBuffer.h"
#include "Renderer.h"

VertexBuffer::VertexBuffer(const void* data,unsigned int size)
    :m_Empty(false)
{
   glGenBuffers(1,&m_RendererID);
   glBindBuffer(GL_ARRAY_BUFFER,m_RendererID);
   glBufferData(GL_ARRAY_BUFFER,size,data,GL_STATIC_DRAW);
}
VertexBuffer::VertexBuffer()
    :m_Empty(true)
{}

void VertexBuffer::loadData(const void* data,unsigned int size)
{
   if (!m_Empty) return; 

   glGenBuffers(1,&m_RendererID);
   glBindBuffer(GL_ARRAY_BUFFER,m_RendererID);
   glBufferData(GL_ARRAY_BUFFER,size,data,GL_STATIC_DRAW);
   m_Empty = false;
}

VertexBuffer::~VertexBuffer()
{
    glDeleteBuffers(1,&m_RendererID);
}

void VertexBuffer::Bind() const
{
   glBindBuffer(GL_ARRAY_BUFFER,m_RendererID);
}


void VertexBuffer::Unbind() const
{
    glBindBuffer(GL_ARRAY_BUFFER,0);
}
