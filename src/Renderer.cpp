#include <ScannerLib/Renderer.h>
#include <iostream>
#include <ScannerLib/VertexArray.h>
#include <ScannerLib/Shader.h>
#include <ScannerLib/IndexBuffer.h>
#include <pcl/io/pcd_io.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>
#include <utility>




Renderer::Renderer(Window& w)
    :_window(&w)
{
    model = glm::translate(glm::mat4(1.0f),glm::vec3(0.f,0.f,-2.f));

}
void Renderer::Draw(Mesh mesh)
{
   
    Shader s_model(M_Basic_Vertex,M_Basic_Fragment);

    s_model.Bind();
    glm::mat4 mvp = _window->GetProjectionMatrix() *_window->GetCamera().getViewMatrix()*model;

    double lastTime = glfwGetTime();
    model =  model *  glm::rotate(glm::mat4(1.0f),glm::radians(180.0f),glm::vec3(0.0f,1.0f,0.0f)) * glm::rotate(glm::mat4(1.0f),glm::radians(180.0f),glm::vec3(0.0f,0.0f,1.0f)) * glm::scale(glm::mat4(1.0f),glm::vec3(5.f,5.f,5.f));

    while(!_window->shouldClose()){
       
       _window->clearScreen(); 
        _window->showFPS();
        double currentTime = glfwGetTime();
        double deltaTime = currentTime- lastTime;
        _window->pollEvents();
        _window->Update(deltaTime);
        mvp = _window->GetProjectionMatrix() * _window->GetCamera().getViewMatrix() * model;

        s_model.SetUniform4Mat("MVP",mvp);
        s_model.Bind();
        mesh.Draw();
         _window->swapBuffers();
        lastTime = currentTime;
    }
   
}


void Renderer::Draw(std::string filename)
{
   VertexArray vao;
   
    Shader s_model(M_Basic_Vertex,M_Basic_Fragment);
    s_model.Bind();
    Mesh mesh;
    bool loaded = mesh.loadOBJ(filename);
    if(!loaded) std::cout << "Mesh couldnt load" << std::endl;

    glm::mat4 mvp = _window->GetProjectionMatrix() *_window->GetCamera().getViewMatrix()*model;

    double lastTime = glfwGetTime();
    model =  model *  glm::rotate(glm::mat4(1.0f),glm::radians(180.0f),glm::vec3(0.0f,1.0f,0.0f)) * glm::rotate(glm::mat4(1.0f),glm::radians(180.0f),glm::vec3(0.0f,0.0f,1.0f)) * glm::scale(glm::mat4(1.0f),glm::vec3(5.f,5.f,5.f));

    while(!_window->shouldClose()){
       
       _window->clearScreen(); 
        _window->showFPS();
        double currentTime = glfwGetTime();
        double deltaTime = currentTime- lastTime;
        _window->pollEvents();
        _window->Update(deltaTime);
        mvp = _window->GetProjectionMatrix() * _window->GetCamera().getViewMatrix() * model;

        s_model.SetUniform4Mat("MVP",mvp);
        mesh.Draw();
         _window->swapBuffers();
        lastTime = currentTime;
    }
   
}

void Renderer::DrawPoints(std::vector<Point*>& points)
{
   VertexArray vao;
   
   std::vector<Point> cloud;
   for(int i =0; i < points.size(); i++)
   {
       cloud.push_back(*points[i]);
   }
   VertexBuffer vb(&cloud[0],points.size()*sizeof(Point));
   
    VertexBufferLayout layout;
    layout.Push(3,false,VertexBufferLayout::FLOAT);
    layout.Push(3,false,VertexBufferLayout::FLOAT);

    vao.AddBuffer(vb,layout);

   Shader s(Point_Vertex,Point_Fragment);
    s.Bind();
    glm::mat4 mvp = _window->GetProjectionMatrix() *_window->GetCamera().getViewMatrix()*model;
    s.SetUniform4Mat("MVP",mvp); 
    float pointSize =10.0f;// _window->GetCamData().radius;//_window->GetWidth()/640;

    double lastTime = glfwGetTime();
    model =  model *  glm::rotate(glm::mat4(1.0f),glm::radians(180.0f),glm::vec3(0.0f,1.0f,0.0f)) * glm::rotate(glm::mat4(1.0f),glm::radians(180.0f),glm::vec3(0.0f,0.0f,1.0f)) * glm::scale(glm::mat4(1.0f),glm::vec3(5.f,5.f,5.f));
    //glPointSize(3 *(_window->GetWidth() /  _window->GetHeight()));
    glPointSize(pointSize);
    //model = glm::translate(glm::mat4(1.0f),glm::vec3(0.0f,0.0f,2.0f)) * glm::rotate(glm::mat4(1.0f),glm::radians(15.f),glm::vec3(0.f,1.0f,0.f))*  glm::rotate(glm::mat4(1.0f),glm::radians(15.f),glm::vec3(1.f,0.0f,0.f))*glm::scale(glm::mat4(1.0f),glm::vec3(1.0f,1.0f,1.0f));
    while(!_window->shouldClose()){
       
        _window->clearScreen(); 
        _window->showFPS();
        double currentTime = glfwGetTime();
        double deltaTime = currentTime- lastTime;
        _window->pollEvents();
        _window->Update(deltaTime);
        
        mvp = _window->GetProjectionMatrix() * _window->GetCamera().getViewMatrix() * model;

        vao.Bind();
        s.SetUniform4Mat("MVP",mvp);

        glDrawArrays(GL_POINTS,0,points.size());
        _window->swapBuffers();
        lastTime = currentTime;
    }
   
}


void Renderer::DrawPoints(std::vector<Point>& points)
{
   VertexArray vao;
   VertexBuffer vb(&points[0],points.size()*sizeof(Point));
   
    VertexBufferLayout layout;
    layout.Push(3,false,VertexBufferLayout::FLOAT);
    layout.Push(3,false,VertexBufferLayout::FLOAT);

    vao.AddBuffer(vb,layout);

   Shader s(Point_Vertex,Point_Fragment);
    s.Bind();
    glm::mat4 mvp = _window->GetProjectionMatrix() *_window->GetCamera().getViewMatrix()*model;
    s.SetUniform4Mat("MVP",mvp); 
    float pointSize =10.0f;// _window->GetCamData().radius;//_window->GetWidth()/640;

    double lastTime = glfwGetTime();

    model =  model *  glm::rotate(glm::mat4(1.0f),glm::radians(180.0f),glm::vec3(0.0f,1.0f,0.0f)) * glm::rotate(glm::mat4(1.0f),glm::radians(180.0f),glm::vec3(0.0f,0.0f,1.0f)) * glm::scale(glm::mat4(1.0f),glm::vec3(5.f,5.f,5.f));
    //model =  model *  glm::rotate(glm::mat4(1.0f),glm::radians(180.0f),glm::vec3(0.0f,1.0f,0.0f)) * glm::scale(glm::mat4(1.0f),glm::vec3(5.f,5.f,5.f));

    glPointSize(3 *(_window->GetWidth() /  _window->GetHeight()));
    //glPointSize(pointSize);
    //model = glm::translate(glm::mat4(1.0f),glm::vec3(0.0f,0.0f,2.0f)) * glm::rotate(glm::mat4(1.0f),glm::radians(15.f),glm::vec3(0.f,1.0f,0.f))*  glm::rotate(glm::mat4(1.0f),glm::radians(15.f),glm::vec3(1.f,0.0f,0.f))*glm::scale(glm::mat4(1.0f),glm::vec3(1.0f,1.0f,1.0f));
    while(!_window->shouldClose()){
       
        _window->clearScreen(); 
        _window->showFPS();
        double currentTime = glfwGetTime();
        double deltaTime = currentTime- lastTime;
        _window->pollEvents();
        _window->Update(deltaTime);
        
        mvp = _window->GetProjectionMatrix() * _window->GetCamera().getViewMatrix() * model;

        vao.Bind();
        s.SetUniform4Mat("MVP",mvp);

        glDrawArrays(GL_POINTS,0,points.size());
        _window->swapBuffers();
        lastTime = currentTime;
    }
   
}

//###########
//NOT TESTED
//############
void Renderer::DrawPoints(std::vector<std::vector<Point>>& frames) 
{
    std::vector<VertexArray> vaos;
    std::vector<VertexBuffer> vbos;
    for(unsigned int i = 0; i < frames.size() ; i++)
    {
        VertexBuffer vb(&frames[i][0],frames[i].size() * sizeof(Point));
        vbos.push_back(vb);
    }

    VertexBufferLayout layout;
    layout.Push(3,false,VertexBufferLayout::FLOAT);
    layout.Push(3,false,VertexBufferLayout::FLOAT);

    for(unsigned int i = 0 ; i < vbos.size() ; i++)
    {
      VertexArray vao;
      vao.AddBuffer(vbos[i],layout);
      vaos.push_back(vao);
    }
   Shader s(Point_Vertex,Point_Fragment);
    s.Bind();
    glm::mat4 mvp = _window->GetProjectionMatrix() *_window->GetCamera().getViewMatrix()*model;
    s.SetUniform4Mat("MVP",mvp); 
    float pointSize =10.0f;

    double lastTime = glfwGetTime();
    model =  model *  glm::rotate(glm::mat4(1.0f),glm::radians(180.0f),glm::vec3(0.0f,1.0f,0.0f)) * glm::rotate(glm::mat4(1.0f),glm::radians(180.0f),glm::vec3(0.0f,0.0f,1.0f)) * glm::scale(glm::mat4(1.0f),glm::vec3(5.f,5.f,5.f));
    //glPointSize(3 *(_window->GetWidth() /  _window->GetHeight()));
    glPointSize(pointSize);
    while(!_window->shouldClose()){
         
        _window->clearScreen(); 
        _window->showFPS();
        double currentTime = glfwGetTime();
        double deltaTime = currentTime- lastTime;
        _window->pollEvents();
        _window->Update(deltaTime);
        
        mvp = _window->GetProjectionMatrix() * _window->GetCamera().getViewMatrix() * model;

        vaos[_window->GetCamData().iter % vaos.size()].Bind();
        s.SetUniform4Mat("MVP",mvp);

        glDrawArrays(GL_POINTS,0,frames[_window->GetCamData().iter % frames.size()].size());
        _window->swapBuffers();
        lastTime = currentTime;
    }
 
}
void Renderer::DrawPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
   VertexArray vao;
   std::vector<Point> points;
   for(unsigned int i = 0; i < cloud->points.size(); i++) {
        //if(cloud->points[i].z && cloud->points[i].z < 15.0f){
            Point p;
            p.x = cloud->points[i].x;
            p.y = cloud->points[i].y;
            p.z = cloud->points[i].z;
            p.r = static_cast<float>(cloud->points[i].r) / 255;
            p.g = static_cast<float>(cloud->points[i].g) / 255;
            p.b = static_cast<float>(cloud->points[i].b) / 255;
            points.push_back(p);
        //}
   }
    VertexBuffer vb(&points[0],points.size()*sizeof(Point));
   
    VertexBufferLayout layout;
    layout.Push(3,false,VertexBufferLayout::FLOAT);
    layout.Push(3,false,VertexBufferLayout::FLOAT);

    vao.AddBuffer(vb,layout);

   Shader s(Point_Vertex,Point_Fragment);
    s.Bind();
    glm::mat4 mvp = _window->GetProjectionMatrix() *_window->GetCamera().getViewMatrix()*model;
    s.SetUniform4Mat("MVP",mvp); 
    float pointSize =10.0f;// _window->GetCamData().radius;//_window->GetWidth()/640;

    double lastTime = glfwGetTime();
    model =  model *  glm::rotate(glm::mat4(1.0f),glm::radians(180.0f),glm::vec3(0.0f,1.0f,0.0f)) * glm::rotate(glm::mat4(1.0f),glm::radians(180.0f),glm::vec3(0.0f,0.0f,1.0f)) * glm::scale(glm::mat4(1.0f),glm::vec3(5.f,5.f,5.f));
    glPointSize(3 *(_window->GetWidth() /  _window->GetHeight()));
    //glPointSize(pointSize);

    //model = glm::translate(glm::mat4(1.0f),glm::vec3(0.0f,0.0f,2.0f)) * glm::rotate(glm::mat4(1.0f),glm::radians(15.f),glm::vec3(0.f,1.0f,0.f))*  glm::rotate(glm::mat4(1.0f),glm::radians(15.f),glm::vec3(1.f,0.0f,0.f))*glm::scale(glm::mat4(1.0f),glm::vec3(1.0f,1.0f,1.0f));
    while(!_window->shouldClose()){
       
        _window->clearScreen(); 
        _window->showFPS();
        double currentTime = glfwGetTime();
        double deltaTime = currentTime- lastTime;
        _window->pollEvents();
        _window->Update(deltaTime);
        
        mvp = _window->GetProjectionMatrix() * _window->GetCamera().getViewMatrix() * model;



        vao.Bind();
        s.SetUniform4Mat("MVP",mvp);

        glDrawArrays(GL_POINTS,0,points.size());
        
        _window->swapBuffers();
        lastTime = currentTime;
    }
   
    lines_vaos.clear();
}


void Renderer::DrawLine(const Point& p1, const Point& p2)
{

    VertexArray vao;
    std::vector<Point> vertices {p1, p2};

  
    Shader line_shader(Point_Vertex,Point_Fragment);
    line_shader.Bind();

    VertexBuffer vb(&vertices[0],2*sizeof(Point));

    VertexBufferLayout layout;
    layout.Push(3,false,VertexBufferLayout::FLOAT);
    layout.Push(3,false,VertexBufferLayout::FLOAT);

    vao.AddBuffer(vb,layout);
    
    this->lines_vaos.push_back({vao,vertices.size()});

//    while(!_window->shouldClose()){
//
//        _window->clearScreen();
//        _window->showFPS();
//        double currentTime = glfwGetTime();
//        double deltaTime = currentTime- lastTime;
//        _window->pollEvents();
//        _window->Update(deltaTime);
//
//        mvp = _window->GetProjectionMatrix() * _window->GetCamera().getViewMatrix() * model;
//
//        vao.Bind();
//        s.SetUniform4Mat("MVP",mvp);
//
//        glDrawArrays(GL_LINES,0,points.size());
//        _window->swapBuffers();
//        lastTime = currentTime;
//    }
//
}
