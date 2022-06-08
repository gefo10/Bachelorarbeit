#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include "glm/gtc/matrix_transform.hpp"
#include "Window.h"

Window::Window(int width, int height, const char* title)
     : _width(width), _height(height), _title(title),_fullScreen(false)//_canvas_left_top_x(0), _canvas_left_top_y(0), _canvas_width(width), _canvas_height(height)
{

    _camData.yaw = 0.0f;
    _camData.pitch = 0.0f;
    //_camData.radius = 10.0f;
    _camData.MOUSE_SENSITIVITY= 0.15f;
    _camData.ZOOM_SENSITIVITY = -3.0;
    //_camData.camera.setLookAt(glm::vec3(0.f,0.f,-1.0f));
    _camData.camera.setPosition(glm::vec3(0.0f,0.0f,5.0f));
    _camData.camera.rotate(_camData.yaw,_camData.pitch);
    //_camData.camera.setRadius(_camData.radius);
    _camData.width = &width;
    _camData.height = &height;
    _camData.projection = glm::perspective(glm::radians(_camData.camera.getFOV()), static_cast<float>(width)/static_cast<float>(height),0.01f,2000.0f);
    _camData.speed = 10.0f;



    if(!glfwInit())
    {
        std::cerr << "GLFW initialization failed" << std::endl;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,3);
    glfwWindowHint(GLFW_OPENGL_PROFILE,GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT,GL_TRUE);
            
    _window = glfwCreateWindow(_width,_height,_title.c_str(),NULL,NULL);
            

     if(_window == NULL)
     {
         std::cerr << "Failed to create GLFW window" << std::endl;
         glfwTerminate();
     }

     glfwSetWindowUserPointer(_window,&_camData);
     glfwMakeContextCurrent(_window);
     glfwSetWindowSizeCallback(_window,OnWindowSize);
     glfwSetKeyCallback(_window,OnKey_Event);
   //  glfwSetCursorPosCallback(_window,OnMouseMove);
     glfwSetFramebufferSizeCallback(_window,OnFrameBufferSize);
     glfwSetScrollCallback(_window,OnMouseScroll);
     

   //  glfwSetInputMode(_window,GLFW_CURSOR, GLFW_CURSOR_DISABLED);
     glfwSetCursorPos(_window,_width /2.0f, _height /2.0f);

     glewExperimental = GL_TRUE;

     if(glewInit() != GLEW_OK)
     {
         std::cerr << "GLEW initialization failed" << std::endl;
     }

     glEnable(GL_DEPTH_TEST);
     this->clearScreen();
     this->swapBuffers();

  }

Window:: ~Window()
 {
     glfwDestroyWindow(_window);
     glfwTerminate();
 }

void Window::setCameraLookAt(glm::vec3 target) 
{
    //_camData.camera.setLookAt(target);
   // _camData.yaw = _camData.camera.GetYaw();
   // _camData.pitch = _camData.camera.GetPitch();
}
//##########################
//Handles Key Events 
//###########################
void Window::OnKey_Event(GLFWwindow* window,int key,int scancode,int action,int mode)
{
    CamData& camData = *(CamData*) glfwGetWindowUserPointer(window);
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window,GL_TRUE);
    else if (key == GLFW_KEY_N && action == GLFW_PRESS)
        camData.iter++;

}

void Window::OnWindowSize(GLFWwindow* window,int width, int height)
{
     CamData& camData = *(CamData*) glfwGetWindowUserPointer(window);
     *camData.width = width;
     *camData.height = height;
     camData.projection = glm::perspective(glm::radians(60.0f), static_cast<float>( width)/static_cast<float>(height),0.01f,10.0f);
}
//########################################################
//Handles mouse movement (rotating and zooming for camera)
//########################################################
void Window::OnMouseMove(GLFWwindow* window, double posX, double posY)
{
    
  //  static glm::vec2 lastMousePos = glm::vec2(0,0);
  //  //Update angles based on Left Mouse Button inpit to rotate the camera
  //  CamData& camData = *(CamData*) glfwGetWindowUserPointer(window);
  //  if(glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == 1)
  //  {
  //      camData.yaw -= (static_cast<float>(posX) - lastMousePos.x) * camData.MOUSE_SENSITIVITY;
  //      camData.pitch += (static_cast<float>(posY) - lastMousePos.y) * camData.MOUSE_SENSITIVITY;

  //      camData.camera.rotate(camData.yaw,camData.pitch);
  //  }

  //  if(glfwGetMouseButton(window,GLFW_MOUSE_BUTTON_RIGHT) == 1)
  //  {
  //      float dx = 0.01f * (static_cast<float>(posX) - lastMousePos.x);
  //      float dy = 0.01f * (static_cast<float>(posY) - lastMousePos.y);
  //      camData.radius += dx -dy;
  //      camData.camera.setRadius(camData.radius);
  //      
  //  }


  //  //std::cout << "camRadius: " << w->_camData.radius << "       YAW:" << w->_camData.yaw << "          Pitch:"<<w->_camData.pitch << "    " << std::flush;
  //  lastMousePos.x = static_cast<float>(posX);
  //  lastMousePos.y = static_cast<float>(posY);
    
}
//###########################################
//Handles scrolling and adjusting Field of view
//#############################################
void Window::OnMouseScroll(GLFWwindow* window, double deltaX, double deltaY)
{

    CamData& camData = *(CamData*) glfwGetWindowUserPointer(window);
    double fov = camData.camera.getFOV() * deltaY * camData.ZOOM_SENSITIVITY;
    fov = glm::clamp(fov,1.0,120.0);

    camData.camera.setFOV(fov);
}

//##########################
//Handles window resizing
//##########################
void Window::OnFrameBufferSize(GLFWwindow* window, int width, int height)
{

    glViewport(0,0,width,height);
}
//###################
//Option to Show FPS
//####################
void Window::showFPS()
{
    static double previousSeconds =0.0;
    static int frameCount = 0;
    double elapsedSeconds;
    double currentSeconds = glfwGetTime(); //returns the numbers of seconds since GLFW started,as a double


    elapsedSeconds = currentSeconds - previousSeconds;

    // limit text update 4 times per seconds
    if(elapsedSeconds >0.25)
    {
        previousSeconds = currentSeconds;
        double fps = (double)frameCount / elapsedSeconds;
        double msPerFrame = 1000.0 / fps;

        std::ostringstream outs;
        outs.precision(3);
        outs << std::fixed << _title.c_str() << "                "
            << "FPS: " << fps << "    "
            << "Frame Time: " << msPerFrame << " (ms)";
        glfwSetWindowTitle(_window,outs.str().c_str());

        frameCount = 0;
    }

    frameCount++;
}


void Window::Update(double elapsedTime)
{
    //Camera orientation
    double mouseX,mouseY;
    
    //Get current mouse position
    glfwGetCursorPos(_window, &mouseX, &mouseY);
    
    //Rotate camera the difference in mouse distance from center screen
    _camData.camera.rotate(static_cast<float>(_width /2.0 - mouseX) * _camData.MOUSE_SENSITIVITY, static_cast<float>(_height /2.0 - mouseY) * _camData.MOUSE_SENSITIVITY);

    //clamp mouse cursor to the center of the screen

    glfwSetCursorPos (_window, _width /2.0, _height/2.0);

    //#######################
    //Camera movement
    //####################

    //Forward/Backward
    if(glfwGetKey(_window,GLFW_KEY_W) == GLFW_PRESS)
        _camData.camera.move(_camData.speed * static_cast<float>(elapsedTime) * _camData.camera.getLook());
    else if(glfwGetKey(_window,GLFW_KEY_S) == GLFW_PRESS)
        _camData.camera.move(_camData.speed * static_cast<float>(elapsedTime) * -_camData.camera.getLook());

    //Left/Right
    if(glfwGetKey(_window,GLFW_KEY_A) == GLFW_PRESS)
        _camData.camera.move(_camData.speed * static_cast<float>(elapsedTime) * -_camData.camera.getRight());
    else if(glfwGetKey(_window,GLFW_KEY_D) == GLFW_PRESS)
        _camData.camera.move(_camData.speed * static_cast<float>(elapsedTime) * _camData.camera.getRight());

    //Up/Down
    if(glfwGetKey(_window,GLFW_KEY_E) == GLFW_PRESS)
        _camData.camera.move(_camData.speed * static_cast<float>(elapsedTime) * _camData.camera.getUp());
    else if(glfwGetKey(_window,GLFW_KEY_Q) == GLFW_PRESS)
        _camData.camera.move(_camData.speed * static_cast<float>(elapsedTime) * -_camData.camera.getUp());




}
