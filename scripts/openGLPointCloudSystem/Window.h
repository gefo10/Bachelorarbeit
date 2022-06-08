#pragma once
#include <GL/glew.h>
#include "GLFW/glfw3.h"

#include <iostream>
#include <string>
#include "Camera.h"
#include <map>

struct CamData
{
    float       yaw;
    float       pitch;
    float       radius;
    float MOUSE_SENSITIVITY;
    double ZOOM_SENSITIVITY;
    //OrbitCamera camera;

    FPSCamera camera;
    float speed;
    
    int* width;
    int* height;
    glm::mat4 projection; 
    unsigned int iter = 0;

    void reset() {
        iter = 0;
    }
};

class Window 
{
    public:
       Window(int width, int height, const char* title);
       ~Window();
        
       std::string GetTitle() {return _title;};
       int GetWidth() {return _width;};
       int GetHeight() {return _height;};
       GLFWwindow* GetGLWindow() {return _window;};
        

       //checks GLFW window state if it should close the window (used for main loop)
       bool shouldClose() {return glfwWindowShouldClose(_window);};
       
       
      //static void OnFrameBufferSize(GLFWwindow* window,int width, int height);
      //check Poll Events/Callback Functions 
      void pollEvents() {glfwPollEvents();};

      //used for double buffering (2 canvaces - front and back buffer)
      void swapBuffers() {glfwSwapBuffers(_window);};
      
      //static void OnMouseMove(GLFWwindow* window, double posX, double posY);
      
      void clearScreen() { glClearColor(0.20f,0.30f,0.40f,1.0f); glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);};

      void swapInterval(int interval) { glfwSwapInterval(interval); };

      //static void OnKey_Event(GLFWwindow* window,int key,int scancode,int action,int mode);

      void showFPS();

      void setCameraLookAt(glm::vec3 target);

      CamData&     GetCamData() {return _camData;};
      //OrbitCamera& GetCamera() {return _camData.camera;};
        
      FPSCamera& GetCamera() {return _camData.camera;};
      glm::mat4 GetProjectionMatrix() { return _camData.projection;};
    
      void Update(double elapsedTime);
      
      //static void SetupCamera() {_yaw = 0.0f; _pitch = 0.0f; _radius = 10.0f; MOUSE_SENSITIVITY = 0.25f; };
    private:
        static void OnFrameBufferSize(GLFWwindow* window,int width, int height);
        static void OnMouseMove(GLFWwindow* window, double posX, double posY);
        static void OnKey_Event(GLFWwindow* window,int key,int scancode,int action,int mode);
        static void OnWindowSize(GLFWwindow* window,int width, int height);
        static void OnMouseScroll(GLFWwindow* window,double deltaX, double deltaY);
        
        GLFWwindow* _window;
        std::string _title;
        int _width;
        int _height;
        bool _fullScreen;

        CamData     _camData;
       // //glm::vec2   lastMousePos;
        
};



