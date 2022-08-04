#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

class Camera
{
public:

    glm::mat4 getViewMatrix() const;

    virtual void setPosition(const glm::vec3& position){};
    virtual void rotate(float yaw, float pitch){}; //in degrees
    virtual void move(const glm::vec3& offsetPos){};

    const glm::vec3& getLook() const;
    const glm::vec3& getRight() const;
    const glm::vec3& getUp() const;

    float getFOV() const {return mFOV;};
    void setFOV(float fov) {mFOV = fov;};

protected:
    Camera();

    glm::vec3 mPosition;
    glm::vec3 mTargetPos;
    glm::vec3 mUp;
    
    glm::vec3 mLook;
    glm::vec3 mRight;
    const glm::vec3 WORLD_UP;


    //Euler Angles (in radians)
    float mYaw;
    float mPitch;

    //field of view (for projection matrix)
    float mFOV; //degrees
};


class OrbitCamera : public Camera
{
public:
    OrbitCamera();

    virtual void rotate(float yaw, float pitch); //in degrees 

    void setLookAt(const glm::vec3& target);
    void setRadius(float radius);

    float GetYaw() {return mYaw;};
    float GetPitch() {return mPitch;};

private:

    void updateCameraVectors();
    float mRadius;


};

class FPSCamera : public  Camera
{
public:

    FPSCamera(glm::vec3 position = glm::vec3(0.0f,0.0f,0.0f), float yaw = glm::pi<float>(), float pitch =0.0f); //(yaw) initial angle faces -Z

    virtual void setPosition(const glm::vec3& position);
    virtual void rotate(float yaw, float pitch);
    virtual void move(const glm::vec3& offsetPos);

private:

    void updateCameraVectors();

};
