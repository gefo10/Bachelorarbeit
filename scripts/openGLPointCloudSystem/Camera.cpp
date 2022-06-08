#include "Camera.h"
#include "glm/gtx/transform.hpp"

const float FOV = 60.0f;

Camera::Camera()
    :mPosition(glm::vec3(0.0f,0.0f,0.0f)),
    mTargetPos(glm::vec3(0.0f,0.0f,0.0f)),
    mUp(0.0f,1.0f,0.0f),
    mRight(0.0f,0.0f,0.0f),
    WORLD_UP(0.0f,1.0f,0.0f),
    mYaw(glm::pi<float>()),
    mPitch(0.0f),
    mFOV(FOV)
{

}

glm::mat4 Camera::getViewMatrix() const 
{
    return glm::lookAt(mPosition,mTargetPos,mUp);
}

const glm::vec3& Camera::getLook() const 
{
    return mLook;
}

const glm::vec3& Camera::getRight() const
{
    return mRight;
}

const glm::vec3& Camera::getUp() const
{
    return mUp;
}



//------------------------
// OrbitCamera
//-------------------------
OrbitCamera::OrbitCamera()
    :mRadius(10.0f)
{}

void OrbitCamera::setLookAt(const glm::vec3& target)
{
    mTargetPos = target;
}

void OrbitCamera::setRadius(float radius)
{
    mRadius = glm::clamp(radius,1.0f,80.0f);
}

void OrbitCamera::rotate(float yaw,float pitch)
{
    mYaw = glm::radians(yaw);
    mPitch = glm::radians(pitch);

    mPitch = glm::clamp(mPitch, -glm::pi<float>() / 2.0f + 0.1f, glm::pi<float>() /2.0f - 0.1f);

    updateCameraVectors();
}

void OrbitCamera::updateCameraVectors()
{
    //Spherical to Cartesian coordinates

    mPosition.x = mTargetPos.x + mRadius * cosf(mPitch) * sinf(mYaw);
    mPosition.y = mTargetPos.y + mRadius * sinf(mPitch);
    mPosition.z = mTargetPos.z + mRadius * cosf(mPitch) * cosf(mYaw);
}

//-----------------------------
//FPSCamera
//--------------------------

FPSCamera::FPSCamera(glm::vec3 position, float yaw, float pitch)
{
    mPosition = position;
    mYaw = yaw;
    mPitch = pitch;
}

void FPSCamera::setPosition(const glm::vec3& position)
{
    mPosition = position;
}

void FPSCamera::move (const glm::vec3& offsetPos)
{
    mPosition += offsetPos;
    updateCameraVectors();
}

void FPSCamera::rotate(float yaw, float pitch)
{
    mYaw += glm::radians(yaw);
    mPitch += glm::radians(pitch);

    //constrain pitch
    mPitch = glm::clamp (mPitch, -glm::pi<float>() /2.0f + 0.1f, glm::pi<float>()/2.0f - 0.1f);
    updateCameraVectors();

}

void FPSCamera::updateCameraVectors()
{
    //Spherical to cartesian coordinates

    //Calculate view direction vector based on yaw and pitch angles

    glm::vec3 look;
    look.x = cosf(mPitch) * sinf(mYaw);
    look.y = sinf(mPitch);
    look.z = cosf(mPitch) * cosf(mYaw);

    mLook = glm::normalize(look);

    //recalculate the right and up vector
    mRight = glm::normalize(glm::cross(mLook,WORLD_UP));
    mUp = glm::normalize(glm::cross(mRight,mLook));

    mTargetPos = mPosition + mLook;
}
