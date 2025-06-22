#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

class ArcBallCamera {
public:
    ArcBallCamera(glm::vec3 target = glm::vec3(0.0f), float dist = 3.0f);

    void rotate(float deltaYaw, float deltaPitch);
    void zoom(float delta);
    void roll(float delta);
    void setViewport(float width, float height);

    glm::mat4 getViewMatrix() const;
    glm::mat4 getProjectionMatrix() const;

private:
    glm::vec3 target;
    float distance;
    float yaw;
    float pitch;
    float rollAngle;
    float aspect;
    float fov = glm::radians(45.0f);
};
