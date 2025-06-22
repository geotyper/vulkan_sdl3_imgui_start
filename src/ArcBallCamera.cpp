#include "ArcBallCamera.h"

ArcBallCamera::ArcBallCamera(glm::vec3 target, float dist)
    : target(target), distance(dist), yaw(0.0f), pitch(0.0f), rollAngle(0.0f), aspect(1.0f) {}

void ArcBallCamera::rotate(float deltaYaw, float deltaPitch) {
    yaw += deltaYaw;
    pitch += deltaPitch;
    pitch = glm::clamp(pitch, -glm::half_pi<float>() + 0.01f, glm::half_pi<float>() - 0.01f);
}

void ArcBallCamera::zoom(float delta) {
    distance += delta;
    distance = glm::max(0.1f, distance);
}

void ArcBallCamera::roll(float delta) {
    rollAngle += delta;
}

void ArcBallCamera::setViewport(float width, float height) {
    aspect = width / height;
}

glm::mat4 ArcBallCamera::getViewMatrix() const {
    glm::vec3 pos = target + glm::vec3(
                        distance * cos(pitch) * sin(yaw),
                        distance * sin(pitch),
                        distance * cos(pitch) * cos(yaw)
                        );

    glm::mat4 view = glm::lookAt(pos, target, glm::vec3(0, 1, 0));
    glm::mat4 rollMat = glm::rotate(glm::mat4(1.0f), rollAngle, glm::vec3(0, 0, 1));
    return rollMat * view;
}

glm::mat4 ArcBallCamera::getProjectionMatrix() const {
    glm::mat4 proj = glm::perspective(fov, aspect, 0.1f, 100.0f);
    proj[1][1] *= -1; // for Vulkan
    return proj;
}
