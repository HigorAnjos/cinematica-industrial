#include "camera3d.h"

#include <glm/gtc/matrix_transform.hpp>

#include <cmath>

namespace {
constexpr float kPi   = 3.14159265358979323846f;
constexpr float kHalf = kPi * 0.5f;
}

Camera3D::Camera3D() = default;

void Camera3D::updateAspect(int framebufferWidth, int framebufferHeight) {
    if (framebufferWidth <= 0 || framebufferHeight <= 0) return;
    m_aspect = static_cast<float>(framebufferWidth) / static_cast<float>(framebufferHeight);
}

void Camera3D::orbit(float dx, float dy) {
    constexpr float sensitivity = 0.005f;
    m_yaw   -= dx * sensitivity;
    m_pitch += dy * sensitivity;
    // Clamp pitch to avoid flipping at the poles.
    const float limit = kHalf - 0.01f;
    if (m_pitch >  limit) m_pitch =  limit;
    if (m_pitch < -limit) m_pitch = -limit;
}

void Camera3D::zoom(float factor) {
    m_distance *= factor;
    if (m_distance < 0.5f)   m_distance = 0.5f;
    if (m_distance > 100.0f) m_distance = 100.0f;
}

glm::vec3 Camera3D::eye() const {
    // Spherical coords: pitch is elevation from XY plane; yaw is azimuth around +Z.
    // Using +Z up convention (matches robotics convention: x right, y forward, z up).
    const float cp = std::cos(m_pitch);
    const float sp = std::sin(m_pitch);
    const float cy = std::cos(m_yaw);
    const float sy = std::sin(m_yaw);
    return m_target + glm::vec3{ cp * cy, cp * sy, sp } * m_distance;
}

glm::mat4 Camera3D::view() const {
    return glm::lookAt(eye(), m_target, glm::vec3{ 0.0f, 0.0f, 1.0f });
}

glm::mat4 Camera3D::projection() const {
    return glm::perspective(glm::radians(m_fovYDeg), m_aspect, m_near, m_far);
}
