#pragma once

#include <glm/glm.hpp>

// Orbit camera: always points at `target`, parameterized by yaw (azimuth),
// pitch (elevation), and distance. Mouse drag rotates; scroll zooms.
class Camera3D {
public:
    Camera3D();

    void updateAspect(int framebufferWidth, int framebufferHeight);

    // dx, dy in pixels. Positive dx rotates camera to the right; positive dy tilts up.
    void orbit(float dx, float dy);

    // factor > 1 zooms out; factor < 1 zooms in.
    void zoom(float factor);

    glm::mat4 view() const;
    glm::mat4 projection() const;
    glm::vec3 eye() const;

    void setTarget(glm::vec3 t)   { m_target = t; }
    void setDistance(float d)     { m_distance = d; }

    float yaw()      const { return m_yaw; }
    float pitch()    const { return m_pitch; }
    float distance() const { return m_distance; }

private:
    glm::vec3 m_target{ 0.0f, 0.0f, 0.0f };
    float m_yaw       = 0.6f;        // around Y (azimuth), radians
    float m_pitch     = 0.5f;        // around X (elevation), radians
    float m_distance  = 6.0f;
    float m_aspect    = 1.0f;
    float m_fovYDeg   = 50.0f;
    float m_near      = 0.05f;
    float m_far       = 200.0f;
};
