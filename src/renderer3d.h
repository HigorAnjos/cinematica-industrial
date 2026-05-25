#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>

class Renderer3D {
public:
    Renderer3D();
    ~Renderer3D();

    Renderer3D(const Renderer3D&)            = delete;
    Renderer3D& operator=(const Renderer3D&) = delete;

    void beginFrame(const glm::mat4& view,
                     const glm::mat4& projection,
                     glm::vec3 lightDir = glm::vec3{ -0.4f, -0.6f, -0.7f });

    void drawSphere(glm::vec3 center, float radius, glm::vec3 color);

    // Cylinder spanning from `a` to `b`, with the given radius.
    void drawCylinder(glm::vec3 a, glm::vec3 b, float radius, glm::vec3 color);

    // Axis-aligned box centered at `center` with half-extents `halfExtents`.
    void drawCube(glm::vec3 center, glm::vec3 halfExtents, glm::vec3 color);

    // Grid drawn on the XY plane (Z=0).
    void drawGridXY(float halfSize, float spacing, glm::vec3 color);

    // X (red), Y (green), Z (blue) axes drawn as colored line segments from origin.
    void drawAxes(float length);

    // Plain line (no lighting) between two points.
    void drawLine(glm::vec3 a, glm::vec3 b, glm::vec3 color);

private:
    GLuint m_program       = 0;
    GLint  m_locModel      = -1;
    GLint  m_locView       = -1;
    GLint  m_locProj       = -1;
    GLint  m_locColor      = -1;
    GLint  m_locLightDir   = -1;
    GLint  m_locUseLight   = -1;

    // Cached primitives
    GLuint m_sphereVAO = 0, m_sphereVBO = 0, m_sphereEBO = 0;
    GLsizei m_sphereIndexCount = 0;

    GLuint m_cylinderVAO = 0, m_cylinderVBO = 0, m_cylinderEBO = 0;
    GLsizei m_cylinderIndexCount = 0;

    GLuint m_cubeVAO = 0, m_cubeVBO = 0, m_cubeEBO = 0;
    GLsizei m_cubeIndexCount = 0;

    // Dynamic line VBO for grid/axes/lines
    GLuint m_lineVAO = 0, m_lineVBO = 0;

    void createSphere(int latitudes, int longitudes);
    void createCylinder(int sides);
    void createCube();
    void createLineBuffer();

    void uploadAndDrawMesh(GLuint vao, GLsizei indexCount,
                            const glm::mat4& model, glm::vec3 color, bool useLighting);
};
