#include "renderer3d.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <cmath>
#include <iostream>
#include <vector>

namespace {
constexpr float kPi = 3.14159265358979323846f;

const char* kVertSrc = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProj;
out vec3 vWorldNormal;
out vec3 vWorldPos;
void main() {
    vec4 worldPos = uModel * vec4(aPos, 1.0);
    vWorldPos    = worldPos.xyz;
    vWorldNormal = mat3(uModel) * aNormal;
    gl_Position  = uProj * uView * worldPos;
}
)";

const char* kFragSrc = R"(
#version 330 core
in vec3 vWorldNormal;
in vec3 vWorldPos;
uniform vec3 uColor;
uniform vec3 uLightDir;
uniform bool uUseLighting;
out vec4 FragColor;
void main() {
    if (uUseLighting) {
        vec3 N = normalize(vWorldNormal);
        vec3 L = normalize(-uLightDir);
        float diff = max(dot(N, L), 0.0);
        vec3 ambient = 0.28 * uColor;
        vec3 diffuse = 0.72 * diff * uColor;
        FragColor = vec4(ambient + diffuse, 1.0);
    } else {
        FragColor = vec4(uColor, 1.0);
    }
}
)";

GLuint compile(GLenum type, const char* src) {
    GLuint sh = glCreateShader(type);
    glShaderSource(sh, 1, &src, nullptr);
    glCompileShader(sh);
    GLint ok = GL_FALSE;
    glGetShaderiv(sh, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetShaderInfoLog(sh, sizeof(log), nullptr, log);
        std::cerr << "Renderer3D shader compile: " << log << '\n';
    }
    return sh;
}

GLuint link() {
    GLuint vs = compile(GL_VERTEX_SHADER,   kVertSrc);
    GLuint fs = compile(GL_FRAGMENT_SHADER, kFragSrc);
    GLuint p  = glCreateProgram();
    glAttachShader(p, vs);
    glAttachShader(p, fs);
    glLinkProgram(p);
    GLint ok = GL_FALSE;
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetProgramInfoLog(p, sizeof(log), nullptr, log);
        std::cerr << "Renderer3D program link: " << log << '\n';
    }
    glDeleteShader(vs);
    glDeleteShader(fs);
    return p;
}

GLuint makeMesh(const std::vector<float>& verts,
                const std::vector<unsigned int>& idx,
                GLuint& vbo, GLuint& ebo) {
    GLuint vao;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ebo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(verts.size() * sizeof(float)),
                 verts.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(idx.size() * sizeof(unsigned int)),
                 idx.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          reinterpret_cast<void*>(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);
    return vao;
}

// Build the model matrix that places a Z-axis-aligned unit cylinder so that its
// base (z=0) sits at `a` and tip (z=1) sits at `b`, with given radius.
glm::mat4 cylinderModel(glm::vec3 a, glm::vec3 b, float radius) {
    glm::vec3 d = b - a;
    const float length = glm::length(d);
    if (length < 1e-6f) return glm::mat4(0.0f);
    d /= length;

    // pick a stable arbitrary axis not parallel to d
    const glm::vec3 arbitrary = (std::abs(d.z) < 0.95f)
        ? glm::vec3(0.0f, 0.0f, 1.0f)
        : glm::vec3(1.0f, 0.0f, 0.0f);
    const glm::vec3 u = glm::normalize(glm::cross(arbitrary, d));
    const glm::vec3 v = glm::cross(d, u);

    glm::mat4 m(1.0f);
    m[0] = glm::vec4(u * radius, 0.0f);
    m[1] = glm::vec4(v * radius, 0.0f);
    m[2] = glm::vec4(d * length, 0.0f);
    m[3] = glm::vec4(a, 1.0f);
    return m;
}

} // namespace

Renderer3D::Renderer3D() {
    m_program     = link();
    m_locModel    = glGetUniformLocation(m_program, "uModel");
    m_locView     = glGetUniformLocation(m_program, "uView");
    m_locProj     = glGetUniformLocation(m_program, "uProj");
    m_locColor    = glGetUniformLocation(m_program, "uColor");
    m_locLightDir = glGetUniformLocation(m_program, "uLightDir");
    m_locUseLight = glGetUniformLocation(m_program, "uUseLighting");

    createSphere(24, 32);
    createCylinder(32);
    createCube();
    createLineBuffer();
}

Renderer3D::~Renderer3D() {
    auto del = [](GLuint& id, void (*fn)(GLsizei, const GLuint*)) {
        if (id) { fn(1, &id); id = 0; }
    };
    del(m_sphereVBO,   glDeleteBuffers);
    del(m_sphereEBO,   glDeleteBuffers);
    del(m_sphereVAO,   glDeleteVertexArrays);
    del(m_cylinderVBO, glDeleteBuffers);
    del(m_cylinderEBO, glDeleteBuffers);
    del(m_cylinderVAO, glDeleteVertexArrays);
    del(m_cubeVBO,     glDeleteBuffers);
    del(m_cubeEBO,     glDeleteBuffers);
    del(m_cubeVAO,     glDeleteVertexArrays);
    del(m_lineVBO,     glDeleteBuffers);
    del(m_lineVAO,     glDeleteVertexArrays);
    if (m_program) { glDeleteProgram(m_program); m_program = 0; }
}

void Renderer3D::beginFrame(const glm::mat4& view, const glm::mat4& projection, glm::vec3 lightDir) {
    glUseProgram(m_program);
    glUniformMatrix4fv(m_locView, 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(m_locProj, 1, GL_FALSE, glm::value_ptr(projection));
    glUniform3f(m_locLightDir, lightDir.x, lightDir.y, lightDir.z);
}

void Renderer3D::uploadAndDrawMesh(GLuint vao, GLsizei indexCount,
                                    const glm::mat4& model, glm::vec3 color, bool useLighting) {
    glUseProgram(m_program);
    glUniformMatrix4fv(m_locModel, 1, GL_FALSE, glm::value_ptr(model));
    glUniform3f(m_locColor, color.r, color.g, color.b);
    glUniform1i(m_locUseLight, useLighting ? 1 : 0);
    glBindVertexArray(vao);
    glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}

void Renderer3D::drawSphere(glm::vec3 center, float radius, glm::vec3 color) {
    glm::mat4 model = glm::translate(glm::mat4(1.0f), center);
    model = glm::scale(model, glm::vec3(radius));
    uploadAndDrawMesh(m_sphereVAO, m_sphereIndexCount, model, color, true);
}

void Renderer3D::drawCylinder(glm::vec3 a, glm::vec3 b, float radius, glm::vec3 color) {
    glm::mat4 model = cylinderModel(a, b, radius);
    if (model == glm::mat4(0.0f)) return;
    uploadAndDrawMesh(m_cylinderVAO, m_cylinderIndexCount, model, color, true);
}

void Renderer3D::drawCube(glm::vec3 center, glm::vec3 halfExtents, glm::vec3 color) {
    glm::mat4 model = glm::translate(glm::mat4(1.0f), center);
    model = glm::scale(model, halfExtents);
    uploadAndDrawMesh(m_cubeVAO, m_cubeIndexCount, model, color, true);
}

void Renderer3D::drawLine(glm::vec3 a, glm::vec3 b, glm::vec3 color) {
    float verts[] = { a.x, a.y, a.z,  0.0f, 0.0f, 1.0f,
                      b.x, b.y, b.z,  0.0f, 0.0f, 1.0f };
    glUseProgram(m_program);
    glUniformMatrix4fv(m_locModel, 1, GL_FALSE, glm::value_ptr(glm::mat4(1.0f)));
    glUniform3f(m_locColor, color.r, color.g, color.b);
    glUniform1i(m_locUseLight, 0);
    glBindVertexArray(m_lineVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_lineVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_DYNAMIC_DRAW);
    glDrawArrays(GL_LINES, 0, 2);
    glBindVertexArray(0);
}

void Renderer3D::drawGridXY(float halfSize, float spacing, glm::vec3 color) {
    std::vector<float> verts;
    verts.reserve(static_cast<size_t>((halfSize / spacing) * 24));
    auto push = [&](glm::vec3 p) {
        verts.push_back(p.x); verts.push_back(p.y); verts.push_back(p.z);
        verts.push_back(0.0f); verts.push_back(0.0f); verts.push_back(1.0f);
    };
    for (float x = -halfSize; x <= halfSize + 1e-4f; x += spacing) {
        push({ x, -halfSize, 0.0f });
        push({ x,  halfSize, 0.0f });
    }
    for (float y = -halfSize; y <= halfSize + 1e-4f; y += spacing) {
        push({ -halfSize, y, 0.0f });
        push({  halfSize, y, 0.0f });
    }
    if (verts.empty()) return;

    glUseProgram(m_program);
    glUniformMatrix4fv(m_locModel, 1, GL_FALSE, glm::value_ptr(glm::mat4(1.0f)));
    glUniform3f(m_locColor, color.r, color.g, color.b);
    glUniform1i(m_locUseLight, 0);
    glBindVertexArray(m_lineVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_lineVBO);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(verts.size() * sizeof(float)),
                 verts.data(), GL_DYNAMIC_DRAW);
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(verts.size() / 6));
    glBindVertexArray(0);
}

void Renderer3D::drawAxes(float length) {
    drawLine({ 0, 0, 0 }, { length, 0, 0 }, { 0.95f, 0.30f, 0.30f }); // X red
    drawLine({ 0, 0, 0 }, { 0, length, 0 }, { 0.35f, 0.85f, 0.35f }); // Y green
    drawLine({ 0, 0, 0 }, { 0, 0, length }, { 0.40f, 0.55f, 0.95f }); // Z blue
}

void Renderer3D::createSphere(int latitudes, int longitudes) {
    std::vector<float> verts;
    std::vector<unsigned int> idx;

    for (int la = 0; la <= latitudes; ++la) {
        const float v = static_cast<float>(la) / latitudes; // 0 -> 1
        const float phi = v * kPi;                            // 0 -> pi (angle from +Z)
        const float cp = std::cos(phi), sp = std::sin(phi);
        for (int lo = 0; lo <= longitudes; ++lo) {
            const float u = static_cast<float>(lo) / longitudes;
            const float theta = u * 2.0f * kPi;
            const float ct = std::cos(theta), st = std::sin(theta);
            // unit sphere in +Z up convention
            const float x = sp * ct;
            const float y = sp * st;
            const float z = cp;
            // pos
            verts.push_back(x); verts.push_back(y); verts.push_back(z);
            // normal == pos for unit sphere
            verts.push_back(x); verts.push_back(y); verts.push_back(z);
        }
    }
    const int stride = longitudes + 1;
    for (int la = 0; la < latitudes; ++la) {
        for (int lo = 0; lo < longitudes; ++lo) {
            const unsigned int i0 = la * stride + lo;
            const unsigned int i1 = i0 + 1;
            const unsigned int i2 = i0 + stride;
            const unsigned int i3 = i2 + 1;
            idx.insert(idx.end(), { i0, i2, i1, i1, i2, i3 });
        }
    }
    m_sphereVAO = makeMesh(verts, idx, m_sphereVBO, m_sphereEBO);
    m_sphereIndexCount = static_cast<GLsizei>(idx.size());
}

void Renderer3D::createCylinder(int sides) {
    // Cylinder along +Z, from z=0 to z=1, radius 1.
    std::vector<float> verts;
    std::vector<unsigned int> idx;

    // Side ring at z=0 and z=1, with side normals (radial).
    for (int i = 0; i <= sides; ++i) {
        const float t = static_cast<float>(i) / sides;
        const float a = t * 2.0f * kPi;
        const float ca = std::cos(a), sa = std::sin(a);
        // z=0 vert
        verts.push_back(ca); verts.push_back(sa); verts.push_back(0.0f);
        verts.push_back(ca); verts.push_back(sa); verts.push_back(0.0f);
        // z=1 vert
        verts.push_back(ca); verts.push_back(sa); verts.push_back(1.0f);
        verts.push_back(ca); verts.push_back(sa); verts.push_back(0.0f);
    }
    for (int i = 0; i < sides; ++i) {
        const unsigned int base = i * 2;
        idx.insert(idx.end(), { base, base + 2, base + 1, base + 1, base + 2, base + 3 });
    }

    // Bottom cap (normal -Z): center vertex + ring
    const unsigned int bottomCenterIdx = static_cast<unsigned int>(verts.size() / 6);
    verts.push_back(0.0f); verts.push_back(0.0f); verts.push_back(0.0f);
    verts.push_back(0.0f); verts.push_back(0.0f); verts.push_back(-1.0f);
    for (int i = 0; i <= sides; ++i) {
        const float a = (static_cast<float>(i) / sides) * 2.0f * kPi;
        verts.push_back(std::cos(a)); verts.push_back(std::sin(a)); verts.push_back(0.0f);
        verts.push_back(0.0f);        verts.push_back(0.0f);        verts.push_back(-1.0f);
    }
    for (int i = 0; i < sides; ++i) {
        const unsigned int r0 = bottomCenterIdx + 1 + i;
        const unsigned int r1 = bottomCenterIdx + 1 + i + 1;
        idx.insert(idx.end(), { bottomCenterIdx, r1, r0 });
    }

    // Top cap (normal +Z): center vertex + ring
    const unsigned int topCenterIdx = static_cast<unsigned int>(verts.size() / 6);
    verts.push_back(0.0f); verts.push_back(0.0f); verts.push_back(1.0f);
    verts.push_back(0.0f); verts.push_back(0.0f); verts.push_back(1.0f);
    for (int i = 0; i <= sides; ++i) {
        const float a = (static_cast<float>(i) / sides) * 2.0f * kPi;
        verts.push_back(std::cos(a)); verts.push_back(std::sin(a)); verts.push_back(1.0f);
        verts.push_back(0.0f);        verts.push_back(0.0f);        verts.push_back(1.0f);
    }
    for (int i = 0; i < sides; ++i) {
        const unsigned int r0 = topCenterIdx + 1 + i;
        const unsigned int r1 = topCenterIdx + 1 + i + 1;
        idx.insert(idx.end(), { topCenterIdx, r0, r1 });
    }

    m_cylinderVAO = makeMesh(verts, idx, m_cylinderVBO, m_cylinderEBO);
    m_cylinderIndexCount = static_cast<GLsizei>(idx.size());
}

void Renderer3D::createCube() {
    // 6 faces, 4 vertices each (24 verts) with face normals so lighting is per-face.
    static const float faces[][6] = {
        // pos, normal
        // +X
        { 1,-1,-1,  1, 0, 0}, { 1, 1,-1,  1, 0, 0}, { 1, 1, 1,  1, 0, 0}, { 1,-1, 1,  1, 0, 0},
        // -X
        {-1,-1,-1, -1, 0, 0}, {-1,-1, 1, -1, 0, 0}, {-1, 1, 1, -1, 0, 0}, {-1, 1,-1, -1, 0, 0},
        // +Y
        {-1, 1,-1,  0, 1, 0}, {-1, 1, 1,  0, 1, 0}, { 1, 1, 1,  0, 1, 0}, { 1, 1,-1,  0, 1, 0},
        // -Y
        {-1,-1,-1,  0,-1, 0}, { 1,-1,-1,  0,-1, 0}, { 1,-1, 1,  0,-1, 0}, {-1,-1, 1,  0,-1, 0},
        // +Z
        {-1,-1, 1,  0, 0, 1}, { 1,-1, 1,  0, 0, 1}, { 1, 1, 1,  0, 0, 1}, {-1, 1, 1,  0, 0, 1},
        // -Z
        {-1,-1,-1,  0, 0,-1}, {-1, 1,-1,  0, 0,-1}, { 1, 1,-1,  0, 0,-1}, { 1,-1,-1,  0, 0,-1},
    };
    std::vector<float> verts;
    for (auto& f : faces) {
        for (float v : f) verts.push_back(v);
    }
    std::vector<unsigned int> idx;
    for (unsigned int face = 0; face < 6; ++face) {
        const unsigned int base = face * 4;
        idx.insert(idx.end(), { base, base + 1, base + 2, base, base + 2, base + 3 });
    }
    m_cubeVAO = makeMesh(verts, idx, m_cubeVBO, m_cubeEBO);
    m_cubeIndexCount = static_cast<GLsizei>(idx.size());
}

void Renderer3D::createLineBuffer() {
    glGenVertexArrays(1, &m_lineVAO);
    glGenBuffers(1, &m_lineVBO);
    glBindVertexArray(m_lineVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_lineVBO);
    glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          reinterpret_cast<void*>(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);
}
