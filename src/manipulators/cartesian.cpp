#include "cartesian.h"

#include "../renderer3d.h"

ManipulatorCartesian::ManipulatorCartesian(float xMax, float yMax, float zMax)
    : m_joints{ 0.4f, 0.4f, 0.4f }
    , m_specs{
        { /*isPrismatic*/ true, 0.0f, xMax, "d1 (X)", "u" },
        { /*isPrismatic*/ true, 0.0f, yMax, "d2 (Y)", "u" },
        { /*isPrismatic*/ true, 0.0f, zMax, "d3 (Z)", "u" },
      }
{}

std::vector<LinkParam> ManipulatorCartesian::linkParams() {
    return {
        { "X max", &m_specs[0].maxValue, 0.5f, 4.0f },
        { "Y max", &m_specs[1].maxValue, 0.5f, 4.0f },
        { "Z max", &m_specs[2].maxValue, 0.5f, 4.0f },
    };
}

std::vector<glm::vec3> ManipulatorCartesian::jointPositions() const {
    const float d1 = m_joints[0];
    const float d2 = m_joints[1];
    const float d3 = m_joints[2];
    return {
        { 0.0f, 0.0f, 0.0f },   // base
        { d1,   0.0f, 0.0f },   // after d1 (along +X)
        { d1,   d2,   0.0f },   // after d2 (along +Y)
        { d1,   d2,   d3   },   // efetuador (after d3 along +Z)
    };
}

bool ManipulatorCartesian::inverseKinematics(glm::vec3 target,
                                              std::vector<float>& outJoints,
                                              std::string& outReason) const {
    const float d1Max = m_specs[0].maxValue;
    const float d2Max = m_specs[1].maxValue;
    const float d3Max = m_specs[2].maxValue;
    const float d1Min = m_specs[0].minValue;
    const float d2Min = m_specs[1].minValue;
    const float d3Min = m_specs[2].minValue;

    if (target.x < d1Min - 1e-5f || target.x > d1Max + 1e-5f) {
        outReason = "X fora do range";
        return false;
    }
    if (target.y < d2Min - 1e-5f || target.y > d2Max + 1e-5f) {
        outReason = "Y fora do range";
        return false;
    }
    if (target.z < d3Min - 1e-5f || target.z > d3Max + 1e-5f) {
        outReason = "Z fora do range";
        return false;
    }

    outJoints = { target.x, target.y, target.z };
    outReason.clear();
    return true;
}

void ManipulatorCartesian::draw(Renderer3D& r) const {
    const auto pts = jointPositions();
    const glm::vec3 rail   { 0.42f, 0.45f, 0.52f };
    const glm::vec3 rod    { 0.80f, 0.82f, 0.88f };
    const glm::vec3 joint  { 0.92f, 0.62f, 0.20f };
    const glm::vec3 base   { 0.85f, 0.55f, 0.20f };
    const glm::vec3 effCol { 0.95f, 0.40f, 0.55f };

    // Workspace bounding box (transparent-ish via wireframe lines) — corner at origin to (xmax, ymax, zmax)
    const float xMax = m_specs[0].maxValue;
    const float yMax = m_specs[1].maxValue;
    const float zMax = m_specs[2].maxValue;
    const glm::vec3 boxCol{ 0.28f, 0.45f, 0.55f };
    glm::vec3 c000{ 0,    0,    0    }, c100{ xMax, 0,    0    };
    glm::vec3 c010{ 0,    yMax, 0    }, c110{ xMax, yMax, 0    };
    glm::vec3 c001{ 0,    0,    zMax }, c101{ xMax, 0,    zMax };
    glm::vec3 c011{ 0,    yMax, zMax }, c111{ xMax, yMax, zMax };
    auto edge = [&](glm::vec3 a, glm::vec3 b){ r.drawLine(a, b, boxCol); };
    edge(c000,c100); edge(c100,c110); edge(c110,c010); edge(c010,c000);  // bottom
    edge(c001,c101); edge(c101,c111); edge(c111,c011); edge(c011,c001);  // top
    edge(c000,c001); edge(c100,c101); edge(c110,c111); edge(c010,c011);  // verticals

    // Rails: each prismatic shown as a thicker shell + thinner rod (cylinder pair)
    // segment 0 -> 1: d1 along +X
    r.drawCylinder(pts[0], pts[1], 0.045f, rail);
    r.drawCylinder(pts[0], pts[1], 0.020f, rod);
    // segment 1 -> 2: d2 along +Y
    r.drawCylinder(pts[1], pts[2], 0.045f, rail);
    r.drawCylinder(pts[1], pts[2], 0.020f, rod);
    // segment 2 -> 3: d3 along +Z
    r.drawCylinder(pts[2], pts[3], 0.045f, rail);
    r.drawCylinder(pts[2], pts[3], 0.020f, rod);

    // Joints as small cubes (prismatic indicator)
    r.drawCube(pts[1], glm::vec3(0.05f), joint);
    r.drawCube(pts[2], glm::vec3(0.05f), joint);

    // Base sphere
    r.drawSphere(pts[0], 0.08f, base);
    // End-effector
    r.drawSphere(pts[3], 0.07f, effCol);
}
