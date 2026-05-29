#include "cartesian.h"

#include "../renderer3d.h"

ManipulatorCartesian::ManipulatorCartesian(float d1Min, float d1Max,
                                            float d2Min, float d2Max,
                                            float d3Min, float d3Max)
    : m_joints{ (d1Min + d1Max) * 0.5f,
                (d2Min + d2Max) * 0.5f,
                (d3Min + d3Max) * 0.5f }
    , m_specs{
        { /*isPrismatic*/ true, d1Min, d1Max, "d1 (X)", "u" },
        { /*isPrismatic*/ true, d2Min, d2Max, "d2 (Y)", "u" },
        { /*isPrismatic*/ true, d3Min, d3Max, "d3 (Z)", "u" },
      }
{}

std::vector<LinkParam> ManipulatorCartesian::linkParams() {
    return {
        { "d1 (X) min", &m_specs[0].minValue, 0.0f, 1.0f },
        { "d1 (X) max", &m_specs[0].maxValue, 0.5f, 4.0f },
        { "d2 (Y) min", &m_specs[1].minValue, 0.0f, 1.0f },
        { "d2 (Y) max", &m_specs[1].maxValue, 0.5f, 4.0f },
        { "d3 (Z) min", &m_specs[2].minValue, 0.0f, 1.0f },
        { "d3 (Z) max", &m_specs[2].maxValue, 0.5f, 4.0f },
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

    const float d1 = m_joints[0];
    const float d2 = m_joints[1];
    const float d3 = m_joints[2];

    const float d1Min = m_specs[0].minValue, d1Max = m_specs[0].maxValue;
    const float d2Min = m_specs[1].minValue, d2Max = m_specs[1].maxValue;
    const float d3Min = m_specs[2].minValue, d3Max = m_specs[2].maxValue;

    // ---- palette (consistente com o robo polar) ----
    const glm::vec3 pedestalCol { 0.30f, 0.32f, 0.36f };
    const glm::vec3 railX       { 0.75f, 0.30f, 0.30f };
    const glm::vec3 railY       { 0.30f, 0.70f, 0.35f };
    const glm::vec3 railZ       { 0.35f, 0.45f, 0.80f };
    const glm::vec3 sledCol     { 0.92f, 0.62f, 0.20f };
    const glm::vec3 sleeveCol   { 0.55f, 0.58f, 0.63f };
    const glm::vec3 rodCol      { 0.75f, 0.77f, 0.82f };
    const glm::vec3 effCol      { 0.85f, 0.30f, 0.30f };
    const glm::vec3 boxCol      { 0.28f, 0.45f, 0.55f };
    const glm::vec3 tickCol     { 0.50f, 0.52f, 0.58f };

    // ============================================================
    // (1) Base pedestal at the origin - flat puck, does not move
    // ============================================================
    const float pedestalR = 0.20f;
    const float pedestalH = 0.04f;
    r.drawCylinder({ 0.0f, 0.0f, 0.0f },
                   { 0.0f, 0.0f, pedestalH },
                   pedestalR, pedestalCol);

    // ============================================================
    // (2) Three fixed perpendicular rails (X, Y, Z) from min to max
    // ============================================================
    const float railR = 0.05f;
    const float tickR = 0.025f;

    r.drawCylinder({ d1Min, 0.0f, 0.0f }, { d1Max, 0.0f, 0.0f }, railR, railX);
    r.drawSphere ({ d1Min, 0.0f, 0.0f }, tickR, tickCol);
    r.drawSphere ({ d1Max, 0.0f, 0.0f }, tickR, tickCol);

    r.drawCylinder({ 0.0f, d2Min, 0.0f }, { 0.0f, d2Max, 0.0f }, railR, railY);
    r.drawSphere ({ 0.0f, d2Min, 0.0f }, tickR, tickCol);
    r.drawSphere ({ 0.0f, d2Max, 0.0f }, tickR, tickCol);

    r.drawCylinder({ 0.0f, 0.0f, d3Min }, { 0.0f, 0.0f, d3Max }, railR, railZ);
    r.drawSphere ({ 0.0f, 0.0f, d3Min }, tickR, tickCol);
    r.drawSphere ({ 0.0f, 0.0f, d3Max }, tickR, tickCol);

    // ============================================================
    // (3) Three sleds - small orange cubes marking the current
    //     position of each prismatic joint along its rail
    // ============================================================
    const glm::vec3 sledHalf{ 0.07f, 0.07f, 0.07f };
    r.drawCube({ d1,   0.0f, 0.0f }, sledHalf, sledCol);
    r.drawCube({ 0.0f, d2,   0.0f }, sledHalf, sledCol);
    r.drawCube({ 0.0f, 0.0f, d3   }, sledHalf, sledCol);

    // ============================================================
    // (4) Kinematic chain (telescopic-style sleeve+rod) connecting
    //     the joint positions: base -> after d1 -> after d2 -> eff
    // ============================================================
    const float linkSleeveR = 0.040f;
    const float linkRodR    = 0.018f;
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        r.drawCylinder(pts[i], pts[i + 1], linkSleeveR, sleeveCol);
        r.drawCylinder(pts[i], pts[i + 1], linkRodR,    rodCol);
    }

    // ============================================================
    // (5) End effector
    // ============================================================
    r.drawSphere(pts.back(), 0.07f, effCol);

    // ============================================================
    // (6) Workspace bounding box (wireframe) from (d_min) to (d_max)
    // ============================================================
    const glm::vec3 c000{ d1Min, d2Min, d3Min }, c100{ d1Max, d2Min, d3Min };
    const glm::vec3 c010{ d1Min, d2Max, d3Min }, c110{ d1Max, d2Max, d3Min };
    const glm::vec3 c001{ d1Min, d2Min, d3Max }, c101{ d1Max, d2Min, d3Max };
    const glm::vec3 c011{ d1Min, d2Max, d3Max }, c111{ d1Max, d2Max, d3Max };
    auto edge = [&](glm::vec3 a, glm::vec3 b){ r.drawLine(a, b, boxCol); };
    edge(c000, c100); edge(c100, c110); edge(c110, c010); edge(c010, c000);  // bottom
    edge(c001, c101); edge(c101, c111); edge(c111, c011); edge(c011, c001);  // top
    edge(c000, c001); edge(c100, c101); edge(c110, c111); edge(c010, c011);  // verticals
}
