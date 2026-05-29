#include "polar.h"

#include "../renderer3d.h"

#include <cmath>

namespace {
constexpr float kPi   = 3.14159265358979323846f;
constexpr float kHalf = kPi * 0.5f;
}

ManipulatorPolar::ManipulatorPolar(float d1, float rhoMin, float rhoMax)
    : m_joints{ 0.6f, 0.4f, (rhoMin + rhoMax) * 0.5f }  // theta, phi, rho
    , m_specs{
        { /*isPrismatic*/ false, -kPi,   +kPi,   "theta (azimute)",   "deg" },
        { /*isPrismatic*/ false, -kHalf, +kHalf, "phi (elevacao)",    "deg" },
        { /*isPrismatic*/ true,   rhoMin, rhoMax, "rho (radial)",     "u"   },
      }
    , m_d1{ d1 }
{}

std::vector<LinkParam> ManipulatorPolar::linkParams() {
    return {
        { "d1 (coluna)", &m_d1,               0.10f, 1.50f },
        { "rho min",     &m_specs[2].minValue, 0.0f, 1.5f  },
        { "rho max",     &m_specs[2].maxValue, 0.5f, 4.0f  },
    };
}

std::vector<glm::vec3> ManipulatorPolar::jointPositions() const {
    const float theta = m_joints[0];
    const float phi   = m_joints[1];
    const float rho   = m_joints[2];

    const float cp = std::cos(phi);
    const float sp = std::sin(phi);
    const float ct = std::cos(theta);
    const float st = std::sin(theta);

    const glm::vec3 base    { 0.0f, 0.0f, 0.0f };
    const glm::vec3 shoulder{ 0.0f, 0.0f, m_d1 };
    const glm::vec3 eff     { rho * cp * ct, rho * cp * st, m_d1 + rho * sp };
    return { base, shoulder, eff };
}

bool ManipulatorPolar::inverseKinematics(glm::vec3 target,
                                          std::vector<float>& outJoints,
                                          std::string& outReason) const {
    const float rhoMin = m_specs[2].minValue;
    const float rhoMax = m_specs[2].maxValue;

    // Shift target to the shoulder frame (subtract column height d1 from z)
    const float zr = target.z - m_d1;
    const float rho = std::sqrt(target.x * target.x + target.y * target.y + zr * zr);

    if (rho < rhoMin - 1e-5f) { outReason = "rho abaixo do minimo (alvo perto demais do ombro)"; return false; }
    if (rho > rhoMax + 1e-5f) { outReason = "rho acima do maximo (alvo longe demais do ombro)";  return false; }

    const float theta = std::atan2(target.y, target.x);
    const float xy    = std::sqrt(target.x * target.x + target.y * target.y);
    const float phi   = std::atan2(zr, xy);

    if (phi >  kHalf + 1e-5f || phi < -kHalf - 1e-5f) {
        outReason = "phi fora do range (+/- 90 deg)";
        return false;
    }

    outJoints = { theta, phi, rho };
    outReason.clear();
    return true;
}

void ManipulatorPolar::draw(Renderer3D& r) const {
    const auto pts    = jointPositions();
    const float theta = m_joints[0];
    const float phi   = m_joints[1];
    const float rho   = m_joints[2];

    const glm::vec3 base     = pts[0];
    const glm::vec3 shoulder = pts[1];
    const glm::vec3 effector = pts[2];

    // ---- palette (dark-mode background, grey/silver robot, red motion indicators) ----
    const glm::vec3 pedestalCol { 0.30f, 0.32f, 0.36f };
    const glm::vec3 turntableCol{ 0.45f, 0.47f, 0.52f };
    const glm::vec3 columnCol   { 0.55f, 0.58f, 0.63f };
    const glm::vec3 shoulderCol { 0.50f, 0.53f, 0.58f };
    const glm::vec3 pivotPinCol { 0.40f, 0.42f, 0.47f };
    const glm::vec3 sleeveCol   { 0.55f, 0.58f, 0.63f };
    const glm::vec3 rodCol      { 0.75f, 0.77f, 0.82f };
    const glm::vec3 effCol      { 0.85f, 0.30f, 0.30f };
    const glm::vec3 redInd      { 0.92f, 0.25f, 0.25f };
    const glm::vec3 ringCol     { 0.28f, 0.45f, 0.55f };

    const float ct = std::cos(theta), st = std::sin(theta);
    const float cp = std::cos(phi),   sp = std::sin(phi);

    const float rhoMin = m_specs[2].minValue;
    const float rhoMax = m_specs[2].maxValue;

    // Boom direction unit vector (rotates with theta + phi).
    const glm::vec3 boomDir { cp * ct, cp * st, sp };
    // Lateral vector in the XY plane, perpendicular to the azimuth direction.
    const glm::vec3 lateralDir { -st, ct, 0.0f };

    // ============================================================
    // (1) Floor pedestal (does NOT rotate with theta) - flat puck at z=0
    // ============================================================
    const float pedestalR = 0.30f;
    const float pedestalH = 0.06f;
    r.drawCylinder({ 0.0f, 0.0f, 0.0f },
                   { 0.0f, 0.0f, pedestalH },
                   pedestalR, pedestalCol);

    // ============================================================
    // (2) Turntable (rotates with theta) on top of the pedestal
    //     The visible azimuth comes from the column + shoulder above it.
    // ============================================================
    const float turntableR = 0.22f;
    const float turntableH = 0.04f;
    r.drawCylinder({ 0.0f, 0.0f, pedestalH },
                   { 0.0f, 0.0f, pedestalH + turntableH },
                   turntableR, turntableCol);

    // ============================================================
    // (3) Vertical column of fixed height d1 (rotates with theta)
    //     This is the physical realisation of the d1 fixed link.
    // ============================================================
    const float columnBottom = pedestalH + turntableH;
    const float columnR      = 0.06f;
    r.drawCylinder({ 0.0f, 0.0f, columnBottom },
                   { 0.0f, 0.0f, m_d1 },
                   columnR, columnCol);

    // ============================================================
    // (4) Shoulder housing - small block at the top of the column
    // ============================================================
    r.drawCube(shoulder, { 0.09f, 0.09f, 0.07f }, shoulderCol);

    // ============================================================
    // (5) Phi-axis pin - horizontal axle perpendicular to the boom's
    //     azimuth direction. Rotates with theta. The boom pivots
    //     around this pin when phi changes.
    // ============================================================
    const float pivotHalfLen = 0.13f;
    r.drawCylinder(shoulder - lateralDir * pivotHalfLen,
                   shoulder + lateralDir * pivotHalfLen,
                   0.035f, pivotPinCol);

    // ============================================================
    // (6) Telescopic boom: outer sleeve (fixed = rho_min) + inner rod
    // ============================================================
    const glm::vec3 sleeveEnd = shoulder + boomDir * rhoMin;
    r.drawCylinder(shoulder, sleeveEnd, 0.06f, sleeveCol);

    // Inner rod starts slightly inside the sleeve so it remains visible
    // even at rho = rho_min. It extends to the actual effector position.
    const glm::vec3 rodStart = shoulder + boomDir * (rhoMin * 0.4f);
    r.drawCylinder(rodStart, effector, 0.025f, rodCol);

    // ============================================================
    // (7) End effector
    // ============================================================
    r.drawSphere(effector, 0.07f, effCol);

    // ============================================================
    // (8) Red theta arc on the floor around the pedestal
    // ============================================================
    if (std::abs(theta) > 0.005f) {
        const float arcR    = pedestalR + 0.08f;
        const int   arcSegs = 32;
        const float step    = theta / arcSegs;
        for (int i = 0; i < arcSegs; ++i) {
            const float a0 = step * i;
            const float a1 = step * (i + 1);
            r.drawLine({ arcR * std::cos(a0), arcR * std::sin(a0), 0.003f },
                       { arcR * std::cos(a1), arcR * std::sin(a1), 0.003f },
                       redInd);
        }
        // Tiny arrow head at the +theta end
        const float aEnd = theta;
        const glm::vec3 tip    { arcR * std::cos(aEnd), arcR * std::sin(aEnd), 0.003f };
        const glm::vec3 tangent{ -std::sin(aEnd) * (theta >= 0.0f ? 1.0f : -1.0f),
                                  std::cos(aEnd) * (theta >= 0.0f ? 1.0f : -1.0f),
                                  0.0f };
        const glm::vec3 inward { -std::cos(aEnd), -std::sin(aEnd), 0.0f };
        r.drawLine(tip, tip - tangent * 0.06f + inward * 0.04f, redInd);
        r.drawLine(tip, tip - tangent * 0.06f - inward * 0.04f, redInd);
    }

    // ============================================================
    // (9) Red phi arc in the vertical plane containing the boom,
    //     centred on the shoulder (NOT on the origin anymore).
    // ============================================================
    if (std::abs(phi) > 0.005f) {
        const float arcR    = 0.22f;
        const int   arcSegs = 28;
        const float step    = phi / arcSegs;
        for (int i = 0; i < arcSegs; ++i) {
            const float a0 = step * i;
            const float a1 = step * (i + 1);
            const glm::vec3 p0 = shoulder + glm::vec3{ arcR * std::cos(a0) * ct,
                                                       arcR * std::cos(a0) * st,
                                                       arcR * std::sin(a0) };
            const glm::vec3 p1 = shoulder + glm::vec3{ arcR * std::cos(a1) * ct,
                                                       arcR * std::cos(a1) * st,
                                                       arcR * std::sin(a1) };
            r.drawLine(p0, p1, redInd);
        }
    }

    // ============================================================
    // (10) Red double-headed arrow alongside the boom showing the
    //      rho slide direction. Offset laterally to avoid overlap.
    // ============================================================
    {
        const glm::vec3 offset = lateralDir * 0.12f + glm::vec3{ 0.0f, 0.0f, 0.08f };
        const glm::vec3 a = shoulder + boomDir * (rhoMin * 0.2f) + offset;
        const glm::vec3 b = effector + offset;
        r.drawLine(a, b, redInd);

        // Arrow heads on both ends (V shape using lateralDir and boomDir).
        const glm::vec3 sideOff = lateralDir * 0.04f;
        const glm::vec3 backOff = boomDir    * 0.07f;
        r.drawLine(b, b - backOff + sideOff, redInd);
        r.drawLine(b, b - backOff - sideOff, redInd);
        r.drawLine(a, a + backOff + sideOff, redInd);
        r.drawLine(a, a + backOff - sideOff, redInd);
    }

    // ============================================================
    // (11) Workspace arcs in the vertical plane of the boom
    //      (rho_min inner + rho_max outer), spanning phi in [-90,+90].
    // ============================================================
    {
        const int   arcSegs = 48;
        const float aStart  = -kHalf;
        const float aEnd    = +kHalf;
        const float step    = (aEnd - aStart) / arcSegs;
        for (int i = 0; i < arcSegs; ++i) {
            const float a0 = aStart + step * i;
            const float a1 = aStart + step * (i + 1);
            // rho_min
            const glm::vec3 p0min = shoulder + glm::vec3{ rhoMin * std::cos(a0) * ct,
                                                          rhoMin * std::cos(a0) * st,
                                                          rhoMin * std::sin(a0) };
            const glm::vec3 p1min = shoulder + glm::vec3{ rhoMin * std::cos(a1) * ct,
                                                          rhoMin * std::cos(a1) * st,
                                                          rhoMin * std::sin(a1) };
            r.drawLine(p0min, p1min, ringCol);
            // rho_max
            const glm::vec3 p0max = shoulder + glm::vec3{ rhoMax * std::cos(a0) * ct,
                                                          rhoMax * std::cos(a0) * st,
                                                          rhoMax * std::sin(a0) };
            const glm::vec3 p1max = shoulder + glm::vec3{ rhoMax * std::cos(a1) * ct,
                                                          rhoMax * std::cos(a1) * st,
                                                          rhoMax * std::sin(a1) };
            r.drawLine(p0max, p1max, ringCol);
        }
    }

    // Suppress unused-variable warning for rho (it is implicit in effector position).
    (void)rho;
}
