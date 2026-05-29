#pragma once

#include "../manipulator.h"

class ManipulatorCartesian : public IManipulator {
public:
    ManipulatorCartesian(float d1Min = 0.0f, float d1Max = 2.0f,
                         float d2Min = 0.0f, float d2Max = 2.0f,
                         float d3Min = 0.0f, float d3Max = 2.0f);

    const char* name() const override { return "Cartesiano (PPP)"; }
    int         numJoints() const override { return 3; }

    const std::vector<JointSpec>& jointSpecs() const override { return m_specs; }
          std::vector<float>&     jointValues()       override { return m_joints; }
    const std::vector<float>&     jointValues() const override { return m_joints; }

    std::vector<glm::vec3> jointPositions() const override;

    std::vector<LinkParam> linkParams() override;

    bool inverseKinematics(glm::vec3 target,
                            std::vector<float>& outJoints,
                            std::string& outReason) const override;

    void draw(Renderer3D& r) const override;

private:
    std::vector<float>     m_joints;
    std::vector<JointSpec> m_specs;
};
