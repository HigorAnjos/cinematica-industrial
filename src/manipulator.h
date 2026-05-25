#pragma once

#include <glm/glm.hpp>

#include <string>
#include <vector>

struct JointSpec {
    bool  isPrismatic;   // false = revolute (angle, radians); true = prismatic (length)
    float minValue;
    float maxValue;
    const char* label;
    const char* unit;    // "deg", "mm", "rad", ...
};

struct LinkParam {
    const char* label;
    float*      value;
    float       minValue;
    float       maxValue;
};

// 3D industrial-manipulator base interface.
class IManipulator {
public:
    virtual ~IManipulator() = default;

    virtual const char* name() const = 0;
    virtual int         numJoints() const = 0;

    virtual const std::vector<JointSpec>& jointSpecs() const = 0;
    virtual       std::vector<float>&     jointValues()       = 0;
    virtual const std::vector<float>&     jointValues() const = 0;

    // For rendering: the polyline of joint frames in world coordinates.
    virtual std::vector<glm::vec3> jointPositions() const = 0;

    virtual glm::vec3 endEffector() const {
        const auto p = jointPositions();
        return p.empty() ? glm::vec3{0.0f} : p.back();
    }

    virtual std::vector<LinkParam> linkParams() = 0;

    virtual bool inverseKinematics(glm::vec3 target,
                                    std::vector<float>& outJoints,
                                    std::string& outReason) const = 0;

    // Render the manipulator-specific visualization using the active renderer.
    virtual void draw(class Renderer3D& r) const = 0;
};
