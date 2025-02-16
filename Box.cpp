#include "Box.h"
#include <stack>
#include <tuple>
#include <deps/glm/glm/gtx/transform.hpp>
#include <deps/glm/glm/gtx/string_cast.hpp>
#include <iostream>

inline glm::mat4 rotationBetweenVecs(glm::vec3 a, glm::vec3 b) {
    glm::vec3 v = glm::cross(a, b);
    float s2 = glm::dot(v, v);
    if (s2 < glm::epsilon<float>()) {
        return glm::mat4(1.0f);
    }
    else {
        // Rodrigue's formula
        float c = glm::dot(a, b);
        glm::mat3 vhat;
        vhat[0][0] = vhat[1][1] = vhat[2][2] = 0;
        vhat[2][1] = v[0]; vhat[1][2] = -v[0];
        vhat[0][2] = v[1]; vhat[2][0] = -v[1];
        vhat[1][0] = v[2]; vhat[0][1] = -v[2];
        return glm::mat3(1.0f) + vhat + vhat*vhat*(1 - c)/(s2);
    }
}

glm::mat4 Box::getTransformFromHands(   const glmx::pose& poseState, 
                                        const PoseTree& poseTree, 
                                        const std::string &lhand, 
                                        const std::string &rhand,
                                        const glm::mat4& globalTrans) 
{
    glm::mat4 rootTR, lhandTR, rhandTR;
    
    std::stack<std::tuple<uint32_t, glm::mat4>> recursionStack;
    recursionStack.push({0, glm::translate(globalTrans, poseState.v)});
    while (!recursionStack.empty()) {
        auto [nodeIdx, curTransform] = recursionStack.top();
        recursionStack.pop();

        const PoseTreeNode& node = poseTree[nodeIdx];
        if (!node.isEndSite) {
            // render bone related to current node (we exclude root node)
            if (nodeIdx != 0 && glm::length(node.offset) > glm::epsilon<float>()) {
                glm::vec3 a = glm::normalize(node.offset);
                glm::vec3 b = {0, 1, 0};
                glm::mat4 initialRot, initialTrans;
                if (node.offset.y >= 0) {
                    initialRot = rotationBetweenVecs(a, b);
                    initialTrans = glm::translate(glm::vec3{0.0f, glm::length(node.offset)/2, 0.0f});
                }
                else {
                    initialRot = rotationBetweenVecs(a, -b);
                    initialTrans = glm::translate(glm::vec3{0.0f, -glm::length(node.offset)/2, 0.0f});
                }
                glm::mat4 initialBoneTransform = initialRot * initialTrans;
                glm::mat4 T = curTransform * initialBoneTransform;

                if(node.isRoot())
                    rootTR = T;
                if(node.name == lhand)
                    lhandTR = T;
                if(node.name == rhand)
                    rhandTR = T;
            }

            curTransform = curTransform * glm::translate(node.offset) * glm::mat4_cast(poseState.q[nodeIdx]);
            for (auto childID : node.childJoints) {
                recursionStack.push({childID, curTransform});
            }
        }
    }
    glm::vec4 lpos = lhandTR * glm::vec4(0, 0, 0, 1);
    glm::vec4 rpos = rhandTR * glm::vec4(0, 0, 0, 1);
    glm::vec4 bpos = (lpos + rpos) * 0.5f;
    glm::mat4 ret = glm::translate(glm::vec3(0.0f, 0.0f, 0.0f));
    ret[0][3] = bpos.x;
    ret[1][3] = bpos.y;
    ret[2][3] = bpos.z + 0.2f;
    return ret;     // @TODO : Only consider hand positions now. Have to consider hand rotations also...
}