//
// Created by lasagnaphil on 19. 12. 5..
//

#ifndef ANIMATION_PROJECT_POSEPHYSICSBODYSKEL_H
#define ANIMATION_PROJECT_POSEPHYSICSBODYSKEL_H

#include <tinyxml2.h>

glm::vec3 stringToVec3(const char* str) {
    glm::vec3 v;
    std::sscanf(str, "%f %f %f", &v.x, &v.y, &v.z);
    return v;
}

glm::mat3 stringToMat3(const char* str) {
    glm::mat3 m;
    std::sscanf(str, "%f %f %f %f %f %f %f %f %f",
                &m[0][0], &m[0][1], &m[0][2],
                &m[1][0], &m[1][1], &m[1][2],
                &m[2][0], &m[2][1], &m[2][2]);
    return m;
}

struct PosePhysicsBodySkel {
    enum class ShapeType {
        Capsule, Cylinder, Sphere, Box
    };
    struct CapsuleShape {
        glm::vec3 direction;
        glm::vec3 offset;
        float radius;
        float height;
    };
    struct CylinderShape {
        glm::vec3 direction;
        glm::vec3 offset;
        float radius;
        float height;
    };
    struct SphereShape {
        glm::vec3 offset;
        float radius;
    };
    struct BoxShape {
        glm::vec3 size;
        glm::vec3 offset;
    };
    struct Shape {
        ShapeType type;
        union {
            CapsuleShape capsule;
            CylinderShape cylinder;
            SphereShape sphere;
            BoxShape box;
        };
    };
    struct Joint {
        std::string name;
        uint32_t bvhIdx;
        uint32_t parentIdx;
        std::vector<uint32_t> childIdx;
        glm::vec3 size;
        float mass;
        glm::vec3 bodyTrans;
        glm::vec3 jointTrans;
        Shape shape;
    };

    std::string skeletonName;
    std::vector<Joint> joints;
    std::unordered_map<std::string, uint32_t> jointNameMapping;

    const Joint& getJoint(uint32_t idx) const {
        return joints[idx];
    }

    Joint& getJoint(uint32_t idx) {
        return joints[idx];
    }

    static PosePhysicsBodySkel fromFile(
            const std::string& filename, const PoseTree& poseTree) {

        using namespace tinyxml2;

        PosePhysicsBodySkel map;

        XMLDocument doc;
        doc.LoadFile(filename.c_str());

        XMLNode* root = doc.FirstChild();
        if (root == nullptr) {
            fprintf(stderr, "Error while reading XML file %s!", filename.c_str());
            exit(EXIT_FAILURE);
        }
        XMLElement* jointNode = root->FirstChildElement("Joint");
        if (jointNode == nullptr) {
            fprintf(stderr, "No Joint element found in XML file %s!", filename.c_str());
            exit(EXIT_FAILURE);
        }

        auto findAttributeVec3 = [](XMLElement* node, const char* name) {
            const XMLAttribute* attr = node->FindAttribute(name);
            if (attr) {
                return stringToVec3(attr->Value());
            }
            else {
                return glm::vec3();
            }
        };

        map.skeletonName = jointNode->FindAttribute("name")->Value();

        uint32_t jointIdx = 0;
        while (jointNode != nullptr) {
            Joint joint;
            joint.name = jointNode->FindAttribute("name")->Value();
            std::string parentName = jointNode->FindAttribute("parent_name")->Value();
            if (parentName == "None") {
                joint.parentIdx = 0;
            }
            else {
                if (map.jointNameMapping.count(parentName) > 0) {
                    joint.parentIdx = map.jointNameMapping[parentName];
                }
                else {
                    std::cerr << "Invalid parent joint name: " << parentName << std::endl;
                    exit(EXIT_FAILURE);
                }
            }
            joint.size = stringToVec3(jointNode->FindAttribute("size")->Value());
            joint.mass = jointNode->FindAttribute("mass")->FloatValue();
            std::string bvhNodeName = jointNode->FindAttribute("bvh")->Value();
            joint.bvhIdx = poseTree.findIdx(bvhNodeName);
            if (joint.bvhIdx == (uint32_t) -1) {
                std::cerr << "Invalid BVH node name: " << bvhNodeName << std::endl;
                exit(EXIT_FAILURE);
            }
            XMLElement* bodyPosNode = jointNode->FirstChildElement("BodyPosition");
            joint.bodyTrans = stringToVec3(bodyPosNode->FindAttribute("translation")->Value());
            XMLElement* jointPosNode = jointNode->FirstChildElement("JointPosition");
            joint.jointTrans = stringToVec3(jointPosNode->FindAttribute("translation")->Value());

            XMLElement* capsuleNode = jointNode->FirstChildElement("Capsule");
            if (capsuleNode) {
                joint.shape.type = ShapeType::Capsule;
                joint.shape.capsule.direction = stringToVec3(capsuleNode->FindAttribute("direction")->Value());
                joint.shape.capsule.offset = findAttributeVec3(capsuleNode, "offset");
                joint.shape.capsule.radius = capsuleNode->FindAttribute("radius")->FloatValue();
                joint.shape.capsule.height = capsuleNode->FindAttribute("height")->FloatValue();
            }
            XMLElement* cylinderNode = jointNode->FirstChildElement("Cylinder");
            if (cylinderNode) {
                joint.shape.type = ShapeType::Cylinder;
                joint.shape.cylinder.direction = stringToVec3(cylinderNode->FindAttribute("direction")->Value());
                joint.shape.cylinder.offset = findAttributeVec3(cylinderNode, "offset");
                joint.shape.cylinder.radius = cylinderNode->FindAttribute("radius")->FloatValue();
                joint.shape.cylinder.height = cylinderNode->FindAttribute("height")->FloatValue();
            }
            XMLElement* sphereNode = jointNode->FirstChildElement("Sphere");
            if (sphereNode) {
                joint.shape.type = ShapeType::Box;
                joint.shape.sphere.offset = findAttributeVec3(sphereNode, "offset");
                joint.shape.sphere.radius = sphereNode->FindAttribute("radius")->FloatValue();
            }
            XMLElement* boxNode = jointNode->FirstChildElement("Box");
            if (boxNode) {
                joint.shape.type = ShapeType::Box;
                joint.shape.box.offset = findAttributeVec3(boxNode, "offset");
                joint.shape.box.size = stringToVec3(boxNode->FindAttribute("size")->Value());
            }

            map.jointNameMapping[joint.name] = jointIdx;

            map.joints.push_back(joint);
            jointIdx++;
            jointNode = jointNode->NextSiblingElement("Joint");
        }

        for (int i = 0; i < map.joints.size(); i++) {
            Joint& joint = map.joints[i];
            if (joint.parentIdx == i) { continue; }
            Joint& parentJoint = map.joints[joint.parentIdx];
            parentJoint.childIdx.push_back(i);
        }

        return map;
    }
};


#endif //ANIMATION_PROJECT_POSEPHYSICSBODYSKEL_H
