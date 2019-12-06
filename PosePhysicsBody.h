//
// Created by lasagnaphil on 19. 11. 21..
//

#ifndef ANIMATION_PROJECT_POSEPHYSICSBODY_H
#define ANIMATION_PROJECT_POSEPHYSICSBODY_H

#include "PoseTree.h"
#include "glmx/pose.h"
#include "PosePhysicsBodySkel.h"


struct PosePhysicsBody {
    physx::PxArticulationReducedCoordinate* articulation;

    PxArticulationCache* cache;

    std::vector<uint32_t> dofStarts;
    std::unordered_map<uint32_t, PxArticulationLink*> nodeToLink;

    void init(PhysicsWorld& world, const PoseTree& poseTree, const PosePhysicsBodySkel& skel) {
        // Initialize Articulation
        articulation = world.physics->createArticulationReducedCoordinate();
        articulation->setSolverIterationCounts(32);

        std::stack<uint32_t> recursionStack;
        recursionStack.push(0);
        while (!recursionStack.empty()) {
            uint32_t jointIdx = recursionStack.top();
            recursionStack.pop();

            const PosePhysicsBodySkel::Joint& joint = skel.getJoint(jointIdx);
            const PosePhysicsBodySkel::Joint& parent = skel.getJoint(joint.parentIdx);
            const PoseTreeNode& bvhNode = poseTree[joint.bvhIdx];

            glm::quat shapeRot = glm::identity<glm::quat>();
            glmx::transform shapeTransform = glmx::transform(joint.bodyTrans - joint.jointTrans, shapeRot);

            PxArticulationLink* link;
            if (jointIdx == 0) {
                link = articulation->createLink(nullptr, PxTransform(GLMToPx(joint.bodyTrans)));
            }
            else {
                link = articulation->createLink(nodeToLink[parent.bvhIdx], PxTransform(GLMToPx(joint.jointTrans - parent.jointTrans)));
            }
            link->setName(joint.name.c_str());

            auto geometry = PxBoxGeometry(GLMToPx(joint.size/2.f));
            PxShape* shape = PxRigidActorExt::createExclusiveShape(*link, geometry, *world.defaultMaterial);
            shape->setLocalPose(GLMToPx(shapeTransform));
            PxRigidBodyExt::updateMassAndInertia(*link, 1.0f);
            nodeToLink[joint.bvhIdx] = link;

            auto pxJoint = static_cast<PxArticulationJointReducedCoordinate*>(link->getInboundJoint());
            if (pxJoint) {
                /*
                pxJoint->setParentPose(
                        PxTransform(GLMToPx(joint.jointTrans), GLMToPx(glm::conjugate(shapeTransform.q))));
                        */
                if ((joint.jointTrans - parent.jointTrans).y >= 0) {
                    // pxJoint->setChildPose(PxTransform(GLMToPx(jbk)))
                    PxVec3 p = pxJoint->getParentPose().p;
                    pxJoint->setParentPose(PxTransform(p, GLMToPx(glm::angleAxis(glm::pi<float>(), glm::vec3(0, 0, 1)))));
                    pxJoint->setChildPose(PxTransform(PxIdentity));
                    pxJoint->setChildPose(PxTransform(GLMToPx(glm::angleAxis(glm::pi<float>(), glm::vec3(0, 0, 1)))));
                }
                else {
                    pxJoint->setChildPose(PxTransform(PxIdentity));
                }
                pxJoint->setJointType(PxArticulationJointType::eSPHERICAL);
                pxJoint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
                pxJoint->setMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
                pxJoint->setMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
            }

            if (!joint.childIdx.empty()) {
                for (auto childIdx : joint.childIdx) {
                    recursionStack.push(childIdx);
                }
            }
        }
        /*
        std::stack<std::tuple<uint32_t, glmx::transform>> recursionStack;
        recursionStack.push({0, glmx::transform()});
        while (!recursionStack.empty()) {
            auto [nodeIdx, parentTransform] = recursionStack.top();
            recursionStack.pop();

            const PoseTreeNode& node = poseTree[nodeIdx];
            uint32_t parentIdx = poseTree[nodeIdx].parent;

            // render bone related to current node (we exclude root node)
            if (nodeIdx != 0 && glm::length(node.offset) > glm::epsilon<float>()) {
                float boneLength = glm::length(node.offset);
                glm::mat3 coordT;
                coordT[0] = glm::normalize(node.offset);
                if (node.offset.y >= 0) coordT[2] = {0, 0, 1};
                else coordT[2] = {0, 0, -1};
                coordT[1] = glm::normalize(glm::cross(coordT[2], coordT[0]));
                glmx::transform shapeTransform = glmx::transform(node.offset / 2.f, glm::quat_cast(coordT));

                PxArticulationLink* parentLink = nodeToLink[parentIdx];
                PxArticulationLink* link = articulation->createLink(parentLink, PxTransform(GLMToPx(parentTransform.v)));
                // link->setName(poseTree[node.parent].name.c_str());
                if (node.isEndSite) {
                    link->setName(("End Site " + std::to_string(nodeIdx)).c_str());
                }
                else {
                    link->setName(node.name.c_str());
                }
                auto geometry = PxBoxGeometry(boneLength/2, 0.01f, 0.01f);
                PxShape* shape = PxRigidActorExt::createExclusiveShape(*link, geometry, *world.defaultMaterial);
                shape->setLocalPose(GLMToPx(shapeTransform));
                PxRigidBodyExt::updateMassAndInertia(*link, 1.0f);
                nodeToLink[nodeIdx] = link;

                auto pxJoint = static_cast<PxArticulationJointReducedCoordinate*>(link->getInboundJoint());
                pxJoint->setParentPose(PxTransform(GLMToPx(parentTransform.v), GLMToPx(glm::conjugate(shapeTransform.q))));
                pxJoint->setChildPose(PxTransform(GLMToPx(glm::conjugate(shapeTransform.q))));
                if (parentIdx == 0) {
                    pxJoint->setJointType(PxArticulationJointType::eFIX);
                }
                else {
                    pxJoint->setJointType(PxArticulationJointType::eSPHERICAL);
                    pxJoint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
                    pxJoint->setMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
                    pxJoint->setMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
                }

                parentTransform.v = node.offset;
                parentTransform.q = shapeTransform.q;

            }

            if (!node.isEndSite) {
                for (auto childID : node.childJoints) {
                    recursionStack.push({childID, parentTransform});
                }
            }
        }
        */

        PxAggregate* aggregate = world.physics->createAggregate(articulation->getNbLinks(), false);
        aggregate->addArticulation(*articulation);
        world.scene->addAggregate(*aggregate);

        std::vector<PxArticulationLink*> links;
        links.resize(articulation->getNbLinks());
        articulation->getLinks(links.data(), links.size());

        cache = articulation->createCache();

        dofStarts.resize(links.size());
        dofStarts[0] = 0; // We know that the root link does not have a joint
        for (uint32_t i = 1; i < links.size(); i++) {
            uint32_t llIndex = links[i]->getLinkIndex();
            uint32_t dofs = links[i]->getInboundJointDof();
            dofStarts[llIndex] = dofs;
        }
        uint32_t dofCount = 0;
        for(PxU32 i = 1; i < links.size(); ++i)
        {
            PxU32 dofs = dofStarts[i];
            dofStarts[i] = dofCount;
            dofCount += dofs;
        }
    }

    void setPose(const glmx::pose& pose) {
        PxArticulationCacheFlags flags = PxArticulationCache::eROOT | PxArticulationCache::ePOSITION;
        articulation->copyInternalStateToCache(*cache, flags);

        cache->rootLinkData->transform = PxTransform(GLMToPx(pose.v), GLMToPx(pose.q[0]));

        for (uint32_t i = 1; i < pose.size(); i++) {
            glm::quat q = pose.q[i];
            glm::quat twist = q.x != 0.0f ? glm::normalize(glm::quat(q.w, q.x, 0, 0)) : glm::identity<glm::quat>();
            glm::quat swing = q * glm::conjugate(twist);
            glm::quat swing1 = swing.y != 0.f ? glm::normalize(glm::quat(swing.w, 0.f, swing.y, 0.f)) : glm::identity<glm::quat>();
            swing = swing * glm::conjugate(swing1);
            glm::quat swing2 = swing.z != 0.f ? glm::normalize(glm::quat(swing.w, 0.f, 0.f, swing.z)) : glm::identity<glm::quat>();

            float x = 2 * glm::atan(twist.w, twist.x);
            float y = 2 * glm::atan(swing1.w, swing1.y);
            float z = 2 * glm::atan(swing2.w, swing2.z);
            uint32_t li = nodeToLink[i]->getLinkIndex();
            cache->jointPosition[dofStarts[li]] = x;
            cache->jointPosition[dofStarts[li] + 1] = y;
            cache->jointPosition[dofStarts[li] + 2] = z;
        }

        articulation->applyCache(*cache, flags);
    }

    void getPose(glmx::pose& pose) {
        PxArticulationCacheFlags flags = PxArticulationCache::eROOT | PxArticulationCache::ePOSITION;
        articulation->copyInternalStateToCache(*cache, flags);

        pose.v = PxToGLM(cache->rootLinkData->transform.p);
        pose.q[0] = PxToGLM(cache->rootLinkData->transform.q);

        for (uint32_t i = 1; i < pose.size(); i++) {
            uint32_t li = nodeToLink[i]->getLinkIndex();
            glm::vec3 v;
            v.x = cache->jointPosition[dofStarts[li]];
            v.y = cache->jointPosition[dofStarts[li] + 1];
            v.z = cache->jointPosition[dofStarts[li] + 2];
            glm::quat twist =  glm::quat(cos(0.5f * v.x), sin(0.5f * v.x), 0, 0);
            glm::quat swing1 =  glm::quat(cos(0.5f * v.y), 0, sin(0.5f * v.y), 0);
            glm::quat swing2 =  glm::quat(cos(0.5f * v.z), 0, 0, sin(0.5f * v.z));
            pose.q[i] = swing2 * swing1 * twist;
        }
    }

    void setRoot(const glmx::transform& rootTrans) {
        PxArticulationCacheFlags flags = PxArticulationCache::eROOT;
        articulation->copyInternalStateToCache(*cache, flags);

        cache->rootLinkData->transform.p = GLMToPx(rootTrans.v);
        cache->rootLinkData->transform.q = GLMToPx(rootTrans.q);

        articulation->applyCache(*cache, flags);
    }

    void renderImGui() {
        ImGui::Begin("PoseEntity");

        if (ImGui::TreeNode("Articulation")) {

            PxArticulationCacheFlags flags = PxArticulationCache::eROOT | PxArticulationCache::ePOSITION;
            articulation->copyInternalStateToCache(*cache, flags);

            bool edited = false;
            edited |= ImGui::SliderFloat3("Root Position", (float*)&cache->rootLinkData->transform.p, -10.f, 10.f);
            glm::quat q = PxToGLM(cache->rootLinkData->transform.q);
            glm::vec3 euler = glmx::quatToEuler(q, EulOrdZYXs);
            ImGui::DragFloat3("Root Rotation", (float*)&euler, 0.01f);
            cache->rootLinkData->transform.q = GLMToPx(glmx::eulerToQuat(euler, EulOrdZYXs));

            for (auto [bvhIdx, link] : nodeToLink) {
                uint32_t li = link->getLinkIndex();
                glm::vec3 euler;
                euler.x = cache->jointPosition[dofStarts[li]];
                euler.y = cache->jointPosition[dofStarts[li] + 1];
                euler.z = cache->jointPosition[dofStarts[li] + 2];
                float pi = glm::pi<float>();
                euler.x = fmod(euler.x + pi, 2*pi) - pi;
                euler.y = fmod(euler.y + pi, 2*pi) - pi;
                euler.z = fmod(euler.z + pi, 2*pi) - pi;
                bool linkEdited = ImGui::DragFloat3(link->getName(), (float*)&euler, 0.01f);
                if (linkEdited) {
                    cache->jointPosition[dofStarts[li]] = euler.x;
                    cache->jointPosition[dofStarts[li] + 1] = euler.y;
                    cache->jointPosition[dofStarts[li] + 2] = euler.z;
                }
                edited |= linkEdited;
            }

            if (edited) {
                articulation->applyCache(*cache, PxArticulationCache::eROOT | PxArticulationCache::ePOSITION);
            }

            ImGui::TreePop();
        }


        ImGui::End();
    }

};
#endif //ANIMATION_PROJECT_POSEPHYSICSBODY_H
