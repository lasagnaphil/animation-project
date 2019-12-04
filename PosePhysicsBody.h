//
// Created by lasagnaphil on 19. 11. 21..
//

#ifndef ANIMATION_PROJECT_POSEPHYSICSBODY_H
#define ANIMATION_PROJECT_POSEPHYSICSBODY_H

#include "PoseTree.h"
#include "glmx/pose.h"

struct PosePhysicsBody {
    physx::PxArticulationReducedCoordinate* articulation;
    std::vector<PxArticulationLink*> links;

    PxArticulationCache* cache;

    std::vector<uint32_t> dofStarts;
    std::unordered_map<uint32_t, PxArticulationLink*> nodeToLink;

    void init(PhysicsWorld& world, const PoseTree& poseTree) {
        // Initialize Articulation
        articulation = world.physics->createArticulationReducedCoordinate();
        articulation->setSolverIterationCounts(32);

        nodeToLink[0] = articulation->createLink(nullptr, PxTransform(PxIdentity));
        auto rootBall = PxSphereGeometry(0.1f);
        PxShape* shape = PxRigidActorExt::createExclusiveShape(*nodeToLink[0], rootBall, *world.defaultMaterial);
        PxRigidBodyExt::updateMassAndInertia(*nodeToLink[0], 1.0f);

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
                glm::vec3 a = glm::normalize(node.offset);
                glm::vec3 b = {1, 0, 0};

                PxArticulationLink* parentLink = nodeToLink[parentIdx];
                PxArticulationLink* link = articulation->createLink(parentLink, PxTransform(GLMToPx(parentTransform.v)));
                auto geometry = PxBoxGeometry(boneLength/2, 0.01f, 0.01f);
                PxShape* shape = PxRigidActorExt::createExclusiveShape(*link, geometry, *world.defaultMaterial);
                glmx::transform shapeTransform = glmx::transform(node.offset / 2.f, glmx::quatBetweenVecs(a, b));
                shape->setLocalPose(GLMToPx(shapeTransform));
                PxRigidBodyExt::updateMassAndInertia(*link, 1.0f);
                nodeToLink[nodeIdx] = link;

                if (parentLink) {
                    // PxTransform t_ParentBodyToJoint = PxTransform(parentLink->getGlobalPose().transformInv(jointTransform));
                    // PxTransform t_ChildBodyToJoint = PxTransform(link->getGlobalPose().transformInv(jointTransform));

                    auto pxJoint = static_cast<PxArticulationJointReducedCoordinate*>(link->getInboundJoint());
                    // pxJoint->setChildPose(PxTransform(GLMToPx(glm::angleAxis((float)M_PI/2, glm::vec3(1, 0, 0)))));
                    // pxJoint->setParentPose(PxTransform(GLMToPx(parentTransform.v), GLMToPx(parentTransform.q)));
                    pxJoint->setParentPose(PxTransform(GLMToPx(parentTransform.v), GLMToPx(glm::conjugate(glmx::quatBetweenVecs(a, b)))));
                    pxJoint->setChildPose(PxTransform(GLMToPx(glm::conjugate(glmx::quatBetweenVecs(a, b)))));
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

        PxAggregate* aggregate = world.physics->createAggregate(articulation->getNbLinks(), false);
        aggregate->addArticulation(*articulation);
        world.scene->addAggregate(*aggregate);

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
        PxMemZero(cache->jointPosition, sizeof(PxReal) * articulation->getDofs());

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
            pose.q[i] = glmx::eulerToQuat(v, EulOrdZYXs);
        }
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
            ImGui::SliderFloat3("Root Rotation", (float*)&euler, -M_PI, M_PI);
            cache->rootLinkData->transform.q = GLMToPx(glmx::eulerToQuat(euler, EulOrdZYXs));

            for (auto link : links) {
                uint32_t li = link->getLinkIndex();
                glm::vec3 euler;
                euler.x = cache->jointPosition[dofStarts[li]];
                euler.y = cache->jointPosition[dofStarts[li] + 1];
                euler.z = cache->jointPosition[dofStarts[li] + 2];
                std::string linkName = "link_" + std::to_string(li);
                bool linkEdited = ImGui::SliderFloat3(linkName.c_str(), (float*)&euler, -M_PI, M_PI);
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
