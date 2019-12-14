//
// Created by lasagnaphil on 19. 11. 21..
//

#ifndef ANIMATION_PROJECT_POSEPHYSICSBODY_H
#define ANIMATION_PROJECT_POSEPHYSICSBODY_H

#include "PoseTree.h"
#include "glmx/pose.h"
#include "PosePhysicsBodySkel.h"

void separateSwingTwist(const PxQuat& q, PxQuat& twist, PxQuat& swing1, PxQuat& swing2)
{
    twist = q.x != 0.0f ? PxQuat(q.x, 0, 0, q.w).getNormalized() : PxQuat(PxIdentity);
    PxQuat swing = q * twist.getConjugate();
    swing1 = swing.y != 0.f ? PxQuat(0.f, swing.y, 0.f, swing.w).getNormalized() : PxQuat(PxIdentity);
    swing = swing * swing1.getConjugate();
    swing2 = swing.z != 0.f ? PxQuat(0.f, 0.f, swing.z, swing.w).getNormalized() : PxQuat(PxIdentity);
}

void separateSwingTwist(const PxQuat& q, PxQuat& swing, PxQuat& twist)
{
    twist = q.x != 0.0f ? PxQuat(q.x, 0, 0, q.w).getNormalized() : PxQuat(PxIdentity);
    swing = q * twist.getConjugate();
}

float computeSwingAngle(float swingYZ, float swingW)
{
    return 4.0f * PxAtan2(swingYZ, 1.0f + swingW);	// tan (t/2) = sin(t)/(1+cos t), so this is the quarter angle
}

PxVec3 quatToTwistSwing(PxQuat q)
{
    PxQuat swing, twist;
    separateSwingTwist(q, swing, twist);

    PxVec3 v;
    v.x = twist.getAngle();
    if (twist.x < 0.0f)
        v.x = -v.x;
    if (swing.w < 0.0f)		// choose the shortest rotation
        swing = -swing;

    v.y = computeSwingAngle(swing.y, swing.w);
    PX_ASSERT(v.y > -PxPi && v.y <= PxPi);				// since |y| < w+1, the atan magnitude is < PI/4
    v.z = computeSwingAngle(swing.z, swing.w);
    PX_ASSERT(v.z > -PxPi && v.z <= PxPi);			// since |y| < w+1, the atan magnitude is < PI/4
    return v;
}

PxQuat twistSwingToQuat(PxVec3 v)
{
    PX_ASSERT(v.y > -PxPi && v.y <= PxPi);				// since |y| < w+1, the atan magnitude is < PI/4
    PX_ASSERT(v.z > -PxPi && v.z <= PxPi);			// since |y| < w+1, the atan magnitude is < PI/4
    PxQuat swing, twist;

    float ty = PxTan(v.y / 4);
    float tz = PxTan(v.z / 4);

    swing.w = (1 - ty*ty - tz*tz) / (1 + ty*ty + tz*tz);
    swing.x = 0;
    swing.y = (1+swing.w) * ty;
    swing.z = (1+swing.w) * tz;
    if (swing.w < 0.0f)
        swing = -swing;

    twist.w = PxCos(v.x / 2);
    twist.x = PxSin(v.x / 2);
    twist.y = 0;
    twist.z = 0;
    if (twist.x < 0.0f)
        twist = -twist;

    return swing * twist;
}

struct PosePhysicsBody {
    physx::PxArticulationReducedCoordinate* articulation;

    PxAggregate *aggregate;
    PxArticulationCache* cache;

    std::vector<uint32_t> dofStarts;
    std::unordered_map<uint32_t, PxArticulationLink*> nodeToLink;
    uint32_t dofCount = 0;


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

            PxArticulationLink* parentLink = nullptr;
            PxArticulationLink* link;
            if (jointIdx == 0) {
                link = articulation->createLink(nullptr, PxTransform(GLMToPx(joint.jointTrans)));
            }
            else {
                parentLink = nodeToLink[parent.bvhIdx];
                link = articulation->createLink(parentLink,
                        PxTransform(GLMToPx(joint.jointTrans - parent.jointTrans)));

            }
            link->setName(joint.name.c_str());

            auto geometry = PxBoxGeometry(GLMToPx(joint.size/2.f));
            PxShape* shape = PxRigidActorExt::createExclusiveShape(*link, geometry, *world.defaultMaterial);
            PxRigidBodyExt::updateMassAndInertia(*link, 1.0f);
            nodeToLink[joint.bvhIdx] = link;

            auto pxJoint = static_cast<PxArticulationJointReducedCoordinate*>(link->getInboundJoint());
            if (pxJoint) {
                glmx::transform origin;
                // PxQuat rot = PxShortestRotation(PxVec3(1.0f, 0.0f, 0.0f), GLMToPx(glm::normalize(-joint.xdir)));
                PxQuat rot = PxShortestRotation(PxVec3(1.0f, 0.0f, 0.0f), PxVec3(1.0f, 0.0f, 0.0f));
                PxTransform pPose = PxTransform(GLMToPx(joint.jointTrans - parent.bodyTrans), rot);
                PxTransform cPose = PxTransform(GLMToPx(joint.jointTrans - joint.bodyTrans), rot);
                pxJoint->setParentPose(pPose);
                pxJoint->setChildPose(cPose);

                pxJoint->setJointType(PxArticulationJointType::eSPHERICAL);
                pxJoint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
                pxJoint->setMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
                pxJoint->setMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
                /*
                pxJoint->setLimit(PxArticulationAxis::eTWIST, -M_PI/2, M_PI/2);
                pxJoint->setLimit(PxArticulationAxis::eSWING1, -0.95f * M_PI, 0.95f * M_PI);
                pxJoint->setLimit(PxArticulationAxis::eSWING2, -0.95f * M_PI, 0.95f * M_PI);
                */
            }

            if (!joint.childIdx.empty()) {
                for (auto childIdx : joint.childIdx) {
                    recursionStack.push(childIdx);
                }
            }
        }

        aggregate = world.physics->createAggregate(articulation->getNbLinks(), false);
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
        dofCount = 0;
        for(PxU32 i = 1; i < links.size(); ++i)
        {
            PxU32 dofs = dofStarts[i];
            dofStarts[i] = dofCount;
            dofCount += dofs;
        }
    }

    void setPose(const glmx::pose& pose, const PoseTree& poseTree, bool setVelToZero = false) {
        PxArticulationCacheFlags flags = PxArticulationCache::eROOT | PxArticulationCache::ePOSITION;
        if (setVelToZero) {
            flags |= PxArticulationCache::eVELOCITY;
        }
        articulation->copyInternalStateToCache(*cache, flags);

        cache->rootLinkData->transform = PxTransform(GLMToPx(pose.v), GLMToPx(pose.q[0]));

        for (uint32_t i = 1; i < pose.size(); i++) {
            glm::quat q = pose.q[i];

            PxVec3 v = quatToTwistSwing(GLMToPx(pose.q[i]));
            uint32_t li = nodeToLink[i]->getLinkIndex();
            cache->jointPosition[dofStarts[li]] = v.x;
            cache->jointPosition[dofStarts[li] + 1] = v.y;
            cache->jointPosition[dofStarts[li] + 2] = v.z;
        }
        if (setVelToZero) {
            PxMemZero(cache->jointVelocity, sizeof(PxReal) * dofCount);
        }

        articulation->applyCache(*cache, flags);
    }

    void getPose(glmx::pose& pose, const PoseTree& poseTree) {
        PxArticulationCacheFlags flags = PxArticulationCache::eROOT | PxArticulationCache::ePOSITION;
        articulation->copyInternalStateToCache(*cache, flags);

        pose.v = PxToGLM(cache->rootLinkData->transform.p);
        pose.q[0] = PxToGLM(cache->rootLinkData->transform.q);

        for (uint32_t i = 1; i < pose.size(); i++) {
            uint32_t li = nodeToLink[i]->getLinkIndex();
            PxVec3 v;
            v.x = cache->jointPosition[dofStarts[li]];
            v.y = cache->jointPosition[dofStarts[li] + 1];
            v.z = cache->jointPosition[dofStarts[li] + 2];
            pose.q[i] = PxToGLM(twistSwingToQuat(v));
        }
    }

    void setRoot(const glmx::transform& rootTrans) {
        PxArticulationCacheFlags flags = PxArticulationCache::eROOT;
        articulation->copyInternalStateToCache(*cache, flags);

        cache->rootLinkData->transform.p = GLMToPx(rootTrans.v);
        cache->rootLinkData->transform.q = GLMToPx(rootTrans.q);

        articulation->applyCache(*cache, flags);
    }

    void putToSleep() {
        articulation->putToSleep();
    }

    void wakeUp() {
        articulation->wakeUp();
    }

    void renderImGui() {
        PxArticulationCacheFlags flags = PxArticulationCache::eROOT | PxArticulationCache::ePOSITION;
        articulation->copyInternalStateToCache(*cache, flags);

        bool edited = false;
        edited |= ImGui::DragFloat3("Root Position", (float*)&cache->rootLinkData->transform.p, 0.01f);
        glm::quat q = PxToGLM(cache->rootLinkData->transform.q);
        glm::vec3 euler = glmx::quatToEuler(q, EulOrdXYZs);
        ImGui::DragFloat3("Root Rotation", (float*)&euler, 0.01f);
        cache->rootLinkData->transform.q = GLMToPx(glmx::eulerToQuat(euler, EulOrdXYZs));

        for (auto [bvhIdx, link] : nodeToLink) {
            uint32_t li = link->getLinkIndex();
            glm::vec3 v;
            v.x = cache->jointPosition[dofStarts[li]];
            v.y = cache->jointPosition[dofStarts[li] + 1];
            v.z = cache->jointPosition[dofStarts[li] + 2];
            float pi = glm::pi<float>();
            bool linkEdited = ImGui::DragFloat3(link->getName(), (float*)&v, 0.01f);
            if (linkEdited) {
                cache->jointPosition[dofStarts[li]] = v.z;
                cache->jointPosition[dofStarts[li] + 1] = v.y;
                cache->jointPosition[dofStarts[li] + 2] = v.x;
            }
            edited |= linkEdited;
        }

        if (edited) {
            articulation->applyCache(*cache, PxArticulationCache::eROOT | PxArticulationCache::ePOSITION);
        }
    }
};
#endif //ANIMATION_PROJECT_POSEPHYSICSBODY_H
