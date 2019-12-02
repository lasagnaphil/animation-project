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
    std::unordered_map<uint32_t, uint32_t> nodeToLinkIdx;

    void init(PhysicsWorld& world, const PoseTree& poseTree) {
        // Initialize Articulation
        articulation = world.physics->createArticulationReducedCoordinate();
        articulation->setSolverIterationCounts(32);

        // TODO: create articulation from pose tree

        links.resize(articulation->getNbLinks());
        articulation->getLinks(links.data(), links.size());

        PxAggregate* aggregate = world.physics->createAggregate(articulation->getNbLinks(), false);
        aggregate->addArticulation(*articulation);
        world.scene->addAggregate(*aggregate);

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

    void readPose(const glmx::pose& pose) {
        PxArticulationCacheFlags flags = PxArticulationCache::eROOT | PxArticulationCache::ePOSITION;
        articulation->copyInternalStateToCache(*cache, flags);
        PxMemZero(cache->jointPosition, sizeof(PxReal) * articulation->getDofs());

        cache->rootLinkData->transform = PxTransform(GLMToPx(pose.v), GLMToPx(pose.q[0]));

        for (uint32_t i = 1; i < pose.size(); i++) {
            glm::vec3 euler = glmx::quatToEuler(pose.q[i], EulOrdZYXs);
            uint32_t li = nodeToLinkIdx[i];
            cache->jointPosition[dofStarts[li]] = euler.x;
            cache->jointPosition[dofStarts[li] + 1] = euler.y;
            cache->jointPosition[dofStarts[li] + 2] = euler.z;
        }
    }

    void writePose(glmx::pose& pose) {
        PxArticulationCacheFlags flags = PxArticulationCache::eROOT | PxArticulationCache::ePOSITION;
        articulation->copyInternalStateToCache(*cache, flags);

        pose.v = PxToGLM(cache->rootLinkData->transform.p);
        pose.q[0] = PxToGLM(cache->rootLinkData->transform.q);

        for (uint32_t i = 1; i < pose.size(); i++) {
            uint32_t li = nodeToLinkIdx[i];
            glm::vec3 v;
            v.x = cache->jointPosition[dofStarts[li]];
            v.y = cache->jointPosition[dofStarts[li] + 1];
            v.z = cache->jointPosition[dofStarts[li] + 2];
            pose.q[i] = glmx::eulerToQuat(v, EulOrdZYXs);
        }
        articulation->applyCache(*cache, flags);
    }

};
#endif //ANIMATION_PROJECT_POSEPHYSICSBODY_H
