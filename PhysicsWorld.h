//
// Created by lasagnaphil on 19. 5. 7.
//

#ifndef PHYSICS_BENCHMARKS_PHYSICSWORLD_H
#define PHYSICS_BENCHMARKS_PHYSICSWORLD_H

#ifndef NDEBUG
#define _DEBUG
#endif

#include <PxPhysicsAPI.h>
#include <pvd/PxPvd.h>
#include <extensions/PxDefaultAllocator.h>
#include <extensions/PxDefaultErrorCallback.h>

#include "PhysXGLM.h"

using namespace physx;

struct PhysicsWorld {
    void init(PxFoundation* foundation, uint32_t numThreads);

    bool advance(float dt);

    bool fetchResults();

    void release();

    PxFoundation* foundation;
    PxPhysics* physics;
    // PxCudaContextManager* cudaContextManager;
    // PxCooking* cooking;
    PxCpuDispatcher* cpuDispatcher;
    PxScene* scene;
    PxMaterial* defaultMaterial;
};

#endif //PHYSICS_BENCHMARKS_PHYSICSWORLD_H
