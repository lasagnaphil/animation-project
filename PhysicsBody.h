//
// Created by lasagnaphil on 19. 12. 11..
//

#ifndef ANIMATION_PROJECT_PHYSICSBODY_H
#define ANIMATION_PROJECT_PHYSICSBODY_H

#include <Defer.h>
#include "PhysXGLM.h"


struct PhysicsBody {
    PxRigidDynamic* body = nullptr;

    static PhysicsBody box(PhysicsWorld& world,
                           PxMaterial* mat,
                           glm::vec3 pos = {},
                           glm::quat rot = glm::identity<glm::quat>(),
                           glm::vec3 size = glm::vec3(1.0f)) {

        PhysicsBody prim;
        PxShape* shape = world.physics->createShape(PxBoxGeometry(GLMToPx(size/2.f)), *mat);
        defer { shape->release(); };
        shape->setFlag(PxShapeFlag::eVISUALIZATION, true);

        PxTransform localTm(GLMToPx(pos), GLMToPx(rot));
        prim.body = world.physics->createRigidDynamic(localTm);
        prim.body->attachShape(*shape);
        PxRigidBodyExt::updateMassAndInertia(*prim.body, 10.0f);
        world.scene->addActor(*prim.body);

        return prim;
    }

    static PhysicsBody sphere(PhysicsWorld& world,
                              PxMaterial* mat,
                              glm::vec3 pos = {},
                              float size = 1.0f) {

        PhysicsBody prim;
        PxShape* shape = world.physics->createShape(PxSphereGeometry(size), *mat);
        defer { shape->release(); };
        shape->setFlag(PxShapeFlag::eVISUALIZATION, true);

        PxTransform localTm(GLMToPx(pos));
        prim.body = world.physics->createRigidDynamic(localTm);
        prim.body->attachShape(*shape);
        PxRigidBodyExt::updateMassAndInertia(*prim.body, 10.0f);
        world.scene->addActor(*prim.body);

        return prim;
    }

    static std::optional<PhysicsBody> fromMesh(const PhysicsWorld& world,
                                               const MeshCollider& mesh,
                                               PxMaterial* mat,
                                               glm::vec3 pos = glm::vec3(0.0f),
                                               glm::quat rot = glm::identity<glm::quat>(),
                                               glm::vec3 scale = glm::vec3(1.0f)) {

        PhysicsBody meshBody;
        PxTriangleMeshDesc meshDesc;
        std::vector<glm::vec3> data;
        meshDesc.points.count = mesh.points.size();
        meshDesc.points.stride = sizeof(glm::vec3);
        meshDesc.points.data = mesh.points.data();
        meshDesc.triangles.count = mesh.indices.size() / 3;
        meshDesc.triangles.stride = 3*sizeof(PxU32);
        meshDesc.triangles.data = mesh.indices.data();

        assert(meshDesc.isValid());

        bool res = world.cooking->validateTriangleMesh(meshDesc);
        if (!res) {
            fprintf(stderr, "Error in MeshBody::fromMesh(): triangle mesh validation failed\n");
            return {};
        }

        PxDefaultMemoryOutputStream writeBuffer;
        PxTriangleMeshCookingResult::Enum result;
        bool status = world.cooking->cookTriangleMesh(meshDesc, writeBuffer, &result);
        if (!status) {
            fprintf(stderr, "Error in MeshBody::fromMesh(): cooking triangle mesh failed\n");
            return {};
        }

        PxDefaultMemoryInputData readBuffer(writeBuffer.getData(), writeBuffer.getSize());
        PxTriangleMesh* triangleMesh = world.physics->createTriangleMesh(readBuffer);

        PxTransform localTm(GLMToPx(pos), GLMToPx(rot));
        meshBody.body = world.physics->createRigidDynamic(localTm);

        auto geom = PxTriangleMeshGeometry(triangleMesh, GLMToPx(scale));
        auto shape = world.physics->createShape(geom, *mat);
        meshBody.body->attachShape(*shape);
        PxRigidBodyExt::updateMassAndInertia(*meshBody.body, 10.0f);
        world.scene->addActor(*meshBody.body);

        return meshBody;
    }

    glm::vec3 getPosition() {
        return PxToGLM(body->getGlobalPose().p);
    }

    void setPosition(glm::vec3 v) {
        body->setGlobalPose(PxTransform(GLMToPx(v), body->getGlobalPose().q));
    }

    glm::quat getRotation() {
        return PxToGLM(body->getGlobalPose().q);
    }

    void setRotation(glm::quat q) {
        body->setGlobalPose(PxTransform(body->getGlobalPose().p, GLMToPx(q)));
    }

    void setTransform(const glmx::transform& t) {
        body->setGlobalPose(GLMToPx(t));
    }

    glmx::transform getTransform() {
        return PxToGLM(body->getGlobalPose());
    }

    glm::vec3 getLinearVelocity() {
        return PxToGLM(body->getLinearVelocity());
    }

    void setLinearVelocity(glm::vec3 v, bool autowake = true) {
        return body->setLinearVelocity(GLMToPx(v), autowake);
    }

    glm::vec3 getAngularVelocity() {
        return PxToGLM(body->getAngularVelocity());
    }

    void setAngularVelocity(glm::vec3 w, bool autowake = true) {
        return body->setAngularVelocity(GLMToPx(w), autowake);
    }
};

#endif //ANIMATION_PROJECT_PHYSICSBODY_H
