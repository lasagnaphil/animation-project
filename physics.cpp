#include <InputManager.h>
#include "App.h"
#include "PBRenderer.h"
#include "FlyCamera.h"
#include "PhysicsWorld.h"
#include "PosePhysicsBody.h"
#include "PhysicsBody.h"
#include "PoseRenderBody.h"
#include "PhysXDebugRenderer.h"
#include "MotionClipData.h"
#include "MotionClipPlayer.h"
#include "PoseKinematics.h"

PxDefaultAllocator gAllocator = {};
PxDefaultErrorCallback gErrorCallback = {};

struct PhysicsObject {
    PhysicsBody body;
    Ref<Mesh> mesh;
    Ref<PBRMaterial> material;
};

class MyApp : public App {
public:
    MyApp() : App(false, AppRenderSettings::PBR) {}

    void loadResources() override {
        FlyCamera* camera = initCamera<FlyCamera>();
        camera->movementSpeed = 1.0f;
        Ref<Transform> cameraTransform = camera->transform;
        cameraTransform->setPosition({0.0f, 0.0f, 1.0f});

        pbRenderer.dirLightProjVolume = {
                {-10.f, -10.f, 0.f}, {10.f, 10.f, 1000.f}
        };
        pbRenderer.shadowFramebufferSize = {2048, 2048};

        pbRenderer.dirLight.enabled = true;
        pbRenderer.dirLight.direction = glm::normalize(glm::vec3 {2.0f, -3.0f, -2.0f});
        pbRenderer.dirLight.color = {5.f, 5.f, 5.f};

        /*
        pbRenderer.pointLights[0].enabled = true;
        pbRenderer.pointLights[0].position = {-1.0f, -1.0f + 1.75f, 1.0f};
        pbRenderer.pointLights[0].color = {30.f, 30.f, 30.f};

        pbRenderer.pointLights[1].enabled = true;
        pbRenderer.pointLights[1].position = {1.0f, -1.0f + 1.75f, 1.0f};
        pbRenderer.pointLights[1].color = {30.f, 30.f, 30.f};

        pbRenderer.pointLights[2].enabled = true;
        pbRenderer.pointLights[2].position = {-10.0f, 10.0f + 17.5f, 10.0f};
        pbRenderer.pointLights[2].color = {300.f, 300.f, 300.f};

        pbRenderer.pointLights[3].enabled = true;
        pbRenderer.pointLights[3].position = {10.0f, 10.0f + 17.5f, 10.0f};
        pbRenderer.pointLights[3].color = {300.f, 300.f, 300.f};
         */

        // Initialize the PhysX Engine
        pxFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
        if (!pxFoundation) {
            fprintf(stderr, "PxCreateFoundation Failed!\n");
            exit(EXIT_FAILURE);
        }
        world.init(pxFoundation, 1);

        pxDebugRenderer.init(world);
        pxDebugRenderer.setCamera(camera);

        // Prepare the ground

        Ref<PBRMaterial> groundMat = PBRMaterial::quick(
                "resources/textures/mossy-ground1-albedo.png",
                "resources/textures/mossy-ground1-metal.png",
                "resources/textures/mossy-ground1-roughness.png",
                "resources/textures/mossy-ground1-ao.png");

        Ref<Mesh> groundMesh = Mesh::makePlane(100.0f, 100.0f);

        // Prepare the box
        if (enableBox) {
            box.mesh = Mesh::makeCube(boxSize);
            // box.mesh = Mesh::fromOBJFile("resources/box_edited.obj");
            box.material = Resources::make<PBRMaterial>();
            box.material->texAlbedo = Texture::fromSingleColor(glm::vec3(0.4f));
            box.material->texAO = Texture::fromSingleColor(glm::vec3(1.0f, 0.0f, 0.0f));
            box.material->texMetallic = Texture::fromSingleColor(glm::vec3(0.5f, 0.0f, 0.0f));
            box.material->texRoughness = Texture::fromSingleColor(glm::vec3(0.5f, 0.0f, 0.0f));
            box.mesh->indices.clear(); // We don't need the index buffer
            /*
            auto collider = box.mesh->generateCollider();
            auto bodyOpt = PhysicsBody::fromMesh(world, collider, world.physics->createMaterial(0.5f, 0.5f, 0.6f));
            if (!bodyOpt) {
                fprintf(stderr, "PhysicsBody::fromMesh() failed to generate mesh.\n");
                exit(EXIT_FAILURE);
            }
            box.body = *bodyOpt;
             */
            box.body = PhysicsBody::box(world, world.physics->createMaterial(0.5f, 0.5f, 0.6f), 1.0f,
                    {}, glm::identity<glm::quat>(), boxSize);
            // box.body.body->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);

            box.body.body->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
        }
        if (enableSpheres) {
            // Prepare the spheres
            glm::vec3 colorList[] = {
                    colors::Red, colors::Blue, colors::Green, colors::Black, colors::Yellow, colors::Cyan, colors::Pink,
                    colors::White, colors::Magenta, colors::Gold
            };
            auto texAO = Texture::fromSingleColor({1.0f, 0.0f, 0.0f});
            auto texMetallic = Texture::fromSingleColor({0.5f, 0.0f, 0.0f});
            auto texRoughness = Texture::fromSingleColor({0.5f, 0.0f, 0.0f});

            std::vector<Ref<PBRMaterial>> sphereMats(sphereColorsCount);
            for (int i = 0; i < sphereColorsCount; i++) {
                sphereMats[i] = Resources::make<PBRMaterial>();
                sphereMats[i]->texAlbedo = Texture::fromSingleColor(colorList[i]);
                sphereMats[i]->texAO = texAO;
                sphereMats[i]->texMetallic = texMetallic;
                sphereMats[i]->texRoughness = texRoughness;
            }

            Ref<Mesh> sphereMesh = Mesh::makeSphere(sphereRadius);

            spheres.resize(sphereCount);
            for (int i = 0; i < sphereCount; i++) {
                spheres[i].body = PhysicsBody::sphere(world, world.physics->createMaterial(0.5f, 0.5f, 0.6f), 0.5f,
                        {}, sphereRadius);
                spheres[i].mesh = sphereMesh;
                spheres[i].material = sphereMats[i % sphereColorsCount];
            }
        }

        // Prepare motion clip

        bvh = MotionClipData::loadFromFile("resources/retargetted/111-17(pregnant_pick_up).bvh", 0.01f);
        if (!bvh.valid) {
            fprintf(stderr, "BVH load failed!\n");
            exit(EXIT_FAILURE);
        }
        poseTree = bvh.poseTree;
        // for (auto& poseState : bvh.poseStates) {
            // poseState.v.y -= poseTree[0].offset.y;
        //}
        currentPose = bvh.poseStates[0];
        convertedPose = currentPose;

        motionClipPlayer = MotionClipPlayer(&bvh);
        motionClipPlayer.init();

        // Prepare the human body

        Ref<PBRMaterial> poseBodyMat = Resources::make<PBRMaterial>();
        poseBodyMat->texAlbedo = Texture::fromSingleColor({0.5f, 0.0f, 0.0f});
        poseBodyMat->texAO = Texture::fromSingleColor({1.0f, 0.0f, 0.0f});
        poseBodyMat->texMetallic =
                Texture::fromSingleColor({0.5f, 0.0f, 0.0f});
        poseBodyMat->texRoughness =
                Texture::fromSingleColor({0.5f, 0.0f, 0.0f});

        Ref<PBRMaterial> poseBodyMat2 = Resources::clone<PBRMaterial>(poseBodyMat);
        poseBodyMat2->texAlbedo = Texture::fromSingleColor({0.0f, 0.5f, 0.0f});

        poseRenderBody = PoseRenderBodyPBR::createAsBoxes(poseTree, 0.02f, poseBodyMat);
        poseRenderBody2 = PoseRenderBodyPBR::createAsBoxes(poseTree, 0.02f, poseBodyMat2);

        posePhysicsBodySkel = PosePhysicsBodySkel::fromFile("resources/humanoid_complex_edited.xml", poseTree);
        posePhysicsBody.init(world, poseTree, posePhysicsBodySkel);
        posePhysicsBody.setRoot(glmx::transform(glm::vec3(0.f, 0.9f, 0.f)));

        // Now attach the box to the hands
        /*
        auto [leftHandJoint, rightHandJoint] = posePhysicsBody.attachBoxToHands(world, poseTree, box.body,
                PxTransform(PxVec3(-boxSize.x/2, 0, 0)), PxTransform(PxVec3(boxSize.x/2, 0, 0)), 0.03f);
                */

        reset();
    }

    void reset() {
        enableRagdoll = false;
        motionClipPlayer.setFrame(27);
        currentPose = motionClipPlayer.getPoseState();
        posePhysicsBody.setPose(currentPose, poseTree);

        if (enableBox) {
            box.body.setPosition(glm::vec3(0.4f, 0.0f, 0.4f));
            box.body.setLinearVelocity({});
            box.body.setAngularVelocity({});
        }

        if (enableSpheres) {
            for (int i = 0; i < sphereCount; i++) {
                spheres[i].body.setPosition(
                        glm::vec3((i % 10) * 2 * (sphereRadius + sphereDist),
                                  sphereRadius,
                                  (i / 10) * 2 * (sphereRadius + sphereDist)));
                spheres[i].body.setLinearVelocity({});
                spheres[i].body.setAngularVelocity({});
            }
        }
    }

    void processInput(SDL_Event &event) override {
    }

    void update(float dt) override {
        static float time = 0.f;
        const float physicsDt = 1.0f / 120.0f;

        time += dt;
        auto inputMgr = InputManager::get();
        if (inputMgr->isKeyEntered(SDL_SCANCODE_SPACE)) {
            enablePhysics = !enablePhysics;
        }
        if (inputMgr->isKeyEntered(SDL_SCANCODE_F1)) {
            enableDebugRendering = !enableDebugRendering;
        }
        motionClipPlayer.update(dt);
        while (time >= physicsDt) {
            time -= physicsDt;

            if (enableManipulation) {
                posePhysicsBody.setPose(currentPose, poseTree);
                if (enablePhysics) {
                    bool advanced = world.advance(physicsDt);
                    if (advanced) {
                        world.fetchResults();
                    }
                }
                posePhysicsBody.getPose(convertedPose, poseTree);
            }
            else if (enableRagdoll) {
                if (enablePhysics) {
                    bool advanced = world.advance(physicsDt);
                    if (advanced) {
                        world.fetchResults();
                    }
                }
                posePhysicsBody.getPose(currentPose, poseTree);
            }
            else {
                currentPose = motionClipPlayer.getPoseState();
                uint32_t leftHandIdx = poseTree.findIdx("LeftHand");
                uint32_t rightHandIdx = poseTree.findIdx("RightHand");
                uint32_t leftRelevantIndices[] = {
                        poseTree.findIdx("LeftArm"), poseTree.findIdx("LeftForeArm")
                };
                uint32_t rightRelevantIndices[] = {
                        poseTree.findIdx("RightArm"), poseTree.findIdx("RightForeArm")
                };
                auto leftHandTrans = calcFK(poseTree, currentPose, leftHandIdx);
                auto rightHandTrans = calcFK(poseTree, currentPose, rightHandIdx);
                glmx::transform boxTrans;
                boxTrans.v.x = 0.5f * (leftHandTrans.v.x + rightHandTrans.v.x);
                boxTrans.v.y = 0.5f * (leftHandTrans.v.y + rightHandTrans.v.y);
                boxTrans.v.z = currentPose.v.z - 0.4f;
                boxTrans.q = glm::identity<glm::quat>();
                boxLeftPos = boxTrans.v - glm::rotate(boxTrans.q, {boxSize.x / 2, 0, 0});
                boxRightPos = boxTrans.v + glm::rotate(boxTrans.q, {boxSize.x / 2, 0, 0});
                box.body.setTransform(boxTrans);

                solveTwoJointIK(poseTree, currentPose,
                                poseTree.findIdx("LeftArm"), poseTree.findIdx("LeftForeArm"), poseTree.findIdx("LeftHand"),
                                boxLeftPos);

                solveTwoJointIK(poseTree, currentPose,
                                poseTree.findIdx("RightArm"), poseTree.findIdx("RightForeArm"), poseTree.findIdx("RightHand"),
                                boxRightPos);

                posePhysicsBody.setPose(currentPose, poseTree);

                bool advanced = world.advance(physicsDt);
                if (advanced) {
                    world.fetchResults();
                }
            }
        }
    }


    void render() override {
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // pbRenderer.queueRender({groundMesh, groundMat, rootTransform->getWorldTransform()});
        renderMotionClip(pbRenderer, imRenderer, currentPose, poseTree, poseRenderBody);
        // renderMotionClip(pbRenderer, imRenderer, convertedPose, poseTree, poseRenderBody2);

        if (enableBox) {
            auto boxTrans = box.body.getTransform();
            pbRenderer.queueRender({box.mesh, box.material, glm::translate(boxTrans.v) * glm::mat4_cast(boxTrans.q)});
        }
        if (enableSpheres) {
            for (auto& sphere : spheres) {
                pbRenderer.queueRender({sphere.mesh, sphere.material, glm::translate(sphere.body.getTransform().v)});
            }
        }
        pbRenderer.render();

        // imRenderer.drawPoint(pbRenderer.pointLights[0].position, colors::Yellow, 4.0f, true);
        // imRenderer.drawPoint(pbRenderer.pointLights[1].position, colors::Yellow, 4.0f, true);
        // imRenderer.drawPoint(pbRenderer.pointLights[2].position, colors::Yellow, 4.0f, true);
        // imRenderer.drawPoint(pbRenderer.pointLights[3].position, colors::Yellow, 4.0f, true);
        imRenderer.drawAxisTriad(glm::mat4(1.0f), 0.1f, 1.0f, false);
        imRenderer.drawSphere(boxLeftPos, colors::Blue, 0.05f, true);
        imRenderer.drawSphere(boxRightPos, colors::Blue, 0.05f, true);
        glm::vec3 leftHandPos = calcFK(poseTree, currentPose, poseTree.findIdx("LeftHand")).v;
        glm::vec3 rightHandPos = calcFK(poseTree, currentPose, poseTree.findIdx("RightHand")).v;
        imRenderer.drawSphere(leftHandPos, colors::Green, 0.05f, true);
        imRenderer.drawSphere(rightHandPos, colors::Green, 0.05f, true);
        imRenderer.render();

        if (enableDebugRendering) {
            pxDebugRenderer.render(world);
        }

        motionClipPlayer.renderImGui();

        renderImGui();
    }

    void renderImGui() {
        ImGui::Begin("Character Data");

        ImGui::Checkbox("Enable Manipulation", &enableManipulation);
        if (!enableManipulation) {
            if (ImGui::Button("Ragdoll")) {
                enableRagdoll = true;
                posePhysicsBody.getPose(convertedPose, poseTree);
                posePhysicsBody.setPose(currentPose, poseTree);
            }
            ImGui::SameLine();
            if (ImGui::Button("Reset")) {
                reset();
            }
        }

        if (ImGui::TreeNode("Kinematics")) {
            ImGui::DragFloat3((poseTree[0].name + " pos").c_str(), (float*)&currentPose.v, 0.01f);
            for (uint32_t i = 0; i < poseTree.numJoints; i++) {
                auto& node = poseTree[i];
                PxVec3 v = quatToTwistSwing(GLMToPx(currentPose.q[i]));
                if (ImGui::DragFloat3(node.name.c_str(), (float*)&v, 0.01f)) {
                    currentPose.q[i] = PxToGLM(twistSwingToQuat(v));
                }
            }
            ImGui::TreePop();
        }

        if (ImGui::TreeNode("PhysX")) {
            posePhysicsBody.renderImGui();
            ImGui::TreePop();
        }

        ImGui::End();
    }

    void release() override {
        world.release();
    }

private:
    const int N = 7;

    glmx::pose currentPose, convertedPose;
    PoseTree poseTree;
    MotionClipData bvh;
    MotionClipPlayer motionClipPlayer;
    PoseRenderBodyPBR poseRenderBody, poseRenderBody2;

    PxFoundation* pxFoundation;
    PhysicsWorld world;
    PhysXDebugRenderer pxDebugRenderer;
    PosePhysicsBody posePhysicsBody;
    PosePhysicsBodySkel posePhysicsBodySkel;

    const int sphereColorsCount = 10;
    const int sphereCount = 100;
    const float sphereRadius = 0.025f;
    const float sphereDist = 0.001f;

    glm::vec3 boxSize = {0.3f, 0.3f, 0.3f};

    PhysicsObject box;
    std::vector<PhysicsObject> spheres;

    bool enableDebugRendering = true;

    bool enableRagdoll = false;
    bool enableManipulation = false;
    bool enablePhysics = true;

    bool enableBox = true;
    bool enableSpheres = false;

    glm::vec3 boxLeftPos, boxRightPos;
};

int main(int argc, char** argv) {
    MyApp app;
    app.start();

    return 0;
}
