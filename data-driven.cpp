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

        // Prepare the ground
        groundMesh = Mesh::makePlane(100.0f, 100.0f);
        groundMat = PBRMaterial::quick(
                "resources/textures/mossy-ground1-albedo.png",
                "resources/textures/mossy-ground1-metal.png",
                "resources/textures/mossy-ground1-roughness.png",
                "resources/textures/mossy-ground1-ao.png");
        
        // Prepare the box
        // glm::vec3 boxSize = {0.5f, 0.5f, 0.5f};
        // box.mesh = Mesh::makeCube(boxSize);
        box.mesh = Mesh::fromOBJFile("resources/box.obj");
        box.material = Resources::make<PBRMaterial>();
        box.material->texAlbedo = Texture::fromSingleColor(glm::vec3(0.4f));
        box.material->texAO = Texture::fromSingleColor(glm::vec3(1.0f, 0.0f, 0.0f));
        box.material->texMetallic = Texture::fromSingleColor(glm::vec3(0.5f, 0.0f, 0.0f));
        box.material->texRoughness = Texture::fromSingleColor(glm::vec3(0.5f, 0.0f, 0.0f));
        box.mesh->indices.clear(); // We don't need the index buffer
        // auto collider = box.mesh->generateCollider();
        // auto bodyOpt = PhysicsBody::fromMesh(world, collider, world.physics->createMaterial(0.5f, 0.5f, 0.6f));
        // if (!bodyOpt) {
        //     fprintf(stderr, "PhysicsBody::fromMesh() failed to generate mesh.\n");
        //     exit(EXIT_FAILURE);
        // }
        // box.body = *bodyOpt;
        box.body = PhysicsBody::ourBox(world, 0.25f, 0.25f, 0.15f, 0.05f, world.physics->createMaterial(0.5f, 0.5f, 0.6f));

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
            spheres[i].body = PhysicsBody::sphere(world, world.physics->createMaterial(0.5f, 0.5f, 0.6f), {}, sphereRadius);
            spheres[i].mesh = sphereMesh;
            spheres[i].material = sphereMats[i % sphereColorsCount];

        }

        // Prepare motion clip

        bvh = MotionClipData::loadFromFile("resources/127_25_1.bvh", 0.01f);
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
        posePhysicsBody.putToSleep();

        pxDebugRenderer.init(world);
        pxDebugRenderer.setCamera(camera);

        reset();
    }

    void reset() {
        enableRagdoll = false;
        motionClipPlayer.setFrame(27);
        currentPose = motionClipPlayer.getPoseState();
        posePhysicsBody.setPose(currentPose, poseTree);

        for (int i = 0; i < sphereCount; i++) {
            spheres[i].body.setPosition(
                glm::vec3((i % 10) * 2 * (sphereRadius + sphereDist),
                          sphereRadius,
                          (i / 10) * 2 * (sphereRadius + sphereDist)));
            spheres[i].body.setLinearVelocity({});
            spheres[i].body.setAngularVelocity({});
        }

        // box
        //box.body.setPosition({0.0f, 0.0f, 1.0f});
        //box.body.setRotation(glm::angleAxis((float)-M_PI/2, glm::vec3(1, 0, 0)));
        //box.body.setPosition({0.0f, 0.0f, 1.0f});
        //box.body.setPosition({1.0f, 1.0f, 1.0f});
        box.body.setTransform(glmx::transform(
            glm::vec3(0.0f, 0.0f, 1.0f), 
            glm::angleAxis((float)-M_PI/2, glm::vec3(1, 0, 0))));
        box.body.setLinearVelocity({});
        box.body.setAngularVelocity({});
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
        motionClipPlayer.update(dt);
        while (time >= physicsDt) {
            time -= physicsDt;

            if (enableManipulation) {
                posePhysicsBody.setPose(currentPose, poseTree, true);
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
                // posePhysicsBody.getPose(convertedPose, poseTree);
            }
            else {
                currentPose = motionClipPlayer.getPoseState();
                posePhysicsBody.setPose(currentPose, poseTree, true);
                // posePhysicsBody.getPose(convertedPose, poseTree);

                /*
                bool advanced = world.advance(physicsDt);
                if (advanced) {
                    world.fetchResults();
                }
                 */
            }
        }
    }


    void render() override {
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        pbRenderer.queueRender({groundMesh, groundMat, rootTransform->getWorldTransform()});
        renderMotionClip(pbRenderer, imRenderer, currentPose, poseTree, poseRenderBody);
        renderMotionClip(pbRenderer, imRenderer, convertedPose, poseTree, poseRenderBody2);

        pbRenderer.queueRender({box.mesh, box.material, glmx::mat4_cast(box.body.getTransform())});
        for (auto& sphere : spheres) {
            pbRenderer.queueRender({sphere.mesh, sphere.material, glm::translate(sphere.body.getTransform().v)});
        }
        pbRenderer.render();

        imRenderer.drawPoint(pbRenderer.pointLights[0].position, colors::Yellow, 4.0f, true);
        imRenderer.drawPoint(pbRenderer.pointLights[1].position, colors::Yellow, 4.0f, true);
        // imRenderer.drawPoint(pbRenderer.pointLights[2].position, colors::Yellow, 4.0f, true);
        // imRenderer.drawPoint(pbRenderer.pointLights[3].position, colors::Yellow, 4.0f, true);
        imRenderer.drawAxisTriad(glm::mat4(1.0f), 0.1f, 1.0f, false);
        imRenderer.render();

        pxDebugRenderer.render(world);

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
                glm::vec3 v = glmx::quatToEuler(currentPose.q[i], EulOrdZYXs);
                if (ImGui::DragFloat3(node.name.c_str(), (float*)&v, 0.01f)) {
                    currentPose.q[i] = glmx::eulerToQuat(v, EulOrdZYXs);
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

    PhysicsObject box;
    std::vector<PhysicsObject> spheres;

    Ref<PBRMaterial> groundMat;
    Ref<Mesh> groundMesh;

    bool enableRagdoll = false;
    bool enableManipulation = false;
    bool enablePhysics = true;
};

int main(int argc, char** argv) {
    MyApp app;
    app.start();

    return 0;
}

// //
// // Created by lasagnaphil on 19. 11. 6..
// //

// #include <InputManager.h>
// #include "App.h"
// #include "PhongRenderer.h"
// #include "DebugRenderer.h"
// #include "FlyCamera.h"
// #include "glmx/pose.h"
// #include "PoseRenderBody.h"
// #include "PoseKinematics.h"
// #include "AnimStateMachine.h"

// #include "Box.h"

// #include <glmx/euler.h>

// class MyApp : public App {
// public:
//     MyApp() : App(false) {}

//     void loadResources() override {
//         FlyCamera* camera = initCamera<FlyCamera>();
//         camera->transform->setPosition({0.f, 1.f, 2.f});

//         Ref<Image> checkerImage = Image::fromFile("resources/textures/checker.png");
//         Ref<Texture> planeTexture = Texture::fromImage(checkerImage);
//         checkerImage.release();

//         groundMat = Resources::make<PhongMaterial>();
//         groundMat->ambient = {0.1f, 0.1f, 0.1f, 1.0f};
//         groundMat->specular = {0.7f, 0.7f, 0.7f, 1.0f};
//         groundMat->shininess = 32.0f;
//         groundMat->texDiffuse = planeTexture;
//         groundMat->texSpecular = {};

//         groundMesh = Mesh::makePlane(1000.0f, 100.0f);

//         // Create box
//         // Ref<Image> cubeImage = Image::fromFile("resources/textures/container2.png");
//         // Ref<Texture> cubeTexture = Texture::fromImage(cubeImage);
//         // cubeImage.release();

//         // Ref<Image> cubeSpecularImage = Image::fromFile("resources/textures/container2_specular.png");
//         // Ref<Texture> cubeSpecularTexture = Texture::fromImage(cubeSpecularImage);
//         // cubeSpecularImage.release();

//         // boxMat = Resources::make<PhongMaterial>();
//         // boxMat->ambient = {0.0f, 0.0f, 0.0f, 1.0f};
//         // boxMat->shininess = 32.0f;
//         // boxMat->texDiffuse = cubeTexture;
//         // boxMat->texSpecular = cubeSpecularTexture;

//         // boxMesh = Mesh::makeCube({1.0f, 0.6f, 0.4f});

//         // Create empty pose
//         // Material of human
//         Ref<PhongMaterial> bodyMat = Resources::make<PhongMaterial>();
//         bodyMat->ambient = {0.1f, 0.1f, 0.1f, 1.0f};
//         bodyMat->specular = {0.7f, 0.7f, 0.7f, 1.0f};
//         bodyMat->diffuse = {1.0f, 0.0f, 0.0f, 1.0f};
//         bodyMat->shininess = 64.0f;
//         bodyMat->texDiffuse = {};
//         bodyMat->texSpecular = {};

//         Ref<PhongMaterial> debugBodyMat1 = Resources::make<PhongMaterial>();
//         debugBodyMat1->ambient = {0.1f, 0.1f, 0.1f, 1.0f};
//         debugBodyMat1->specular = {0.7f, 0.7f, 0.7f, 1.0f};
//         debugBodyMat1->diffuse = {0.0f, 1.0f, 0.0f, 1.0f};
//         debugBodyMat1->shininess = 64.0f;
//         debugBodyMat1->texDiffuse = {};
//         debugBodyMat1->texSpecular = {};

//         Ref<PhongMaterial> debugBodyMat2 = Resources::make<PhongMaterial>();
//         debugBodyMat2->ambient = {0.1f, 0.1f, 0.1f, 1.0f};
//         debugBodyMat2->specular = {0.7f, 0.7f, 0.7f, 1.0f};
//         debugBodyMat2->diffuse = {0.0f, 0.0f, 1.0f, 1.0f};
//         debugBodyMat2->shininess = 64.0f;
//         debugBodyMat2->texDiffuse = {};
//         debugBodyMat2->texSpecular = {};

//         auto idleBVH = MotionClipData::loadFromFile("resources/retargetted/111-36(pregnant_carry).bvh", 0.1f);
//         auto walkBVH = MotionClipData::loadFromFile("resources/retargetted/111-36(pregnant_carry).bvh", 0.1f);

//         poseTree = idleBVH.poseTree;
//         currentPose = glmx::pose::empty(poseTree.numJoints);
//         //currentPose.v.y = 1.05f;

//         poseRenderBody = PoseRenderBody::createAsBoxes(poseTree, 0.05f, bodyMat);
//         debugPoseRenderBody1 = PoseRenderBody::createAsBoxes(poseTree, 0.05f, debugBodyMat1);
//         debugPoseRenderBody2 = PoseRenderBody::createAsBoxes(poseTree, 0.05f, debugBodyMat2);

//         // auto runBVH = MotionClipData::loadFromFile("resources/motion/cmu/16_55_run.bvh", 0.01f);
//         //auto jumpBVH = MotionClipData::loadFromFile("../resources/motion/16_03_high jump.bvh", 0.01f);
//         // auto forwardJumpBVH = MotionClipData::loadFromFile("resources/motion/cmu/16_05_forward jump.bvh", 0.01f);
//         // auto walkVeerLeftBVH = MotionClipData::loadFromFile("resources/motion/cmu/16_11_walk, veer left.bvh", 0.01f);
//         // auto walkVeerRightBVH = MotionClipData::loadFromFile("resources/motion/cmu/16_13_walk, veer right.bvh", 0.01f);
//         // auto walkTurnLeftBVH = MotionClipData::loadFromFile("resources/motion/cmu/16_17_walk, 90-degree left turn.bvh", 0.01f);
//         // auto walkTurnRightBVH = MotionClipData::loadFromFile("resources/motion/cmu/16_19_walk, 90-degree right turn.bvh", 0.01f);
//         // auto runVeerLeftBVH = MotionClipData::loadFromFile("resources/motion/cmu/16_48_run, veer left.bvh", 0.01f);
//         // auto runVeerRightBVH = MotionClipData::loadFromFile("resources/motion/cmu/16_49_run, veer right.bvh", 0.01f);
//         // auto runTurnLeftBVH = MotionClipData::loadFromFile("resources/motion/cmu/16_51_run, 90-degree left turn.bvh", 0.01f);
//         // auto runTurnRightBVH = MotionClipData::loadFromFile("resources/motion/cmu/16_53_run, 90-degree right turn.bvh", 0.01f);

//         auto idlePoses = std::vector<glmx::pose>(30, idleBVH.poseStates[0]);
//         auto walkPoses = nonstd::span<glmx::pose>(walkBVH.poseStates.data() + 230, 72);
//         //auto walkPoses = nonstd::span<glmx::pose>(walkBVH.poseStates.data() + 1010, 300);
//         // auto walkTurnLeftPoses = nonstd::span<glmx::pose>(walkTurnLeftBVH.poseStates.data() + 40, walkTurnLeftBVH.poseStates.size() - 60);
//         // auto walkTurnRightPoses = nonstd::span<glmx::pose>(walkTurnRightBVH.poseStates.data() + 40, walkTurnRightBVH.poseStates.size() - 60);
//         // auto runPoses = nonstd::span<glmx::pose>(runBVH.poseStates.data() + 24, runBVH.poseStates.size() - 24);
//         // auto runTurnLeftPoses = nonstd::span<glmx::pose>(runTurnLeftBVH.poseStates.data() + 10, runTurnLeftBVH.poseStates.size() - 20);
//         // auto runTurnRightPoses = nonstd::span<glmx::pose>(runTurnRightBVH.poseStates.data() + 10, runTurnRightBVH.poseStates.size() - 20);

//         auto idleAnim = animFSM.addAnimation("idle", nonstd::span<glmx::pose>(idlePoses.data(), idlePoses.size()));
//         auto walkAnim = animFSM.addAnimation("walk", walkPoses, 30);
//         // auto walkVeerLeftAnim = animFSM.addAnimation("walk_veer_left", walkVeerLeftBVH.poseStates);
//         // auto walkVeerRightAnim = animFSM.addAnimation("walk_veer_right", walkVeerRightBVH.poseStates);
//         // auto walkTurnLeftAnim = animFSM.addAnimation("walk_turn_left", walkTurnLeftPoses);
//         // auto walkTurnRightAnim = animFSM.addAnimation("walk_turn_right", walkTurnRightPoses);
//         // auto runAnim = animFSM.addAnimation("run", runPoses);
//         // auto runVeerLeftAnim = animFSM.addAnimation("run_veer_left", runVeerLeftBVH.poseStates);
//         // auto runVeerRightAnim = animFSM.addAnimation("run_veer_right", runVeerRightBVH.poseStates);
//         // auto runTurnLeftAnim = animFSM.addAnimation("run_turn_left", runTurnLeftBVH.poseStates);
//         // auto runTurnRightAnim = animFSM.addAnimation("run_turn_right", runTurnRightBVH.poseStates);
//         // auto jumpAnim = animFSM.addAnimation("jump", jumpBVH.poseStates);
//         // auto forwardJumpAnim = animFSM.addAnimation("forward_jump", forwardJumpBVH.poseStates);

//         for (Ref<Animation> anim : { idleAnim, walkAnim })
//         // , walkAnim, walkVeerLeftAnim, walkVeerRightAnim,
//         //                             walkTurnLeftAnim, walkTurnRightAnim,
//         //                             runAnim, runVeerLeftAnim, runVeerRightAnim, runTurnLeftAnim, runTurnRightAnim,
//         //                             jumpAnim, forwardJumpAnim}) {
//         {

//             animFSM.get(anim)->setStartingRootPos(0.0f, 0.0f);
//         }

//         auto idleState = animFSM.addState("idle", idleAnim);
//         auto walkState = animFSM.addState("walk", walkAnim);
//         // auto runState = animFSM.addState("run", runAnim);
//         // auto jumpState = animFSM.addState("jump", jumpAnim);
//         // auto forwardJumpState = animFSM.addState("forward_jump", forwardJumpAnim);
//         // auto walkVeerLeftState = animFSM.addState("walk_veer_left", walkVeerLeftAnim);
//         // auto walkVeerRightState = animFSM.addState("walk_veer_right", walkVeerRightAnim);
//         // auto walkTurnLeftState = animFSM.addState("walk_turn_left", walkTurnLeftAnim);
//         // auto walkTurnRightState = animFSM.addState("walk_turn_right", walkTurnRightAnim);
//         // auto runVeerLeftState = animFSM.addState("run_veer_left", runVeerLeftAnim);
//         // auto runVeerRightState = animFSM.addState("run_veer_right", runVeerRightAnim);
//         // auto runTurnLeftState = animFSM.addState("run_turn_left", runTurnLeftAnim);
//         // auto runTurnRightState = animFSM.addState("run_turn_right", runTurnRightAnim);

//         animFSM.addParam("is_walking", false);
//         // animFSM.addParam("is_walking_veer_left");
//         // animFSM.addParam("is_walking_veer_right");
//         // animFSM.addParam("is_walking_turn_left");
//         // animFSM.addParam("is_walking_turn_right");
//         // animFSM.addParam("is_running", false);
//         // animFSM.addParam("is_running_veer_left");
//         // animFSM.addParam("is_running_veer_right");
//         // animFSM.addParam("is_running_turn_left");
//         // animFSM.addParam("is_running_turn_right");
//         // animFSM.addParam("jump");
//         // animFSM.addParam("forward_jump");

//         auto repeatIdleTrans = animFSM.addTransition("repeat_idle", idleState, idleState, 0.0f, 0.0f, 0.0f);

//         auto repeatWalkingTrans = animFSM.addTransition("repeat_walking", walkState, walkState, 1.0f, 1.0f, 1.0f);

//         auto startWalkingTrans = animFSM.addTransition("start_walking", idleState, walkState, 1.0f, 1.0f, 1.0f);
//         animFSM.setTransitionCondition(startWalkingTrans, "is_walking", true);

//         auto stopWalkingTrans = animFSM.addTransition("stop_walking", walkState, idleState, 1.0f, 1.0f, 1.0f);
//         animFSM.setTransitionCondition(stopWalkingTrans, "is_walking", false);

//         // auto startWalkVeerLeftTrans = animFSM.addTransition("start_walk_veer_left", walkState, walkVeerLeftState,
//         //         0.2f, 0.2f, 0.2f);
//         // animFSM.setTransitionTrigger(startWalkVeerLeftTrans, "is_walking_veer_left");

//         // auto stopWalkVeerLeftTrans = animFSM.addTransition("stop_walk_veer_left", walkVeerLeftState, walkState,
//         //                                                     0.2f, 0.2f, 0.2f);

//         // auto startWalkVeerRightTrans = animFSM.addTransition("start_walk_veer_right", walkState, walkVeerRightState,
//         //         0.2f, 0.2f, 0.2f);
//         // animFSM.setTransitionTrigger(startWalkVeerRightTrans, "is_walking_veer_right");

//         // auto stopWalkVeerRightTrans = animFSM.addTransition("stop_walk_veer_right", walkVeerRightState, walkState,
//         //                                                      0.2f, 0.2f, 0.2f);

//         // auto startWalkTurnLeftTrans = animFSM.addTransition("start_walk_turn_left", walkState, walkTurnLeftState,
//         //         0.4f, 0.4f, 0.4f);
//         // animFSM.setTransitionTrigger(startWalkTurnLeftTrans, "is_walking_turn_left");

//         // auto stopWalkTurnLeftTrans = animFSM.addTransition("stop_walk_turn_left", walkTurnLeftState, walkState,
//         //         0.4f, 0.4f, 0.4f);

//         // auto startWalkTurnRightTrans = animFSM.addTransition("start_walk_turn_right", walkState, walkTurnRightState,
//         //                                                     0.4f, 0.4f, 0.4f);
//         // animFSM.setTransitionTrigger(startWalkTurnRightTrans, "is_walking_turn_right");

//         // auto stopWalkTurnRightTrans = animFSM.addTransition("stop_walk_turn_right", walkTurnRightState, walkState,
//         //                                                    0.4f, 0.4f, 0.4f);

//         // auto repeatRunningTrans = animFSM.addTransition("repeat_running", runState, runState, 0.1f, 0.1f, 0.1f);

//         // auto startRunningTrans = animFSM.addTransition("start_running", walkState, runState, 0.2f, 0.2f, 0.2f);
//         // animFSM.setTransitionCondition(startRunningTrans, "is_running", true);

//         // auto stopRunningTrans = animFSM.addTransition("stop_running", runState, walkState, 0.2f, 0.2f, 0.2f);
//         // animFSM.setTransitionCondition(stopRunningTrans, "is_running", false);

//         // auto startRunVeerLeftTrans = animFSM.addTransition("start_run_veer_left", runState, runVeerLeftState,
//         //                                                    0.3f, 0.3f, 0.3f);
//         // animFSM.setTransitionTrigger(startRunVeerLeftTrans, "is_running_veer_left");

//         // auto stopRunVeerLeftTrans = animFSM.addTransition("stop_run_veer_left", runVeerLeftState, runState,
//         //                                                   0.3f, 0.3f, 0.3f);

//         // auto startRunVeerRightTrans = animFSM.addTransition("start_run_veer_right", runState, runVeerRightState,
//         //                                                     0.3f, 0.3f, 0.3f);
//         // animFSM.setTransitionTrigger(startRunVeerRightTrans, "is_running_veer_right");

//         // auto stopRunVeerRightTrans = animFSM.addTransition("stop_run_veer_right", runVeerRightState, runState,
//         //                                                   0.3f, 0.3f, 0.3f);

//         // auto startRunTurnLeftTrans = animFSM.addTransition("start_run_turn_left", runState, runTurnLeftState,
//         //                                                     0.3f, 0.3f, 0.3f);
//         // animFSM.setTransitionTrigger(startRunTurnLeftTrans, "is_running_turn_left");

//         // auto stopRunTurnLeftTrans = animFSM.addTransition("stop_run_turn_left", runTurnLeftState, runState,
//         //                                                    0.3f, 0.3f, 0.3f);

//         // auto startRunTurnRightTrans = animFSM.addTransition("start_run_turn_right", runState, runTurnRightState,
//         //                                                      0.3f, 0.3f, 0.3f);
//         // animFSM.setTransitionTrigger(startRunTurnRightTrans, "is_running_turn_right");

//         // auto stopRunTurnRightTrans = animFSM.addTransition("stop_run_turn_right", runTurnRightState, runState,
//         //                                                     0.3f, 0.3f, 0.3f);

//         // auto jumpTrans = animFSM.addTransition("jumping", idleState, jumpState, 0.2f, 0.2f, 0.2f);
//         // animFSM.setTransitionTrigger(jumpTrans, "jump");

//         // auto stopJumpTrans = animFSM.addTransition("stop_jumping", jumpState, idleState, 0.2f, 0.2f, 0.2f);

//         // auto forwardJumpTrans = animFSM.addTransition("forward_jumping", walkState, forwardJumpState, 0.2f, 0.2f, 0.2f);
//         // animFSM.setTransitionTrigger(forwardJumpTrans, "forward_jump");

//         // auto stopForwardJumpTrans = animFSM.addTransition("stop_forward_jumping", forwardJumpState, walkState,
//         //         0.2f, 0.2f, 0.2f);

//         animFSM.setCurrentState(idleState);
//     }

//     void processInput(SDL_Event &event) override {
//     }

//     void update(float dt) override {
//         if (fixCamera) {
//             glm::vec3 cameraPos = currentPose.v;
//             cameraPos.z += 5.f;
//             cameraPos.y += 1.f;
//             FlyCamera* camera = dynamic_cast<FlyCamera*>(this->camera.get());
//             camera->transform->setPosition(cameraPos);
//             camera->pitch = -40.0f;
//         }

//         static float time = 0.0f;
//         time += dt;
//         auto inputMgr = InputManager::get();
//         if (inputMgr->isMousePressed(SDL_BUTTON_LEFT)) {
//             glm::vec2 mPos = inputMgr->getMousePos();
//             Ray ray = camera->screenPointToRay(mPos);
//         }
//         if (inputMgr->isKeyEntered(SDL_SCANCODE_1)) {
//             fixCamera = !fixCamera;
//         }
//         if (inputMgr->isKeyPressed(SDL_SCANCODE_UP)) {
//             animFSM.setParam("is_walking", true);

//             if (inputMgr->isKeyPressed(SDL_SCANCODE_LSHIFT)) {
//                 animFSM.setParam("is_running", true);
//                 if (inputMgr->isKeyPressed(SDL_SCANCODE_LALT)) {
//                     if (inputMgr->isKeyPressed(SDL_SCANCODE_LEFT)) {
//                         animFSM.setTrigger("is_running_veer_left");
//                     }
//                     if (inputMgr->isKeyPressed(SDL_SCANCODE_RIGHT)) {
//                         animFSM.setTrigger("is_running_veer_right");
//                     }
//                 }
//                 else {
//                     if (inputMgr->isKeyPressed(SDL_SCANCODE_LEFT)) {
//                         animFSM.setTrigger("is_running_turn_left");
//                     }
//                     if (inputMgr->isKeyPressed(SDL_SCANCODE_RIGHT)) {
//                         animFSM.setTrigger("is_running_turn_right");
//                     }
//                 }
//             }

//             else {
//                 animFSM.setParam("is_running", false);
//                 if (inputMgr->isKeyPressed(SDL_SCANCODE_LALT)) {
//                     if (inputMgr->isKeyPressed(SDL_SCANCODE_LEFT)) {
//                         animFSM.setTrigger("is_walking_veer_left");
//                     }
//                     if (inputMgr->isKeyPressed(SDL_SCANCODE_RIGHT)) {
//                         animFSM.setTrigger("is_walking_veer_right");
//                     }
//                 }
//                 else {
//                     if (inputMgr->isKeyPressed(SDL_SCANCODE_LEFT)) {
//                         animFSM.setTrigger("is_walking_turn_left");
//                     }
//                     if (inputMgr->isKeyPressed(SDL_SCANCODE_RIGHT)) {
//                         animFSM.setTrigger("is_walking_turn_right");
//                     }
//                 }
//                 if (inputMgr->isKeyPressed(SDL_SCANCODE_SPACE)) {
//                     animFSM.setTrigger("forward_jump");
//                 }
//             }
//         }
//         else {
//             animFSM.setParam("is_walking", false);
//             animFSM.setParam("is_running", false);
//         }
//         if (inputMgr->isKeyPressed(SDL_SCANCODE_SPACE)) {
//             animFSM.setTrigger("jump");
//         }
//         animFSM.update(dt);
//         currentPose = animFSM.getCurrentPose();
//     }

//     void render() override {
//         // Render ground.
//         phongRenderer.queueRender({groundMesh, groundMat, rootTransform->getWorldTransform()});

//         // Render body.
//         renderMotionClip(phongRenderer, imRenderer, currentPose, poseTree, poseRenderBody);

//         // Render box.
//         // glm::mat4 boxTransform = Box::getTransformFromHands(currentPose, poseTree, "LeftHand", "RightHand");
//         // phongRenderer.queueRender({boxMesh, boxMat, boxTransform});

//         phongRenderer.render();

//         imRenderer.drawAxisTriad(glm::mat4(1.0f), 0.1f, 1.0f, false);
//         /*
//         if (animFSM.p1.size() > 0)
//             renderMotionClip(phongRenderer, imRenderer, animFSM.p1, poseTree, debugPoseRenderBody1);
//         if (animFSM.p2.size() > 0)
//             renderMotionClip(phongRenderer, imRenderer, animFSM.p2, poseTree, debugPoseRenderBody2);
//         */

//         glm::mat4 rootTransMat = glm::translate(currentPose.v) * glm::mat4_cast(currentPose.q[0]);
//         imRenderer.drawAxisTriad(rootTransMat, 0.05f, 0.5f, false);

//         imRenderer.render();

//         ImGui::SetNextWindowPos(ImVec2(1556, 13));
//         ImGui::SetNextWindowSize(ImVec2(339, 277));
//         phongRenderer.renderImGui();

//         ImGui::SetNextWindowPos(ImVec2(23, 158));
//         ImGui::SetNextWindowSize(ImVec2(447, 844));
//         animFSM.renderImGui(poseTree);
//     }

//     void release() override {
//     }

// private:
//     glmx::pose currentPose;

//     PoseTree poseTree;
//     PoseRenderBody poseRenderBody;
//     PoseRenderBody debugPoseRenderBody1, debugPoseRenderBody2;

//     AnimStateMachine animFSM;

//     Ref<PhongMaterial> groundMat;
//     Ref<Mesh> groundMesh;

//     Ref<PhongMaterial> boxMat;
//     Ref<Mesh> boxMesh;

//     bool fixCamera = false;
// };

// int main(int argc, char** argv) {
//     MyApp app;
//     app.start();

//     return 0;
// }

