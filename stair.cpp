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
#include "AnimStateMachine.h"

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

    void initGround() {
        groundMesh = Mesh::makePlane(100.0f, 100.0f);
        groundMat = PBRMaterial::quick(
                "resources/textures/mossy-ground1-albedo.png",
                "resources/textures/mossy-ground1-metal.png",
                "resources/textures/mossy-ground1-roughness.png",
                "resources/textures/mossy-ground1-ao.png");
    }

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

        initGround();

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
            box.mesh = Mesh::fromOBJFile("resources/box_edited2.obj");
            box.material = Resources::make<PBRMaterial>();
            box.material->texAlbedo = Texture::fromSingleColor(glm::vec3(0.4f));
            box.material->texAO = Texture::fromSingleColor(glm::vec3(1.0f, 0.0f, 0.0f));
            box.material->texMetallic = Texture::fromSingleColor(glm::vec3(0.5f, 0.0f, 0.0f));
            box.material->texRoughness = Texture::fromSingleColor(glm::vec3(0.5f, 0.0f, 0.0f));
            box.mesh->indices.clear(); // We don't need the index buffer
            box.body = PhysicsBody::ourBox(world, boxSize.x, boxSize.y, boxSize.z, boxThickness,
                    world.physics->createMaterial(0.5f, 0.5f, 0.6f));
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

        // Prepare stairs
        stairs.resize(stairCount);
        for(int i = 0; i < stairCount; i++) {
            glm::vec3 size;
            if(i == 0)
                size = glm::vec3(10.0f, 0.10f, stairWidth);
            else if(i == 1)
                size = glm::vec3(10.0f, 0.30f, stairWidth);
            else
                size = glm::vec3(10.0f, 0.46f, stairWidth);
            stairs[i].body = PhysicsBody::box(world, world.physics->createMaterial(0.5f, 0.5f, 0.6f), 1.0f, { 0, 0, stairZ - (float)(i * stairWidth) }, glm::identity<glm::quat>(), size);
            stairs[i].body.body->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
            if(i == 0)
                stairs[i].mesh = Mesh::makeCube(size);
            else if(i == 1)
                stairs[i].mesh = Mesh::makeCube(size);
            else
                stairs[i].mesh = Mesh::makeCube(size);
            stairs[i].material = Resources::make<PBRMaterial>();
            stairs[i].material->texAlbedo = Texture::fromSingleColor(glm::vec3(0.4f));
            stairs[i].material->texAO = Texture::fromSingleColor(glm::vec3(1.0f, 0.0f, 0.0f));
            stairs[i].material->texMetallic = Texture::fromSingleColor(glm::vec3(0.5f, 0.0f, 0.0f));
            stairs[i].material->texRoughness = Texture::fromSingleColor(glm::vec3(0.5f, 0.0f, 0.0f));
        }


        // Prepare motion clip
        pickupBVH = MotionClipData::loadFromFile("resources/retargetted/111-17(pregnant_pick_up).bvh", 0.01f);
        auto idleBVH = MotionClipData::loadFromFile("resources/retargetted/111-36(pregnant_carry).bvh", 0.01f);
        auto walkBVH = MotionClipData::loadFromFile("resources/retargetted/111-36(pregnant_carry).bvh", 0.01f);
        auto climbBVH = MotionClipData::loadFromFile("resources/retargetted/output.bvh", 0.01f);

        //bvh = walkBVH;
        poseTree = pickupBVH.poseTree;
        currentPose = glmx::pose::empty(poseTree.numJoints);

        auto pickupPoses = pickupBVH.slice(1, pickupBVH.numFrames);
        auto idlePoses = std::vector<glmx::pose>(30, pickupBVH.poseStates[pickupBVH.numFrames - 1]);
        auto walkPoses = nonstd::span<glmx::pose>(walkBVH.poseStates.data() + 258, 87);
        auto climbPoses = nonstd::span<glmx::pose>(climbBVH.poseStates.data() + 180, 330);

        auto pickupAnim = animFSM.addAnimation("pickup", pickupPoses, 30);
        auto idleAnim = animFSM.addAnimation("idle", idlePoses, 30);
        auto walkAnim = animFSM.addAnimation("walk", walkPoses, 30);
        auto climbAnim = animFSM.addAnimation("climb", climbPoses, 30);

        for (Ref<Animation> anim : { pickupAnim, idleAnim, walkAnim, climbAnim })
        {
            animFSM.get(anim)->setStartingRootPos(0.0f, 0.0f);
        }

        pickupState = animFSM.addState("pickup", pickupAnim);
        auto idleState = animFSM.addState("idle", idleAnim);
        auto walkState = animFSM.addState("walk", walkAnim);
        auto climbState = animFSM.addState("climb", climbAnim);

        animFSM.addParam("is_walking", false);
        animFSM.addParam("is_climbing", false);

        auto finishPickupTrans = animFSM.addTransition("finish_pickup", pickupState, idleState, 0.2f, 0.2f, 0.2f);

        auto repeatIdleTrans = animFSM.addTransition("repeat_idle", idleState, idleState, 0.0f, 0.0f, 0.0f);

        auto repeatWalkingTrans = animFSM.addTransition("repeat_walking", walkState, walkState, 0.4f, 0.4f, 0.4f);

        auto startWalkingTrans = animFSM.addTransition("start_walking", idleState, walkState, 0.1f, 0.1f, 0.1f);
        animFSM.setTransitionCondition(startWalkingTrans, "is_walking", true);

        auto stopWalkingTrans = animFSM.addTransition("stop_walking", walkState, idleState, 0.1f, 0.1f, 0.1f);
        animFSM.setTransitionCondition(stopWalkingTrans, "is_walking", false);

        auto startClimbingTrans = animFSM.addTransition("start_climbing", idleState, climbState, 0.1f, 0.1f, 0.1f);
        animFSM.setTransitionCondition(startClimbingTrans, "is_climbing", true);

        animFSM.setCurrentState(pickupState);

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
        animFSM.setCurrentState(pickupState);
        currentPose = animFSM.getCurrentPose();

        posePhysicsBody.setPose(currentPose, poseTree);

        if (enableBox) {
            auto pickupPose = animFSM.getCurrentAnim().poses[startPickupFrameIdx];
            auto leftHandTrans = calcFK(poseTree, pickupPose, poseTree.findIdx("LeftHand"));
            auto rightHandTrans = calcFK(poseTree, pickupPose, poseTree.findIdx("RightHand"));
            glmx::transform boxTrans;
            boxTrans.v.x = 0.5f * (leftHandTrans.v.x + rightHandTrans.v.x);
            boxTrans.v.y = 0.0f;
            boxTrans.v.z = pickupPose.v.z - 0.4f;
            boxTrans.q = glm::identity<glm::quat>();
            box.body.setTransform(boxTrans);

            box.body.setLinearVelocity({});
            box.body.setAngularVelocity({});
        }

        if (enableSpheres) {
            glm::vec3 boxPos = box.body.getTransform().v;
            for(int x = 0; x < sphereCountWidth; x++) {
                for(int y = 0; y < sphereCountWidth; y++) {
                    for(int z = 0; z < sphereCountWidth; z++) {
                        int i = x * sphereCountWidth * sphereCountWidth + y * sphereCountWidth + z;
                        spheres[i].body.setPosition(
                                glm::vec3(boxPos.x - boxSize.x + 3.0f * sphereRadius + x * 2.0f * (sphereRadius + sphereDist),
                                          boxPos.y + sphereRadius + boxThickness * 2.0f + y * 2.0f * (sphereRadius + sphereDist),
                                          boxPos.z - boxSize.z + 3.0f * sphereRadius + z * 2.0f * (sphereRadius + sphereDist)));
                        spheres[i].body.setLinearVelocity({});
                        spheres[i].body.setAngularVelocity({});
                    }
                }
            }
        }
    }

    void startRagdoll(float offset) {
        float dt = 1.0f / 120.0f;
        enableRagdoll = true;
        posePhysicsBody.setPose(currentPose, poseTree);

        // Set velocity to articulated body (for more realism)
        float t = animFSM.getStateTime();
        glmx::pose beforePose = animFSM.getCurrentAnim().getFrame(t - dt);
        beforePose.v.y -= offset;
        posePhysicsBody.setPoseVelocityFromTwoPoses(beforePose, currentPose, dt);
        box.body.setKinematic(false);

        // To trip even more spectacularily, add some forces and torques
        posePhysicsBody.applyImpulseTorqueAtRoot(glm::vec3(-0.002, 0, 0));
        posePhysicsBody.applyImpulseAtRoot(glm::vec3(0, 0.014, -0.074));
        //posePhysicsBody.applyVelocityChangeAtRoot(glm::vec3(0, 0, -0.05));
        box.body.body->addTorque(PxVec3(-0.05, 0, 0), PxForceMode::eIMPULSE, true);
        box.body.body->addForce(PxVec3(0, 0, -0.05), PxForceMode::eIMPULSE, true);
    }

    void processInput(SDL_Event &event) override {
    }

    void update(float dt) override {
        static float time = 0.f;
        const float physicsDt = 1.0f / 120.0f;

        static bool hitStopZ = false;

        time += dt;

        auto inputMgr = InputManager::get();
        if (inputMgr->isKeyEntered(SDL_SCANCODE_SPACE)) {
            enablePhysics = !enablePhysics;
        }
        if (inputMgr->isKeyEntered(SDL_SCANCODE_F1)) {
            enableDebugRendering = !enableDebugRendering;
        }
        if (inputMgr->isKeyEntered(SDL_SCANCODE_RETURN)) {
            //startRagdoll();
        }
        if(inputMgr->isKeyPressed(SDL_SCANCODE_DOWN)) {
            animFSM.setParam("is_climbing", true);
        }
        if (inputMgr->isKeyPressed(SDL_SCANCODE_UP) && !hitStopZ)
            animFSM.setParam("is_walking", true);
        else
            animFSM.setParam("is_walking", false);

        // Update animation
        if (!enableRagdoll) {
            animFSM.update(dt);
            currentPose = animFSM.getCurrentPose();

            auto boxTrans = box.body.getTransform();
            boxLeftPos = boxTrans.v - glm::vec3(boxSize.x + 0.02f, 0, 0);
            boxRightPos = boxTrans.v + glm::vec3(boxSize.x + 0.02f, 0, 0);
            boxLeftPos.y += 2 * boxSize.y;
            boxRightPos.y += 2 * boxSize.y;

            // If animation controller arrives at hard-coded state, then start holding the box
            auto currentState = animFSM.getCurrentState();
            if (currentState && animFSM.get(currentState)->name == "pickup") {
                if (animFSM.getStateTime() >= 36.f / 30.f) {
                    isHoldingBox = true;
                }
            }

            if (isHoldingBox) {
                uint32_t leftHandIdx = poseTree.findIdx("LeftHand");
                uint32_t rightHandIdx = poseTree.findIdx("RightHand");
                auto leftHandTrans = calcFK(poseTree, currentPose, leftHandIdx);
                auto rightHandTrans = calcFK(poseTree, currentPose, rightHandIdx);
                glmx::transform boxTrans;
                boxTrans.v.x = 0.5f * (leftHandTrans.v.x + rightHandTrans.v.x);
                boxTrans.v.y = 0.5f * (leftHandTrans.v.y + rightHandTrans.v.y) - 2 * boxSize.y;
                if (boxTrans.v.y < 0.0f) boxTrans.v.y = 0.0f;
                boxTrans.v.z = currentPose.v.z - 0.4f;
                float theta = atan2(rightHandTrans.v.z - leftHandTrans.v.z, rightHandTrans.v.x - leftHandTrans.v.x);
                boxTrans.q = glm::rotate(-theta, glm::vec3(0, 1, 0));
                boxLeftPos = boxTrans.v - boxTrans.q * glm::vec3(boxSize.x + 0.02f, 0, 0);
                boxRightPos = boxTrans.v + boxTrans.q * glm::vec3(boxSize.x + 0.02f, 0, 0);
                boxLeftPos.y += 2 * boxSize.y;
                boxRightPos.y += 2 * boxSize.y;
                box.body.setTransform(boxTrans);

                solveTwoJointIK(poseTree, currentPose,
                                poseTree.findIdx("LeftArm"), poseTree.findIdx("LeftForeArm"),
                                poseTree.findIdx("LeftHand"),
                                boxLeftPos);

                solveTwoJointIK(poseTree, currentPose,
                                poseTree.findIdx("RightArm"), poseTree.findIdx("RightForeArm"),
                                poseTree.findIdx("RightHand"),
                                boxRightPos);
            }

            if (!hitStopZ && currentState && animFSM.get(currentState)->name == "walk") {
                uint32_t rootIdx = poseTree.findIdx("Hips");
                auto rootTrans = calcFK(poseTree, currentPose, rootIdx);    
                static float stopZ = -4.18f;

                if(rootTrans.v.z < stopZ) {
                    animFSM.setParam("is_walking", false);
                    hitStopZ = true;
                }
            }

            if(currentState && animFSM.get(currentState)->name == "climb") {
                if(animFSM.getStateTime() >= 329.0f / 30.0f) {
                    //currentPose.v.y += 0.11f;
                    
                    uint32_t leftFootIdx = poseTree.findIdx("LeftFoot");
                    uint32_t rightFootIdx = poseTree.findIdx("RightFoot");
                    // auto leftFootTrans = calcFK(poseTree, currentPose, leftFootIdx);
                    // auto rightFootTrans = calcFK(poseTree, currentPose, rightFootIdx);
                    // glm::vec3 leftTarget = leftFootTrans.v;
                    // glm::vec3 rightTarget = rightFootTrans.v;

                    // leftTarget.y = 0.24;
                    // rightTarget.y = 0.24;
                    // solveTwoJointIK(poseTree, currentPose,
                    //                 poseTree.findIdx("LeftUpLeg"), poseTree.findIdx("LeftLeg"),
                    //                 poseTree.findIdx("LeftFoot"),
                    //                 leftTarget);

                    // solveTwoJointIK(poseTree, currentPose,
                    //                 poseTree.findIdx("RightUpLeg"), poseTree.findIdx("RightLeg"),
                    //                 poseTree.findIdx("RightFoot"),
                    //                 rightTarget);

                    uint32_t leftFootEndIdx = poseTree.getChildIdx(leftFootIdx, "End Site");
                    uint32_t rightFootEndIdx = poseTree.getChildIdx(rightFootIdx, "End Site");
                    auto leftFootEndTrans = calcFK(poseTree, currentPose, leftFootEndIdx);
                    auto rightFootEndTrans = calcFK(poseTree, currentPose, rightFootEndIdx);
                    glm::vec3 leftTarget = leftFootEndTrans.v;
                    glm::vec3 rightTarget = rightFootEndTrans.v;

                    float offset = rightTarget.y - 0.23f;
                    currentPose.v.y -= offset;
                    // solveTwoJointIK(poseTree, currentPose,
                    //                 poseTree.findIdx("LeftLeg"), poseTree.findIdx("LeftFoot"),
                    //                 leftFootEndIdx,
                    //                 leftTarget);

                    // solveTwoJointIK(poseTree, currentPose,
                    //                 poseTree.findIdx("RightLeg"), poseTree.findIdx("RightFoot"),
                    //                 rightFootEndIdx,
                    //                 rightTarget);

                    startRagdoll(offset);
                }
            }
        }

        // Update physics
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
                posePhysicsBody.setPose(currentPose, poseTree);
                if (enablePhysics) {
                    bool advanced = world.advance(physicsDt);
                    if (advanced) {
                        world.fetchResults();
                    }
                }
            }
        }
    }


    void render() override {
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        pbRenderer.queueRender({groundMesh, groundMat, rootTransform->getWorldTransform()});
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
        for(int i = 0; i < stairCount; i++) {
            pbRenderer.queueRender({stairs[i].mesh, stairs[i].material, glm::translate(glm::vec3{0.0f, 0.0f, stairZ - (float)(i * stairWidth)})});
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
        //imRenderer.render();

        if (enableDebugRendering) {
            pxDebugRenderer.render(world);
        }

        renderImGui();
    }

    void renderImGui() {
        animFSM.renderImGui(poseTree);

        ImGui::Begin("Character Data");

        ImGui::Checkbox("Enable Manipulation", &enableManipulation);
        if (!enableManipulation) {
            if (ImGui::Button("Ragdoll")) {
                //startRagdoll();
                //enableRagdoll = true;
                // posePhysicsBody.getPose(convertedPose, poseTree);
                // posePhysicsBody.setPose(currentPose, poseTree);
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
    MotionClipData pickupBVH;
    AnimStateMachine animFSM;
    Ref<AnimState> pickupState;
    uint32_t startPickupFrameIdx = 36;

    PoseRenderBodyPBR poseRenderBody, poseRenderBody2;

    PxFoundation* pxFoundation;
    PhysicsWorld world;
    PhysXDebugRenderer pxDebugRenderer;
    PosePhysicsBody posePhysicsBody;
    PosePhysicsBodySkel posePhysicsBodySkel;

    Ref<PBRMaterial> groundMat;
    Ref<Mesh> groundMesh;

    const int sphereColorsCount = 10;
    const int sphereCountWidth = 5;
    const int sphereCount = sphereCountWidth * sphereCountWidth * sphereCountWidth;
    const float sphereRadius = 0.035f;
    const float sphereDist = 0.001f;

    glm::vec3 boxSize = {0.25f, 0.15f, 0.25f};
    float boxThickness = 0.025f;

    PhysicsObject box;
    std::vector<PhysicsObject> spheres;

    std::vector<PhysicsObject> stairs;      // Stairs are comprised of box objects.
    int stairCount = 3;
    float stairHeight = 0.5f;
    float stairWidth = 0.27f;
    float stairZ = -4.85f;

    bool enableDebugRendering = true;

    bool enableRagdoll = false;
    bool enableManipulation = false;
    bool enablePhysics = true;

    bool enableBox = true;
    bool enableSpheres = true;

    bool isHoldingBox = false;

    glm::vec3 boxLeftPos, boxRightPos;

};

int main(int argc, char** argv) {
    MyApp app;
    app.start();

    return 0;
}
