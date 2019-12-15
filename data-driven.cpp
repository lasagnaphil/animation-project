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
#include "AnimStateMachine.h"
#include "Box.h"

#define DEBUG_ANIM 1

PxDefaultAllocator gAllocator = {};
PxDefaultErrorCallback gErrorCallback = {};

struct PhysicsObject {
    PhysicsBody body;
    Ref<Mesh> mesh;
    Ref<PBRMaterial> material;
};

FlyCamera* fCamera;
class MyApp : public App {
public:
    MyApp() : App(false, AppRenderSettings::PBR) {}

    void setCamera() {
        fCamera = initCamera<FlyCamera>();
        fCamera->movementSpeed = 1.0f;
        Ref<Transform> cameraTransform = fCamera->transform;
        cameraTransform->setPosition({0.0f, 0.0f, 1.0f});
    }
    void setDirLight() {
        pbRenderer.dirLightProjVolume = {
                {-10.f, -10.f, 0.f}, {10.f, 10.f, 1000.f}
        };
        pbRenderer.shadowFramebufferSize = {2048, 2048};

        pbRenderer.dirLight.enabled = true;
        pbRenderer.dirLight.direction = glm::normalize(glm::vec3 {2.0f, -3.0f, -2.0f});
        pbRenderer.dirLight.color = {5.f, 5.f, 5.f};
    }
    void initPhysX() {
        pxFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
        if (!pxFoundation) {
            fprintf(stderr, "PxCreateFoundation Failed!\n");
            exit(EXIT_FAILURE);
        }
        world.init(pxFoundation, 1);
    }

    void initGround() {
        groundMesh = Mesh::makePlane(100.0f, 100.0f);
        groundMat = PBRMaterial::quick(
                "resources/textures/mossy-ground1-albedo.png",
                "resources/textures/mossy-ground1-metal.png",
                "resources/textures/mossy-ground1-roughness.png",
                "resources/textures/mossy-ground1-ao.png");
    }
    void initBox() {
        box.mesh = Mesh::fromOBJFile("resources/box.obj");

        box.material = Resources::make<PBRMaterial>();
        box.material->texAlbedo = Texture::fromSingleColor(glm::vec3(0.4f));
        box.material->texAO = Texture::fromSingleColor(glm::vec3(1.0f, 0.0f, 0.0f));
        box.material->texMetallic = Texture::fromSingleColor(glm::vec3(0.5f, 0.0f, 0.0f));
        box.material->texRoughness = Texture::fromSingleColor(glm::vec3(0.5f, 0.0f, 0.0f));
        box.mesh->indices.clear(); // We don't need the index buffer
        
        boxHX = 0.25f; boxHY = 0.25f; boxHZ = 0.15f; boxHT = 0.025f;    // @TODO : Hardcoding.
        box.body = PhysicsBody::ourBox(world, boxHX, boxHY, boxHZ, boxHT, world.physics->createMaterial(0.5f, 0.5f, 0.6f));
    }
    void initSphere() {
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
            spheres[i].body = PhysicsBody::sphere(world, world.physics->createMaterial(0.5f, 0.5f, 0.6f), 1.0f, {}, sphereRadius);
            spheres[i].mesh = sphereMesh;
            spheres[i].material = sphereMats[i % sphereColorsCount];
        }
    }

    void initAnim() {
#if DEBUG_ANIM
        bvh = MotionClipData::loadFromFile("resources/retargetted/output.bvh", 0.01f);
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
#else
        auto idleBVH = MotionClipData::loadFromFile("resources/retargetted/111-36(pregnant_carry).bvh", 0.01f);
        auto walkBVH = MotionClipData::loadFromFile("resources/retargetted/111-36(pregnant_carry).bvh", 0.01f);

        //bvh = walkBVH;
        poseTree = idleBVH.poseTree;
        currentPose = glmx::pose::empty(poseTree.numJoints);
        convertedPose = currentPose;

        auto idlePoses = std::vector<glmx::pose>(30, idleBVH.poseStates[0]);
        auto walkPoses = nonstd::span<glmx::pose>(walkBVH.poseStates.data() + 1032, 310);

        auto idleAnim = animFSM.addAnimation("idle", nonstd::span<glmx::pose>(idlePoses.data(), idlePoses.size()), 120);
        auto walkAnim = animFSM.addAnimation("walk", walkPoses, 120);

        for (Ref<Animation> anim : { idleAnim, walkAnim })
        {
            animFSM.get(anim)->setStartingRootPos(0.0f, 0.0f);
        }

        auto idleState = animFSM.addState("idle", idleAnim);
        auto walkState = animFSM.addState("walk", walkAnim);

        animFSM.addParam("is_walking", false);

        auto repeatIdleTrans = animFSM.addTransition("repeat_idle", idleState, idleState, 0.0f, 0.0f, 0.0f);

        auto repeatWalkingTrans = animFSM.addTransition("repeat_walking", walkState, walkState, 0.3f, 0.3f, 0.3f);

        auto startWalkingTrans = animFSM.addTransition("start_walking", idleState, walkState, 0.1f, 0.1f, 0.1f);
        animFSM.setTransitionCondition(startWalkingTrans, "is_walking", true);

        auto stopWalkingTrans = animFSM.addTransition("stop_walking", walkState, idleState, 0.1f, 0.1f, 0.1f);
        animFSM.setTransitionCondition(stopWalkingTrans, "is_walking", false);

        animFSM.setCurrentState(idleState);
#endif
    }
    void initBody() {
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
        
        // At first, character is in kinematic state.
        // world.scene->removeAggregate(*posePhysicsBody.aggregate);

        pxDebugRenderer.init(world);
        pxDebugRenderer.setCamera(fCamera);
    }

    void loadResources() override {
        setCamera();
        setDirLight();
        initPhysX();

        // Scene setting
        initGround();
        initBox();
        initSphere();
        
        // Animation Setting
        initAnim();
        initBody();

        // Reset
        reset();
    }

    void reset() {
        kinematicChar = true;
        pickedBox = false;
        enablePhysics = false;

        // world.scene->removeAggregate(*posePhysicsBody.aggregate);

#if DEBUG_ANIM
        motionClipPlayer.setFrame(27);
        currentPose = motionClipPlayer.getPoseState();
        posePhysicsBody.setPose(currentPose, poseTree);
#else
        currentPose = animFSM.getCurrentPose();
#endif

        // Box and Spheres.
        glm::vec3 boxPos(0.0f, 0.1f, 15.0f);
        for(int x = 0; x < sphereCountWidth; x++) {
            for(int y = 0; y < sphereCountWidth; y++) {
                for(int z = 0; z < sphereCountWidth; z++) {
                    int i = x * sphereCountWidth * sphereCountWidth + y * sphereCountWidth + z;
                    spheres[i].body.setPosition(
                        glm::vec3(boxPos.x + 3.0f * sphereRadius + x * 2.0f * (sphereRadius + sphereDist),
                        boxPos.y + sphereRadius + boxHT * 2.0f + y * 2.0f * (sphereRadius + sphereDist),
                        boxPos.z - 3.0f * sphereRadius - z * 2.0f * (sphereRadius + sphereDist)));
                    spheres[i].body.setLinearVelocity({});
                    spheres[i].body.setAngularVelocity({});
                }
            }
        }

        // box
        box.body.setTransform(glmx::transform(
            boxPos, 
            glm::angleAxis((float)(-M_PI / 2.0), glm::vec3(1, 0, 0))));
        
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
#if DEBUG_ANIM
        motionClipPlayer.update(dt);
#endif
        while (time >= physicsDt) {
            time -= physicsDt;

            if(kinematicChar) {
                // Before character falls down.
#if DEBUG_ANIM
                currentPose = motionClipPlayer.getPoseState();
                posePhysicsBody.setPose(currentPose, poseTree);
#else
                if (inputMgr->isKeyPressed(SDL_SCANCODE_UP))
                    animFSM.setParam("is_walking", true);
                else 
                    animFSM.setParam("is_walking", false);
                animFSM.update(dt);

                currentPose = animFSM.getCurrentPose();

                glm::vec3 boxOffset = box.body.getTransform().v - currentPose.getRoot().v;
                boxOffset.y = 0.0f;
                const static float nearBoxThres = 1.0f;
                float len = glm::length(boxOffset);
                if(len < nearBoxThres) {
                    // pick up box.
                    pickedBox = true;
                }

                if(pickedBox) {
                    // Do not have to do physics on box and spheres.
                    // Set box position to rootpos. @TODO
                    glm::vec3 rootPos = currentPose.getRoot().v;
                    glm::vec3 boxPos = rootPos + glm::vec3(0.0f, 0.0f, 0.2f);
                    glm::vec3 boxOffset = boxPos - box.body.getPosition();
                    
                    box.body.setPosition(boxPos);

                    // Also move spheres...
                    for (int i = 0; i < sphereCount; i++) 
                        spheres[i].body.setPosition(spheres[i].body.getPosition() + boxOffset);
                }
                else {
                    if(enablePhysics) {
                        bool advanced = world.advance(physicsDt);
                        if (advanced) 
                            world.fetchResults();
                    }
                }
#endif
            }
            else {
                // After character falls down.
                if (enablePhysics) {
                    bool advanced = world.advance(physicsDt);
                    if (advanced)
                        world.fetchResults();
                }
                posePhysicsBody.getPose(currentPose, poseTree);
            }
        }
    }

    glmx::transform pxToRender(const glmx::transform &tr) {
        glmx::transform rtr;
        rtr.q = tr.q;
        rtr.v = glm::transpose(glm::toMat3(rtr.q)) * tr.v;
        return rtr;
    }

    void render() override {
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        pbRenderer.queueRender({groundMesh, groundMat, rootTransform->getWorldTransform()});
        renderMotionClip(pbRenderer, imRenderer, currentPose, poseTree, poseRenderBody);
        renderMotionClip(pbRenderer, imRenderer, convertedPose, poseTree, poseRenderBody2);

        pbRenderer.queueRender({box.mesh, box.material, glmx::mat4_cast(pxToRender(box.body.getTransform()))});
        for (auto& sphere : spheres) {
            pbRenderer.queueRender({sphere.mesh, sphere.material, glmx::mat4_cast(pxToRender(sphere.body.getTransform()))});
        }
        pbRenderer.render();

        imRenderer.drawPoint(pbRenderer.pointLights[0].position, colors::Yellow, 4.0f, true);
        imRenderer.drawPoint(pbRenderer.pointLights[1].position, colors::Yellow, 4.0f, true);
        // imRenderer.drawPoint(pbRenderer.pointLights[2].position, colors::Yellow, 4.0f, true);
        // imRenderer.drawPoint(pbRenderer.pointLights[3].position, colors::Yellow, 4.0f, true);
        imRenderer.drawAxisTriad(glm::mat4(1.0f), 0.1f, 1.0f, false);
        imRenderer.render();

        pxDebugRenderer.render(world);

#if DEBUG_ANIM
        motionClipPlayer.renderImGui();
#else
        ImGui::SetNextWindowPos(ImVec2(23, 158));
        ImGui::SetNextWindowSize(ImVec2(447, 844));
        animFSM.renderImGui(poseTree);
#endif

        renderImGui();
    }

    void renderImGui() {
        ImGui::Begin("Character Data");

        if(kinematicChar) {
            if(ImGui::Button("Enable Character Ragdoll")) {
                kinematicChar = false;
                //world.scene->addAggregate(*posePhysicsBody.aggregate);
                currentPose = animFSM.getCurrentPose();
                posePhysicsBody.setPose(currentPose, poseTree);
            }
        }
        else {
            if(ImGui::Button("Enable Character Control")) {
                kinematicChar = true;
                //world.scene->removeAggregate(*posePhysicsBody.aggregate);
            }
        }

        ImGui::Checkbox("Enable Physics", &enablePhysics);
        if(ImGui::Button("Reset"))
            reset();

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
    const int sphereCountWidth = 5;
    const int sphereCount = sphereCountWidth * sphereCountWidth * sphereCountWidth;
    const float sphereRadius = 0.035f;
    const float sphereDist = 0.001f;

    PhysicsObject box;
    float boxHX, boxHY, boxHZ, boxHT;   // Half X width, ... , Half Thickness.
    std::vector<PhysicsObject> spheres;

    Ref<PBRMaterial> groundMat;
    Ref<Mesh> groundMesh;

    bool enablePhysics = false;
    bool pickedBox = false;     // If it is true, character is carrying box now.
    bool kinematicChar = true;  // If it is true, character is in kinematic state.

    AnimStateMachine animFSM;
};

int main(int argc, char** argv) {
    MyApp app;
    app.start();

    return 0;
}

