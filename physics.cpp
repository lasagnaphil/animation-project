#include <InputManager.h>
#include "App.h"
#include "PBRenderer.h"
#include "FlyCamera.h"
#include "PhysicsWorld.h"
#include "PosePhysicsBody.h"
#include "PoseRenderBody.h"
#include "PhysXDebugRenderer.h"
#include "MotionClipData.h"
#include "MotionClipPlayer.h"

PxDefaultAllocator gAllocator = {};
PxDefaultErrorCallback gErrorCallback = {};

class MyApp : public App {
public:
    MyApp() : App(false, AppRenderSettings::PBR) {}

    void loadResources() override {
        FlyCamera* camera = initCamera<FlyCamera>();
        camera->movementSpeed = 1.0f;
        Ref<Transform> cameraTransform = camera->transform;
        cameraTransform->setPosition({0.0f, 0.0f, 1.0f});

        pbRenderer.dirLightProjVolume = {
                {-20.f, -20.f, 0.f}, {20.f, 20.f, 1000.f}
        };
        pbRenderer.shadowFramebufferSize = {2048, 2048};

        pbRenderer.dirLight.enabled = true;
        pbRenderer.dirLight.direction = glm::normalize(glm::vec3 {2.0f, -3.0f, -2.0f});
        pbRenderer.dirLight.color = {0.5f, 0.5f, 0.5f};

        pbRenderer.pointLights[0].enabled = true;
        pbRenderer.pointLights[0].position = {-1.0f, -1.0f + 1.75f, 1.0f};
        pbRenderer.pointLights[0].color = {30.f, 30.f, 30.f};

        pbRenderer.pointLights[1].enabled = true;
        pbRenderer.pointLights[1].position = {1.0f, -1.0f + 1.75f, 1.0f};
        pbRenderer.pointLights[1].color = {30.f, 30.f, 30.f};

        /*
        pbRenderer.pointLights[2].enabled = true;
        pbRenderer.pointLights[2].position = {-10.0f, 10.0f + 17.5f, 10.0f};
        pbRenderer.pointLights[2].color = {300.f, 300.f, 300.f};

        pbRenderer.pointLights[3].enabled = true;
        pbRenderer.pointLights[3].position = {10.0f, 10.0f + 17.5f, 10.0f};
        pbRenderer.pointLights[3].color = {300.f, 300.f, 300.f};
         */

        groundMat = PBRMaterial::quick(
                "resources/textures/mossy-ground1-albedo.png",
                "resources/textures/mossy-ground1-metal.png",
                "resources/textures/mossy-ground1-roughness.png",
                "resources/textures/mossy-ground1-ao.png");

        groundMesh = Mesh::makePlane(100.0f, 100.0f);

        bvh = MotionClipData::loadFromFile("resources/16_15_walk.bvh", 0.01f);
        if (!bvh.valid) {
            fprintf(stderr, "BVH load failed!\n");
            exit(EXIT_FAILURE);
        }
        poseTree = bvh.poseTree;
        currentPose = glmx::pose::empty(19);
        convertedPose = currentPose;

        motionClipPlayer = MotionClipPlayer(&bvh);
        motionClipPlayer.init();

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

        pxFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
        if (!pxFoundation) {
            fprintf(stderr, "PxCreateFoundation Failed!\n");
            exit(EXIT_FAILURE);
        }
        world.init(pxFoundation, 1);

        posePhysicsBody.init(world, poseTree);

        pxDebugRenderer.init(world);
        pxDebugRenderer.setCamera(camera);

    }

    void processInput(SDL_Event &event) override {
    }

    void update(float dt) override {
        auto inputMgr = InputManager::get();
        motionClipPlayer.update(dt);
        // if (motionClipPlayer.shouldUpdate) {
            // currentPose = motionClipPlayer.getPoseState();
            posePhysicsBody.setPose(currentPose);
            convertedPose = currentPose;
            posePhysicsBody.getPose(convertedPose);
        // }
        bool advanced = world.advance(dt);
        if (advanced) {
            world.fetchResults();
        }
    }

    void render() override {
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // pbRenderer.queueRender({groundMesh, groundMat, rootTransform->getWorldTransform()});
        renderMotionClip(pbRenderer, imRenderer, currentPose, poseTree, poseRenderBody);
        renderMotionClip(pbRenderer, imRenderer, convertedPose, poseTree, poseRenderBody2);
        pbRenderer.render();

        imRenderer.drawPoint(pbRenderer.pointLights[0].position, colors::Yellow, 4.0f, true);
        imRenderer.drawPoint(pbRenderer.pointLights[1].position, colors::Yellow, 4.0f, true);
        // imRenderer.drawPoint(pbRenderer.pointLights[2].position, colors::Yellow, 4.0f, true);
        // imRenderer.drawPoint(pbRenderer.pointLights[3].position, colors::Yellow, 4.0f, true);
        imRenderer.render();

        pxDebugRenderer.render(world);

        motionClipPlayer.renderImGui();
        posePhysicsBody.renderImGui();

        renderImGui();
    }

    void renderImGui() {
        ImGui::Begin("Pose");

        for (uint32_t i = 0; i < poseTree.numJoints; i++) {
            auto& node = poseTree[i];
            glm::vec3 v = glmx::quatToEuler(currentPose.q[i], EulOrdXYZs);
            ImGui::PushItemWidth(ImGui::GetFontSize() * 24);
            ImGui::SliderFloat3(node.name.c_str(), (float*)&v, -M_PI/2, M_PI/2);
            ImGui::PopItemWidth();
            currentPose.q[i] = glmx::eulerToQuat(v);
        }

        ImGui::End();
    }

    void release() override {
        world.release();
    }

private:
    const int N = 7;

    Ref<PBRMaterial> groundMat;
    std::vector<Ref<PBRMaterial>> sphereMats;
    Ref<Mesh> groundMesh;
    Ref<Mesh> sphereMesh;
    std::vector<Ref<Transform>> sphereTransforms;

    glmx::pose currentPose, convertedPose;
    PoseTree poseTree;
    MotionClipData bvh;
    MotionClipPlayer motionClipPlayer;
    PoseRenderBodyPBR poseRenderBody, poseRenderBody2;

    PxFoundation* pxFoundation;
    PhysicsWorld world;
    PhysXDebugRenderer pxDebugRenderer;
    PosePhysicsBody posePhysicsBody;
};

int main(int argc, char** argv) {
    MyApp app;
    app.start();

    return 0;
}
