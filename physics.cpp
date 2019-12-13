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

        Ref<PBRMaterial> groundMat = PBRMaterial::quick(
                "resources/textures/mossy-ground1-albedo.png",
                "resources/textures/mossy-ground1-metal.png",
                "resources/textures/mossy-ground1-roughness.png",
                "resources/textures/mossy-ground1-ao.png");

        Ref<Mesh> groundMesh = Mesh::makePlane(100.0f, 100.0f);

        // Prepare the box
        if (enableBox) {
            glm::vec3 boxSize = {0.5f, 0.5f, 0.5f};
            // box.mesh = Mesh::makeCube(boxSize);
            box.mesh = Mesh::fromOBJFile("resources/box.obj");
            box.material = Resources::make<PBRMaterial>();
            box.material->texAlbedo = Texture::fromSingleColor(glm::vec3(0.4f));
            box.material->texAO = Texture::fromSingleColor(glm::vec3(1.0f, 0.0f, 0.0f));
            box.material->texMetallic = Texture::fromSingleColor(glm::vec3(0.5f, 0.0f, 0.0f));
            box.material->texRoughness = Texture::fromSingleColor(glm::vec3(0.5f, 0.0f, 0.0f));
            box.mesh->indices.clear(); // We don't need the index buffer
            auto collider = box.mesh->generateCollider();
            auto bodyOpt = PhysicsBody::fromMesh(world, collider, world.physics->createMaterial(0.5f, 0.5f, 0.6f));
            if (!bodyOpt) {
                fprintf(stderr, "PhysicsBody::fromMesh() failed to generate mesh.\n");
                exit(EXIT_FAILURE);
            }
            box.body = *bodyOpt;

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
                spheres[i].body = PhysicsBody::sphere(world, world.physics->createMaterial(0.5f, 0.5f, 0.6f), {}, sphereRadius);
                spheres[i].mesh = sphereMesh;
                spheres[i].material = sphereMats[i % sphereColorsCount];

            }
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

        pxDebugRenderer.init(world);
        pxDebugRenderer.setCamera(camera);

        reset();
    }

    void reset() {
        enableRagdoll = false;
        motionClipPlayer.setFrame(27);
        currentPose = motionClipPlayer.getPoseState();
        posePhysicsBody.setPose(currentPose, poseTree);

        if (enableBox) {
            box.body.setTransform(
                    glmx::transform(glm::vec3(0.0f, 0.0f, 0.0f), glm::angleAxis((float)-M_PI/2, glm::vec3(1, 0, 0))));
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

                /*
                bool advanced = world.advance(physicsDt);
                if (advanced) {
                    world.fetchResults();
                }

                posePhysicsBody.getPose(convertedPose, poseTree);
                 */
            }
        }
    }


    void render() override {
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // pbRenderer.queueRender({groundMesh, groundMat, rootTransform->getWorldTransform()});
        renderMotionClip(pbRenderer, imRenderer, currentPose, poseTree, poseRenderBody);
        renderMotionClip(pbRenderer, imRenderer, convertedPose, poseTree, poseRenderBody2);

        if (enableBox) {
            pbRenderer.queueRender({box.mesh, box.material, glmx::mat4_cast(box.body.getTransform())});
        }
        if (enableSpheres) {
            for (auto& sphere : spheres) {
                pbRenderer.queueRender({sphere.mesh, sphere.material, glm::translate(sphere.body.getTransform().v)});
            }
        }
        pbRenderer.render();

        imRenderer.drawPoint(pbRenderer.pointLights[0].position, colors::Yellow, 4.0f, true);
        imRenderer.drawPoint(pbRenderer.pointLights[1].position, colors::Yellow, 4.0f, true);
        // imRenderer.drawPoint(pbRenderer.pointLights[2].position, colors::Yellow, 4.0f, true);
        // imRenderer.drawPoint(pbRenderer.pointLights[3].position, colors::Yellow, 4.0f, true);
        imRenderer.drawAxisTriad(glm::mat4(1.0f), 0.1f, 1.0f, false);
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

    PhysicsObject box;
    std::vector<PhysicsObject> spheres;

    bool enableDebugRendering = true;

    bool enableRagdoll = false;
    bool enableManipulation = false;
    bool enablePhysics = true;

    bool enableBox = true;
    bool enableSpheres = false;
};

int main(int argc, char** argv) {
    MyApp app;
    app.start();

    return 0;
}
