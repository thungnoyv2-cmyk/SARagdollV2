#pragma once
#include <btBulletDynamicsCommon.h>

// GTA SA skeleton node ids (RpHAnim node ids used by CPed). Verify in-game.
enum BoneId {
    B_PELVIS = 1, B_SPINE = 3, B_NECK = 5, B_HEAD = 8,
    B_R_SHOULDER = 22, B_R_ELBOW = 23, B_R_HAND = 25,
    B_L_SHOULDER = 32, B_L_ELBOW = 33, B_L_HAND = 35,
    B_R_HIP = 41, B_R_KNEE = 42, B_R_FOOT = 44,
    B_L_HIP = 51, B_L_KNEE = 52, B_L_FOOT = 54,
};

enum Part {
    P_PELVIS, P_TORSO, P_HEAD,
    P_R_UARM, P_R_LARM, P_L_UARM, P_L_LARM,
    P_R_THIGH, P_R_CALF, P_L_THIGH, P_L_CALF,
    P_COUNT
};

struct RagdollParams {      // fill from your .ini
    float restitution  = 0.55f;   // bounciness
    float friction     = 0.8f;
    float linDamping   = 0.15f;
    float angDamping   = 0.35f;
    float timeScale    = 0.75f;   // <1 = slower fall
    float gravity      = -9.81f;
};

struct Ragdoll {
    btRigidBody* body[P_COUNT] = {};
    btTypedConstraint* joint[P_COUNT] = {};
    bool active = false;
};

namespace RD {
    void Init(const RagdollParams& p);
    Ragdoll* Create(const float pelvisPos[3], const float velocity[3]);
    void Destroy(Ragdoll* r);
    void Step(float dtSeconds);
    // Fills out 4x4 column-major world matrix (16 floats) for a body part
    void GetPartMatrix(const Ragdoll* r, Part part, float out[16]);
    // Ground height supplier: set this to a function that raycasts GTA's world
    typedef float (*GroundFn)(float x, float y, float z);
    void SetGroundFn(GroundFn fn);
}
