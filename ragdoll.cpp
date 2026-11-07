#include "ragdoll.h"
#include <vector>

namespace {
    btDefaultCollisionConfiguration* g_cfg;
    btCollisionDispatcher*           g_disp;
    btDbvtBroadphase*                g_broad;
    btSequentialImpulseConstraintSolver* g_solver;
    btDiscreteDynamicsWorld*         g_world;
    btRigidBody*                     g_ground;
    RagdollParams                    g_p;
    RD::GroundFn                     g_groundFn;
    std::vector<Ragdoll*>            g_all;

    struct Def { float radius, halfLen, mass; };   // capsule along Y
    // Rough human proportions in meters
    const Def DEFS[P_COUNT] = {
        {0.14f, 0.06f, 12.f}, // pelvis
        {0.15f, 0.20f, 18.f}, // torso
        {0.11f, 0.03f,  5.f}, // head
        {0.05f, 0.12f,  2.f}, {0.04f, 0.12f, 1.5f}, // R arm
        {0.05f, 0.12f,  2.f}, {0.04f, 0.12f, 1.5f}, // L arm
        {0.07f, 0.17f,  7.f}, {0.05f, 0.17f, 4.f},  // R leg
        {0.07f, 0.17f,  7.f}, {0.05f, 0.17f, 4.f},  // L leg
    };
    // Local offset of each part center from pelvis (standing pose, Z up)
    const float OFF[P_COUNT][3] = {
        {0,0,0}, {0,0,0.30f}, {0,0,0.62f},
        {0.25f,0,0.45f}, {0.25f,0,0.20f},
        {-0.25f,0,0.45f}, {-0.25f,0,0.20f},
        {0.10f,0,-0.25f}, {0.10f,0,-0.65f},
        {-0.10f,0,-0.25f}, {-0.10f,0,-0.65f},
    };

    btRigidBody* MakeBody(const Def& d, const btVector3& pos) {
        // Capsule oriented along Z (Bullet capsuleZ) to match GTA's Z-up
        auto* shape = new btCapsuleShapeZ(d.radius, d.halfLen * 2.f);
        btVector3 inertia(0,0,0);
        shape->calculateLocalInertia(d.mass, inertia);
        auto* ms = new btDefaultMotionState(btTransform(btQuaternion::getIdentity(), pos));
        btRigidBody::btRigidBodyConstructionInfo ci(d.mass, ms, shape, inertia);
        ci.m_restitution = g_p.restitution;
        ci.m_friction = g_p.friction;
        ci.m_linearDamping = g_p.linDamping;
        ci.m_angularDamping = g_p.angDamping;
        auto* rb = new btRigidBody(ci);
        rb->setActivationState(DISABLE_DEACTIVATION);
        return rb;
    }

    btTypedConstraint* Cone(btRigidBody* a, btRigidBody* b, const btVector3& worldPivot,
                            float swing1, float swing2, float twist) {
        btTransform fa = btTransform::getIdentity(), fb = btTransform::getIdentity();
        fa.setOrigin(a->getWorldTransform().inverse() * worldPivot);
        fb.setOrigin(b->getWorldTransform().inverse() * worldPivot);
        auto* c = new btConeTwistConstraint(*a, *b, fa, fb);
        c->setLimit(swing1, swing2, twist);
        return c;
    }

    btTypedConstraint* Hinge(btRigidBody* a, btRigidBody* b, const btVector3& worldPivot,
                             float lo, float hi) {
        btTransform fa = btTransform::getIdentity(), fb = btTransform::getIdentity();
        fa.setOrigin(a->getWorldTransform().inverse() * worldPivot);
        fb.setOrigin(b->getWorldTransform().inverse() * worldPivot);
        auto* h = new btHingeConstraint(*a, *b, fa, fb);
        h->setLimit(lo, hi);
        return h;
    }
}

void RD::Init(const RagdollParams& p) {
    g_p = p;
    g_cfg    = new btDefaultCollisionConfiguration();
    g_disp   = new btCollisionDispatcher(g_cfg);
    g_broad  = new btDbvtBroadphase();
    g_solver = new btSequentialImpulseConstraintSolver();
    g_world  = new btDiscreteDynamicsWorld(g_disp, g_broad, g_solver, g_cfg);
    g_world->setGravity(btVector3(0, 0, p.gravity));

    // Big static box used as ground; moved under the ragdoll each step
    auto* gs = new btBoxShape(btVector3(200, 200, 1));
    auto* ms = new btDefaultMotionState(btTransform(btQuaternion::getIdentity(), btVector3(0,0,-1)));
    btRigidBody::btRigidBodyConstructionInfo ci(0, ms, gs);
    ci.m_restitution = p.restitution;
    ci.m_friction = p.friction;
    g_ground = new btRigidBody(ci);
    g_world->addRigidBody(g_ground);
}

void RD::SetGroundFn(GroundFn fn) { g_groundFn = fn; }

Ragdoll* RD::Create(const float pelvis[3], const float vel[3]) {
    auto* r = new Ragdoll();
    btVector3 base(pelvis[0], pelvis[1], pelvis[2]);
    for (int i = 0; i < P_COUNT; i++) {
        r->body[i] = MakeBody(DEFS[i], base + btVector3(OFF[i][0], OFF[i][1], OFF[i][2]));
        r->body[i]->setLinearVelocity(btVector3(vel[0], vel[1], vel[2]));
        g_world->addRigidBody(r->body[i]);
    }
    auto pivot = [&](float x, float y, float z) { return base + btVector3(x, y, z); };
    const float D = SIMD_PI / 180.f;
    int j = 0;
    auto add = [&](btTypedConstraint* c) {
        g_world->addConstraint(c, true); // true = no collision between linked parts
        r->joint[j++] = c;
    };
    add(Cone (r->body[P_PELVIS], r->body[P_TORSO], pivot(0,0,0.14f), 40*D, 40*D, 30*D));
    add(Cone (r->body[P_TORSO],  r->body[P_HEAD],  pivot(0,0,0.52f), 45*D, 45*D, 45*D));
    add(Cone (r->body[P_TORSO],  r->body[P_R_UARM],pivot(0.20f,0,0.52f), 90*D, 90*D, 40*D));
    add(Hinge(r->body[P_R_UARM], r->body[P_R_LARM],pivot(0.25f,0,0.33f), 0, 130*D));
    add(Cone (r->body[P_TORSO],  r->body[P_L_UARM],pivot(-0.20f,0,0.52f), 90*D, 90*D, 40*D));
    add(Hinge(r->body[P_L_UARM], r->body[P_L_LARM],pivot(-0.25f,0,0.33f), 0, 130*D));
    add(Cone (r->body[P_PELVIS], r->body[P_R_THIGH],pivot(0.10f,0,-0.08f), 70*D, 70*D, 25*D));
    add(Hinge(r->body[P_R_THIGH],r->body[P_R_CALF], pivot(0.10f,0,-0.45f), -130*D, 0));
    add(Cone (r->body[P_PELVIS], r->body[P_L_THIGH],pivot(-0.10f,0,-0.08f), 70*D, 70*D, 25*D));
    add(Hinge(r->body[P_L_THIGH],r->body[P_L_CALF], pivot(-0.10f,0,-0.45f), -130*D, 0));
    r->active = true;
    g_all.push_back(r);
    return r;
}

void RD::Destroy(Ragdoll* r) {
    if (!r) return;
    for (auto*& c : r->joint) if (c) { g_world->removeConstraint(c); delete c; c = nullptr; }
    for (auto*& b : r->body) if (b) {
        g_world->removeRigidBody(b);
        delete b->getMotionState(); delete b->getCollisionShape(); delete b; b = nullptr;
    }
    for (size_t i = 0; i < g_all.size(); i++) if (g_all[i] == r) { g_all.erase(g_all.begin()+i); break; }
    delete r;
}

void RD::Step(float dt) {
    if (g_all.empty()) return;
    if (g_groundFn) {   // move ground under the first active ragdoll
        btVector3 p = g_all[0]->body[P_PELVIS]->getWorldTransform().getOrigin();
        float gz = g_groundFn(p.x(), p.y(), p.z());
        btTransform t = btTransform::getIdentity();
        t.setOrigin(btVector3(p.x(), p.y(), gz - 1.f));
        g_ground->setWorldTransform(t);
    }
    g_world->stepSimulation(dt * g_p.timeScale, 4, 1.f / 60.f);
}

void RD::GetPartMatrix(const Ragdoll* r, Part part, float out[16]) {
    btTransform t;
    r->body[part]->getMotionState()->getWorldTransform(t);
    t.getOpenGLMatrix(out);
}
