// SARagdoll - skeleton. GTA SA Android v2.00, 32-bit (armeabi-v7a), AML.
// Check names/signatures against the AML header you actually use (RusJJ/AML).
#include <mod/amlmod.h>
#include <mod/logger.h>
#include <android/log.h>
#include <time.h>
#include "ragdoll.h"

MYMOD(net.user.saragdoll, SARagdoll, 0.1, user)

#define LOG(...) __android_log_print(ANDROID_LOG_INFO, "SARagdoll", __VA_ARGS__)

uintptr_t g_GTASA = 0;      // base address of libGTASA.so
void*     g_hGTASA = nullptr;

// ---- Minimal view of the game structs (offsets MUST be filled in by you) ----
// Find them with IDA/Ghidra on libGTASA 2.00 or from public SA mobile mod sources.
struct CVector { float x, y, z; };

// ---- Hook targets: resolve by exported symbol when possible ----
// Example symbol names (check with: nm -D libGTASA.so | grep CPed):
//   _ZN4CPed14ProcessControlEv
typedef void (*ProcessControl_t)(void* ped);
ProcessControl_t CPed_ProcessControl_orig = nullptr;

static float GroundHeight(float x, float y, float z) {
    // TODO: call CWorld::FindGroundZForCoord (resolve via GetSym) and return the result
    return z - 1.0f;
}

static double Now() {
    timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

// Per-frame hook: advances Bullet once per frame and writes bones back.
static void CPed_ProcessControl_hook(void* ped) {
    static double last = Now();
    double t = Now();
    float dt = (float)(t - last);
    last = t;
    if (dt > 0.1f) dt = 0.1f;

    CPed_ProcessControl_orig(ped);

    // TODO 1: detect "ped should ragdoll" (knocked down / dead / hit by car).
    //         On trigger: RD::Create(pelvisPos, velocity) and store it per ped.
    // TODO 2: for a ped with an active ragdoll, copy RD::GetPartMatrix(...) into the
    //         RpHAnimHierarchy bone matrices (use the BoneId enum -> node index).
    // TODO 3: skip the game's own animation update for that ped while ragdolled.
    RD::Step(dt);
}

extern "C" void OnModLoad() {
    logger->SetTag("SARagdoll");
    g_GTASA  = aml->GetLib("libGTASA.so");
    g_hGTASA = aml->GetLibHandle("libGTASA.so");
    LOG("libGTASA base = 0x%x", (unsigned)g_GTASA);

    RagdollParams p;       // TODO: load from SARagdoll.ini
    RD::Init(p);
    RD::SetGroundFn(GroundHeight);

    void* fn = aml->GetSym(g_hGTASA, "_ZN4CPed14ProcessControlEv");
    if (!fn) { LOG("CPed::ProcessControl not found - use an offset instead"); return; }
    aml->Hook(fn, (void*)CPed_ProcessControl_hook, (void**)&CPed_ProcessControl_orig);
    LOG("hook installed");
}
