extern "C" {
#include "z64.h"
#include "macros.h"
#include "functions.h"
#include "variables.h"
#include "objects/object_gi_key/object_gi_key.h"
#include "objects/object_gi_bosskey/object_gi_bosskey.h"
extern PlayState* gPlayState;
}

#include "VrCombat.h"
#include "VrPocket.h"
#include "soh/Enhancements/game-interactor/GameInteractor.h"
#include "soh/ShipInit.hpp"
#include "soh/frame_interpolation.h"
#include <libultraship/bridge/consolevariablebridge.h>
#include <vr_interface.h>

// Phase one of physical locked doors. This exposes real inventory keys to the hands but never
// spends one; phase two will take ownership only after a compatible lock accepts the held key.
namespace {
enum class Kind { None, Small, Boss };
Kind sHeld = Kind::None;
int sHand = -1;
bool sGripPrev[2] = { true, true };
bool sNear[2] = { false, false };
VrPocket::Grab sGrab;
int sSmallInterpKey, sBossInterpKey, sHeldInterpKey;

float Cm(float cm) { return cm * 0.01f * VrPocket::WorldScale(); }

bool Gate(PlayState* play, Player* player) {
    return play && player && CVarGetInteger("gVrPhysicalKeys", 1) && CVarGetInteger("gVrPhysicalDoors", 1) &&
           VR_IsInitialized() && VR_GetFirstPerson() && !VR_IsFlatScreen() &&
           CVarGetInteger("gVrMotionHands", 1) && VrItemSelect_ModeActive() &&
           VrItemSelect_SelectionAllowed() && play->pauseCtx.state == 0 && play->csCtx.state == CS_STATE_IDLE &&
           play->transitionTrigger == TRANS_TRIGGER_OFF && play->bombchuBowlingStatus == 0 &&
           play->shootingGalleryStatus == 0 && !(player->stateFlags1 &
               (PLAYER_STATE1_ON_HORSE | PLAYER_STATE1_IN_WATER | PLAYER_STATE1_DEAD));
}

bool HasSmall() {
    const int map = gSaveContext.mapIndex;
    return map >= 0 && map < (int)ARRAY_COUNT(gSaveContext.inventory.dungeonKeys) &&
           gSaveContext.inventory.dungeonKeys[map] > 0;
}
bool HasBoss() {
    const int map = gSaveContext.mapIndex;
    return map >= 0 && map < (int)ARRAY_COUNT(gSaveContext.inventory.dungeonItems) &&
           CHECK_DUNGEON_ITEM(DUNGEON_KEY_BOSS, map);
}

bool WristFrame(float pos[3], float side[3], float up[3], float forward[3]) {
    float handPos[3], q[4];
    if (!VR_GetHandPose(VR_HAND_LEFT, handPos, q)) return false;
    const float xAxis[3] = { 1, 0, 0 }, yAxis[3] = { 0, 1, 0 }, fAxis[3] = { 0, 0, -1 };
    VrPocket::QuatRot(q, xAxis, side);
    VrPocket::QuatRot(q, yAxis, up);
    VrPocket::QuatRot(q, fAxis, forward);
    const float x = Cm(CVarGetFloat("gVrKeyAnchorX", 10.0f));
    const float y = Cm(CVarGetFloat("gVrKeyAnchorY", 8.0f));
    const float z = Cm(CVarGetFloat("gVrKeyAnchorZ", 5.0f));
    for (int i = 0; i < 3; ++i) pos[i] = handPos[i] + side[i] * x + up[i] * y + forward[i] * z;
    return true;
}

bool Anchor(Kind kind, float pos[3], float side[3], float up[3], float forward[3]) {
    if (!WristFrame(pos, side, up, forward)) return false;
    if (kind == Kind::Boss) {
        const float offset = Cm(10.0f);
        for (int i = 0; i < 3; ++i) pos[i] += side[i] * offset;
    }
    return true;
}

bool HandNear(int hand, const float point[3]) {
    float p[3], q[4];
    if (!VR_GetHandPose(hand, p, q)) return false;
    const float dx = p[0] - point[0], dy = p[1] - point[1], dz = p[2] - point[2];
    const float reach = Cm(CVarGetFloat("gVrKeyGrabRadius", 12.0f));
    return dx * dx + dy * dy + dz * dz <= reach * reach;
}

bool Pose(Kind kind, float pos[3], float side[3], float up[3], float forward[3]) {
    if (sHeld != kind) return Anchor(kind, pos, side, up, forward);
    // A held key has a deliberate grip-local pose rather than preserving the floating anchor's
    // world orientation. It is therefore rigidly part of the controller/closed fist and cannot
    // appear to rotate independently as the wrist turns.
    float handPos[3], q[4];
    if (!VR_GetHandPose(sHand, handPos, q)) return false;
    const float sideLocal[3] = { 0, 1, 0 };
    const float upLocal[3] = { 0, 0, -1 };
    const float forwardLocal[3] = { -1, 0, 0 };
    VrPocket::QuatRot(q, sideLocal, side);
    VrPocket::QuatRot(q, upLocal, up);
    VrPocket::QuatRot(q, forwardLocal, forward);
    const float localOffset[3] = { Cm(1.5f), Cm(-1.0f), Cm(-2.5f) };
    float worldOffset[3];
    VrPocket::QuatRot(q, localOffset, worldOffset);
    for (int i = 0; i < 3; ++i) pos[i] = handPos[i] + worldOffset[i];
    return true;
}
void Clear() {
    sHeld = Kind::None; sHand = -1; sGrab.Clear();
    sNear[0] = sNear[1] = false;
}
void Take(Kind kind, int hand) {
    sGrab.Clear();
    sHeld = kind; sHand = hand;
    VR_TriggerHaptic(hand, 0.5f, 0.0f, 35.0f);
}
} // namespace

extern "C" void VrKeys_Reset(void) {
    Clear(); sGripPrev[0] = sGripPrev[1] = true;
}

extern "C" bool VrKeys_GripConsumed(int32_t hand, uint16_t mask) {
    return (mask & VR_BTN_GRIP) && hand == VR_HAND_RIGHT &&
           ((sHeld != Kind::None && sHand == hand) || sNear[hand]);
}

extern "C" bool VrKeys_HoldsHand(int32_t hand) {
    return sHeld != Kind::None && sHand == hand;
}

extern "C" void VrKeys_Tick(PlayState* play, Player* player) {
    if (!Gate(play, player)) {
        VrKeys_Reset();
        return;
    }
    if ((sHeld == Kind::Small && !HasSmall()) || (sHeld == Kind::Boss && !HasBoss())) Clear();
    bool pressed[2], released[2];
    for (int h = 0; h < 2; ++h) {
        const bool grip = (VR_GetControllerButton(h) & VR_BTN_GRIP) != 0;
        pressed[h] = grip && !sGripPrev[h]; released[h] = !grip && sGripPrev[h];
        sGripPrev[h] = grip; sNear[h] = false;
    }
    if (sHeld != Kind::None) {
        if (released[sHand]) {
            VR_TriggerHaptic(sHand, 0.18f, 0.0f, 18.0f);
            Clear();
        }
        return;
    }
    float pos[3], side[3], up[3], forward[3];
    const Kind available[2] = { HasSmall() ? Kind::Small : Kind::None, HasBoss() ? Kind::Boss : Kind::None };
    for (Kind kind : available) {
        if (kind == Kind::None || !Anchor(kind, pos, side, up, forward)) continue;
        // The key lives on the left wrist HUD, so the right hand takes it. Letting the wrist's own
        // grip qualify would make ordinary left-grip input grab a key without reaching for it.
        for (int h = VR_HAND_RIGHT; h <= VR_HAND_RIGHT; ++h) {
            if (!HandNear(h, pos)) continue;
            sNear[h] = true;
            if (pressed[h]) { Take(kind, h); return; }
        }
    }
}

static void DrawKey(Kind kind, const void* interpKey) {
    float pos[3], side[3], up[3], forward[3];
    if (!Pose(kind, pos, side, up, forward)) return;
    Vec3s rot;
    VrPocket::SetRotFromAxes(&rot, side, up, forward);
    GraphicsContext* gfxCtx = gPlayState->state.gfxCtx;
    const float anchorScale = kind == Kind::Boss ? 0.018f : 0.022f;
    const float heldScale = CVarGetFloat("gVrKeyHeldScale", 180.0f) / 100.0f;
    const float scale = anchorScale * (sHeld == kind ? heldScale : 1.0f);
    OPEN_DISPS(gfxCtx);
    FrameInterpolation_RecordOpenChild(interpKey, 0);
    Matrix_SetTranslateRotateYXZ(pos[0], pos[1], pos[2], &rot);
    Matrix_RotateX(1.57079632679f, MTXMODE_APPLY);
    Matrix_Scale(scale, scale, scale, MTXMODE_APPLY);
    if (sHeld == kind) {
        // The get-item models are authored lengthwise with the decorative bow/head on +Y.
        // Move that end to the controller origin so the closed fist surrounds the non-blade
        // portion and the shaft/teeth extend away from the hand. Applied after scale so these
        // remain model-space offsets as the user changes Held Key Size.
        Matrix_Translate(0.0f, kind == Kind::Boss ? -34.0f : -24.0f, 0.0f, MTXMODE_APPLY);
    }
    MtxF cur; Matrix_Get(&cur);
    Mtx* mtx = MATRIX_NEWMTX(gfxCtx);
    const int weldHand = sHeld == kind ? sHand : VR_HAND_LEFT;
    MtxF hand;
    if (weldHand >= 0 && VR_GetHandMatrix(weldHand, hand.mf)) {
        MtxF inv, local;
        SkinMatrix_Invert(&hand, &inv);
        SkinMatrix_MtxFMtxFMult(&inv, &cur, &local);
        VR_RegisterHandChildMatrix(mtx, weldHand, &local.mf[0][0]);
    }
    POLY_OPA_DISP = Play_SetFog(gPlayState, POLY_OPA_DISP);
    Gfx_SetupDL_25Opa(gfxCtx);
    gSPMatrix(POLY_OPA_DISP++, mtx, G_MTX_NOPUSH | G_MTX_LOAD | G_MTX_MODELVIEW);
    gSPDisplayList(POLY_OPA_DISP++, (Gfx*)(kind == Kind::Boss ? gGiBossKeyDL : gGiSmallKeyDL));
    if (kind == Kind::Boss) {
        Gfx_SetupDL_25Xlu(gfxCtx);
        gSPMatrix(POLY_XLU_DISP++, mtx, G_MTX_NOPUSH | G_MTX_LOAD | G_MTX_MODELVIEW);
        gSPDisplayList(POLY_XLU_DISP++, (Gfx*)gGiBossKeyGemDL);
    }
    FrameInterpolation_RecordCloseChild();
    CLOSE_DISPS(gfxCtx);
}

extern "C" void VrKeys_Draw(void) {
    if (!gPlayState || !Gate(gPlayState, GET_PLAYER(gPlayState))) return;
    const bool anchors = CVarGetInteger("gVrKeyShowAnchors", 1) != 0;
    if (sHeld == Kind::Small || (anchors && HasSmall()))
        DrawKey(Kind::Small, sHeld == Kind::Small ? &sHeldInterpKey : &sSmallInterpKey);
    if (sHeld == Kind::Boss || (anchors && HasBoss()))
        DrawKey(Kind::Boss, sHeld == Kind::Boss ? &sHeldInterpKey : &sBossInterpKey);
}

namespace {
void RegisterVrKeys() { COND_HOOK(OnPlayDrawEnd, true, VrKeys_Draw); }
static RegisterShipInitFunc initVrKeys(RegisterVrKeys);
}
