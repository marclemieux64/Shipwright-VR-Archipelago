extern "C" {
#include "z64.h"
#include "macros.h"
#include "functions.h"
#include "objects/gameplay_keep/gameplay_keep.h"
#include "overlays/actors/ovl_En_Door/z_en_door.h"
#include "overlays/actors/ovl_Door_Shutter/z_door_shutter.h"
extern PlayState* gPlayState;
s32 Player_HandleExitsAndVoids(PlayState* play, Player* player, CollisionPoly* poly, u32 bgId);
void VrDoor_ReleaseRetainedObjects(PlayState* play, bool keepDestination);
void VrDoor_ResetRetainedObjects(void);
}

#include "VrCombat.h"
#include "soh/Enhancements/game-interactor/GameInteractor.h"
#include "soh/ShipInit.hpp"
#include "soh/frame_interpolation.h"
#include <libultraship/bridge/consolevariablebridge.h>
#include <vr_interface.h>
#include <cfloat>
#include <cmath>
#include <cstring>

// Physical ordinary doors. En_Door's skeleton offsets the rotating hierarchy by 2700 model units
// (27 world units) before applying limb 3's opening rotation. We keep all geometry in the actor's
// closed-door frame, then write the same limb rotation channel used by the stock animation.
namespace {
constexpr float kTurnDeg = 22.0f;
constexpr float kMaxOpenDeg = 90.0f;
constexpr float kExitDeg = 55.0f;
// Unlock the doorway early enough that a door pushed away from the player can be walked around
// before its handle leaves comfortable reach.
constexpr float kWalkThroughDeg = 25.0f;

EnDoor* sDoor = nullptr;
EnDoor* sCandidate = nullptr;
int sHand = -1;
bool sUnlatched = false;
bool sRoomRequested = false;
bool sPassagePending = false;
bool sPassageCommitted = false;
bool sGripPrev[2] = { true, true };
bool sGripNow[2] = { false, false };
bool sGripFresh[2] = { false, false };
uint32_t sInputFrame = UINT32_MAX;
float sStartQuat[4] = {};
float sPullStartRad = 0.0f;
float sPullStartDeg = 0.0f;
bool sPullRefValid = false;
bool sOpenSoundPlayed = false;
bool sCloseSoundPlayed = false;
float sSide = 1.0f;
float sDisplaySide = 1.0f;
float sHandleSide = 1.0f;
float sKnobFace = 1.0f;
s16 sAngle = 0;
uint32_t sTickFrame = 0;
bool sNear[2] = { false, false };
float sNearFace[2] = { 1.0f, 1.0f };
float sNearHandleSide[2] = { 1.0f, 1.0f };
float sGrabOffsetLocal[3] = {};
float sPrevPlaneDistance = 1e30f;

DoorShutter* sJabuDoor = nullptr;
DoorShutter* sJabuLatchDoor = nullptr;
bool sJabuNear[2] = { false, false };
bool sJabuLatched[2] = { false, false };

DoorShutter* sShutterCandidate = nullptr;
DoorShutter* sShutterDoor = nullptr;
DoorShutter* sShutterLatchDoor = nullptr;
DoorShutter* sShutterStreamDoor = nullptr;
DoorShutter* sShutterJustClosedDoor = nullptr;
bool sShutterLatched[2] = { false, false };
bool sShutterGripNow[2] = { false, false };
uint32_t sShutterInputFrame = UINT32_MAX;
float sShutterLatchSide[2] = {};
struct ShutterHandState {
    DoorShutter* door = nullptr;
    uint32_t frame = UINT32_MAX;
    bool active = false;
    bool latched = false;
    float score = FLT_MAX;
    float localX = 0.0f;
    float localY = 0.0f;
    float surfaceFace = 1.0f;
    float visualFace = 1.0f;
};
ShutterHandState sShutterHandState[2];
DoorShutter* sShutterPrevHoverDoor[2] = { nullptr, nullptr };
float sShutterPrevHoverFace[2] = { 1.0f, 1.0f };
float sShutterFrameHandPos[2][3] = {};
float sShutterPrevHandPos[2][3] = {};
bool sShutterFrameHandValid[2] = { false, false };
bool sShutterPrevHandValid[2] = { false, false };
float sShutterGrabFace[2] = {};
float sShutterGrabLocal[2][2] = {};
float sShutterStartMeanY = 0.0f;
float sShutterStartHeight = 0.0f;
float sShutterEntrySide = 1.0f;
bool sShutterRoomRequested = false;
bool sShutterFullyOpen = false;
bool sShutterLiftArmed = false;
bool sShutterCloseRollbackPending = false;
bool sShutterCloseForNextDoor = false;

// Room streaming is identical for every physical doorway.  Interaction code (turning a handle,
// lifting a slab, or touching a membrane) only drives the door's animation; this object owns the
// shared preload/commit/rollback lifecycle.
struct PhysicalDoorwayTransition {
    Actor* owner = nullptr;
    float entrySide = 1.0f;
    bool requested = false;
    bool crossed = false;
};

PhysicalDoorwayTransition sHingeTransition;
PhysicalDoorwayTransition sShutterTransition;
bool sPhysicalRoomRequest = false;

void SetDoorwayActorRoom(Actor* actor, int room, bool includeRelatives) {
    if (actor == nullptr) return;
    actor->room = room;
    if (includeRelatives && actor->child != nullptr) actor->child->room = room;
    if (includeRelatives && actor->parent != nullptr) actor->parent->room = room;
}

bool BeginDoorwayTransition(PhysicalDoorwayTransition& transition, Actor* actor, PlayState* play,
                            float entrySide, bool includeRelatives) {
    if (actor == nullptr || play == nullptr || transition.requested || play->roomCtx.status != 0 ||
        play->roomCtx.prevRoom.num >= 0) {
        return transition.requested && transition.owner == actor;
    }
    const int index = GET_TRANSITION_ACTOR_INDEX(actor);
    const TransitionActorEntry* entry = &play->transiActorCtx.list[index];
    int destination;
    if (entry->sides[0].room == play->roomCtx.curRoom.num) {
        destination = entry->sides[1].room;
    } else if (entry->sides[1].room == play->roomCtx.curRoom.num) {
        destination = entry->sides[0].room;
    } else {
        destination = entry->sides[entrySide > 0.0f ? 0 : 1].room;
    }
    if (destination < 0 || destination == play->roomCtx.curRoom.num) {
        return false;
    }
    sPhysicalRoomRequest = true;
    const bool requested = Room_RequestNewRoom(play, &play->roomCtx, destination) != 0;
    sPhysicalRoomRequest = false;
    if (!requested) return false;
    transition.owner = actor;
    transition.entrySide = entrySide;
    transition.requested = true;
    transition.crossed = false;
    SetDoorwayActorRoom(actor, -1, includeRelatives);
    return true;
}

void FinishDoorwayTransition(PhysicalDoorwayTransition& transition, PlayState* play, float currentSide,
                             bool includeRelatives) {
    if (!transition.requested || play == nullptr || play->roomCtx.status != 0) return;
    const bool keepDestination = currentSide != transition.entrySide;
    transition.crossed = keepDestination;
    if (play->roomCtx.prevRoom.num >= 0) {
        if (!keepDestination) {
            Room room = play->roomCtx.curRoom;
            play->roomCtx.curRoom = play->roomCtx.prevRoom;
            play->roomCtx.prevRoom = room;
        }
        Room_FinishRoomChange(play, &play->roomCtx);
    }
    VrDoor_ReleaseRetainedObjects(play, keepDestination);
    if (keepDestination) Play_SetupRespawnPoint(play, RESPAWN_MODE_DOWN, 0xDFF);
    SetDoorwayActorRoom(transition.owner, play->roomCtx.curRoom.num, includeRelatives);
    transition = {};
}
constexpr float kShutterSurface = 12.0f;
constexpr float kBossShutterRaisedHeight = 240.0f;
constexpr float kShutterReleaseLiftMeters = 0.30f;

bool ShutterLiftEligible(const DoorShutter* door) {
    return door != nullptr && door->gfxType != 3 && door->doorType != SHUTTER_PG_BARS &&
           door->doorType != SHUTTER_GOHMA_BLOCK && door->unlockTimer == 0 &&
           door->barsClosedAmount <= 0.001f;
}

float ShutterGrabHalfWidth(const DoorShutter* door) {
    if (door == nullptr) return 0.0f;
    if (door->gfxType == 2) return 70.0f; // Dodongo's Cavern stone shutter
    if (door->gfxType == 6 || door->gfxType == 7) return 50.0f; // Spirit and Boss shutters
    // The remaining temple/Ganon shutters have a visibly broad slab as well.  The former 30-unit
    // half-width restricted both hands to a narrow strip around the centre; on doors such as the
    // Light Trial shutter, reaching naturally for its two sides could never enter either zone.
    return 50.0f;
}

float ShutterRaisedHeight(const DoorShutter* door) {
    return door != nullptr && door->doorType == SHUTTER_BOSS ? kBossShutterRaisedHeight : 200.0f;
}

float WorldScale() {
    const float ws = VR_GetWorldScale();
    return ws < 1.0f ? 35.0f : ws;
}

float HingeOffset() {
    return CVarGetFloat("gVrDoorHingeOffset", 27.0f);
}

void Axes(const EnDoor* door, float right[3], float normal[3]) {
    const float s = Math_SinS(door->actor.shape.rot.y), c = Math_CosS(door->actor.shape.rot.y);
    right[0] = c; right[1] = 0.0f; right[2] = -s;
    normal[0] = s; normal[1] = 0.0f; normal[2] = c;
}

float PlayerSide(const EnDoor* door, const Player* player) {
    float r[3], n[3]; Axes(door, r, n);
    const float d = (player->actor.world.pos.x - door->actor.world.pos.x) * n[0] +
                    (player->actor.world.pos.z - door->actor.world.pos.z) * n[2];
    return d >= 0.0f ? 1.0f : -1.0f;
}

bool PlayerInsideDoorway(const EnDoor* door, const Player* player, float halfWidth) {
    if (door == nullptr || player == nullptr) return false;
    float right[3], normal[3]; Axes(door, right, normal);
    const float dx = player->actor.world.pos.x - door->actor.world.pos.x;
    const float dz = player->actor.world.pos.z - door->actor.world.pos.z;
    const float lateral = dx * right[0] + dz * right[2];
    const float y = player->actor.world.pos.y - door->actor.world.pos.y;
    // Link's actor position is at his feet. Only crossing the finite opening counts; crossing the
    // door plane above the lintel must leave the preloaded destination uncommitted.
    return fabsf(lateral) < halfWidth && y > -8.0f && y < 80.0f;
}

float EyeDistanceXZ(const EnDoor* door, const PlayState* play) {
    const float dx = play->view.eye.x - door->actor.world.pos.x;
    const float dz = play->view.eye.z - door->actor.world.pos.z;
    return sqrtf(dx * dx + dz * dz);
}

void HandlePoint(const EnDoor* door, float side, float face, float out[3]) {
    float r[3], n[3]; Axes(door, r, n);
    // Match the side-specific face display list: positive calibration means the visually right
    // side of the door from whichever face the player is standing on.
    float localX = CVarGetFloat("gVrDoorHandleRight", 23.0f) * side;
    float localZ = CVarGetFloat("gVrDoorHandleDepth", 3.0f) * face;
    if (sDoor == door && sAngle != 0) {
        const float a = sAngle * (2.0f * M_PI / 65536.0f);
        const float hingeOffset = HingeOffset();
        const float hingeX = localX >= 0.0f ? -hingeOffset : hingeOffset;
        const float relX = localX - hingeX;
        const float rotatedX = relX * cosf(a) + localZ * sinf(a);
        localZ = -relX * sinf(a) + localZ * cosf(a);
        localX = hingeX + rotatedX;
    }
    out[0] = door->actor.world.pos.x + r[0] * localX + n[0] * localZ;
    out[1] = door->actor.world.pos.y + CVarGetFloat("gVrDoorHandleY", 38.0f);
    out[2] = door->actor.world.pos.z + r[2] * localX + n[2] * localZ;
}

bool HandPosition(int hand, float out[3]) {
    float q[4];
    return VR_GetHandPose(hand, out, q);
}

bool HandPose(int hand, float outPos[3], float outQuat[4]) {
    return VR_GetHandPose(hand, outPos, outQuat);
}

bool HandMarkerDistance(int hand, const float marker[3], float* outDistance) {
    float m[4][4];
    if (!VR_GetHandMatrix(hand, m)) {
        float p[3];
        if (!HandPosition(hand, p)) return false;
        const float dx = p[0] - marker[0], dy = p[1] - marker[1], dz = p[2] - marker[2];
        *outDistance = sqrtf(dx * dx + dy * dy + dz * dz);
        return true;
    }
    const float len = VrHand_ChildSized() ? 548.0f : 862.0f;
    float bestSq = 1e30f;
    for (int i = 0; i <= 2; ++i) {
        const float x = len * 0.5f * i;
        const float p[3] = { m[0][0] * x + m[3][0], m[0][1] * x + m[3][1], m[0][2] * x + m[3][2] };
        const float dx = p[0] - marker[0], dy = p[1] - marker[1], dz = p[2] - marker[2];
        bestSq = fminf(bestSq, dx * dx + dy * dy + dz * dz);
    }
    *outDistance = sqrtf(bestSq);
    return true;
}

float HandTurnDeg(int hand) {
    float p[3], q[4];
    if (!HandPose(hand, p, q)) return 0.0f;
    float dot = fabsf(q[0] * sStartQuat[0] + q[1] * sStartQuat[1] + q[2] * sStartQuat[2] + q[3] * sStartQuat[3]);
    dot = fmaxf(0.0f, fminf(1.0f, dot));
    return 2.0f * acosf(dot) * (180.0f / M_PI);
}

float HandPolar(const EnDoor* door, int hand, float side) {
    float p[3], right[3], normal[3];
    if (!HandPosition(hand, p)) return 0.0f;
    Axes(door, right, normal);
    const float handleX = CVarGetFloat("gVrDoorHandleRight", 23.0f) * side;
    const float hingeOffset = HingeOffset();
    const float hingeLocal = handleX >= 0.0f ? -hingeOffset : hingeOffset;
    const float hingeX = door->actor.world.pos.x + right[0] * hingeLocal;
    const float hingeZ = door->actor.world.pos.z + right[2] * hingeLocal;
    const float x = (p[0] - hingeX) * right[0] + (p[2] - hingeZ) * right[2];
    const float z = (p[0] - hingeX) * normal[0] + (p[2] - hingeZ) * normal[2];
    return atan2f(z, x);
}

bool Eligible(const EnDoor* door, PlayState* play, Player* player) {
    const int type = (door->actor.params >> 7) & 7;
    return CVarGetInteger("gVrPhysicalDoors", 1) && VR_IsInitialized() && VR_GetFirstPerson() &&
           !VR_IsFlatScreen() && CVarGetInteger("gVrMotionHands", 1) &&
           (type == DOOR_ROOMLOAD || type == DOOR_ROOMLOAD2 || type == DOOR_ROOMLOAD7 || type == DOOR_SCENEEXIT ||
            type == DOOR_LOCKED) &&
           door->lockTimer == 0 && play->csCtx.state == CS_STATE_IDLE &&
           play->transitionTrigger == TRANS_TRIGGER_OFF && !(player->stateFlags1 &
               (PLAYER_STATE1_ON_HORSE | PLAYER_STATE1_IN_WATER | PLAYER_STATE1_DEAD));
}

void Clear() {
    sDoor = nullptr; sHand = -1; sUnlatched = false; sRoomRequested = false; sPassagePending = false;
    sPassageCommitted = false; sAngle = 0;
    sPullRefValid = false; sOpenSoundPlayed = false; sCloseSoundPlayed = false;
    sPrevPlaneDistance = 1e30f;
    sNear[0] = sNear[1] = false;
    sHingeTransition = {};
}

void ClearAllDoorState() {
    Clear();
    sCandidate = nullptr;
    sJabuDoor = sJabuLatchDoor = nullptr;
    sJabuNear[0] = sJabuNear[1] = false;
    sJabuLatched[0] = sJabuLatched[1] = false;
    sShutterCandidate = sShutterDoor = sShutterLatchDoor = sShutterStreamDoor = sShutterJustClosedDoor = nullptr;
    sShutterLatched[0] = sShutterLatched[1] = false;
    sShutterHandState[0] = {};
    sShutterHandState[1] = {};
    sShutterPrevHoverDoor[0] = sShutterPrevHoverDoor[1] = nullptr;
    sShutterFrameHandValid[0] = sShutterFrameHandValid[1] = false;
    sShutterPrevHandValid[0] = sShutterPrevHandValid[1] = false;
    sShutterRoomRequested = sShutterFullyOpen = sShutterLiftArmed = sShutterCloseRollbackPending = false;
    sShutterCloseForNextDoor = false;
    sShutterTransition = {};
}

void ForgetDestroyedDoor(Actor* actor) {
    if (actor == nullptr) return;
    if (actor == reinterpret_cast<Actor*>(sDoor)) Clear();
    if (actor == reinterpret_cast<Actor*>(sCandidate)) sCandidate = nullptr;
    if (actor == reinterpret_cast<Actor*>(sJabuDoor)) sJabuDoor = nullptr;
    if (actor == reinterpret_cast<Actor*>(sJabuLatchDoor)) sJabuLatchDoor = nullptr;
    if (actor == reinterpret_cast<Actor*>(sShutterCandidate)) sShutterCandidate = nullptr;
    if (actor == reinterpret_cast<Actor*>(sShutterDoor)) sShutterDoor = nullptr;
    if (actor == reinterpret_cast<Actor*>(sShutterJustClosedDoor)) sShutterJustClosedDoor = nullptr;
    for (int hand = 0; hand < 2; ++hand) {
        if (actor == reinterpret_cast<Actor*>(sShutterHandState[hand].door)) sShutterHandState[hand] = {};
        if (actor == reinterpret_cast<Actor*>(sShutterPrevHoverDoor[hand])) sShutterPrevHoverDoor[hand] = nullptr;
    }
    if (actor == reinterpret_cast<Actor*>(sShutterStreamDoor)) {
        sShutterStreamDoor = nullptr;
        sShutterRoomRequested = false;
    }
    if (actor == sHingeTransition.owner) sHingeTransition = {};
    if (actor == sShutterTransition.owner) sShutterTransition = {};
    if (actor == reinterpret_cast<Actor*>(sShutterLatchDoor)) {
        sShutterLatchDoor = nullptr;
        sShutterFullyOpen = sShutterLiftArmed = sShutterCloseRollbackPending = false;
        sShutterCloseForNextDoor = false;
    }
}

void CaptureGrabOffset(const EnDoor* door, int hand, float side) {
    float handPos[3], handle[3], right[3], normal[3];
    if (!HandPosition(hand, handPos)) return;
    HandlePoint(door, sHandleSide, sKnobFace, handle);
    Axes(door, right, normal);
    const float d[3] = { handPos[0] - handle[0], handPos[1] - handle[1], handPos[2] - handle[2] };
    sGrabOffsetLocal[0] = d[0] * right[0] + d[2] * right[2];
    sGrabOffsetLocal[1] = d[1];
    sGrabOffsetLocal[2] = d[0] * normal[0] + d[2] * normal[2];
}

void Grab(EnDoor* door, int hand, float side, float face, const float quat[4], bool preserveAngle) {
    sDoor = door; sHand = hand; sSide = side; sUnlatched = false;
    if (!preserveAngle) sDisplaySide = side;
    sHandleSide = sNearHandleSide[hand];
    sKnobFace = face;
    if (!preserveAngle) sAngle = 0;
    for (int i = 0; i < 4; ++i) sStartQuat[i] = quat[i];
    sPullRefValid = false;
    sCloseSoundPlayed = false;
    CaptureGrabOffset(door, hand, side);
    sPrevPlaneDistance = 1e30f;
    VR_TriggerHaptic(hand, 0.35f, 0.0f, 25.0f);
}

void RequestRoom(EnDoor* door, PlayState* play, Player* player);

void AssistWalkThrough(EnDoor* door, PlayState* play, Player* player) {
    float right[3], normal[3]; Axes(door, right, normal);
    const float dx = player->actor.world.pos.x - door->actor.world.pos.x;
    const float dz = player->actor.world.pos.z - door->actor.world.pos.z;
    const float across = dx * normal[0] + dz * normal[2];
    const float distance = fabsf(across);

    // Approaching the open centre starts streaming the adjacent room. Once it is ready, the local
    // wall-collision aperture below lets the player cross under their own locomotion.
    if (sPrevPlaneDistance < 1e20f && distance < sPrevPlaneDistance - 0.02f && distance < 36.0f &&
        PlayerInsideDoorway(door, player, 24.0f)) {
        sPassagePending = true;
    }
    if (sPassagePending && !sRoomRequested) {
        RequestRoom(door, play, player);
    }
    if (sPassagePending && sRoomRequested && play->roomCtx.status == 0 && PlayerSide(door, player) != sSide &&
        PlayerInsideDoorway(door, player, 18.0f)) {
        sPassageCommitted = true;
    }
    sPrevPlaneDistance = distance;
}

void RequestRoom(EnDoor* door, PlayState* play, Player* player) {
    sRoomRequested = BeginDoorwayTransition(sHingeTransition, &door->actor, play, sSide, true);
}

void ResolveLoadedRooms(EnDoor*, PlayState* play, Player*) {
    if (!sRoomRequested || play->roomCtx.status != 0) return;
    // A side change alone is insufficient: Link can cross the door's infinite plane above its
    // frame. Preserve the destination only after a crossing inside the physical aperture.
    const float resolvedSide = sPassageCommitted ? -sHingeTransition.entrySide : sHingeTransition.entrySide;
    FinishDoorwayTransition(sHingeTransition, play, resolvedSide, true);
    sRoomRequested = false;
}
} // namespace

extern "C" bool VrJabuDoor_Tick(DoorShutter* door, PlayState* play) {
    if (door == nullptr || play == nullptr || door->gfxType != 3 ||
        !CVarGetInteger("gVrPhysicalDoors", 1) || !VR_IsInitialized() || !VR_GetFirstPerson() ||
        VR_IsFlatScreen() || !CVarGetInteger("gVrMotionHands", 1) || play->csCtx.state != CS_STATE_IDLE) {
        if (sJabuDoor == door) sJabuDoor = nullptr;
        if (sJabuLatchDoor == door) sJabuLatchDoor = nullptr;
        sJabuNear[0] = sJabuNear[1] = false;
        sJabuLatched[0] = sJabuLatched[1] = false;
        return false;
    }
    if ((sJabuLatchDoor != nullptr && sJabuLatchDoor != door) ||
        (sShutterLatchDoor != nullptr && sShutterLatchDoor != door)) return false;

    Player* player = GET_PLAYER(play);
    const float s = Math_SinS(door->dyna.actor.shape.rot.y);
    const float c = Math_CosS(door->dyna.actor.shape.rot.y);
    bool touching = false;
    for (int hand = 0; hand < 2; ++hand) {
        float p[3];
        if (!HandPosition(hand, p)) {
            sJabuNear[hand] = false;
            continue;
        }
        const float dx = p[0] - door->dyna.actor.world.pos.x;
        const float dy = p[1] - (door->dyna.actor.world.pos.y + 80.0f);
        const float dz = p[2] - door->dyna.actor.world.pos.z;
        const float localX = dx * c - dz * s;
        const float localZ = dx * s + dz * c;
        sJabuNear[hand] = fabsf(localZ) <= 18.0f && fabsf(dy) <= 75.0f && fabsf(localX) <= 70.0f;
        if (sJabuNear[hand]) {
            touching = true;
            if (!sJabuLatched[hand]) {
                sJabuLatched[hand] = true;
                sJabuLatchDoor = door;
                VR_TriggerHaptic(hand, 0.3f, 0.0f, 25.0f);
            }
        } else {
            sJabuLatched[hand] = false;
        }
    }

    if (!touching) {
        if (sJabuDoor == door && !sShutterTransition.requested) sJabuDoor = nullptr;
        if (!sJabuLatched[0] && !sJabuLatched[1] && !sShutterTransition.requested) sJabuLatchDoor = nullptr;
        return false;
    }

    const float pdx = player->actor.world.pos.x - door->dyna.actor.world.pos.x;
    const float pdz = player->actor.world.pos.z - door->dyna.actor.world.pos.z;
    const float entrySide = (pdx * s + pdz * c) >= 0.0f ? 1.0f : -1.0f;
    if (!sShutterTransition.requested) {
        if (!BeginDoorwayTransition(sShutterTransition, &door->dyna.actor, play, entrySide, false)) {
            return false;
        }
        sJabuDoor = door;
        sShutterStreamDoor = door;
        sShutterRoomRequested = true;
        sShutterEntrySide = entrySide;
    }
    if (play->roomCtx.status != 0) return false;

    // From this point the membrane uses the same open/cross/close lifecycle as every rigid shutter.
    // Only its activation differs: an open palm touching the flesh replaces the two-hand lift.
    sShutterLatchDoor = door;
    sShutterFullyOpen = true;
    sShutterCloseRollbackPending = false;
    for (int hand = 0; hand < 2; ++hand) {
        if (sJabuLatched[hand]) {
            VR_TriggerHaptic(hand, 0.3f, 0.0f, 25.0f);
        }
    }
    return true;
}

extern "C" bool VrJabuDoor_GripConsumed(int32_t hand, uint16_t mask) {
    return false;
}

extern "C" int32_t VrShutter_Tick(DoorShutter* door, PlayState* play) {
    Player* player = play != nullptr ? GET_PLAYER(play) : nullptr;
    const bool ownsActivePreload = door != nullptr && door == sShutterLatchDoor && door == sShutterStreamDoor &&
                                   sShutterRoomRequested && !sShutterFullyOpen;
    const bool eligible = player != nullptr && (ShutterLiftEligible(door) || ownsActivePreload) &&
                          CVarGetInteger("gVrPhysicalDoors", 1) &&
                          VR_IsInitialized() && VR_GetFirstPerson() && !VR_IsFlatScreen() &&
                          CVarGetInteger("gVrMotionHands", 1) && play->csCtx.state == CS_STATE_IDLE &&
                          play->transitionTrigger == TRANS_TRIGGER_OFF;
    if (!eligible) {
        if (sShutterCandidate == door) sShutterCandidate = nullptr;
        if (sShutterDoor == door) sShutterDoor = nullptr;
        // Door_Shutter actors update independently. An unrelated locked/barred/special shutter
        // must never clear the hands attached to the active shutter merely because *it* is not
        // eligible. Central dispatch made that old global cleanup run for every shutter and broke
        // lifting immediately after a successful grab.
        if (sShutterLatchDoor == door && !sShutterFullyOpen) {
            sShutterLatchDoor = nullptr;
            sShutterLatched[0] = sShutterLatched[1] = false;
            sShutterHandState[0] = {};
            sShutterHandState[1] = {};
            sShutterLiftArmed = false;
        }
        return false;
    }
    // Destination room setup may try to bar its entrance as soon as it is preloaded. The player
    // has not crossed yet, and this exact shutter was already valid when grabbed, so defer that
    // lock-behind state until the physical close on the destination side.
    if (ownsActivePreload) door->barsClosedAmount = 0.0f;
    if (sShutterLatchDoor != nullptr && sShutterLatchDoor != door) {
        // The room system can safely retain only the two rooms around one physical doorway. In a
        // short room Link may reach the next shutter before the previous one's distance threshold.
        // Ask the crossed shutter to close instead of making this new one appear permanently inert.
        if (sShutterFullyOpen && door->dyna.actor.xzDistToPlayer < 100.0f) sShutterCloseForNextDoor = true;
        return false;
    }

    // Every shutter actor ticks independently. Sample grip edges once per game frame so a distant
    // shutter cannot consume the press before the door under the player's hands sees it.
    if (sShutterInputFrame != play->state.frames) {
        sShutterInputFrame = play->state.frames;
        if (sShutterLatchDoor == nullptr) {
            sShutterCandidate = nullptr;
        }
        for (int hand = 0; hand < 2; ++hand) {
            const ShutterHandState previous = sShutterHandState[hand];
            const bool previousFrame = previous.frame != UINT32_MAX && previous.frame + 1 == play->state.frames;
            sShutterPrevHoverDoor[hand] =
                previousFrame && previous.active && !previous.latched ? previous.door : nullptr;
            sShutterPrevHoverFace[hand] = previous.surfaceFace;
            sShutterHandState[hand] = {};
            sShutterHandState[hand].frame = play->state.frames;
            sShutterGripNow[hand] = (VR_GetControllerButton(hand) & VR_BTN_GRIP) != 0;
            if (sShutterFrameHandValid[hand]) {
                memcpy(sShutterPrevHandPos[hand], sShutterFrameHandPos[hand], sizeof(sShutterPrevHandPos[hand]));
                sShutterPrevHandValid[hand] = true;
            }
            sShutterFrameHandValid[hand] = HandPosition(hand, sShutterFrameHandPos[hand]);
            if (!sShutterPrevHandValid[hand] && sShutterFrameHandValid[hand]) {
                memcpy(sShutterPrevHandPos[hand], sShutterFrameHandPos[hand], sizeof(sShutterPrevHandPos[hand]));
                sShutterPrevHandValid[hand] = true;
            }
        }
    }

    // Releasing a grabbed door before lifting it restores the room the player still occupies.
    // This is a streaming operation, not a physical close, so it deliberately has no door sound.
    if (sShutterRoomRequested && sShutterStreamDoor == door && !sShutterFullyOpen &&
        !sShutterLatched[0] && !sShutterLatched[1] &&
        play->roomCtx.status == 0) {
        FinishDoorwayTransition(sShutterTransition, play, sShutterEntrySide, false);
        sShutterRoomRequested = false;
        sShutterStreamDoor = nullptr;
        sShutterCandidate = nullptr;
        sShutterLatchDoor = nullptr;
        sShutterLiftArmed = false;
        sShutterCloseForNextDoor = false;
        return false;
    }

    const float sn = Math_SinS(door->dyna.actor.shape.rot.y);
    const float cs = Math_CosS(door->dyna.actor.shape.rot.y);
    const float pdx = player->actor.world.pos.x - door->dyna.actor.world.pos.x;
    const float pdz = player->actor.world.pos.z - door->dyna.actor.world.pos.z;
    const float playerAcross = pdx * sn + pdz * cs;
    // Use the shutter actor's own side resolver whenever it has an answer. It includes the native
    // facing/distance rules and therefore reflects the actor's initialized orientation rather
    // than duplicating constructor assumptions in the VR layer.
    const float bodyFace = playerAcross >= 0.0f ? 1.0f : -1.0f;

    float handY[2] = {};
    bool grip[2] = {};
    const float grabHalfWidth = ShutterGrabHalfWidth(door);
    for (int hand = 0; hand < 2; ++hand) {
        const float* p = sShutterFrameHandPos[hand];
        if (!sShutterFrameHandValid[hand]) {
            continue;
        }
        const float dx = p[0] - door->dyna.actor.world.pos.x;
        const float dz = p[2] - door->dyna.actor.world.pos.z;
        const float localX = dx * cs - dz * sn;
        const float localZ = dx * sn + dz * cs;
        const float localY = p[1] - door->dyna.actor.world.pos.y;
        float contactX = localX, contactY = localY, contactZ = localZ;
        float handMtx[4][4];
        if (VR_GetHandMatrix(hand, handMtx)) {
            const float len = VrHand_ChildSized() ? 548.0f : 862.0f;
            float best = fabsf(localZ);
            for (int point = 0; point < 3; ++point) {
                const float along = len * 0.5f * point;
                const float px = handMtx[3][0] + handMtx[0][0] * along;
                const float py = handMtx[3][1] + handMtx[0][1] * along;
                const float pz = handMtx[3][2] + handMtx[0][2] * along;
                const float pdx = px - door->dyna.actor.world.pos.x;
                const float pdz = pz - door->dyna.actor.world.pos.z;
                const float lx = pdx * cs - pdz * sn;
                const float lz = pdx * sn + pdz * cs;
                const float ly = py - door->dyna.actor.world.pos.y;
                if (fabsf(lx) <= grabHalfWidth && ly >= 20.0f && ly <= 125.0f && fabsf(lz) < best) {
                    best = fabsf(lz);
                    contactX = lx; contactY = ly; contactZ = lz;
                }
            }
        }
        handY[hand] = p[1];
        const bool near = fabsf(contactX) <= grabHalfWidth && fabsf(contactZ) <= 28.0f &&
                          contactY >= 20.0f && contactY <= 125.0f;
        const float eyeDx = play->view.eye.x - door->dyna.actor.world.pos.x;
        const float eyeDz = play->view.eye.z - door->dyna.actor.world.pos.z;
        const float visualFace = (eyeDx * sn + eyeDz * cs) >= 0.0f ? 1.0f : -1.0f;
        // On first contact, use the hand's previous-frame position relative to this exact door.
        // This records the side from which the hand crossed into the hover zone before current-
        // frame penetration can reverse it, and is rotationally invariant for every shutter yaw.
        float hoverFace = bodyFace;
        if (sShutterPrevHoverDoor[hand] == door) {
            hoverFace = sShutterPrevHoverFace[hand];
        } else if (near) {
            float approachFace = bodyFace;
            float prevLocalZ = localZ;
            if (sShutterPrevHandValid[hand]) {
                const float prevDx = sShutterPrevHandPos[hand][0] - door->dyna.actor.world.pos.x;
                const float prevDz = sShutterPrevHandPos[hand][2] - door->dyna.actor.world.pos.z;
                prevLocalZ = prevDx * sn + prevDz * cs;
                if (fabsf(prevLocalZ) > 0.5f) approachFace = prevLocalZ >= 0.0f ? 1.0f : -1.0f;
            }
            hoverFace = approachFace;
        }
        grip[hand] = sShutterGripNow[hand];
        if (sShutterLatchDoor == door && sShutterLatched[hand] && !grip[hand]) {
            sShutterLatched[hand] = false;
        }

        // A latched hand always owns its visual state. Otherwise accumulate the closest valid
        // shutter for this hand; a later, distant actor may never overwrite it with "not near".
        if (sShutterLatchDoor == door && sShutterLatched[hand]) {
            ShutterHandState& state = sShutterHandState[hand];
            state.door = door;
            state.frame = play->state.frames;
            state.active = true;
            state.latched = true;
            state.score = 0.0f;
            state.localX = sShutterGrabLocal[hand][0];
            state.localY = sShutterGrabLocal[hand][1];
            state.surfaceFace = sShutterGrabFace[hand] >= 0.0f ? 1.0f : -1.0f;
            state.visualFace = state.surfaceFace;
        } else if (near && (sShutterLatchDoor == nullptr || sShutterLatchDoor == door) &&
                   fabsf(contactZ) < sShutterHandState[hand].score) {
            ShutterHandState& state = sShutterHandState[hand];
            state.door = door;
            state.frame = play->state.frames;
            state.active = true;
            state.latched = false;
            state.score = fabsf(contactZ);
            state.localX = contactX;
            state.localY = contactY;
            state.surfaceFace = hoverFace;
            state.visualFace = visualFace;
            sShutterCandidate = door;
        }

        // Grip only the owner resolved on the previous completed frame. This one-frame handover
        // removes actor-order dependence when two shutters share a compact room.
        if (!sShutterLatched[hand] && grip[hand] && near && sShutterPrevHoverDoor[hand] == door &&
            (sShutterLatchDoor == nullptr || sShutterLatchDoor == door)) {
            sShutterLatchDoor = door;
            sShutterCandidate = door;
            sShutterLatched[hand] = true;
            sShutterLatchSide[hand] = contactX >= 0.0f ? 1.0f : -1.0f;
            const float surface = hoverFace * kShutterSurface;
            // As with climbing, snap the closest part of the rendered hand to the surface rather
            // than putting the controller origin there. This keeps fingertips/palm out of the slab.
            sShutterGrabFace[hand] = localZ + (surface - contactZ);
            sShutterGrabLocal[hand][0] = localX;
            sShutterGrabLocal[hand][1] = localY;
            ShutterHandState& state = sShutterHandState[hand];
            state.door = door;
            state.frame = play->state.frames;
            state.active = true;
            state.latched = true;
            state.score = 0.0f;
            state.localX = localX;
            state.localY = localY;
            state.surfaceFace = sShutterGrabFace[hand] >= 0.0f ? 1.0f : -1.0f;
            state.visualFace = state.surfaceFace;
            VR_TriggerHaptic(hand, 0.3f, 0.0f, 25.0f);
        }
    }

    // A room preload belongs exclusively to the shutter the player actually grabbed. Proximity
    // alone is not sufficient: several shutters can coexist in compact dungeon rooms and used to
    // overwrite this global interaction state, producing flicker and incomplete openings.
    if (sShutterLatchDoor == door && !sShutterRoomRequested && play->roomCtx.status == 0 &&
        play->roomCtx.prevRoom.num < 0) {
        const float entrySide = playerAcross >= 0.0f ? 1.0f : -1.0f;
        if (BeginDoorwayTransition(sShutterTransition, &door->dyna.actor, play, entrySide, false)) {
            sShutterRoomRequested = true;
            sShutterStreamDoor = door;
            sShutterEntrySide = entrySide;
        }
    }

    const int transitionIndex = GET_TRANSITION_ACTOR_INDEX(&door->dyna.actor);
    const TransitionActorEntry* transitionEntry = &play->transiActorCtx.list[transitionIndex];
    const int adjacentRoom = transitionEntry->sides[0].room == play->roomCtx.curRoom.num
                                 ? transitionEntry->sides[1].room
                                 : transitionEntry->sides[0].room;
    const bool needsAdjacentRoom = adjacentRoom >= 0 && adjacentRoom != play->roomCtx.curRoom.num;

    // Any two points on the slab form a valid lift. Requiring opposite halves made the grip
    // needlessly dependent on the player's hand spacing and was unlike the climbing interaction.
    const bool holding = sShutterLatched[0] && sShutterLatched[1] && grip[0] && grip[1];
    if (!holding) {
        // A comfortable 30 cm two-handed lift commits the interaction. Releasing either grip
        // then hands control back to the native shutter action, which smoothly finishes raising
        // the full-height slab instead of requiring the player to lift it over their head.
        if (sShutterLiftArmed) {
            door->dyna.actor.velocity.y = 1.0f;
            sShutterFullyOpen = true;
            sShutterLiftArmed = false;
            VR_TriggerHaptic(VR_HAND_LEFT, 0.75f, 0.0f, 45.0f);
            VR_TriggerHaptic(VR_HAND_RIGHT, 0.75f, 0.0f, 45.0f);
            sShutterDoor = nullptr;
            sShutterLatched[0] = sShutterLatched[1] = false;
            return 2;
        }
        // Keep ownership while an abandoned preload finishes; the rollback above resolves it on
        // the next tick. Otherwise another nearby shutter could inherit the wrong room request.
        if (!sShutterLatched[0] && !sShutterLatched[1] && !sShutterRoomRequested) sShutterLatchDoor = nullptr;
        sShutterDoor = nullptr;
        return (sShutterLatched[0] || sShutterLatched[1]) ? 1 : 0;
    }

    const float meanY = (handY[0] + handY[1]) * 0.5f;
    // Capture the physical lift origin immediately, including while the destination room is still
    // streaming. Previously sShutterDoor was deliberately cleared during loading, which discarded
    // all hand travel made in that interval and made some shutters appear to stop partway open.
    if (sShutterDoor != door) {
        sShutterDoor = door;
        sShutterStartMeanY = meanY;
        sShutterStartHeight = door->dyna.actor.world.pos.y;
        sShutterEntrySide = playerAcross >= 0.0f ? 1.0f : -1.0f;
        sShutterLiftArmed = false;
    }


    // Never reveal an unloaded doorway. If another engine transition temporarily owns the room
    // context, retain the two-hand grip and retry rather than allowing the slab to expose a void.
    if (needsAdjacentRoom && (!sShutterRoomRequested || sShutterStreamDoor != door)) return 1;

    // Keep both hands attached while the destination loads, but do not begin moving the slab yet.
    // Once ready, all hand travel since the initial two-hand grip is applied in one safe step.
    if (sShutterRoomRequested && sShutterStreamDoor == door && play->roomCtx.status != 0) {
        return 1;
    }

    // Hand travel is amplified 3x below so Link can raise a full-height slab comfortably. Match
    // the release threshold to 30 cm of resulting door travel, not 30 cm of controller travel.
    if (meanY - sShutterStartMeanY >= WorldScale() * kShutterReleaseLiftMeters / 3.0f) {
        sShutterLiftArmed = true;
    }

    // The slab is taller than Link, so a comfortable full overhead lift must move the complete
    // door rather than demanding several metres of real hand travel.
    const float raisedHeight = ShutterRaisedHeight(door);
    const float target = fmaxf(door->dyna.actor.home.pos.y,
                               fminf(door->dyna.actor.home.pos.y + raisedHeight,
                                     sShutterStartHeight + (meanY - sShutterStartMeanY) * 3.0f));
    if (target > door->dyna.actor.world.pos.y &&
        door->dyna.actor.world.pos.y <= door->dyna.actor.home.pos.y + 1.0f) {
        Audio_PlayActorSound2(&door->dyna.actor, NA_SE_EV_SLIDE_DOOR_OPEN);
    }
    door->dyna.actor.world.pos.y = target;

    if (target >= door->dyna.actor.home.pos.y + raisedHeight - 5.0f) {
        door->dyna.actor.world.pos.y = door->dyna.actor.home.pos.y + raisedHeight;
        door->dyna.actor.velocity.y = 1.0f;
        sShutterFullyOpen = true;
        sShutterLiftArmed = false;
        VR_TriggerHaptic(VR_HAND_LEFT, 0.75f, 0.0f, 45.0f);
        VR_TriggerHaptic(VR_HAND_RIGHT, 0.75f, 0.0f, 45.0f);
        sShutterDoor = nullptr;
        sShutterLatched[0] = sShutterLatched[1] = false;
        return 2;
    }
    return 1;
}

extern "C" bool VrShutter_GripConsumed(int32_t hand, uint16_t mask) {
    const bool hovering = hand >= 0 && hand < 2 && gPlayState != nullptr &&
                          sShutterHandState[hand].frame == gPlayState->state.frames &&
                          sShutterHandState[hand].active;
    return (mask & VR_BTN_GRIP) && hand >= 0 && hand < 2 &&
           (sShutterLatched[hand] || hovering || sShutterDoor != nullptr);
}

extern "C" bool VrShutter_HoldsHand(int32_t hand) {
    // Mesh selection and pinning consume this same resolved state; neither performs another hit test.
    return hand >= 0 && hand < 2 && gPlayState != nullptr && !sShutterFullyOpen &&
           sShutterHandState[hand].frame == gPlayState->state.frames && sShutterHandState[hand].active;
}

extern "C" bool VrShutter_KeepOpen(DoorShutter* door, PlayState* play) {
    if (door == nullptr || play == nullptr || door != sShutterLatchDoor || !sShutterFullyOpen) return false;
    Player* player = GET_PLAYER(play);
    const float sn = Math_SinS(door->dyna.actor.shape.rot.y);
    const float cs = Math_CosS(door->dyna.actor.shape.rot.y);
    const float dx = player->actor.world.pos.x - door->dyna.actor.world.pos.x;
    const float dz = player->actor.world.pos.z - door->dyna.actor.world.pos.z;
    const float side = (dx * sn + dz * cs) >= 0.0f ? 1.0f : -1.0f;
    if (side == sShutterEntrySide) {
        if (door->dyna.actor.xzDistToPlayer < 140.0f) return true;
        // The player opened the shutter but walked away on the side they started from. Let the
        // native action close it first; only the closed-position callback may unload the visible
        // destination room and its retained object bank.
        sShutterCloseRollbackPending = true;
        sShutterCandidate = nullptr;
        sShutterLiftArmed = false;
        sShutterHandState[0] = {};
        sShutterHandState[1] = {};
        sShutterLatched[0] = sShutterLatched[1] = false;
        return false;
    }
    if (sShutterCloseForNextDoor) {
        sShutterCloseRollbackPending = true;
        sShutterCandidate = nullptr;
        sShutterLiftArmed = false;
        sShutterHandState[0] = {};
        sShutterHandState[1] = {};
        sShutterLatched[0] = sShutterLatched[1] = false;
        return false;
    }
    if (door->dyna.actor.xzDistToPlayer < 70.0f) return true;
    // Close the slab before releasing either room. FinishClose keeps whichever side Link actually
    // occupies then, so turning around during the close can never expose an unloaded void.
    sShutterCloseRollbackPending = true;
    sShutterCandidate = nullptr;
    sShutterLiftArmed = false;
    sShutterHandState[0] = {};
    sShutterHandState[1] = {};
    sShutterLatched[0] = sShutterLatched[1] = false;
    return false;
}

extern "C" void VrShutter_FinishClose(DoorShutter* door, PlayState* play) {
    if (door == nullptr || play == nullptr || door != sShutterLatchDoor || !sShutterCloseRollbackPending) return;
    if (sShutterRoomRequested && play->roomCtx.status == 0 && play->roomCtx.prevRoom.num >= 0) {
        Player* player = GET_PLAYER(play);
        const float sn = Math_SinS(door->dyna.actor.shape.rot.y);
        const float cs = Math_CosS(door->dyna.actor.shape.rot.y);
        const float dx = player->actor.world.pos.x - door->dyna.actor.world.pos.x;
        const float dz = player->actor.world.pos.z - door->dyna.actor.world.pos.z;
        const float side = (dx * sn + dz * cs) >= 0.0f ? 1.0f : -1.0f;
        FinishDoorwayTransition(sShutterTransition, play, side, false);
    }
    door->dyna.actor.room = play->roomCtx.curRoom.num;
    sShutterJustClosedDoor = door;
    sShutterLatchDoor = nullptr;
    sShutterCandidate = nullptr;
    sShutterFullyOpen = false;
    sShutterLiftArmed = false;
    sShutterRoomRequested = false;
    sShutterStreamDoor = nullptr;
    sShutterCloseRollbackPending = false;
    sShutterCloseForNextDoor = false;
}

extern "C" bool VrShutter_ConsumePhysicalClose(DoorShutter* door) {
    if (door == nullptr || door != sShutterJustClosedDoor) return false;
    sShutterJustClosedDoor = nullptr;
    return true;
}

extern "C" float VrShutter_OpenHeight(DoorShutter* door) {
    return door != nullptr && door == sShutterLatchDoor && sShutterFullyOpen ? ShutterRaisedHeight(door) : 200.0f;
}

extern "C" bool VrShutter_PinnedHandMatrix(Player* player, int32_t hand, float* mf16) {
    if (player == nullptr || mf16 == nullptr || hand < 0 || hand > 1 ||
        gPlayState == nullptr || player != GET_PLAYER(gPlayState) || sShutterFullyOpen) return false;
    const ShutterHandState& state = sShutterHandState[hand];
    if (!state.active || state.frame != gPlayState->state.frames || state.door == nullptr) return false;
    DoorShutter* door = state.door;
    const float sn = Math_SinS(door->dyna.actor.shape.rot.y);
    const float cs = Math_CosS(door->dyna.actor.shape.rot.y);
    const auto placeOpenPalm = [&](float localX, float localY, float orientationSide, float surfaceSide,
                                   float faceOffset) {
        const float sx = sqrtf(mf16[0] * mf16[0] + mf16[1] * mf16[1] + mf16[2] * mf16[2]);
        const float sy = sqrtf(mf16[4] * mf16[4] + mf16[5] * mf16[5] + mf16[6] * mf16[6]);
        const float sz = sqrtf(mf16[8] * mf16[8] + mf16[9] * mf16[9] + mf16[10] * mf16[10]);
        // The open-hand display lists use local Y from wrist to fingertips, so map Y to world-up.
        // This proper half-turn seats the controller-native hand mesh on the selected face.
        const float palmTurn = hand == VR_HAND_LEFT ? 1.0f : -1.0f;
        mf16[0] = orientationSide * palmTurn * cs * sx;
        mf16[1] = 0.0f;
        mf16[2] = -orientationSide * palmTurn * sn * sx;
        mf16[4] = 0.0f;
        mf16[5] = sy;
        mf16[6] = 0.0f;
        mf16[8] = orientationSide * palmTurn * sn * sz;
        mf16[9] = 0.0f;
        mf16[10] = orientationSide * palmTurn * cs * sz;

        const float halfLength = (VrHand_ChildSized() ? 548.0f : 862.0f) * 0.5f;
        const float face = surfaceSide * (kShutterSurface + faceOffset);
        mf16[12] = door->dyna.actor.world.pos.x + localX * cs + face * sn;
        mf16[13] = door->dyna.actor.world.pos.y + localY - sy * halfLength;
        mf16[14] = door->dyna.actor.world.pos.z - localX * sn + face * cs;
    };
    // Hover detection already resolved the exact door, point and faces during the actor update.
    // Reusing that result guarantees the open mesh and the pinned pose cannot disagree.
    const float faceOffset = state.latched ? 0.0f : 1.0f;
    const float orientationSide = state.latched ? state.surfaceFace : state.visualFace;
    placeOpenPalm(state.localX, state.localY, orientationSide, state.surfaceFace, faceOffset);
    return true;
}

extern "C" bool VrDoor_Tick(EnDoor* door, PlayState* play) {
    Player* player = GET_PLAYER(play);
    sTickFrame = play->state.frames;
    // EnDoor_Tick runs once per door actor. Sample edge state only once per game frame so whichever
    // door happens to update first cannot consume the fresh press before the nearby door sees it.
    if (sInputFrame != sTickFrame) {
        sInputFrame = sTickFrame;
        sCandidate = nullptr;
        for (int h = 0; h < 2; ++h) {
            sGripNow[h] = (VR_GetControllerButton(h) & VR_BTN_GRIP) != 0;
            sGripFresh[h] = sGripNow[h] && !sGripPrev[h];
            sGripPrev[h] = sGripNow[h];
        }
    }

    if (!Eligible(door, play, player)) {
        if (sDoor == door) {
            // Do not discard ownership while a synchronous room request is between request and
            // completion. Once ready, resolve from Link's real side before leaving physical mode.
            if (sRoomRequested && play->roomCtx.status != 0) return true;
            ResolveLoadedRooms(door, play, player);
            Clear();
        }
        if (sCandidate == door) sCandidate = nullptr;
        return false;
    }
    if (sDoor != nullptr && sDoor != door) return false;
    if (sDoor == nullptr && door->actor.xzDistToPlayer >= 70.0f && EyeDistanceXZ(door, play) >= 70.0f) return false;

    const float side = sDoor == door ? sSide : PlayerSide(door, player);
    if (sDoor == nullptr) {
        const int type = (door->actor.params >> 7) & 7;
        // At arm's length the VR eye can see through the small clearance between the leaf and its
        // frame. Preload the adjoining room as soon as a room-loading door enters interaction range
        // so that clearance reveals real geometry instead of the void. Keep scene-exit doors on the
        // stock transition path, and do not disturb a room transition already owned by the engine.
        if (type != DOOR_SCENEEXIT && play->roomCtx.status == 0 && play->roomCtx.prevRoom.num < 0) {
            sSide = side;
            sDisplaySide = side;
            RequestRoom(door, play, player);
            if (sRoomRequested) {
                sDoor = door;
            }
        }
    }
    if (sDoor == nullptr) sCandidate = door;
    for (int h = 0; h < 2; ++h) {
        float bestDistance = 1e30f;
        for (int faceIndex = 0; faceIndex < 2; ++faceIndex) {
            const float face = faceIndex == 0 ? side : -side;
            // Front and back knobs share the same physical spindle and therefore the same leaf edge.
            // The actor's local +X edge is the handle edge. A paired leaf is already rotated 180°
            // when spawned, so this remains correct for both halves and from either room.
            const float handleSide = 1.0f;
            float handle[3], distance;
            HandlePoint(door, handleSide, face, handle);
            if (HandMarkerDistance(h, handle, &distance) && distance < bestDistance) {
                bestDistance = distance;
                sNearFace[h] = face;
                sNearHandleSide[h] = handleSide;
            }
        }
        sNear[h] = bestDistance / WorldScale() * 100.0f <= CVarGetFloat("gVrDoorGrabRadius", 18.0f);
    }

    if (sDoor == nullptr) {
        for (int h = 0; h < 2; ++h) {
            float handPos[3], handQuat[4];
            if (sGripFresh[h] && sNear[h] && HandPose(h, handPos, handQuat)) {
                Grab(door, h, side, sNearFace[h], handQuat, false);
                break;
            }
        }
        return sDoor == door;
    }

    // Keep the display face stable while Link is within the doorway plane; update it only after his
    // body is clearly on one side. The headset may cross this plane repeatedly at arm's length.
    {
        float right[3], normal[3]; Axes(door, right, normal);
        const float dx = player->actor.world.pos.x - door->actor.world.pos.x;
        const float dz = player->actor.world.pos.z - door->actor.world.pos.z;
        const float across = dx * normal[0] + dz * normal[2];
        if (fabsf(across) > 8.0f) sDisplaySide = across >= 0.0f ? 1.0f : -1.0f;
    }

    // Crossing is a geometric fact, not a side effect of the streaming helper. Detect it on every
    // tick once the adjacent room is ready so automatic closing cannot be missed.
    const int doorType = (door->actor.params >> 7) & 7;
    if (doorType == DOOR_SCENEEXIT && sHand < 0 &&
        fabsf(sAngle * (360.0f / 65536.0f)) >= kExitDeg && PlayerSide(door, player) != sSide) {
        if (PlayerInsideDoorway(door, player, 28.0f)) {
            // The separate destination scene cannot coexist with this one. Hand the transition to
            // the original entrance/exit logic only after Link has physically crossed the opening.
            Vec3f checkPos = player->actor.world.pos;
            checkPos.y = door->actor.world.pos.y + 10.0f;
            CollisionPoly* groundPoly = nullptr;
            BgCheck_EntityRaycastFloor1(&play->colCtx, &groundPoly, &checkPos);
            if (groundPoly != nullptr && Player_HandleExitsAndVoids(play, player, groundPoly, BGCHECK_SCENE)) {
                gSaveContext.entranceSpeed = 2.0f;
                gSaveContext.entranceSound = NA_SE_OC_DOOR_OPEN;
                door->jointTable[3].z = 0;
                Clear();
                return false;
            }
        }
    }
    if (sRoomRequested && play->roomCtx.status == 0 && PlayerSide(door, player) != sSide) {
        if (PlayerInsideDoorway(door, player, 18.0f)) {
            sPassageCommitted = true;
        }
    }

    if (sHand < 0) {
        // An opened physical door stays where the player left it. This both matches a real hinged
        // door and leaves the doorway usable; it may be grabbed again at its moved handle.
        for (int h = 0; h < 2; ++h) {
            float handPos[3], handQuat[4];
            if (sGripFresh[h] && sNear[h] && HandPose(h, handPos, handQuat)) {
                Grab(door, h, sSide, sNearFace[h], handQuat, true);
                break;
            }
        }
        if (sHand < 0 && fabsf(sAngle * (360.0f / 65536.0f)) >= kWalkThroughDeg) {
            AssistWalkThrough(door, play, player);
        }
        if (sHand < 0) {
            float right[3], normal[3]; Axes(door, right, normal);
            const float dx = player->actor.world.pos.x - door->actor.world.pos.x;
            const float dz = player->actor.world.pos.z - door->actor.world.pos.z;
            const float across = fabsf(dx * normal[0] + dz * normal[2]);
            const bool clearedAfterPassage = sPassageCommitted && across > 20.0f;
            const bool abandonedOnOriginalSide = !sPassageCommitted && door->actor.xzDistToPlayer > 120.0f &&
                                                   EyeDistanceXZ(door, play) > 120.0f;
            // Close after a completed passage, or when Link changes his mind and walks away on the
            // original side. ResolveLoadedRooms keeps whichever side he actually occupies.
            if (clearedAfterPassage || abandonedOnOriginalSide) {
                constexpr int kAutoCloseStep = 0x300;
                // A proximity preload can own sDoor while the leaf is still closed. Resolving that
                // unused preload is not a door action and must remain silent.
                if (!sCloseSoundPlayed && (sOpenSoundPlayed || sAngle != 0)) {
                    Audio_PlayActorSound2(&door->actor,
                        (play->sceneNum == SCENE_SHADOW_TEMPLE || play->sceneNum == SCENE_BOTTOM_OF_THE_WELL ||
                         play->sceneNum == SCENE_FIRE_TEMPLE) ? NA_SE_EV_IRON_DOOR_CLOSE : NA_SE_EV_DOOR_CLOSE);
                    sCloseSoundPlayed = true;
                }
                if (abs(sAngle) <= kAutoCloseStep) {
                    sAngle = 0;
                    door->jointTable[3].z = 0;
                    ResolveLoadedRooms(door, play, player);
                    door->actor.room = play->roomCtx.curRoom.num;
                    if (door->actor.child != nullptr) door->actor.child->room = play->roomCtx.curRoom.num;
                    if (door->actor.parent != nullptr) door->actor.parent->room = play->roomCtx.curRoom.num;
                    Clear();
                    return false;
                }
                sAngle += sAngle > 0 ? -kAutoCloseStep : kAutoCloseStep;
                door->jointTable[3].z = sAngle;
            }
        }
        return true;
    }

    if (!sGripNow[sHand]) {
        sHand = -1;
        sPrevPlaneDistance = 1e30f;
        if (abs(sAngle) < 0x180) {
            door->jointTable[3].z = 0;
            if (sOpenSoundPlayed) {
                Audio_PlayActorSound2(&door->actor,
                    (play->sceneNum == SCENE_SHADOW_TEMPLE || play->sceneNum == SCENE_BOTTOM_OF_THE_WELL ||
                     play->sceneNum == SCENE_FIRE_TEMPLE) ? NA_SE_EV_IRON_DOOR_CLOSE : NA_SE_EV_DOOR_CLOSE);
            }
            // Keep both rooms and both sets of actors alive after passage. Closing the physical
            // door is the commit point that releases the room behind the player.
            ResolveLoadedRooms(door, play, player);
            if (sRoomRequested) {
                door->actor.room = play->roomCtx.curRoom.num;
                if (door->actor.child != nullptr) door->actor.child->room = play->roomCtx.curRoom.num;
                if (door->actor.parent != nullptr) door->actor.parent->room = play->roomCtx.curRoom.num;
            }
            Clear();
            return false;
        }
        door->jointTable[3].z = sAngle;
        return true;
    }

    if (!sUnlatched) {
        if (HandTurnDeg(sHand) >= kTurnDeg) {
            sUnlatched = true;
            sPullStartRad = HandPolar(door, sHand, sHandleSide);
            sPullStartDeg = sAngle * (360.0f / 65536.0f);
            sPullRefValid = true;
            VR_TriggerHaptic(sHand, 0.75f, 0.0f, 45.0f);
        }
        return true;
    }

    if (sPullRefValid) {
        float delta = HandPolar(door, sHand, sHandleSide) - sPullStartRad;
        while (delta > M_PI) delta -= 2.0f * M_PI;
        while (delta < -M_PI) delta += 2.0f * M_PI;
        // The skeleton's hinge channel turns opposite the controller's polar motion. The gain also
        // maps OoT's oversized 60-unit leaf to a comfortable real-world arm arc.
        float deg = sPullStartDeg - delta * (180.0f / M_PI) * CVarGetFloat("gVrDoorSwingGain", 1.75f);
        deg = fmaxf(-kMaxOpenDeg, fminf(kMaxOpenDeg, deg));
        sAngle = (s16)(deg * (65536.0f / 360.0f));
        door->jointTable[3].z = sAngle;

        const float openDeg = fabsf(deg);
        const int type = (door->actor.params >> 7) & 7;
        if (!sOpenSoundPlayed && openDeg >= 3.0f) {
            Audio_PlayActorSound2(&door->actor,
                (play->sceneNum == SCENE_SHADOW_TEMPLE || play->sceneNum == SCENE_BOTTOM_OF_THE_WELL ||
                 play->sceneNum == SCENE_FIRE_TEMPLE) ? NA_SE_EV_IRON_DOOR_OPEN : NA_SE_OC_DOOR_OPEN);
            sOpenSoundPlayed = true;
        }
        // Stream the destination as soon as a normal physical door visibly begins to swing. The
        // engine retains the old room as prevRoom, so both geometries and actor sets coexist until
        // the automatic close commits the transition.
        if (type != DOOR_SCENEEXIT && !sRoomRequested && openDeg >= 3.0f) {
            RequestRoom(door, play, player);
        }
        // Scene exits stay physically open here. Their stock entrance transition is requested only
        // after Link crosses the doorway plane (above), never merely because the leaf opened.
    }

    return true;
}

extern "C" bool VrDoor_GripConsumed(int32_t hand, uint16_t mask) {
    if (!(mask & VR_BTN_GRIP) || hand < 0 || hand > 1 || gPlayState == nullptr) return false;
    if ((uint32_t)gPlayState->state.frames - sTickFrame > 2) return false;
    return (sDoor != nullptr && sHand == hand) || sNear[hand];
}

extern "C" bool VrDoor_KeepPreviousRoomActors(void) {
    const bool ordinaryDoorStreaming = sDoor != nullptr && sRoomRequested;
    const bool shutterStreaming = (sShutterCandidate != nullptr || sShutterLatchDoor != nullptr) && sShutterRoomRequested;
    return (ordinaryDoorStreaming || shutterStreaming) && gPlayState != nullptr &&
           CVarGetInteger("gVrPhysicalDoors", 1) && VR_IsInitialized() && !VR_IsFlatScreen();
}

extern "C" bool VrDoor_IsPhysicalRoomRequest(void) {
    return sPhysicalRoomRequest && CVarGetInteger("gVrPhysicalDoors", 1) && VR_IsInitialized() &&
           !VR_IsFlatScreen();
}

extern "C" int32_t VrDoor_EnvironmentBehaviorType2(PlayState* play) {
    if (play != nullptr && sRoomRequested && !sPassageCommitted && play->roomCtx.prevRoom.num >= 0 &&
        CVarGetInteger("gVrPhysicalDoors", 1) && VR_IsInitialized() && !VR_IsFlatScreen()) {
        // Room_Request makes the preloaded destination curRoom immediately. Until Link actually
        // crosses the doorway, environmental effects must still come from the room he occupies.
        return play->roomCtx.prevRoom.behaviorType2;
    }
    if (play != nullptr && sShutterRoomRequested && !sShutterFullyOpen && play->roomCtx.prevRoom.num >= 0 &&
        CVarGetInteger("gVrPhysicalDoors", 1) && VR_IsInitialized() && !VR_IsFlatScreen()) {
        return play->roomCtx.prevRoom.behaviorType2;
    }
    return play != nullptr ? play->roomCtx.curRoom.behaviorType2 : 0;
}

extern "C" bool VrDoor_IsPhysicalActive(EnDoor* door) {
    if (door == nullptr || sDoor == nullptr || !CVarGetInteger("gVrPhysicalDoors", 1) ||
        !VR_IsInitialized() || VR_IsFlatScreen()) {
        return false;
    }
    return door == sDoor || sDoor->actor.child == &door->actor || sDoor->actor.parent == &door->actor;
}

extern "C" bool VrDoor_StableFaceView(EnDoor* door, float* outPos3) {
    if (door == nullptr || outPos3 == nullptr || sDoor != door) return false;
    float right[3], normal[3]; Axes(door, right, normal);
    outPos3[0] = door->actor.world.pos.x + normal[0] * sDisplaySide * 100.0f;
    outPos3[1] = door->actor.world.pos.y + 40.0f;
    outPos3[2] = door->actor.world.pos.z + normal[2] * sDisplaySide * 100.0f;
    return true;
}

extern "C" bool VrDoor_AllowsWallPass(Actor* actor, const float* nextPos3, const float* prevPos3) {
    if (actor == nullptr || nextPos3 == nullptr || prevPos3 == nullptr || gPlayState == nullptr ||
        actor != &GET_PLAYER(gPlayState)->actor) {
        return false;
    }
    if (sShutterLatchDoor != nullptr && sShutterFullyOpen && sShutterRoomRequested && gPlayState->roomCtx.status == 0) {
        const float sn = Math_SinS(sShutterLatchDoor->dyna.actor.shape.rot.y);
        const float cs = Math_CosS(sShutterLatchDoor->dyna.actor.shape.rot.y);
        const float* points[2] = { nextPos3, prevPos3 };
        for (int i = 0; i < 2; ++i) {
            const float dx = points[i][0] - sShutterLatchDoor->dyna.actor.world.pos.x;
            const float dz = points[i][2] - sShutterLatchDoor->dyna.actor.world.pos.z;
            const float lateral = dx * cs - dz * sn;
            const float across = dx * sn + dz * cs;
            // Cover the full boss-door opening and a complete movement sweep, while remaining
            // local enough that neighboring walls retain their collision.
            if (fabsf(lateral) < 65.0f && fabsf(across) < 100.0f) return true;
        }
    }
    if (sDoor == nullptr || sHand >= 0) return false;
    const int type = (sDoor->actor.params >> 7) & 7;
    const float requiredOpen = type == DOOR_SCENEEXIT ? kExitDeg : kWalkThroughDeg;
    if ((type != DOOR_SCENEEXIT && (!sRoomRequested || gPlayState->roomCtx.status != 0)) ||
        fabsf(sAngle * (360.0f / 65536.0f)) < requiredOpen) return false;
    float right[3], normal[3]; Axes(sDoor, right, normal);
    const float* points[2] = { nextPos3, prevPos3 };
    for (int i = 0; i < 2; ++i) {
        const float dx = points[i][0] - sDoor->actor.world.pos.x;
        const float dz = points[i][2] - sDoor->actor.world.pos.z;
        const float lateral = dx * right[0] + dz * right[2];
        const float across = dx * normal[0] + dz * normal[2];
        const float y = points[i][1] - sDoor->actor.world.pos.y;
        if (fabsf(lateral) < 24.0f && fabsf(across) < 40.0f && y > -8.0f && y < 80.0f) {
            return true;
        }
    }
    return false;
}

extern "C" void VrDoor_ConstrainPlayerToLeaf(Actor* actor) {
    if (actor == nullptr || sDoor == nullptr || gPlayState == nullptr ||
        actor != &GET_PLAYER(gPlayState)->actor || fabsf(sAngle * (360.0f / 65536.0f)) < 3.0f) {
        return;
    }
    const float relY = actor->world.pos.y - sDoor->actor.world.pos.y;
    if (relY < -8.0f || relY > 80.0f) return;

    float right[3], normal[3]; Axes(sDoor, right, normal);
    const float handleX = CVarGetFloat("gVrDoorHandleRight", 23.0f);
    const float hingeOffset = HingeOffset();
    const float hingeLocal = handleX >= 0.0f ? -hingeOffset : hingeOffset;
    const float leafDir = handleX >= 0.0f ? 1.0f : -1.0f;
    const float a = sAngle * (2.0f * M_PI / 65536.0f);
    const float hx = sDoor->actor.world.pos.x + right[0] * hingeLocal;
    const float hz = sDoor->actor.world.pos.z + right[2] * hingeLocal;
    const float ex = hx + right[0] * (leafDir * 2.0f * hingeOffset * cosf(a)) +
                     normal[0] * (-leafDir * 2.0f * hingeOffset * sinf(a));
    const float ez = hz + right[2] * (leafDir * 2.0f * hingeOffset * cosf(a)) +
                     normal[2] * (-leafDir * 2.0f * hingeOffset * sinf(a));
    const float sx = ex - hx, sz = ez - hz;
    const float lengthSq = sx * sx + sz * sz;
    if (lengthSq < 0.001f) return;
    float t = ((actor->world.pos.x - hx) * sx + (actor->world.pos.z - hz) * sz) / lengthSq;
    t = fmaxf(0.0f, fminf(1.0f, t));
    const float cx = hx + sx * t, cz = hz + sz * t;
    float dx = actor->world.pos.x - cx, dz = actor->world.pos.z - cz;
    float distance = sqrtf(dx * dx + dz * dz);
    constexpr float kPlayerLeafRadius = 8.0f;
    if (distance >= kPlayerLeafRadius) return;
    if (distance < 0.001f) {
        dx = -sz;
        dz = sx;
        if ((actor->prevPos.x - cx) * dx + (actor->prevPos.z - cz) * dz < 0.0f) {
            dx = -dx; dz = -dz;
        }
        distance = sqrtf(dx * dx + dz * dz);
    }
    const float push = (kPlayerLeafRadius - distance) / distance;
    actor->world.pos.x += dx * push;
    actor->world.pos.z += dz * push;
}

extern "C" bool VrDoor_PinnedHandMatrix(Player* player, int32_t hand, float* mf16) {
    if (mf16 == nullptr || hand < 0 || hand > 1 || sDoor == nullptr || sHand != hand ||
        gPlayState == nullptr || player != GET_PLAYER(gPlayState) || !sGripNow[hand]) {
        return false;
    }
    float handle[3], right[3], normal[3];
    HandlePoint(sDoor, sHandleSide, sKnobFace, handle);
    Axes(sDoor, right, normal);
    const float a = sAngle * (2.0f * M_PI / 65536.0f);
    const float localX = sGrabOffsetLocal[0] * cosf(a) + sGrabOffsetLocal[2] * sinf(a);
    const float localZ = -sGrabOffsetLocal[0] * sinf(a) + sGrabOffsetLocal[2] * cosf(a);
    mf16[12] = handle[0] + right[0] * localX + normal[0] * localZ;
    mf16[13] = handle[1] + sGrabOffsetLocal[1];
    mf16[14] = handle[2] + right[2] * localX + normal[2] * localZ;
    return true;
}

// Two miniature Deku Nuts expose the independent knob zones on the front and back faces.
extern "C" void VrDoor_DrawHandleMarker(void) {
    if (gPlayState == nullptr || (sCandidate == nullptr && sDoor == nullptr) ||
        !CVarGetInteger("gVrPhysicalDoors", 1) || !CVarGetInteger("gVrDoorShowGrabMarkers", 1)) return;
    Player* player = GET_PLAYER(gPlayState);
    EnDoor* markerDoor = sDoor != nullptr ? sDoor : sCandidate;
    if (player == nullptr || markerDoor->actor.xzDistToPlayer >= 70.0f) return;
    const float side = sDoor != nullptr ? sSide : PlayerSide(markerDoor, player);

    OPEN_DISPS(gPlayState->state.gfxCtx);
    for (int faceIndex = 0; faceIndex < 2; ++faceIndex) {
        const float face = faceIndex == 0 ? side : -side;
        float anchor[3];
        HandlePoint(markerDoor, 1.0f, face, anchor);
        FrameInterpolation_RecordOpenChild((const void*)&sCandidate, faceIndex);
        Matrix_Translate(anchor[0], anchor[1], anchor[2], MTXMODE_NEW);
        Matrix_ReplaceRotation(&gPlayState->billboardMtxF);
        float scale = 0.0045f;
        if (sNear[0] || sNear[1]) scale *= 1.4f;
        Matrix_Scale(scale, scale, scale, MTXMODE_APPLY);
        POLY_OPA_DISP = Play_SetFog(gPlayState, POLY_OPA_DISP);
        POLY_OPA_DISP = Gfx_SetupDL_66(POLY_OPA_DISP);
        gSPSegment(POLY_OPA_DISP++, 0x08, (uintptr_t)SEGMENTED_TO_VIRTUAL(gDropDekuNutTex));
        gSPMatrix(POLY_OPA_DISP++, MATRIX_NEWMTX(gPlayState->state.gfxCtx), G_MTX_MODELVIEW | G_MTX_LOAD);
        gSPDisplayList(POLY_OPA_DISP++, (Gfx*)gItemDropDL);
        FrameInterpolation_RecordCloseChild();
    }
    CLOSE_DISPS(gPlayState->state.gfxCtx);
}

namespace {
void RegisterVrDoor() {
    COND_HOOK(OnPlayDrawEnd, true, VrDoor_DrawHandleMarker);
    COND_HOOK(OnSceneInit, true, [](int16_t) {
        ClearAllDoorState();
        VrDoor_ResetRetainedObjects();
    });
    COND_HOOK(OnActorDestroy, true, [](void* actor) { ForgetDestroyedDoor(static_cast<Actor*>(actor)); });
}
static RegisterShipInitFunc initVrDoor(RegisterVrDoor);
}
