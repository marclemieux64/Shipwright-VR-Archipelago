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
#include <cmath>

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
float sJabuLatchSide[2] = {};
float sJabuStartSeparation = 0.0f;

DoorShutter* sBossCandidate = nullptr;
DoorShutter* sBossDoor = nullptr;
DoorShutter* sBossLatchDoor = nullptr;
bool sBossNear[2] = { false, false };
bool sBossLatched[2] = { false, false };
bool sBossGripPrev[2] = { true, true };
float sBossLatchSide[2] = {};
float sBossGrabFace[2] = {};
float sBossGrabLocal[2][2] = {};
float sBossStartMeanY = 0.0f;
float sBossStartHeight = 0.0f;
float sBossEntrySide = 1.0f;
bool sBossRoomRequested = false;
bool sBossFullyOpen = false;
bool sBossLiftArmed = false;
constexpr float kBossDoorSurface = 12.0f;
constexpr float kBossDoorRaisedHeight = 240.0f;
constexpr float kBossDoorReleaseLiftMeters = 0.30f;

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
}

void ClearAllDoorState() {
    Clear();
    sCandidate = nullptr;
    sJabuDoor = sJabuLatchDoor = nullptr;
    sJabuNear[0] = sJabuNear[1] = false;
    sJabuLatched[0] = sJabuLatched[1] = false;
    sBossCandidate = sBossDoor = sBossLatchDoor = nullptr;
    sBossNear[0] = sBossNear[1] = false;
    sBossLatched[0] = sBossLatched[1] = false;
    sBossRoomRequested = sBossFullyOpen = sBossLiftArmed = false;
}

void ForgetDestroyedDoor(Actor* actor) {
    if (actor == nullptr) return;
    if (actor == reinterpret_cast<Actor*>(sDoor)) Clear();
    if (actor == reinterpret_cast<Actor*>(sCandidate)) sCandidate = nullptr;
    if (actor == reinterpret_cast<Actor*>(sJabuDoor)) sJabuDoor = nullptr;
    if (actor == reinterpret_cast<Actor*>(sJabuLatchDoor)) sJabuLatchDoor = nullptr;
    if (actor == reinterpret_cast<Actor*>(sBossCandidate)) sBossCandidate = nullptr;
    if (actor == reinterpret_cast<Actor*>(sBossDoor)) sBossDoor = nullptr;
    if (actor == reinterpret_cast<Actor*>(sBossLatchDoor)) {
        sBossLatchDoor = nullptr;
        sBossFullyOpen = sBossLiftArmed = false;
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
    const float lateral = dx * right[0] + dz * right[2];
    const float distance = fabsf(across);

    // Approaching the open centre starts streaming the adjacent room. Once it is ready, the local
    // wall-collision aperture below lets the player cross under their own locomotion.
    if (sPrevPlaneDistance < 1e20f && distance < sPrevPlaneDistance - 0.02f && distance < 36.0f &&
        fabsf(lateral) < 24.0f) {
        sPassagePending = true;
    }
    if (sPassagePending && !sRoomRequested) {
        RequestRoom(door, play, player);
    }
    if (sPassagePending && sRoomRequested && play->roomCtx.status == 0 && PlayerSide(door, player) != sSide) {
        sPassageCommitted = true;
    }
    sPrevPlaneDistance = distance;
}

void RequestRoom(EnDoor* door, PlayState* play, Player* player) {
    const int index = GET_TRANSITION_ACTOR_INDEX(&door->actor);
    const TransitionActorEntry* entry = &play->transiActorCtx.list[index];
    int front;
    if (entry->sides[0].room == play->roomCtx.curRoom.num) {
        front = entry->sides[1].room;
    } else if (entry->sides[1].room == play->roomCtx.curRoom.num) {
        front = entry->sides[0].room;
    } else {
        front = entry->sides[(sSide > 0.0f) ? 0 : 1].room;
    }
    if (front >= 0 && front != play->roomCtx.curRoom.num && Room_RequestNewRoom(play, &play->roomCtx, front)) {
        sRoomRequested = true;
        // Keep the physical leaf (and the other half of a paired door) independent of room culling
        // until the door is closed and the transition is committed.
        door->actor.room = -1;
        if (door->actor.child != nullptr) door->actor.child->room = -1;
        if (door->actor.parent != nullptr) door->actor.parent->room = -1;
    }
}

void ResolveLoadedRooms(EnDoor* door, PlayState* play, Player* player) {
    if (!sRoomRequested || play->roomCtx.status != 0) return;
    const bool enteredDestination = PlayerSide(door, player) != sSide;
    if (play->roomCtx.prevRoom.num >= 0) {
        // Room_Request makes the destination curRoom immediately. Decide which room survives from
        // the player's actual side now, never from a cached crossing flag that may miss a frame.
        if (PlayerSide(door, player) == sSide) {
            Room tempRoom = play->roomCtx.curRoom;
            play->roomCtx.curRoom = play->roomCtx.prevRoom;
            play->roomCtx.prevRoom = tempRoom;
        }
        Room_FinishRoomChange(play, &play->roomCtx);
    }
    VrDoor_ReleaseRetainedObjects(play, enteredDestination);
    if (enteredDestination) Play_SetupRespawnPoint(play, RESPAWN_MODE_DOWN, 0xDFF);
    door->actor.room = play->roomCtx.curRoom.num;
    if (door->actor.child != nullptr) door->actor.child->room = play->roomCtx.curRoom.num;
    if (door->actor.parent != nullptr) door->actor.parent->room = play->roomCtx.curRoom.num;
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
    if (sJabuLatchDoor != nullptr && sJabuLatchDoor != door) return false;

    float localX[2] = {};
    bool grip[2] = {};
    const float s = Math_SinS(door->dyna.actor.shape.rot.y);
    const float c = Math_CosS(door->dyna.actor.shape.rot.y);
    for (int hand = 0; hand < 2; ++hand) {
        float p[3];
        if (!HandPosition(hand, p)) {
            sJabuNear[hand] = false;
            continue;
        }
        const float dx = p[0] - door->dyna.actor.world.pos.x;
        const float dy = p[1] - (door->dyna.actor.world.pos.y + 80.0f);
        const float dz = p[2] - door->dyna.actor.world.pos.z;
        localX[hand] = dx * c - dz * s;
        const float localZ = dx * s + dz * c;
        sJabuNear[hand] = fabsf(localZ) <= 25.0f && fabsf(dy) <= 75.0f && fabsf(localX[hand]) <= 70.0f;
        grip[hand] = (VR_GetControllerButton(hand) & VR_BTN_GRIP) != 0;
        if (!grip[hand]) {
            sJabuLatched[hand] = false;
        } else if (!sJabuLatched[hand] && sJabuNear[hand] && fabsf(localX[hand]) >= 4.0f) {
            sJabuLatchDoor = door;
            sJabuLatched[hand] = true;
            sJabuLatchSide[hand] = localX[hand] >= 0.0f ? 1.0f : -1.0f;
            VR_TriggerHaptic(hand, 0.3f, 0.0f, 25.0f);
        }
    }

    const bool oppositeSides = sJabuLatchSide[0] * sJabuLatchSide[1] < 0.0f;
    const bool holding = sJabuLatched[0] && sJabuLatched[1] && oppositeSides && grip[0] && grip[1];
    const float separation = fabsf(localX[0] - localX[1]);
    if (!holding) {
        if (sJabuDoor == door) sJabuDoor = nullptr;
        if (!sJabuLatched[0] && !sJabuLatched[1]) sJabuLatchDoor = nullptr;
        return false;
    }
    if (sJabuDoor != door) {
        sJabuDoor = door;
        sJabuStartSeparation = separation;
        return false;
    }
    if (separation - sJabuStartSeparation >= 10.0f) {
        VR_TriggerHaptic(VR_HAND_LEFT, 0.75f, 0.0f, 45.0f);
        VR_TriggerHaptic(VR_HAND_RIGHT, 0.75f, 0.0f, 45.0f);
        sJabuDoor = nullptr;
        sJabuLatchDoor = nullptr;
        sJabuNear[0] = sJabuNear[1] = false;
        sJabuLatched[0] = sJabuLatched[1] = false;
        return true;
    }
    return false;
}

extern "C" bool VrJabuDoor_GripConsumed(int32_t hand, uint16_t mask) {
    return (mask & VR_BTN_GRIP) && hand >= 0 && hand < 2 &&
           (sJabuLatched[hand] || sJabuDoor != nullptr);
}

extern "C" int32_t VrBossDoor_Tick(DoorShutter* door, PlayState* play) {
    Player* player = play != nullptr ? GET_PLAYER(play) : nullptr;
    const bool eligible = door != nullptr && player != nullptr && door->doorType == SHUTTER_BOSS &&
                          door->unlockTimer == 0 && CVarGetInteger("gVrPhysicalDoors", 1) &&
                          VR_IsInitialized() && VR_GetFirstPerson() && !VR_IsFlatScreen() &&
                          CVarGetInteger("gVrMotionHands", 1) && play->csCtx.state == CS_STATE_IDLE &&
                          play->transitionTrigger == TRANS_TRIGGER_OFF;
    if (!eligible) {
        if (sBossCandidate == door) sBossCandidate = nullptr;
        if (sBossDoor == door) sBossDoor = nullptr;
        if (sBossLatchDoor == door && !sBossFullyOpen) sBossLatchDoor = nullptr;
        sBossNear[0] = sBossNear[1] = false;
        sBossLatched[0] = sBossLatched[1] = false;
        sBossLiftArmed = false;
        return false;
    }
    if (sBossLatchDoor != nullptr && sBossLatchDoor != door) return false;
    if (door->dyna.actor.xzDistToPlayer < 100.0f) sBossCandidate = door;

    // Walking away without touching the preloaded door restores the room the player still occupies.
    // This is a streaming operation, not a physical close, so it deliberately has no door sound.
    if (sBossRoomRequested && !sBossFullyOpen && !sBossLatched[0] && !sBossLatched[1] &&
        door->dyna.actor.xzDistToPlayer > 140.0f && play->roomCtx.status == 0) {
        if (play->roomCtx.prevRoom.num >= 0) {
            Room room = play->roomCtx.curRoom;
            play->roomCtx.curRoom = play->roomCtx.prevRoom;
            play->roomCtx.prevRoom = room;
            Room_FinishRoomChange(play, &play->roomCtx);
        }
        VrDoor_ReleaseRetainedObjects(play, false);
        door->dyna.actor.room = play->roomCtx.curRoom.num;
        sBossRoomRequested = false;
        sBossCandidate = nullptr;
        sBossLatchDoor = nullptr;
        sBossLiftArmed = false;
        return false;
    }

    const float sn = Math_SinS(door->dyna.actor.shape.rot.y);
    const float cs = Math_CosS(door->dyna.actor.shape.rot.y);
    const float pdx = player->actor.world.pos.x - door->dyna.actor.world.pos.x;
    const float pdz = player->actor.world.pos.z - door->dyna.actor.world.pos.z;
    const float playerAcross = pdx * sn + pdz * cs;

    // Prepare the boss room before lifting exposes the doorway. Proximity streaming is silent;
    // only an actual physical lift is allowed to play the sliding-door sound.
    if (!sBossRoomRequested && door->dyna.actor.xzDistToPlayer < 100.0f && play->roomCtx.status == 0 &&
        play->roomCtx.prevRoom.num < 0) {
        const int index = GET_TRANSITION_ACTOR_INDEX(&door->dyna.actor);
        const TransitionActorEntry* entry = &play->transiActorCtx.list[index];
        const int destination = entry->sides[0].room == play->roomCtx.curRoom.num ? entry->sides[1].room
                                                                                  : entry->sides[0].room;
        if (destination >= 0 && destination != play->roomCtx.curRoom.num &&
            Room_RequestNewRoom(play, &play->roomCtx, destination)) {
            sBossRoomRequested = true;
            sBossEntrySide = playerAcross >= 0.0f ? 1.0f : -1.0f;
            door->dyna.actor.room = -1;
        }
    }
    float handY[2] = {};
    bool grip[2] = {};
    bool fresh[2] = {};
    for (int hand = 0; hand < 2; ++hand) {
        float p[3];
        if (!HandPosition(hand, p)) {
            sBossNear[hand] = false;
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
                if (fabsf(lx) <= 70.0f && ly >= 20.0f && ly <= 125.0f && fabsf(lz) < best) {
                    best = fabsf(lz);
                    contactX = lx; contactY = ly; contactZ = lz;
                }
            }
        }
        handY[hand] = p[1];
        sBossNear[hand] = fabsf(contactX) <= 70.0f && fabsf(contactZ) <= 28.0f &&
                          contactY >= 20.0f && contactY <= 125.0f;
        grip[hand] = (VR_GetControllerButton(hand) & VR_BTN_GRIP) != 0;
        fresh[hand] = grip[hand] && !sBossGripPrev[hand];
        sBossGripPrev[hand] = grip[hand];
        if (!grip[hand]) {
            sBossLatched[hand] = false;
        } else if (fresh[hand] && !sBossLatched[hand] && sBossNear[hand]) {
            sBossLatchDoor = door;
            sBossLatched[hand] = true;
            sBossLatchSide[hand] = contactX >= 0.0f ? 1.0f : -1.0f;
            const float surface = playerAcross >= 0.0f ? kBossDoorSurface : -kBossDoorSurface;
            // As with climbing, snap the closest part of the rendered hand to the surface rather
            // than putting the controller origin there. This keeps fingertips/palm out of the slab.
            sBossGrabFace[hand] = localZ + (surface - contactZ);
            sBossGrabLocal[hand][0] = localX;
            sBossGrabLocal[hand][1] = localY;
            VR_TriggerHaptic(hand, 0.3f, 0.0f, 25.0f);
        }
    }

    // Any two points on the slab form a valid lift. Requiring opposite halves made the grip
    // needlessly dependent on the player's hand spacing and was unlike the climbing interaction.
    const bool holding = sBossLatched[0] && sBossLatched[1] && grip[0] && grip[1];
    if (!holding) {
        // A comfortable 30 cm two-handed lift commits the interaction. Releasing either grip
        // then hands control back to the native shutter action, which smoothly finishes raising
        // the full-height slab instead of requiring the player to lift it over their head.
        if (sBossLiftArmed) {
            door->dyna.actor.velocity.y = 1.0f;
            sBossFullyOpen = true;
            sBossLiftArmed = false;
            VR_TriggerHaptic(VR_HAND_LEFT, 0.75f, 0.0f, 45.0f);
            VR_TriggerHaptic(VR_HAND_RIGHT, 0.75f, 0.0f, 45.0f);
            sBossDoor = nullptr;
            sBossLatched[0] = sBossLatched[1] = false;
            return 2;
        }
        if (!sBossLatched[0] && !sBossLatched[1]) sBossLatchDoor = nullptr;
        sBossDoor = nullptr;
        return (sBossLatched[0] || sBossLatched[1]) ? 1 : 0;
    }
    const float meanY = (handY[0] + handY[1]) * 0.5f;
    if (sBossDoor != door) {
        sBossDoor = door;
        sBossStartMeanY = meanY;
        sBossStartHeight = door->dyna.actor.world.pos.y;
        sBossEntrySide = playerAcross >= 0.0f ? 1.0f : -1.0f;
        sBossLiftArmed = false;
        return 1;
    }

    if (meanY - sBossStartMeanY >= WorldScale() * kBossDoorReleaseLiftMeters) {
        sBossLiftArmed = true;
    }

    // The slab is taller than Link, so a comfortable full overhead lift must move the complete
    // door rather than demanding several metres of real hand travel.
    const float target = fmaxf(door->dyna.actor.home.pos.y,
                               fminf(door->dyna.actor.home.pos.y + kBossDoorRaisedHeight,
                                     sBossStartHeight + (meanY - sBossStartMeanY) * 3.0f));
    if (target > door->dyna.actor.world.pos.y && door->dyna.actor.world.pos.y <= door->dyna.actor.home.pos.y + 1.0f) {
        Audio_PlayActorSound2(&door->dyna.actor, NA_SE_EV_SLIDE_DOOR_OPEN);
    }
    door->dyna.actor.world.pos.y = target;

    if (target >= door->dyna.actor.home.pos.y + kBossDoorRaisedHeight - 5.0f) {
        door->dyna.actor.world.pos.y = door->dyna.actor.home.pos.y + kBossDoorRaisedHeight;
        door->dyna.actor.velocity.y = 1.0f;
        sBossFullyOpen = true;
        sBossLiftArmed = false;
        VR_TriggerHaptic(VR_HAND_LEFT, 0.75f, 0.0f, 45.0f);
        VR_TriggerHaptic(VR_HAND_RIGHT, 0.75f, 0.0f, 45.0f);
        sBossDoor = nullptr;
        sBossLatched[0] = sBossLatched[1] = false;
        return 2;
    }
    return 1;
}

extern "C" bool VrBossDoor_GripConsumed(int32_t hand, uint16_t mask) {
    return (mask & VR_BTN_GRIP) && hand >= 0 && hand < 2 &&
           (sBossLatched[hand] || sBossNear[hand] || sBossDoor != nullptr);
}

extern "C" bool VrBossDoor_HoldsHand(int32_t hand) {
    return hand >= 0 && hand < 2 && sBossLatched[hand] && !sBossFullyOpen;
}

extern "C" bool VrBossDoor_KeepOpen(DoorShutter* door, PlayState* play) {
    if (door == nullptr || play == nullptr || door != sBossLatchDoor || !sBossFullyOpen) return false;
    Player* player = GET_PLAYER(play);
    const float sn = Math_SinS(door->dyna.actor.shape.rot.y);
    const float cs = Math_CosS(door->dyna.actor.shape.rot.y);
    const float dx = player->actor.world.pos.x - door->dyna.actor.world.pos.x;
    const float dz = player->actor.world.pos.z - door->dyna.actor.world.pos.z;
    const float side = (dx * sn + dz * cs) >= 0.0f ? 1.0f : -1.0f;
    if (side == sBossEntrySide || door->dyna.actor.xzDistToPlayer < 70.0f) return true;
    if (sBossRoomRequested && play->roomCtx.status == 0 && play->roomCtx.prevRoom.num >= 0) {
        Room_FinishRoomChange(play, &play->roomCtx);
        VrDoor_ReleaseRetainedObjects(play, true);
        Play_SetupRespawnPoint(play, RESPAWN_MODE_DOWN, 0xDFF);
    }
    sBossLatchDoor = nullptr;
    sBossCandidate = nullptr;
    sBossFullyOpen = false;
    sBossLiftArmed = false;
    sBossRoomRequested = false;
    return false;
}

extern "C" float VrBossDoor_OpenHeight(DoorShutter* door) {
    return door != nullptr && door == sBossLatchDoor && sBossFullyOpen ? kBossDoorRaisedHeight : 200.0f;
}

extern "C" bool VrBossDoor_PinnedHandMatrix(Player* player, int32_t hand, float* mf16) {
    DoorShutter* door = sBossLatchDoor != nullptr ? sBossLatchDoor : sBossCandidate;
    if (door == nullptr || player == nullptr || mf16 == nullptr || hand < 0 || hand > 1 ||
        gPlayState == nullptr || player != GET_PLAYER(gPlayState) || sBossFullyOpen) return false;
    const float sn = Math_SinS(door->dyna.actor.shape.rot.y);
    const float cs = Math_CosS(door->dyna.actor.shape.rot.y);
    if (sBossLatched[hand]) {
        const float face = sBossGrabFace[hand];
        mf16[12] = door->dyna.actor.world.pos.x + sBossGrabLocal[hand][0] * cs + face * sn;
        mf16[13] = door->dyna.actor.world.pos.y + sBossGrabLocal[hand][1];
        mf16[14] = door->dyna.actor.world.pos.z - sBossGrabLocal[hand][0] * sn + face * cs;
        return true;
    }
    const float dx = mf16[12] - door->dyna.actor.world.pos.x;
    const float dz = mf16[14] - door->dyna.actor.world.pos.z;
    const float localX = dx * cs - dz * sn;
    const float localZ = dx * sn + dz * cs;
    const float localY = mf16[13] - door->dyna.actor.world.pos.y;
    const float pdx = player->actor.world.pos.x - door->dyna.actor.world.pos.x;
    const float pdz = player->actor.world.pos.z - door->dyna.actor.world.pos.z;
    const float playerSide = (pdx * sn + pdz * cs) >= 0.0f ? 1.0f : -1.0f;
    float contactX = localX, contactY = localY, contactZ = localZ;
    const float len = VrHand_ChildSized() ? 548.0f : 862.0f;
    float best = fabsf(localZ);
    for (int point = 1; point < 3; ++point) {
        const float along = len * 0.5f * point;
        const float px = mf16[12] + mf16[0] * along;
        const float py = mf16[13] + mf16[1] * along;
        const float pz = mf16[14] + mf16[2] * along;
        const float hdx = px - door->dyna.actor.world.pos.x;
        const float hdz = pz - door->dyna.actor.world.pos.z;
        const float lx = hdx * cs - hdz * sn;
        const float lz = hdx * sn + hdz * cs;
        const float ly = py - door->dyna.actor.world.pos.y;
        if (fabsf(lx) <= 72.0f && ly >= 0.0f && ly <= 150.0f && fabsf(lz) < best) {
            best = fabsf(lz); contactX = lx; contactY = ly; contactZ = lz;
        }
    }
    if (fabsf(contactX) > 72.0f || contactY < 0.0f || contactY > 150.0f ||
        contactZ * playerSide >= kBossDoorSurface) return false;
    const float correction = playerSide * kBossDoorSurface - contactZ;
    mf16[12] += correction * sn;
    mf16[14] += correction * cs;
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
        float right[3], normal[3]; Axes(door, right, normal);
        const float dx = player->actor.world.pos.x - door->actor.world.pos.x;
        const float dz = player->actor.world.pos.z - door->actor.world.pos.z;
        if (fabsf(dx * right[0] + dz * right[2]) < 28.0f) {
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
        float right[3], normal[3]; Axes(door, right, normal);
        const float dx = player->actor.world.pos.x - door->actor.world.pos.x;
        const float dz = player->actor.world.pos.z - door->actor.world.pos.z;
        if (fabsf(dx * right[0] + dz * right[2]) < 18.0f) {
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
    const bool bossDoorStreaming = (sBossCandidate != nullptr || sBossLatchDoor != nullptr) && sBossRoomRequested;
    return (ordinaryDoorStreaming || bossDoorStreaming) && gPlayState != nullptr &&
           CVarGetInteger("gVrPhysicalDoors", 1) && VR_IsInitialized() && !VR_IsFlatScreen();
}

extern "C" int32_t VrDoor_EnvironmentBehaviorType2(PlayState* play) {
    if (play != nullptr && sRoomRequested && !sPassageCommitted && play->roomCtx.prevRoom.num >= 0 &&
        CVarGetInteger("gVrPhysicalDoors", 1) && VR_IsInitialized() && !VR_IsFlatScreen()) {
        // Room_Request makes the preloaded destination curRoom immediately. Until Link actually
        // crosses the doorway, environmental effects must still come from the room he occupies.
        return play->roomCtx.prevRoom.behaviorType2;
    }
    if (play != nullptr && sBossRoomRequested && !sBossFullyOpen && play->roomCtx.prevRoom.num >= 0 &&
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
    if (sBossLatchDoor != nullptr && sBossFullyOpen && sBossRoomRequested && gPlayState->roomCtx.status == 0) {
        const float sn = Math_SinS(sBossLatchDoor->dyna.actor.shape.rot.y);
        const float cs = Math_CosS(sBossLatchDoor->dyna.actor.shape.rot.y);
        const float* points[2] = { nextPos3, prevPos3 };
        for (int i = 0; i < 2; ++i) {
            const float dx = points[i][0] - sBossLatchDoor->dyna.actor.world.pos.x;
            const float dz = points[i][2] - sBossLatchDoor->dyna.actor.world.pos.z;
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
