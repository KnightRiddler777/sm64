#include <PR/ultratypes.h>

#include "engine/behavior_script.h"
#include "engine/math_util.h"
#include "engine/surface_collision.h"
#include "level_update.h"
#include "object_fields.h"
#include "object_helpers.h"
#include "object_list_processor.h"
#include "platform_displacement.h"
#include "types.h"
#include "mario.h"
#include "sm64.h"
#include "memory.h"

u16 D_8032FEC0 = 0;

u32 unused_8032FEC4[4] = { 0 };

struct Object *gMarioPlatform = NULL;

/**
 * Determine if Mario is standing on a platform object, meaning that he is
 * within 4 units of the floor. Set his referenced platform object accordingly.
 */
void update_mario_platform(void) {
    struct Surface *floor;
    UNUSED u8 filler[4];
    f32 marioX;
    f32 marioY;
    f32 marioZ;
    f32 floorHeight;
    u32 awayFromFloor;
    return;
    if (gMarioObject == NULL) {
        return;
    }

    //! If Mario moves onto a rotating platform in a PU, the find_floor call
    //  will detect the platform and he will end up receiving a large amount
    //  of displacement since he is considered to be far from the platform's
    //  axis of rotation.

    marioX = gMarioObject->oPosX;
    marioY = gMarioObject->oPosY;
    marioZ = gMarioObject->oPosZ;
    floorHeight = find_floor(marioX, marioY, marioZ, &floor);

    if (absf(marioY - floorHeight) < 4.0f) {
        awayFromFloor = 0;
    } else {
        awayFromFloor = 1;
    }

    switch (awayFromFloor) {
        case 1:
            gMarioPlatform = NULL;
            gMarioObject->platform = NULL;
            break;

        case 0:
            if (floor != NULL && floor->object != NULL) {
                gMarioPlatform = floor->object;
                gMarioObject->platform = floor->object;
            } else {
                gMarioPlatform = NULL;
                gMarioObject->platform = NULL;
            }
            break;
    }
}

/**
 * Get Mario's position and store it in x, y, and z.
 */
void get_mario_pos(f32 *x, f32 *y, f32 *z) {
    *x = gMarioStates[0].pos[0];
    *y = gMarioStates[0].pos[1];
    *z = gMarioStates[0].pos[2];
}

/**
 * Set Mario's position.
 */
void set_mario_pos(f32 x, f32 y, f32 z) {
    gMarioStates[0].pos[0] = x;
    gMarioStates[0].pos[1] = y;
    gMarioStates[0].pos[2] = z;
}

static struct PlatformDisplacementInfo sMarioDisplacementInfo;
extern s32 gGlobalTimer;

static void scale_vec3f(Vec3f dst, Vec3f src, Vec3f scale, u32 doInverted) {
	if (doInverted) {
		dst[0] = src[0] / scale[0];
		dst[1] = src[1] / scale[1];
		dst[2] = src[2] / scale[2];
	} else {
		dst[0] = src[0] * scale[0];
		dst[1] = src[1] * scale[1];
		dst[2] = src[2] * scale[2];
	}
}

extern void *bhvTTCTreadmill;

/**
 * Apply one frame of platform displacement to Mario or an object using the given
 * platform.
 */
void apply_platform_displacement(struct PlatformDisplacementInfo *displaceInfo, Vec3f pos, s16 *yaw, struct Object *platform) {
	Vec3f platformPos;
	Vec3f posDifference;
	Vec3f yawVec;
	Vec3f scaledPos;
	// Determine how much Mario turned on his own since last frame
	s16 yawDifference = *yaw - displaceInfo->prevYaw;

	// Avoid a crash if the platform unloaded its collision while stood on
	if (platform->header.gfx.throwMatrix == NULL) return;

	vec3f_copy(platformPos, (*(Mat4 *)platform->header.gfx.throwMatrix)[3]);

	// Determine how far Mario moved on his own since last frame
	vec3f_copy(posDifference, pos);
	vec3f_sub(posDifference, displaceInfo->prevPos);

	if ((platform == displaceInfo->prevPlatform) && (gGlobalTimer == displaceInfo->prevTimer + 1)) {
		// Transform from relative positions to world positions
		scale_vec3f(scaledPos, displaceInfo->prevTransformedPos, platform->header.gfx.scale, FALSE);
		linear_mtxf_mul_vec3f(*(Mat4 *)platform->header.gfx.throwMatrix, pos, scaledPos);

		// Add on how much Mario moved in the previous frame
		vec3f_add(pos, posDifference);

		// Calculate new yaw
		linear_mtxf_mul_vec3f(*(Mat4 *)platform->header.gfx.throwMatrix, yawVec, displaceInfo->prevTransformedYawVec);
		mtxf_mul_vec3f(gWorldToLocalGravRotationMtx, yawVec);
		*yaw = atan2s(yawVec[2], yawVec[0]) + yawDifference;
	} else {
		// First frame of standing on the platform, don't calculate a new position
		vec3f_sub(pos, platformPos);
	}

	// Apply velocity-based displacement for certain objects (like the TTC Treadmills)
	if ((platform->behavior == segmented_to_virtual(&bhvTTCTreadmill)) && (gGravityVector[1] > 0.9f)) {
	    pos[0] += platform->oVelX;
	    pos[1] += platform->oVelY;
	    pos[2] += platform->oVelZ;
	}

	// Transform from world positions to relative positions for use next frame
	linear_mtxf_transpose_mul_vec3f(*(Mat4 *)platform->header.gfx.throwMatrix, scaledPos, pos);
	scale_vec3f(displaceInfo->prevTransformedPos, scaledPos, platform->header.gfx.scale, TRUE);
	vec3f_add(pos, platformPos);

	// Update info for next frame
	// Update position
	vec3f_copy(displaceInfo->prevPos, pos);

	// Set yaw info
	vec3f_set(yawVec, sins(*yaw), 0, coss(*yaw));
	mtxf_mul_vec3f(gLocalToWorldGravRotationMtx, yawVec);
	linear_mtxf_transpose_mul_vec3f(*(Mat4 *)platform->header.gfx.throwMatrix, displaceInfo->prevTransformedYawVec, yawVec);
	displaceInfo->prevYaw = *yaw;

	// Update platform and timer
	displaceInfo->prevPlatform = platform;
	displaceInfo->prevTimer = gGlobalTimer;
}

/**
 * Apply platform displacement or inertia if required
 */
void apply_mario_platform_displacement(void) {
    struct Object *platform = gMarioPlatform;

    if (!(gTimeStopState & TIME_STOP_ACTIVE) && gMarioObject != NULL) {
	if (platform != NULL) {
            Vec3f tempPos;
            vec3f_copy(tempPos,gMarioState->pos);
            mtxf_mul_vec3f(gLocalToWorldGravTransformMtx,tempPos); 
	    apply_platform_displacement(&sMarioDisplacementInfo, tempPos, &gMarioState->faceAngle[1], platform);
            mtxf_mul_vec3f(gWorldToLocalGravTransformMtx,tempPos);
            vec3f_copy(gMarioState->pos,tempPos);
        }
    }
}

#ifndef VERSION_JP
/**
 * Set Mario's platform to NULL.
 */
void clear_mario_platform(void) {
    gMarioPlatform = NULL;
}
#endif
