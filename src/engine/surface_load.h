#ifndef SURFACE_LOAD_H
#define SURFACE_LOAD_H

#include <PR/ultratypes.h>

#include "surface_collision.h"
#include "types.h"

#define NUM_CELLS       (2 * LEVEL_BOUNDARY_MAX / CELL_SIZE)
#define NUM_CELLS_INDEX (NUM_CELLS - 1)

struct SurfaceNode {
    struct SurfaceNode *next;
    struct Surface *surface;
};

enum {
    SPATIAL_PARTITION_FLOORS,
    SPATIAL_PARTITION_CEILS,
    SPATIAL_PARTITION_WALLS
};

typedef struct SurfaceNode SpatialPartitionCell[3];

// Needed for bs bss reordering memes.
extern s32 unused8038BE90;

extern struct SurfaceNode gStaticSurfaces[128];
extern struct SurfaceNode gDynamicSurfaces;

extern SpatialPartitionCell gStaticSurfacePartition;
extern struct SurfaceNode *sSurfaceNodePool;
extern struct Surface *sSurfacePool;
extern s16 sSurfacePoolSize;

s16 min_3(s16 a0, s16 a1, s16 a2);
s16 max_3(s16 a0, s16 a1, s16 a2);
s16 get_cell(s16 x, s16 y, s16 z);
void alloc_surface_pools(void);
#ifdef NO_SEGMENTED_MEMORY
u32 get_area_terrain_size(s16 *data);
#endif
void load_area_terrain(s16 index, s16 *data, s8 *surfaceRooms, s16 *macroObjects);
void clear_dynamic_and_transformed_surfaces(void);
void create_transformed_surfaces(Vec3f);
void load_object_collision_model(void);

#endif // SURFACE_LOAD_H
