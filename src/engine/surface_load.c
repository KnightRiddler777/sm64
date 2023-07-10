#include <PR/ultratypes.h>

#include "sm64.h"
#include "game/ingame_menu.h"
#include "graph_node.h"
#include "behavior_script.h"
#include "behavior_data.h"
#include "game/memory.h"
#include "game/object_helpers.h"
#include "game/macro_special_objects.h"
#include "surface_collision.h"
#include "game/mario.h"
#include "game/object_list_processor.h"
#include "surface_load.h"
#include "math_util.h"

s32 unused8038BE90;

struct SurfaceNode gStaticSurfaces[128];
struct SurfaceNode gDynamicSurfaces;

SpatialPartitionCell gStaticSurfacePartition;

/**
 * Pools of data to contain either surface nodes or surfaces.
 */
struct SurfaceNode *sSurfaceNodePool;
struct Surface *sSurfacePool;

/**
 * The size of the surface pool (2300).
 */
s16 sSurfacePoolSize;

u8 unused8038EEA8[0x30];

/**
 * Allocate the part of the surface node pool to contain a surface node.
 */
static struct SurfaceNode *alloc_surface_node(void) {
    struct SurfaceNode *node = &sSurfaceNodePool[gSurfaceNodesAllocated];
    gSurfaceNodesAllocated++;

    node->next = NULL;

    //! A bounds check! If there's more surface nodes than 7000 allowed,
    //  we, um...
    // Perhaps originally just debug feedback?
    if (gSurfaceNodesAllocated >= 7000) {
    }

    return node;
}

/**
 * Allocate the part of the surface pool to contain a surface and
 * initialize the surface.
 */
static struct Surface *alloc_surface(void) {

    struct Surface *surface = &sSurfacePool[gSurfacesAllocated];
    gSurfacesAllocated++;

    //! A bounds check! If there's more surfaces than the 2300 allowed,
    //  we, um...
    // Perhaps originally just debug feedback?
    if (gSurfacesAllocated >= sSurfacePoolSize) {
    }

    surface->type = 0;
    surface->force = 0;
    surface->flags = 0;
    surface->room = 0;
    surface->object = NULL;

    return surface;
}

/**
 * Clears the static (level) surface partitions for new use.
 */
static void clear_static_surfaces(void) {
    s32 i;
    for (i = 0; i < 128; i++) {
        gStaticSurfaces[i].next = NULL;
    }
}

/**
 * Returns the lowest of three values.
 */
s16 min_3(s16 a0, s16 a1, s16 a2) {
    if (a1 < a0) {
        a0 = a1;
    }

    if (a2 < a0) {
        a0 = a2;
    }

    return a0;
}

/**
 * Returns the highest of three values.
 */
s16 max_3(s16 a0, s16 a1, s16 a2) {
    if (a1 > a0) {
        a0 = a1;
    }

    if (a2 > a0) {
        a0 = a2;
    }

    return a0;
}

u32 gCellGridX = 8;
u32 gCellSizeX = 2048;
u32 gCellGridY = 1;
u32 gCellSizeY = 40000;
u32 gCellGridZ = 8;
u32 gCellSizeZ = 2048;

enum CellIndexOffsets {
    CELL_OFFSET_NONE,
    CELL_OFFSET_LOWER,
    CELL_OFFSET_UPPER
};

// Get the one-dimensional index of the cell array from the three-dimensional indices.
s16 cell_index_to_array(s16 x, s16 y, s16 z) {
    return x + gCellGridX * (z + gCellGridZ * y);
}

// Get the cell index of any dimension from its position.
s16 get_cell_index(s16 pos, u16 numCells, u32 cellSize, u32 offset) {
    s16 index;

    if (numCells == 1) return 0;

    if (offset == CELL_OFFSET_LOWER) {
        pos -= 20;
    } else if (offset == CELL_OFFSET_UPPER) {
        pos += 20;
    }

    if ((numCells % 2)) pos += (cellSize / 2);

    index = ((f32)pos / cellSize) + (numCells/2);

    if (index < 0) {
        index = 0;
    } else if (index >= numCells) {
        index = numCells - 1;
    }

    return index;
}

// Get the one-dimensional index of the cell array from a position in 3D space.
s16 get_cell(s16 x, s16 y, s16 z) {
    return cell_index_to_array(get_cell_index(x, gCellGridX, gCellSizeX, CELL_OFFSET_NONE),
                               get_cell_index(y, gCellGridY, gCellSizeY, CELL_OFFSET_NONE),
                               get_cell_index(z, gCellGridZ, gCellSizeZ, CELL_OFFSET_NONE));
}

// Add a surface to a specific cell.
static void add_surface_to_cell(s16 cellX, s16 cellY, s16 cellZ, struct Surface *surface) {
    struct SurfaceNode *newNode = alloc_surface_node();
    struct SurfaceNode *list;

    newNode->surface = surface;

    if (0) {
        list = &gDynamicSurfaces;
    } else {
        list = &gStaticSurfaces[cell_index_to_array(cellX, cellY, cellZ)];
    }

    newNode->next = list->next;
    list->next = newNode;
}

// Add a surface to all the appropriate cells it needs to be added to.
static void add_surface(struct Surface *surface, s32 dynamic) {
    s16 minX, minY, minZ, maxX, maxY, maxZ;
    u16 minCellX, minCellY, minCellZ, maxCellX, maxCellY, maxCellZ;
    u16 cellZ, cellY, cellX;

    if (dynamic) {
        struct SurfaceNode *newNode = alloc_surface_node();
        newNode->surface = surface;
        newNode->next = gDynamicSurfaces.next;
        gDynamicSurfaces.next = newNode;
        return;
    }

    minX = min_3(surface->vertex1[0], surface->vertex2[0], surface->vertex3[0]);
    minY = min_3(surface->vertex1[1], surface->vertex2[1], surface->vertex3[1]);
    minZ = min_3(surface->vertex1[2], surface->vertex2[2], surface->vertex3[2]);
    maxX = max_3(surface->vertex1[0], surface->vertex2[0], surface->vertex3[0]);
    maxY = max_3(surface->vertex1[1], surface->vertex2[1], surface->vertex3[1]);
    maxZ = max_3(surface->vertex1[2], surface->vertex2[2], surface->vertex3[2]);

    minCellX = get_cell_index(minX, gCellGridX, gCellSizeX, CELL_OFFSET_LOWER);
    maxCellX = get_cell_index(maxX, gCellGridX, gCellSizeX, CELL_OFFSET_UPPER);
    minCellY = get_cell_index(minY, gCellGridY, gCellSizeY, CELL_OFFSET_LOWER);
    maxCellY = get_cell_index(maxY, gCellGridY, gCellSizeY, CELL_OFFSET_UPPER);
    minCellZ = get_cell_index(minZ, gCellGridZ, gCellSizeZ, CELL_OFFSET_LOWER);
    maxCellZ = get_cell_index(maxZ, gCellGridZ, gCellSizeZ, CELL_OFFSET_UPPER);

    for (cellZ = minCellZ; cellZ <= maxCellZ; cellZ++) {
        for (cellY = minCellY; cellY <= maxCellY; cellY++) {
            for (cellX = minCellX; cellX <= maxCellX; cellX++) {
                add_surface_to_cell(cellX, cellY, cellZ, surface);
            }
        }
    }
}

/**
 * Initializes a Surface struct using the given vertex data
 * @param vertexData The raw data containing vertex positions
 * @param vertexIndices Helper which tells positions in vertexData to start reading vertices
 */
static struct Surface *read_surface_data(s16 *vertexData, s16 **vertexIndices) {
    struct Surface *surface;
    register s32 x1, y1, z1;
    register s32 x2, y2, z2;
    register s32 x3, y3, z3;
    s32 maxY, minY;
    f32 nx, ny, nz;
    f32 mag;
    s16 offset1, offset2, offset3;

    offset1 = 3 * (*vertexIndices)[0];
    offset2 = 3 * (*vertexIndices)[1];
    offset3 = 3 * (*vertexIndices)[2];

    x1 = *(vertexData + offset1 + 0);
    y1 = *(vertexData + offset1 + 1);
    z1 = *(vertexData + offset1 + 2);

    x2 = *(vertexData + offset2 + 0);
    y2 = *(vertexData + offset2 + 1);
    z2 = *(vertexData + offset2 + 2);

    x3 = *(vertexData + offset3 + 0);
    y3 = *(vertexData + offset3 + 1);
    z3 = *(vertexData + offset3 + 2);

    // (v2 - v1) x (v3 - v2)
    nx = (y2 - y1) * (z3 - z2) - (z2 - z1) * (y3 - y2);
    ny = (z2 - z1) * (x3 - x2) - (x2 - x1) * (z3 - z2);
    nz = (x2 - x1) * (y3 - y2) - (y2 - y1) * (x3 - x2);
    mag = sqrtf(nx * nx + ny * ny + nz * nz);

    // Could have used min_3 and max_3 for this...
    minY = y1;
    if (y2 < minY) {
        minY = y2;
    }
    if (y3 < minY) {
        minY = y3;
    }

    maxY = y1;
    if (y2 > maxY) {
        maxY = y2;
    }
    if (y3 > maxY) {
        maxY = y3;
    }

    // Checking to make sure no DIV/0
    if (mag < 0.0001) {
        return NULL;
    }
    mag = (f32)(1.0 / mag);
    nx *= mag;
    ny *= mag;
    nz *= mag;

    surface = alloc_surface();

    surface->vertex1[0] = x1;
    surface->vertex2[0] = x2;
    surface->vertex3[0] = x3;

    surface->vertex1[1] = y1;
    surface->vertex2[1] = y2;
    surface->vertex3[1] = y3;

    surface->vertex1[2] = z1;
    surface->vertex2[2] = z2;
    surface->vertex3[2] = z3;

    surface->normal.x = nx;
    surface->normal.y = ny;
    surface->normal.z = nz;

    surface->originOffset = -(nx * x1 + ny * y1 + nz * z1);

    return surface;
}

/**
 * Returns whether a surface has exertion/moves Mario
 * based on the surface type.
 */
static s32 surface_has_force(s16 surfaceType) {
    s32 hasForce = FALSE;

    switch (surfaceType) {
        case SURFACE_0004: // Unused
        case SURFACE_FLOWING_WATER:
        case SURFACE_DEEP_MOVING_QUICKSAND:
        case SURFACE_SHALLOW_MOVING_QUICKSAND:
        case SURFACE_MOVING_QUICKSAND:
        case SURFACE_HORIZONTAL_WIND:
        case SURFACE_INSTANT_MOVING_QUICKSAND:
            hasForce = TRUE;
            break;

        default:
            break;
    }
    return hasForce;
}

/**
 * Returns whether a surface should have the
 * SURFACE_FLAG_NO_CAM_COLLISION flag.
 */
static s32 surf_has_no_cam_collision(s16 surfaceType) {
    s32 flags = 0;

    switch (surfaceType) {
        case SURFACE_NO_CAM_COLLISION:
        case SURFACE_NO_CAM_COLLISION_77: // Unused
        case SURFACE_NO_CAM_COL_VERY_SLIPPERY:
        case SURFACE_SWITCH:
            flags = SURFACE_FLAG_NO_CAM_COLLISION;
            break;

        default:
            break;
    }

    return flags;
}

/**
 * Load in the surfaces for a given surface type. This includes setting the flags,
 * exertion, and room.
 */
static void load_static_surfaces(s16 **data, s16 *vertexData, s16 surfaceType, s8 **surfaceRooms) {
    s32 i;
    s32 numSurfaces;
    struct Surface *surface;
    s8 room = 0;
    s16 hasForce = surface_has_force(surfaceType);
    s16 flags = surf_has_no_cam_collision(surfaceType);

    numSurfaces = *(*data);
    (*data)++;

    for (i = 0; i < numSurfaces; i++) {
        if (*surfaceRooms != NULL) {
            room = *(*surfaceRooms);
            (*surfaceRooms)++;
        }

        surface = read_surface_data(vertexData, data);
        if (surface != NULL) {
            surface->room = room;
            surface->type = surfaceType;
            surface->flags = (s8) flags;

            if (hasForce) {
                surface->force = *(*data + 3);
            } else {
                surface->force = 0;
            }

            add_surface(surface, FALSE);
        }

        *data += 3;
        if (hasForce) {
            (*data)++;
        }
    }
}

/**
 * Read the data for vertices for reference by triangles.
 */
static s16 *read_vertex_data(s16 **data) {
    s32 numVertices;
    UNUSED u8 filler[16];
    s16 *vertexData;

    numVertices = *(*data);
    (*data)++;

    vertexData = *data;
    *data += 3 * numVertices;

    return vertexData;
}

/**
 * Loads in special environmental regions, such as water, poison gas, and JRB fog.
 */
static void load_environmental_regions(s16 **data) {
    s32 numRegions;
    s32 i;

    gEnvironmentRegions = *data;
    numRegions = *(*data)++;

    if (numRegions > 20) {
    }

    for (i = 0; i < numRegions; i++) {
        UNUSED s16 val, loX, loZ, hiX, hiZ;
        s16 height;

        val = *(*data)++;

        loX = *(*data)++;
        hiX = *(*data)++;
        loZ = *(*data)++;
        hiZ = *(*data)++;

        height = *(*data)++;

        gEnvironmentLevels[i] = height;
    }
}

/**
 * Allocate some of the main pool for surfaces (2300 surf) and for surface nodes (7000 nodes).
 */
void alloc_surface_pools(void) {
    sSurfacePoolSize = 5000;
    sSurfaceNodePool = main_pool_alloc(7000 * sizeof(struct SurfaceNode), MEMORY_POOL_LEFT);
    sSurfacePool = main_pool_alloc(sSurfacePoolSize * sizeof(struct Surface), MEMORY_POOL_LEFT);

    gCCMEnteredSlide = 0;
    reset_red_coins_collected();
}

#ifdef NO_SEGMENTED_MEMORY
/**
 * Get the size of the terrain data, to get the correct size when copying later.
 */
u32 get_area_terrain_size(s16 *data) {
    s16 *startPos = data;
    s32 end = FALSE;
    s16 terrainLoadType;
    s32 numVertices;
    s32 numRegions;
    s32 numSurfaces;
    s16 hasForce;

    while (!end) {
        terrainLoadType = *data++;

        switch (terrainLoadType) {
            case TERRAIN_LOAD_VERTICES:
                numVertices = *data++;
                data += 3 * numVertices;
                break;

            case TERRAIN_LOAD_OBJECTS:
                data += get_special_objects_size(data);
                break;

            case TERRAIN_LOAD_ENVIRONMENT:
                numRegions = *data++;
                data += 6 * numRegions;
                break;

            case TERRAIN_LOAD_CONTINUE:
                continue;

            case TERRAIN_LOAD_END:
                end = TRUE;
                break;

            default:
                numSurfaces = *data++;
                hasForce = surface_has_force(terrainLoadType);
                data += (3 + hasForce) * numSurfaces;
                break;
        }
    }

    return data - startPos;
}
#endif


/**
 * Process the level file, loading in vertices, surfaces, some objects, and environmental
 * boxes (water, gas, JRB fog).
 */
void load_area_terrain(s16 index, s16 *data, s8 *surfaceRooms, s16 *macroObjects) {
    s16 terrainLoadType;
    s16 *vertexData;
    UNUSED u8 filler[4];

    // Initialize the data for this.
    gEnvironmentRegions = NULL;
    unused8038BE90 = 0;
    gSurfaceNodesAllocated = 0;
    gSurfacesAllocated = 0;

    clear_static_surfaces();

    // A while loop iterating through each section of the level data. Sections of data
    // are prefixed by a terrain "type." This type is reused for surfaces as the surface
    // type.
    while (TRUE) {
        terrainLoadType = *data;
        data++;

        if (TERRAIN_LOAD_IS_SURFACE_TYPE_LOW(terrainLoadType)) {
            load_static_surfaces(&data, vertexData, terrainLoadType, &surfaceRooms);
        } else if (terrainLoadType == TERRAIN_LOAD_VERTICES) {
            vertexData = read_vertex_data(&data);
        } else if (terrainLoadType == TERRAIN_LOAD_OBJECTS) {
            spawn_special_objects(index, &data);
        } else if (terrainLoadType == TERRAIN_LOAD_ENVIRONMENT) {
            load_environmental_regions(&data);
        } else if (terrainLoadType == TERRAIN_LOAD_CONTINUE) {
            continue;
        } else if (terrainLoadType == TERRAIN_LOAD_END) {
            break;
        } else if (TERRAIN_LOAD_IS_SURFACE_TYPE_HIGH(terrainLoadType)) {
            load_static_surfaces(&data, vertexData, terrainLoadType, &surfaceRooms);
            continue;
        }
    }

    if (macroObjects != NULL && *macroObjects != -1) {
        // If the first macro object presetID is within the range [0, 29].
        // Generally an early spawning method, every object is in BBH (the first level).
        if (0 <= *macroObjects && *macroObjects < 30) {
            spawn_macro_objects_hardcoded(index, macroObjects);
        }
        // A more general version that can spawn more objects.
        else {
            spawn_macro_objects(index, macroObjects);
        }
    }

    gNumStaticSurfaceNodes = gSurfaceNodesAllocated;
    gNumStaticSurfaces = gSurfacesAllocated;
}

/**
 * If not in time stop, clear the surface partitions.
 */
void clear_dynamic_and_transformed_surfaces(void) {
    gSurfacesAllocated = gNumStaticSurfaces;
    gSurfaceNodesAllocated = gNumStaticSurfaceNodes;

    gDynamicSurfaces.next = NULL;
    gStaticSurfacePartition[SPATIAL_PARTITION_FLOORS].next = NULL;
    gStaticSurfacePartition[SPATIAL_PARTITION_CEILS].next = NULL;
    gStaticSurfacePartition[SPATIAL_PARTITION_WALLS].next = NULL;
}

/**
 * Transform surfaces with a gravity matrix.
 * This will create a new surface for every existing surface.
 */
#define VERTEX_BOX_RADIUS 20000

s32 vertex_out_of_box(Vec3s vertex, Vec3s pos) {
    return (vertex[0] > (pos[0]+VERTEX_BOX_RADIUS)) || (vertex[0] < (pos[0]-VERTEX_BOX_RADIUS)) ||
           (vertex[1] > (pos[1]+VERTEX_BOX_RADIUS)) || (vertex[1] < (pos[1]-VERTEX_BOX_RADIUS)) ||
           (vertex[2] > (pos[2]+VERTEX_BOX_RADIUS)) || (vertex[2] < (pos[2]-VERTEX_BOX_RADIUS));
}

void create_transformed_surfaces(Vec3f pos) {
    Vec3s v1,v2,v3;
    Vec3f n;
    s32 i = 0;
    struct Surface *surf, *newSurf;
    struct SurfaceNode *surfaceNode = gStaticSurfaces[get_cell((s16)pos[0],(s16)pos[1],(s16)pos[2])].next;
    s32 listIndex;
    Vec3s marioPos;
    s32 count = 0;

    vec3f_to_vec3s(marioPos, pos);

    if (gMarioObject == NULL)
        return;

    while (surfaceNode != NULL) {
        struct SurfaceNode *newNode = alloc_surface_node();
        struct SurfaceNode *list;

        count += 1;

        surf = surfaceNode->surface;
        surfaceNode = surfaceNode->next;

        if ((surfaceNode == NULL) && !i) {
            i = 1;
            surfaceNode = gDynamicSurfaces.next;
        }

	if (surf->type == SURFACE_INTANGIBLE) {
	    continue;
	}

        // Transform vertices and normals
        vec3f_copy(n,&surf->normal.x);
        vec3s_copy(v1,surf->vertex1);
        vec3s_copy(v2,surf->vertex2);
        vec3s_copy(v3,surf->vertex3);
        mtxf_mul_vec3f(gWorldToLocalGravRotationMtx,n);
        mtxf_mul_vec3s(gWorldToLocalGravTransformMtx,v1);
        mtxf_mul_vec3s(gWorldToLocalGravTransformMtx,v2);
        mtxf_mul_vec3s(gWorldToLocalGravTransformMtx,v3);

        // Find appropiate partition
        if (n[1] > 0.01f) {
            listIndex = SPATIAL_PARTITION_FLOORS;
        } else if (n[1] < -0.01f) {
            listIndex = SPATIAL_PARTITION_CEILS;
        } else {
            listIndex = SPATIAL_PARTITION_WALLS;
        }

        // Create the new surface
        newSurf = alloc_surface();

        vec3s_copy(newSurf->vertex1, v1);
        vec3s_copy(newSurf->vertex2, v2);
        vec3s_copy(newSurf->vertex3, v3);
        vec3f_copy(&newSurf->normal.x, n);
        newSurf->originOffset = -(n[0] * v1[0] + n[1] * v1[1] + n[2] * v1[2]);

        // Copy over some important data
        newSurf->type = surf->type;
        newSurf->force = surf->force;
        newSurf->room = surf->room;
        newSurf->object = surf->object;
	newSurf->flags = surf->flags;
        newSurf->origSurf = surf;

        newNode->surface = newSurf;

        list = &gStaticSurfacePartition[listIndex];

        newNode->next = list->next;
        list->next = newNode;
    }
}

/**
 * Applies an object's transformation to the object's vertices.
 */
void transform_object_vertices(s16 **data, s16 *vertexData) {
    register s16 *vertices;
    register f32 vx, vy, vz;
    register s32 numVertices;

    Mat4 *objectTransform;
    Mat4 m;

    objectTransform = &gCurrentObject->transform;

    numVertices = *(*data);
    (*data)++;

    vertices = *data;

    if (gCurrentObject->header.gfx.throwMatrix == NULL) {
        gCurrentObject->header.gfx.throwMatrix = objectTransform;
        obj_build_transform_from_pos_and_angle(gCurrentObject, O_POS_INDEX, O_FACE_ANGLE_INDEX);
    }

    obj_apply_scale_to_matrix(gCurrentObject, m, *objectTransform);

    // Go through all vertices, rotating and translating them to transform the object.
    while (numVertices--) {
        vx = *(vertices++);
        vy = *(vertices++);
        vz = *(vertices++);

        //! No bounds check on vertex data
        *vertexData++ = (s16)(vx * m[0][0] + vy * m[1][0] + vz * m[2][0] + m[3][0]);
        *vertexData++ = (s16)(vx * m[0][1] + vy * m[1][1] + vz * m[2][1] + m[3][1]);
        *vertexData++ = (s16)(vx * m[0][2] + vy * m[1][2] + vz * m[2][2] + m[3][2]);
    }

    *data = vertices;
}

/**
 * Load in the surfaces for the gCurrentObject. This includes setting the flags, exertion, and room.
 */
void load_object_surfaces(s16 **data, s16 *vertexData) {
    s32 surfaceType;
    s32 i;
    s32 numSurfaces;
    s16 hasForce;
    s16 flags;
    s16 room;

    surfaceType = *(*data);
    (*data)++;

    numSurfaces = *(*data);
    (*data)++;

    hasForce = surface_has_force(surfaceType);

    flags = surf_has_no_cam_collision(surfaceType);
    flags |= SURFACE_FLAG_DYNAMIC;

    // The DDD warp is initially loaded at the origin and moved to the proper
    // position in paintings.c and doesn't update its room, so set it here.
    if (gCurrentObject->behavior == segmented_to_virtual(bhvDddWarp)) {
        room = 5;
    } else {
        room = 0;
    }

    for (i = 0; i < numSurfaces; i++) {
        struct Surface *surface = read_surface_data(vertexData, data);

        if (surface != NULL) {
            surface->object = gCurrentObject;
            surface->type = surfaceType;

            if (hasForce) {
                surface->force = *(*data + 3);
            } else {
                surface->force = 0;
            }

            surface->flags |= flags;
            surface->room = (s8) room;
            add_surface(surface, TRUE);
        }

        if (hasForce) {
            *data += 4;
        } else {
            *data += 3;
        }
    }
}

/**
 * Transform an object's vertices, reload them, and render the object.
 */
void load_object_collision_model(void) {
    UNUSED u8 filler[4];
    s16 vertexData[600];

    s16 *collisionData = gCurrentObject->collisionData;
    f32 marioDist = gCurrentObject->oDistanceToMario;
    f32 tangibleDist = gCurrentObject->oCollisionDistance;

    // On an object's first frame, the distance is set to 19000.0f.
    // If the distance hasn't been updated, update it now.
    if (gCurrentObject->oDistanceToMario == 19000.0f) {
        marioDist = dist_between_objects(gCurrentObject, gMarioObject);
    }

    // If the object collision is supposed to be loaded more than the
    // drawing distance of 4000, extend the drawing range.
    if (gCurrentObject->oCollisionDistance > 4000.0f) {
        gCurrentObject->oDrawingDistance = gCurrentObject->oCollisionDistance;
    }

    // Update if no Time Stop, in range, and in the current room.
    if (!(gTimeStopState & TIME_STOP_ACTIVE) && marioDist < tangibleDist
        && !(gCurrentObject->activeFlags & ACTIVE_FLAG_IN_DIFFERENT_ROOM)) {
        collisionData++;
        transform_object_vertices(&collisionData, vertexData);

        // TERRAIN_LOAD_CONTINUE acts as an "end" to the terrain data.
        while (*collisionData != TERRAIN_LOAD_CONTINUE) {
            load_object_surfaces(&collisionData, vertexData);
        }
    }

    if (marioDist < gCurrentObject->oDrawingDistance) {
        gCurrentObject->header.gfx.node.flags |= GRAPH_RENDER_ACTIVE;
    } else {
        gCurrentObject->header.gfx.node.flags &= ~GRAPH_RENDER_ACTIVE;
    }
}
