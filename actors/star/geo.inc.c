#include "game/rendering_graph_node.h"

// 0x16000EA0
const GeoLayout star_geo[] = {
   GEO_SHADOW(SHADOW_CIRCLE_4_VERTS, 0x9B, 100),
   GEO_OPEN_NODE(),
      GEO_SCALE(0x00, 16384),
      GEO_OPEN_NODE(),
#if MIRROR_MODE == 1
	 GEO_ASM(0, geo_invert_cond_off),
#endif
         GEO_DISPLAY_LIST(LAYER_OPAQUE, star_seg3_dl_0302B870),
         GEO_DISPLAY_LIST(LAYER_ALPHA, star_seg3_dl_0302BA18),
#if MIRROR_MODE == 1
	 GEO_ASM(0, geo_invert_cond),
#endif
      GEO_CLOSE_NODE(),
   GEO_CLOSE_NODE(),
   GEO_END(),
};
