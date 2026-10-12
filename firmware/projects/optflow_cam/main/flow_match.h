/*
 * flow_match - sparse block-matching optical flow for a downward camera.
 *
 * Deliberately portable: this header and its .c file have NO ESP-IDF, no
 * FreeRTOS and no libc-beyond-freestanding dependencies, so the identical code
 * that runs on the camera also compiles on a host with gcc. That is what makes
 * it testable without hardware (see host/test_flow_match.c).
 *
 * The problem being solved is narrow on purpose. A blimp hovering over a floor
 * sees, to a very good approximation, a single rigid translation of the whole
 * image between consecutive frames. We do not need a dense flow field, we need
 * one (dx, dy) that we can trust - so the algorithm takes a sparse grid of
 * blocks, matches each one independently, and takes the MEDIAN. The median is
 * the entire point: it is what makes a person walking under the airframe, or a
 * patch of featureless floor, not matter.
 */
#pragma once

#include <stddef.h>   /* size_t, used by the row-offset arithmetic */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Caps so the SAD surface can live on the stack. Raising FLOW_MAX_SEARCH costs
   (2*s+1)^2 * 4 bytes of stack per call: 1764 B at 10, 2916 B at 13. */
#define FLOW_MAX_SEARCH   10
#define FLOW_MAX_BLOCKS   64

/* Sub-pixel estimators. The displacement a downward camera sees while hovering
   is a small fraction of a pixel, so the whole measurement comes out of this
   refinement and its bias IS the scale error. Measured on the host by
   host/bias_sweep at 0.12 px per frame, which is what the bench produces at
   1.5 m:
 *
 *   PARABOLA  gain 0.50 - fits a parabola to what is really a V, so estimates
 *             are pulled toward integer offsets ("pixel locking") and
 *             integrated distance comes out roughly half of truth.
 *   GAUSSIAN  fits a parabola to the logarithms. Intended for correlation
 *             PEAKS, where the extremum is large; a SAD minimum tends to zero,
 *             so log s0 diverges and the estimate is driven toward zero.
 *   VFIT      two straight lines through the minimum and its higher neighbour.
 *             Exact for an ideal V-shaped cost surface, which is what an
 *             absolute-difference metric produces.
 */
#define FLOW_SUBPIX_PARABOLA  0
#define FLOW_SUBPIX_GAUSSIAN  1
#define FLOW_SUBPIX_VFIT      2

typedef struct {
    int width;          /* frame width in pixels (also the row stride)        */
    int height;         /* frame height in pixels                            */
    int block;          /* block edge in pixels; 16 is a good default        */
    int search;         /* search radius in pixels, <= FLOW_MAX_SEARCH       */
    int grid_x;         /* blocks across; grid_x*grid_y <= FLOW_MAX_BLOCKS   */
    int grid_y;         /* blocks down                                       */
    uint32_t min_var;   /* reject blocks whose variance is below this        */
    int subpix;         /* FLOW_SUBPIX_*; sets the scale gain, see above      */
} flow_cfg_t;

typedef struct {
    float   dx;         /* median flow, pixels, prev -> cur, +x = right      */
    float   dy;         /* median flow, pixels, prev -> cur, +y = down       */
    int     n_valid;    /* blocks that produced a usable vector              */
    int     n_total;    /* blocks attempted                                  */
    int     n_flat;     /* rejected: below min_var (featureless floor)       */
    int     n_edge;     /* rejected: best match sat on the search boundary   */
    int     n_ambig;    /* rejected: SAD surface had no clean minimum        */
    float   spread;     /* median absolute deviation of the block vectors,
                           in pixels. Low = the blocks agree. This is the
                           honest confidence number - use it, not n_valid.   */
} flow_result_t;

/*
 * Validate a configuration against the frame geometry.
 * Returns 1 if usable, 0 if not (frame too small for block+search, grid too
 * large, search beyond FLOW_MAX_SEARCH, ...).
 */
int flow_cfg_valid(const flow_cfg_t *cfg);

/*
 * Match `cur` against `prev`, both `width * height` 8-bit grayscale.
 *
 * Returns 1 if a flow vector was produced, 0 if too few blocks survived (in
 * which case out->dx/dy are zeroed and the reject counters say why). A return
 * of 0 is a normal, expected outcome over a blank floor - the caller must treat
 * it as "no measurement", never as "no motion".
 */
int flow_match(const flow_cfg_t *cfg, const uint8_t *prev, const uint8_t *cur,
               flow_result_t *out);

#ifdef __cplusplus
}
#endif
