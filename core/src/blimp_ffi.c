/* blimp_ffi.c - exported glue so the host-side MuJoCo sim can
 * drive the REAL compiled C core through Python's ctypes, per blimp.h's own
 * design goal: whatever the sim exercises is the exact same code that runs
 * on the vehicle, not a reimplementation.
 *
 * THIS FILE IS NOT PORTABLE core/ CODE. core/CMakeLists.txt excludes it by
 * name (`*_ffi.c`) from the ESP-IDF component build, and tools/build.py's
 * build_dll() drops -Wconversion for it specifically - both say so in their
 * own comments. It is glue for a host-only shared library (build/libblimp.dll
 * on Windows, build/libblimp.so on Linux), compiled only by tools/build.py's
 * `dll` command,
 * and must never be compiled by xtensa-gcc. Accordingly, and ONLY here,
 * this file uses <stddef.h>'s size_t and standard, boring value/pointer
 * passing - nothing an embedded target would ever see.
 *
 * DESIGN: "sizeof helper" over "create/destroy" (blimp.h task brief offers
 * either). blimp_t is a plain, self-contained, pointer-free struct (every
 * field is a float/bool/uint32_t/nested plain struct - no allocation inside
 * it anywhere in the core/ headers). So Python can own the memory itself
 * (e.g. `ctypes.create_string_buffer(lib.blimp_ffi_sizeof())`) and this file
 * never needs to malloc/free anything or hand back an opaque handle whose
 * lifetime Python has to manage correctly across a language boundary - one
 * less way for the two sides to disagree about ownership.
 *
 * EVERY EXPORTED FUNCTION - EXACT SIGNATURES FOR THE SIM HARNESS -----------
 *
 *   size_t blimp_ffi_sizeof(void);
 *       Bytes needed for one blimp_t. Allocate a buffer at least this large
 *       (e.g. ctypes.create_string_buffer) and pass its address as `b` to
 *       every other call below.
 *
 *   void blimp_ffi_init(void *b);
 *       Wraps blimp_init(). b must point at >= blimp_ffi_sizeof() bytes.
 *       Call once before any blimp_ffi_step().
 *
 *   void blimp_ffi_step(void *b, const blimp_input_t *in, blimp_actuators_t *out);
 *       Wraps blimp_step(). in and out are plain blimp/types.h structs -
 *       mirror them field-for-field as ctypes.Structure. out is ALWAYS
 *       written (blimp_step()'s own contract), even if in is malformed.
 *
 *   void blimp_ffi_notify_link(void *b, uint32_t t_us);
 *       Wraps blimp_notify_link(). Call once per valid ground packet.
 *
 *   const char *blimp_ffi_version(void);
 *       Wraps blimp_version(). Returns a pointer to a static, NUL-terminated,
 *       never-freed string literal - safe to read from Python
 *       (ctypes.c_char_p) but never write through or free.
 *
 *   int blimp_ffi_set_params(void *b, const blimp_params_t *p);
 *       Copies *p into b's params, then rulebook-validates and clamps them
 *       in place (blimp_params_validate()) - the same guarantee blimp_step()
 *       itself enforces every tick. Returns 1 if p was already fully valid,
 *       0 if something was out-of-spec and got clamped (the clamped values
 *       are what actually took effect; read them back with
 *       blimp_ffi_get_params() if the caller cares which fields changed).
 *
 *   void blimp_ffi_get_params(const void *b, blimp_params_t *out_p);
 *   void blimp_ffi_get_state(const void *b, blimp_state_t *out_s);
 *   void blimp_ffi_get_guidance(const void *b, blimp_guidance_t *out_g);
 *   void blimp_ffi_get_demand(const void *b, blimp_wrench_t *out_w);
 *   void blimp_ffi_get_actuators(const void *b, blimp_actuators_t *out_a);
 *       Copy-out accessors. Each *_out pointer must be non-NULL and point at
 *       a real instance of the named struct; on any NULL argument (b or the
 *       out pointer) the out struct, if writable, is zero-filled instead of
 *       left uninitialised. blimp_wrench_t/blimp_state_t/blimp_actuators_t/
 *       blimp_params_t/blimp_guidance_t are all in blimp/types.h,
 *       blimp/params.h and blimp/guidance.h - mirror them directly, they are
 *       already plain floats/bools/ints/uint32_t with no bitfields.
 *
 *   int blimp_ffi_get_armed(const void *b);
 *       1 if armed, 0 if disarmed or b is NULL.
 *
 * ALL POINTER PARAMETERS ABOVE ARE PLAIN ADDRESSES - pass them from Python
 * as ctypes.c_void_p (for the opaque `b` buffer) or as
 * ctypes.POINTER(YourMirrorStructure) (for in/out/p) via ctypes.byref() or
 * .from_buffer(); none of them need to be the exact C pointer TYPE, only
 * the exact address and the exact bytes-on-the-wire layout of whatever they
 * point at. No function here uses varargs, a bitfield, or a union anywhere
 * in its signature or in any struct it touches.
 */
#include "blimp/blimp.h"

#include <stddef.h>

/* Windows exports only what is marked dllexport; on Linux and macOS every
 * non-static symbol is visible by default, and the attribute keeps it so if
 * the library is ever built with -fvisibility=hidden. */
#if defined(_WIN32)
#define BLIMP_API __declspec(dllexport)
#else
#define BLIMP_API __attribute__((visibility("default")))
#endif

BLIMP_API size_t blimp_ffi_sizeof(void)
{
    return sizeof(blimp_t);
}

BLIMP_API void blimp_ffi_init(void *b)
{
    blimp_init((blimp_t *)b);
}

BLIMP_API void blimp_ffi_step(void *b, const blimp_input_t *in,
                              blimp_actuators_t *out)
{
    blimp_step((blimp_t *)b, in, out);
}

BLIMP_API void blimp_ffi_notify_link(void *b, uint32_t t_us)
{
    blimp_notify_link((blimp_t *)b, t_us);
}

BLIMP_API const char *blimp_ffi_version(void)
{
    return blimp_version();
}

BLIMP_API int blimp_ffi_set_params(void *b, const blimp_params_t *p)
{
    if (!b || !p) {
        return 0;
    }
    blimp_t *bb = (blimp_t *)b;
    bb->params = *p;
    return blimp_params_validate(&bb->params) ? 1 : 0;
}

BLIMP_API void blimp_ffi_get_params(const void *b, blimp_params_t *out_p)
{
    if (!out_p) {
        return;
    }
    const blimp_t *bb = (const blimp_t *)b;
    *out_p = bb ? bb->params : (blimp_params_t){0};
}

BLIMP_API void blimp_ffi_get_state(const void *b, blimp_state_t *out_s)
{
    if (!out_s) {
        return;
    }
    const blimp_t *bb = (const blimp_t *)b;
    *out_s = bb ? bb->state : (blimp_state_t){0};
}

BLIMP_API void blimp_ffi_get_guidance(const void *b, blimp_guidance_t *out_g)
{
    if (!out_g) {
        return;
    }
    const blimp_t *bb = (const blimp_t *)b;
    *out_g = bb ? bb->guid : (blimp_guidance_t){0};
}

BLIMP_API void blimp_ffi_get_demand(const void *b, blimp_wrench_t *out_w)
{
    if (!out_w) {
        return;
    }
    const blimp_t *bb = (const blimp_t *)b;
    *out_w = bb ? bb->demand : (blimp_wrench_t){0};
}

BLIMP_API void blimp_ffi_get_actuators(const void *b, blimp_actuators_t *out_a)
{
    if (!out_a) {
        return;
    }
    const blimp_t *bb = (const blimp_t *)b;
    *out_a = bb ? bb->act : (blimp_actuators_t){0};
}

BLIMP_API int blimp_ffi_get_armed(const void *b)
{
    const blimp_t *bb = (const blimp_t *)b;
    return (bb && bb->armed) ? 1 : 0;
}

/* ---- Layout reporting: the authoritative cross-language check -------------
 *
 * WHY THIS EXISTS. blimp_ffi.py used to validate its ctypes mirrors against a
 * hardcoded table of sizes captured from the headers at the time it was
 * written. Both sides of that comparison came from the SAME snapshot, so it
 * validated Python against Python and could never detect a C-side change.
 * It duly kept passing after blimp_guidance_t grew two fields (48 -> 56
 * bytes), and the sim then read guidance state at the wrong offsets.
 *
 * This reports what THE COMPILER actually laid out, so the check becomes
 * genuinely cross-language. Any future field added to a mirrored struct now
 * fails loudly at import instead of corrupting a simulation run. */
BLIMP_API size_t blimp_ffi_struct_size(int id)
{
    switch (id) {
    case 0:  return sizeof(blimp_pid_params_t);
    case 1:  return sizeof(blimp_params_t);
    case 2:  return sizeof(blimp_state_t);
    case 3:  return sizeof(blimp_imu_sample_t);
    case 4:  return sizeof(blimp_wrench_t);
    case 5:  return sizeof(blimp_actuators_t);
    case 6:  return sizeof(blimp_waypoint_t);
    case 7:  return sizeof(blimp_target_obs_t);
    case 8:  return sizeof(blimp_guidance_t);
    case 9:  return sizeof(blimp_input_t);
    case 10: return sizeof(blimp_t);
    case 11: return sizeof(blimp_hold_t);
    default: return (size_t)0;
    }
}
