/* protocol.h - wire format for the ESP-NOW ground link.
 *
 * Two packet types, both fixed-size and packed so both ends agree on layout
 * byte-for-byte without a serialisation library:
 *
 *   blimp_cmd_packet_t        ground -> vehicle. Carries the perception
 *                              observation (bearing/elevation/range/
 *                              confidence/visible - matches
 *                              blimp_target_obs_t in core/include/blimp/
 *                              guidance.h exactly, since vision runs
 *                              OFFBOARD on a laptop per docs/03 section 6.1's
 *                              Rule 1/15-driven decision) plus an arm/disarm
 *                              request. Every valid packet, of either type,
 *                              is treated as proof the link is alive - see
 *                              espnow_link.c's use of blimp_notify_link().
 *
 *   blimp_telemetry_packet_t  vehicle -> ground. Streams state, demand,
 *                              actuators and the mixer's clamp flags, so a
 *                              saturation event shows up in a log instead of
 *                              being silently absorbed (docs/07 section 3.3's
 *                              explicit warning against that).
 *
 * Both carry a magic number + version (reject anything from an incompatible
 * build), a rolling sequence number (detect drops/reordering, not currently
 * acted on beyond logging) and a CRC32 (reject anything corrupted in the
 * air). NEITHER carries the waypoint running order (Rule 2's "specified
 * prior to each run") - that is a one-shot configuration concern, not a
 * per-tick one, and is out of scope for this skeleton; see docs/08 for the
 * gap this leaves.
 *
 * This header has no ESP-IDF dependency beyond <stdint.h>/<stdbool.h> plus
 * the GCC/Clang `packed` attribute (safe under xtensa-gcc - both are GCC-
 * family compilers) - it is firmware-only, not part of core/'s portable-C11
 * contract, but keeping it dependency-light means a ground-station tool
 * written in C (or a Python ctypes/struct mirror) can read this file
 * directly as the one source of truth for the wire layout.
 */
#ifndef BLIMP_NET_PROTOCOL_H
#define BLIMP_NET_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Arbitrary 16-bit constant, chosen to be unlikely to collide with garbage
 * on the air; not derived from anything. Bump BLIMP_PROTO_VERSION (not this)
 * on any wire-format change so old ground-station builds fail closed
 * (rejected by the magic/version check in espnow_link.c) instead of
 * mis-parsing a shifted struct layout. */
#define BLIMP_PROTO_MAGIC    ((uint16_t)0xB717u)
#define BLIMP_PROTO_VERSION  ((uint8_t)3u)   /* bumped: telemetry's actuator
                                              * fields changed from
                                              * thrust_l/thrust_r/tilt/
                                              * thrust_lat to
                                              * thrust_fl/fr/rl/rr, and
                                              * clamped_tilt was renamed
                                              * clamped_heave, for the
                                              * confirmed 4-fixed-motor
                                              * airframe (types.h) */

typedef struct __attribute__((packed)) {
    uint16_t magic;          /* BLIMP_PROTO_MAGIC                            */
    uint8_t  version;        /* BLIMP_PROTO_VERSION                          */
    uint8_t  seq;            /* rolling, wraps silently - not a reliability
                              * mechanism, just a drop/reorder observability
                              * aid in telemetry logs                        */

    /* Mirrors blimp_target_obs_t (core/include/blimp/guidance.h) field for
     * field, so espnow_link.c's translation is a straight copy, not a
     * reinterpretation. */
    float    bearing_rad;
    float    elevation_rad;
    float    range_m;
    float    confidence;
    float    hint_yaw_rad;   /* SEARCH hint: absolute heading (vehicle yaw
                              * frame, zeroed at arm) where the current
                              * target is expected - computed on the ground
                              * from the course geometry + telemetry yaw.
                              * See blimp_target_obs_t.hint_yaw_rad.        */
    float    alt_fix_m;      /* altitude above the floor from ANY balloon in
                              * the image (all sit at 2.0 m, Rule 22); see
                              * blimp_target_obs_t.alt_fix_m               */
    uint8_t  visible;        /* bool as uint8 - booleans are not a portable
                              * fixed width across compilers/ABIs, avoid them
                              * in anything that crosses the wire            */
    uint8_t  hint_valid;     /* bool as uint8; 0 -> blind scan              */
    uint8_t  alt_fix_valid;  /* bool as uint8; 0 -> no altitude fix         */

    uint8_t  arm_request;    /* bool as uint8, see above. The four uint8s
                              * above end the payload on a 4-byte boundary. */

    uint32_t crc32;          /* over every byte of this struct EXCEPT this
                              * field, which is treated as zero while
                              * computing/checking it - see
                              * blimp_proto_crc32() below                    */
} blimp_cmd_packet_t;

typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t  version;
    uint8_t  seq;

    uint32_t t_us;           /* vehicle's own monotonic clock (esp_timer_get_time
                              * truncated to 32 bits - wraps at ~71 minutes;
                              * fine for a race lasting well under that, not
                              * fine as a general-purpose uptime counter)     */

    /* state (blimp_state_t subset - core/include/blimp/types.h) */
    float roll, pitch, yaw;      /* rad   */
    float altitude;              /* m     */
    float vz;                    /* m/s   */
    float yaw_rate;               /* rad/s */

    /* demand (blimp_wrench_t - what guidance+control asked for) */
    float fx, fz, tz;

    /* actuators (blimp_actuators_t - what the mixer actually produced).
     * Four fixed unidirectional motors on this airframe - no tilt, no
     * lateral thruster (types.h). */
    float thrust_fl, thrust_fr, thrust_rl, thrust_rr;

    /* mixer clamp flags - docs/07 section 3.3: log these, never silently
     * absorb them */
    uint8_t clamped_total;
    uint8_t clamped_yaw;
    uint8_t clamped_heave;   /* heave demand clipped: negative (cannot push
                              * down) or above the heave budget - renamed
                              * from clamped_tilt, this airframe has no tilt */

    uint8_t guid_state;      /* blimp_guid_state_t, narrowed to uint8 -
                              * blimp_guidance_state_name() (guidance.h)
                              * turns this back into a string on the ground */
    uint8_t armed;           /* blimp_t.armed                                */
    uint8_t _reserved[3];    /* alignment padding, zero on send              */

    uint32_t crc32;
} blimp_telemetry_packet_t;

/* Plain CRC-32 (IEEE 802.3 / zlib / PNG polynomial 0xEDB88320, reflected,
 * final XOR) - the same variant as Python's zlib.crc32()/binascii.crc32(),
 * for easy cross-checking from a ground-station script without needing a
 * matching table-based implementation on that side.
 *
 * `static inline` in a header (not a separate protocol.c, which does not
 * exist - see the file list this was built from) is deliberate: it is safe
 * under C11's inline-linkage rules as long as every translation unit that
 * uses it also has this definition visible, which is guaranteed by
 * `#include`ing this header, and avoids a one-function .c file for a
 * three-line utility. */
static inline uint32_t blimp_proto_crc32(const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= p[i];
        for (int b = 0; b < 8; b++) {
            const uint32_t mask = (uint32_t)(-(int32_t)(crc & 1u));
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

#endif /* BLIMP_NET_PROTOCOL_H */
