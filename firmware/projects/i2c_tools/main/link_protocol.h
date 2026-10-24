/*
 * link_protocol.h - wire format of the Wi-Fi ground link (UDP datagrams).
 *
 * One packet per datagram. Fixed-size packed structs, little-endian: the
 * ESP32-S3 and every laptop we will meet are little-endian, so nothing is
 * byte-swapped anywhere. Every packet starts with link_header_t and ends with
 * a CRC-32 over the whole struct computed with the crc field zeroed - the
 * same shape as DroneUN's firmware/main/net/protocol.h, so the two firmwares
 * can share a ground-station parser later.
 *
 * The magic and version reject anything that is not ours or that an older
 * build sent; mis-parsing a shifted struct would otherwise drive a motor with
 * a number that used to mean something else. The sequence number is for
 * drop and reorder statistics only. Nothing is retransmitted: a lost
 * MOTOR_SET is replaced by the next one a few tens of milliseconds later, and
 * a lost telemetry packet is one missing sample in a log.
 *
 * Transport-agnostic on purpose. These packets go over UDP today; ESP-NOW
 * frames carry up to 250 bytes and would take them unchanged.
 *
 * Only <stdint.h> and the GCC/Clang packed attribute, so a Python tool can
 * read this file as the one source of truth: software/blimp_link.py mirrors
 * it with struct.Struct and checks the sizes asserted below.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Arbitrary; bump LINK_PROTO_VERSION, not this, on any layout change. */
#define LINK_PROTO_MAGIC    ((uint16_t)0xB1A7u)
#define LINK_PROTO_VERSION  ((uint8_t)1u)

/* Larger than any packet below, so a datagram that fills the receive buffer
   is by construction the wrong size and gets rejected. */
#define LINK_MAX_PACKET_BYTES   128

typedef enum {
    /* host -> blimp */
    LINK_PKT_HEARTBEAT  = 0x01,  /* keeps the link alive; drives nothing       */
    LINK_PKT_MOTOR_SET  = 0x02,  /* standing thrust commands, repeat to hold   */
    LINK_PKT_MOTOR_STOP = 0x03,  /* coast (or brake) all four, once is enough  */
    /* blimp -> host */
    LINK_PKT_TELEMETRY  = 0x81,
    LINK_PKT_SENSORS    = 0x82,  /* every sensor on the bench, see below       */
} link_pkt_type_t;

typedef struct __attribute__((packed)) {
    uint16_t magic;      /* LINK_PROTO_MAGIC                                  */
    uint8_t  version;    /* LINK_PROTO_VERSION                                */
    uint8_t  type;       /* link_pkt_type_t                                   */
    uint16_t seq;        /* rolling, per sender, wraps silently               */
    uint16_t _reserved;  /* zero; keeps the payload 4-byte aligned            */
} link_header_t;

typedef struct __attribute__((packed)) {
    link_header_t h;
    uint32_t      crc32;
} link_heartbeat_t;

/*
 * cmd[] is in motor_id_t order - FL, RL, FR, RR - exactly the array that
 * motors_set_all() takes, so the firmware applies it with one call and no
 * reordering. Each entry is a signed thrust command, -1.0 to +1.0; 0 coasts.
 * NaN or infinity is rejected by the motor driver, not mapped to anything.
 */
typedef struct __attribute__((packed)) {
    link_header_t h;
    float         cmd[4];
    uint32_t      crc32;
} link_motor_set_t;

typedef struct __attribute__((packed)) {
    link_header_t h;
    uint8_t       brake;       /* 0: coast (props freewheel), 1: brake        */
    uint8_t       _reserved[3];
    uint32_t      crc32;
} link_motor_stop_t;

/* motors_failsafe_state() narrowed to a byte. */
enum {
    LINK_FAILSAFE_OFF     = 0,
    LINK_FAILSAFE_IDLE    = 1,
    LINK_FAILSAFE_ARMED   = 2,
    LINK_FAILSAFE_TRIPPED = 3,
};

typedef struct __attribute__((packed)) {
    link_header_t h;
    uint32_t t_us;           /* blimp clock, esp_timer, wraps after ~71 min   */
    float    motor_cmd[4];   /* what the driver is holding, FL RL FR RR,
                              * before deadband mapping - the readback that
                              * proves a command arrived                      */
    uint8_t  failsafe;       /* LINK_FAILSAFE_*                               */
    uint8_t  link_alive;     /* 1 while a valid packet arrived within
                              * CONFIG_NET_LINK_TIMEOUT_MS                    */
    uint8_t  link_motors;    /* 1 if the standing commands came over the air  */
    uint8_t  stations;       /* Wi-Fi stations associated with the AP         */
    uint32_t rx_ok;          /* valid packets since boot                      */
    uint32_t rx_bad;         /* datagrams rejected: size, magic, version, CRC */
    uint32_t free_heap;      /* bytes, a cheap leak detector for long runs    */
    uint16_t link_age_ms;    /* since the last valid packet, saturates        */
    uint16_t _reserved;
    uint32_t crc32;
} link_telemetry_t;

_Static_assert(sizeof(link_header_t)     ==  8, "header layout");
_Static_assert(sizeof(link_heartbeat_t)  == 12, "heartbeat layout");
_Static_assert(sizeof(link_motor_set_t)  == 28, "motor_set layout");
_Static_assert(sizeof(link_motor_stop_t) == 16, "motor_stop layout");
_Static_assert(sizeof(link_telemetry_t)  == 52, "telemetry layout");

/*
 * The bench's sensors, one packet at a fixed rate for the ground station's
 * displays and logs. Units are the ones the console commands print: g and
 * deg/s from the MPU-6050, gauss from the magnetometer (HMC5883L on the
 * GY-88, QMC5883L on the breakout that replaced it), pascal and metres from
 * the barometer (BMP180 or BME280), millimetres and ST's range status from
 * the VL53L1X. Which chip answered is the driver's business, not the wire's.
 *
 * `valid` says which blocks carry a fresh reading; a clear bit means the
 * fields of that block are zero and must not be plotted. The rates differ
 * per sensor (the barometer is slow, the ToF depends on its timing budget),
 * so the same reading may repeat across packets.
 *
 * Sent by net_link.c from the samples sensor_stream.c keeps, one packet per
 * new sample, after each TELEMETRY. The stream pauses while a console sensor
 * command runs, and then nothing is sent: a gap on the ground, never a
 * frozen number. software/mock_blimp.py sends the same packet for work
 * without hardware. Adding a packet type did not change LINK_PROTO_VERSION:
 * older receivers reject it by type.
 */
#define LINK_SENSORS_IMU    (1u << 0)
#define LINK_SENSORS_MAG    (1u << 1)
#define LINK_SENSORS_BARO   (1u << 2)
#define LINK_SENSORS_TOF    (1u << 3)

typedef struct __attribute__((packed)) {
    link_header_t h;
    uint32_t t_us;
    float    ax_g, ay_g, az_g;         /* MPU-6050 accelerometer, body axes  */
    float    gx_dps, gy_dps, gz_dps;   /* MPU-6050 gyroscope                 */
    float    imu_temp_c;
    float    mx_gauss, my_gauss, mz_gauss;
    float    heading_deg;              /* from the magnetometer, 0..360      */
    float    pressure_pa;
    float    baro_temp_c;
    float    baro_alt_m;               /* standard-atmosphere altitude minus
                                        * the first reading since boot, so 0
                                        * at the bench; 'baro' prints the
                                        * absolute one                       */
    uint16_t tof_mm;
    uint8_t  tof_status;               /* ST ULD range status, 0 = valid     */
    uint8_t  valid;                    /* LINK_SENSORS_* bits                */
    uint32_t crc32;
} link_sensors_t;

_Static_assert(sizeof(link_sensors_t)    == 76, "sensors layout");

/*
 * CRC-32 as zlib, PNG and Python's zlib.crc32() compute it (polynomial
 * 0xEDB88320 reflected, init and final XOR 0xFFFFFFFF). Check value: the
 * nine ASCII bytes "123456789" give 0xCBF43926. Bitwise, no table: a packet
 * is under 64 bytes and this runs a few hundred times a second at most.
 */
static inline uint32_t link_crc32(const void *data, size_t len)
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

#ifdef __cplusplus
}
#endif
