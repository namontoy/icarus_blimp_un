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

/* Larger than any packet the blimp receives, so a datagram that fills the
   receive buffer is by construction the wrong size and gets rejected. The
   LOG_CHUNK the blimp sends is bigger; nothing receives it here. */
#define LINK_MAX_PACKET_BYTES   128

typedef enum {
    /* host -> blimp */
    LINK_PKT_HEARTBEAT  = 0x01,  /* keeps the link alive; drives nothing       */
    LINK_PKT_MOTOR_SET  = 0x02,  /* standing thrust commands, repeat to hold   */
    LINK_PKT_MOTOR_STOP = 0x03,  /* coast (or brake) all four, once is enough  */
    LINK_PKT_LOG_CMD    = 0x04,  /* the capture: start, stop, clear, info, get */
    /* blimp -> host */
    LINK_PKT_TELEMETRY  = 0x81,
    LINK_PKT_SENSORS    = 0x82,  /* every sensor on the bench, see below       */
    LINK_PKT_LOG_INFO   = 0x83,  /* the capture's state, the reply to LOG_CMD  */
    LINK_PKT_LOG_CHUNK  = 0x84,  /* 16 log records, the reply to LOG_CMD GET   */
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
 * The capture: the control loop's log records (log_record_t in ctl_types.h,
 * LINK_LOG_RECORD_BYTES each) recorded in PSRAM for a bench session
 * (capture.c) and fetched afterwards. One host-to-blimp packet carries every
 * command; the blimp answers INFO, START, STOP and CLEAR with a LOG_INFO,
 * and GET with `count` LOG_CHUNKs of 16 records each, sent back to back, or
 * a LOG_INFO with an error when the range is empty. The host asks for the
 * chunks it is missing until it has them all: UDP drops, and the buffer
 * lives in RAM, so nothing is lost by asking again. Records below `count`
 * never change while `capture_id` holds, so a running capture can be read
 * too; a chunk whose capture_id differs from the LOG_INFO the host started
 * from belongs to another capture and is discarded.
 *
 * LOG_INFO also carries what a decoder needs that the records do not: the
 * ranges the IMU counts were taken at, the magnetometer's LSB per gauss and
 * which chips answered. The barometer fields of a record are already
 * compensated on the chip (0.1 Pa, 0.01 C, 0.01 %RH), so no calibration
 * travels here. LINK_LOG_FORMAT is the record layout's version; it changes
 * when log_record_t does.
 */
#define LINK_LOG_FORMAT         1
#define LINK_LOG_RECORD_BYTES   72
#define LINK_LOG_CHUNK_RECORDS  16
#define LINK_LOG_GET_MAX_CHUNKS 8       /* per GET: 8 x 1172 B in one burst */

enum {
    LINK_LOG_CMD_INFO  = 0,
    LINK_LOG_CMD_START = 1,     /* seconds: length, 0 = the firmware's default */
    LINK_LOG_CMD_STOP  = 2,
    LINK_LOG_CMD_CLEAR = 3,
    LINK_LOG_CMD_GET   = 4,     /* first: record index; count: chunks wanted   */
};

enum {
    LINK_LOG_IDLE    = 0,       /* nothing stored                               */
    LINK_LOG_RUNNING = 1,
    LINK_LOG_DONE    = 2,       /* records stored, not recording                */
};

enum {
    LINK_LOG_OK            = 0,
    LINK_LOG_ERR_NO_BUFFER = 1, /* the allocation failed at boot                */
    LINK_LOG_ERR_BUSY      = 2, /* START while one is running                   */
    LINK_LOG_ERR_RANGE     = 3, /* GET at or past count                         */
    LINK_LOG_ERR_CMD       = 4, /* unknown cmd                                  */
    LINK_LOG_ERR_LENGTH    = 5, /* START longer than the buffer: max_seconds says how long */
};

typedef struct __attribute__((packed)) {
    link_header_t h;
    uint8_t  cmd;               /* LINK_LOG_CMD_*                               */
    uint8_t  count;             /* GET: chunks, 1..LINK_LOG_GET_MAX_CHUNKS      */
    uint16_t seconds;           /* START: length                                */
    uint32_t first;             /* GET: index of the first record               */
    uint32_t crc32;
} link_log_cmd_t;

typedef struct __attribute__((packed)) {
    link_header_t h;
    uint8_t  state;             /* LINK_LOG_IDLE / RUNNING / DONE               */
    uint8_t  result;            /* LINK_LOG_OK or the error of the command      */
    uint8_t  record_size;       /* LINK_LOG_RECORD_BYTES                        */
    uint8_t  format;            /* LINK_LOG_FORMAT                              */
    uint16_t capture_id;        /* increments per START                         */
    uint16_t period_us;         /* the loop's period: records are this far apart */
    uint32_t count;             /* records stored                               */
    uint32_t capacity;          /* records the buffer holds                     */
    uint32_t wanted;            /* records this capture stops at                */
    uint32_t t_start_us;        /* the first record's clock                     */
    uint16_t imu_accel_g;       /* full scale, g; 0: no IMU attached            */
    uint16_t imu_gyro_dps;      /* full scale, deg/s                            */
    float    mag_lsb_per_gauss; /* 0: no magnetometer                           */
    uint8_t  mag_chip;          /* 0 none, 1 HMC5883L, 2 QMC5883L               */
    uint8_t  baro_chip;         /* 0 none, 1 BMP180, 2 BMP280, 3 BME280         */
    uint16_t max_seconds;       /* capacity in seconds, the START ceiling       */
    char     fw[16];            /* firmware version (git describe), NUL-padded  */
    uint32_t crc32;
} link_log_info_t;

typedef struct __attribute__((packed)) {
    link_header_t h;
    uint16_t capture_id;
    uint8_t  count;             /* records in data[], the last chunk may be short */
    uint8_t  _reserved;
    uint32_t first;             /* index of data[0]                             */
    uint8_t  data[LINK_LOG_CHUNK_RECORDS * LINK_LOG_RECORD_BYTES];  /* zero past count */
    uint32_t crc32;
} link_log_chunk_t;

_Static_assert(sizeof(link_log_cmd_t)   ==   20, "log_cmd layout");
_Static_assert(sizeof(link_log_info_t)  ==   64, "log_info layout");
_Static_assert(sizeof(link_log_chunk_t) == 1172, "log_chunk layout");

/*
 * CRC-32 as zlib, PNG and Python's zlib.crc32() compute it (polynomial
 * 0xEDB88320 reflected, init and final XOR 0xFFFFFFFF). Check value: the
 * nine ASCII bytes "123456789" give 0xCBF43926. Bitwise, no table: a packet
 * is under 64 bytes and this runs a few hundred times a second at most;
 * the 1172-byte LOG_CHUNK goes out a few hundred times per capture fetch,
 * which at about 1 ms each is still nothing.
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
