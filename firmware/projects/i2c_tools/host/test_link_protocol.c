/* test_link_protocol.c - the wire format of the Wi-Fi link (main/link_protocol.h),
 * on the computer: the size of every packet, the offset of every field of the
 * CAMERA packet, and the CRC's check value. software/blimp_link.py mirrors
 * these numbers with struct.Struct; a change here that is not made there (or
 * the other way round) would shift a field silently on one side, so the
 * numbers are written out in full rather than derived from the header.
 * Run by tools/fw_host_test.py, which tools/verify.py calls.
 */
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "link_protocol.h"
#include "test_util.h"

#define OFF(field, expect) CHECK_MSG(offsetof(link_camera_t, field) == (expect), \
                                     "offsetof(%s) = %zu, expected %d", #field, \
                                     offsetof(link_camera_t, field), (expect))

int main(void)
{
    printf("test_link_protocol\n");

    TEST_GROUP("packet sizes");
    {
        CHECK(sizeof(link_header_t) == 8);
        CHECK(sizeof(link_heartbeat_t) == 12);
        CHECK(sizeof(link_motor_set_t) == 28);
        CHECK(sizeof(link_motor_stop_t) == 16);
        CHECK(sizeof(link_telemetry_t) == 52);
        CHECK(sizeof(link_sensors_t) == 76);
        CHECK(sizeof(link_log_cmd_t) == 20);
        CHECK(sizeof(link_log_info_t) == 64);
        CHECK(sizeof(link_log_chunk_t) == 1172);
        CHECK(sizeof(link_camera_t) == 152);
    }

    TEST_GROUP("packet types");
    {
        CHECK(LINK_PKT_CAMERA == 0x85);
        CHECK(LINK_PKT_LOG_CHUNK == 0x84);
        CHECK(LINK_PROTO_VERSION == 1);     /* a new type did not bump it */
        CHECK(LINK_CAM_ROW == 1 && LINK_CAM_DETECTED == 2 && LINK_CAM_CLIPPED == 4);
        CHECK(LINK_CAM_UART == 8 && LINK_CAM_STREAMING_OFF == 16);
        CHECK(LINK_CAM_IDLE_MS == 1000);
    }

    TEST_GROUP("link_camera_t field offsets");
    {
        OFF(t_us, 8);        OFF(row_t_us, 12);   OFF(frame_seq, 16);  OFF(cam_t_ms, 20);
        OFF(flags, 24);      OFF(csv_version, 25); OFF(blobs, 26);     OFF(stage, 27);
        OFF(cx_px, 28);      OFF(cy_px, 32);      OFF(az_deg, 36);     OFF(el_deg, 40);
        OFF(area_px, 44);    OFF(w_px, 48);       OFF(h_px, 50);       OFF(fill, 52);
        OFF(range_mm, 56);   OFF(conf, 60);       OFF(b_az_deg, 64);   OFF(b_el_deg, 76);
        OFF(b_range_mm, 88); OFF(b_conf, 100);    OFF(matched_px, 112); OFF(dt_ms, 116);
        OFF(proc_us, 120);   OFF(loop_us, 124);   OFF(bytes, 128);     OFF(rejected, 132);
        OFF(overflows, 136); OFF(info_lines, 140); OFF(row_age_ms, 144);
        OFF(rows_last_s, 146); OFF(bearing_sign, 147); OFF(crc32, 148);
    }

    TEST_GROUP("CRC-32");
    {
        CHECK(link_crc32("123456789", 9) == 0xCBF43926u);
    }

    TEST_REPORT();
}
