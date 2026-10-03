/* espnow_link.c - ESP-NOW bring-up, command RX queue, telemetry TX.
 *
 * CONFIDENCE FLAG: esp_now_register_recv_cb()'s callback signature changed
 * within the ESP-IDF v5.x line - older releases pass
 * `(const uint8_t *mac_addr, const uint8_t *data, int len)`, newer ones pass
 * `(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)`
 * (recv_info exposes src/dst MAC and rx_ctrl - RSSI, channel, etc.). This
 * file uses the NEWER signature, on the reasoning that a project starting
 * now should target a current v5.x release. If the installed IDF predates
 * that change, this callback's signature will fail to compile - check
 * esp_now.h's `esp_now_recv_cb_t` typedef for the version actually
 * installed and adjust `recv_cb`'s parameter type accordingly (only the
 * signature changes; `recv_info->src_addr` in place of a bare `mac_addr`
 * is the only place that matters, and this file does not even use the
 * sender's address today).
 */
#include "espnow_link.h"

#include <string.h>

#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "board_config.h"

static const char *TAG = "espnow_link";

/* Length-1 "mailbox" queue: only the newest command matters for a live
 * control link (an old, superseded target observation is worse than no
 * observation - it would drive guidance at a stale bearing). xQueueOverwrite
 * is FreeRTOS's documented mechanism for exactly this pattern and requires
 * the queue length to be 1. */
static QueueHandle_t s_cmd_queue;
static const uint8_t s_peer_mac[6] = BLIMP_ESPNOW_PEER_MAC;

static void recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    (void)recv_info;   /* sender address/RSSI not used yet - see this file's
                        * header comment */

    if (!data || len != (int)sizeof(blimp_cmd_packet_t)) {
        return;   /* wrong size: either a foreign sender or a version skew -
                   * either way, not a packet this build understands */
    }

    blimp_cmd_packet_t pkt;
    memcpy(&pkt, data, sizeof(pkt));

    if (pkt.magic != BLIMP_PROTO_MAGIC || pkt.version != BLIMP_PROTO_VERSION) {
        return;
    }

    const uint32_t received_crc = pkt.crc32;
    pkt.crc32 = 0;   /* CRC is computed with this field zeroed, both here and
                      * on the sender's side - see protocol.h */
    if (blimp_proto_crc32(&pkt, sizeof(pkt)) != received_crc) {
        return;
    }
    pkt.crc32 = received_crc;

    if (s_cmd_queue) {
        (void)xQueueOverwrite(s_cmd_queue, &pkt);
    }
}

esp_err_t blimp_espnow_link_init(void)
{
    s_cmd_queue = xQueueCreate(1, sizeof(blimp_cmd_packet_t));
    if (!s_cmd_queue) {
        return ESP_ERR_NO_MEM;
    }

    /* Standard ESP-IDF WiFi bring-up boilerplate, required by esp_wifi_init()
     * regardless of whether ESP-NOW ever associates to an AP (it does not,
     * here - board_config.h's BLIMP_ESPNOW_WIFI_CHANNEL comment explains
     * why STA-without-association is this skeleton's assumption). Requires
     * nvs_flash_init() to already have run (main.c does this first) - WiFi
     * reads/writes calibration data through NVS. */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    const wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_channel((uint8_t)BLIMP_ESPNOW_WIFI_CHANNEL,
                                         WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(recv_cb));

    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, s_peer_mac, sizeof(peer.peer_addr));
    peer.channel = (uint8_t)BLIMP_ESPNOW_WIFI_CHANNEL;
    peer.ifidx   = WIFI_IF_STA;
    peer.encrypt = false;   /* PLACEHOLDER: unencrypted - see board_config.h's
                             * BLIMP_ESPNOW_PEER_MAC comment on hardening this
                             * before flying anywhere but an isolated bench */
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    ESP_LOGI(TAG, "ESP-NOW link up on channel %d", (int)BLIMP_ESPNOW_WIFI_CHANNEL);
    return ESP_OK;
}

void blimp_espnow_link_deinit(void)
{
    (void)esp_now_del_peer(s_peer_mac);
    (void)esp_now_unregister_recv_cb();
    (void)esp_now_deinit();
    (void)esp_wifi_stop();
    if (s_cmd_queue) {
        vQueueDelete(s_cmd_queue);
        s_cmd_queue = NULL;
    }
}

bool blimp_espnow_link_get_command(blimp_cmd_packet_t *out)
{
    if (!out || !s_cmd_queue) {
        return false;
    }
    return xQueueReceive(s_cmd_queue, out, 0) == pdTRUE;
}

esp_err_t blimp_espnow_link_send_telemetry(const blimp_telemetry_packet_t *pkt)
{
    if (!pkt) {
        return ESP_ERR_INVALID_ARG;
    }
    return esp_now_send(s_peer_mac, (const uint8_t *)pkt, sizeof(*pkt));
}
