/*
 * net_wifi.c - the blimp's own Wi-Fi access point.
 *
 * Why an access point and not a station: neither the bench nor the arena
 * has a network we control, and a laptop joining the blimp's own SSID needs
 * no router, no discovery and no extra hardware - the laptop's built-in
 * radio is the whole ground side. The alternative we weighed, ESP-NOW, is a
 * few milliseconds faster and recovers from a dropout without
 * re-associating, but a laptop can only speak it through a second ESP32 on
 * USB, and there is exactly one working S3-Zero in the lab. If a crowded
 * hall ever forces the change, link_protocol.h's packets go over ESP-NOW
 * unchanged; only net_link.c's transport moves.
 *
 * Power: the radio listens continuously at roughly 100 mA and transmit
 * bursts reach about 300 mA, on the same 5 V step-up that feeds the motors
 * and the sensors' AMS1117 (see the power note at the top of cmd_motor.c).
 * If the IMU drops off the bus the moment the access point comes up, suspect
 * the rail before the code. The wiring plan budgets for a radio that listens
 * all the time, so this is a check, not a surprise.
 *
 * The SSID carries the last three bytes of the board's MAC, so two blimps in
 * one room are distinguishable. Prefix, key, channel and station limit are
 * the "Wireless link" settings in Kconfig.
 */
#include <stdio.h>
#include <string.h>
#include "argtable3/argtable3.h"
#include "esp_console.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "net_wifi.h"

static const char *TAG = "net_wifi";

static bool          s_inited;
static volatile bool s_up;
static volatile int  s_stations;
static esp_netif_t  *s_ap_netif;
static char          s_ssid[33];

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)base;
    switch (id) {
    case WIFI_EVENT_AP_START: {
        s_up = true;
        s_stations = 0;
        char ip[16] = "?";
        net_wifi_ip_str(ip, sizeof(ip));
        ESP_LOGI(TAG, "access point '%s' up on channel %d, address %s",
                 s_ssid, CONFIG_NET_WIFI_CHANNEL, ip);
        break;
    }
    case WIFI_EVENT_AP_STOP:
        s_up = false;
        s_stations = 0;
        ESP_LOGI(TAG, "access point down");
        break;
    case WIFI_EVENT_AP_STACONNECTED: {
        const wifi_event_ap_staconnected_t *e = data;
        s_stations++;
        ESP_LOGI(TAG, "station " MACSTR " joined (aid %d)", MAC2STR(e->mac), e->aid);
        break;
    }
    case WIFI_EVENT_AP_STADISCONNECTED: {
        const wifi_event_ap_stadisconnected_t *e = data;
        if (s_stations > 0) {
            s_stations--;
        }
        ESP_LOGI(TAG, "station " MACSTR " left (aid %d, reason %d)",
                 MAC2STR(e->mac), e->aid, e->reason);
        break;
    }
    default:
        break;
    }
}

esp_err_t net_wifi_init(void)
{
    if (s_inited) {
        return ESP_OK;
    }

    /* The Wi-Fi driver keeps its PHY calibration in NVS; without a mounted
       NVS partition esp_wifi_init() fails outright. A partition from an
       older NVS version or with no free pages is simply erased - nothing
       else in this firmware stores anything there. */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS needs erasing (%s)", esp_err_to_name(err));
        err = nvs_flash_erase();
        if (err == ESP_OK) {
            err = nvs_flash_init();
        }
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_netif_init();
    if (err != ESP_OK) {
        return err;
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    s_ap_netif = esp_netif_create_default_wifi_ap();
    if (s_ap_netif == NULL) {
        ESP_LOGE(TAG, "could not create the AP network interface");
        return ESP_FAIL;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init: %s", esp_err_to_name(err));
        return err;
    }
    err = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                              &wifi_event_handler, NULL, NULL);
    if (err != ESP_OK) {
        return err;
    }

    /* The configuration is rebuilt from Kconfig at every boot, so keep the
       driver from persisting a copy in NVS: a reflash with a new key is then
       the whole story, with no stale credentials surviving it. */
    err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t mac[6];
    err = esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    if (err != ESP_OK) {
        return err;
    }
    snprintf(s_ssid, sizeof(s_ssid), "%s-%02X%02X%02X",
             CONFIG_NET_WIFI_SSID_PREFIX, mac[3], mac[4], mac[5]);

    const size_t key_len = strlen(CONFIG_NET_WIFI_PASSWORD);
    if (key_len > 0 && key_len < 8) {
        ESP_LOGE(TAG, "CONFIG_NET_WIFI_PASSWORD must be empty (open network) "
                      "or 8 to 63 characters for WPA2");
        return ESP_ERR_INVALID_ARG;
    }

    wifi_config_t wc = { 0 };
    strlcpy((char *)wc.ap.ssid, s_ssid, sizeof(wc.ap.ssid));
    wc.ap.ssid_len       = (uint8_t)strlen(s_ssid);
    strlcpy((char *)wc.ap.password, CONFIG_NET_WIFI_PASSWORD, sizeof(wc.ap.password));
    wc.ap.channel        = CONFIG_NET_WIFI_CHANNEL;
    wc.ap.max_connection = CONFIG_NET_WIFI_MAX_STA;
    /* WPA2-PSK with management-frame protection optional: every laptop
       adapter of the last decade joins that, which is not true of WPA3-only.
       The key is the only thing between a stray phone and the 'motor'
       command, so an open network is a bench-only setting. */
    wc.ap.authmode         = key_len ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    wc.ap.pmf_cfg.capable  = true;
    wc.ap.pmf_cfg.required = false;

    err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err != ESP_OK) {
        return err;
    }
    err = esp_wifi_set_config(WIFI_IF_AP, &wc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config: %s", esp_err_to_name(err));
        return err;
    }

    s_inited = true;
    ESP_LOGI(TAG, "access point '%s' configured: channel %d, %s, up to %d station(s)",
             s_ssid, CONFIG_NET_WIFI_CHANNEL, key_len ? "WPA2" : "OPEN",
             CONFIG_NET_WIFI_MAX_STA);

#if CONFIG_NET_WIFI_AT_BOOT
    return net_wifi_start();
#else
    ESP_LOGI(TAG, "not started at boot (CONFIG_NET_WIFI_AT_BOOT=n); use 'wifi --start'");
    return ESP_OK;
#endif
}

esp_err_t net_wifi_start(void)
{
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_up) {
        return ESP_OK;
    }
    esp_err_t err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t net_wifi_stop(void)
{
    if (!s_inited || !s_up) {
        return ESP_OK;
    }
    return esp_wifi_stop();
}

bool net_wifi_is_up(void)
{
    return s_up;
}

int net_wifi_station_count(void)
{
    return s_stations;
}

const char *net_wifi_ssid(void)
{
    return s_ssid;
}

esp_err_t net_wifi_ip_str(char *buf, size_t len)
{
    if (buf == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_ap_netif == NULL) {
        buf[0] = '\0';
        return ESP_ERR_INVALID_STATE;
    }
    esp_netif_ip_info_t info;
    esp_err_t err = esp_netif_get_ip_info(s_ap_netif, &info);
    if (err != ESP_OK) {
        buf[0] = '\0';
        return err;
    }
    snprintf(buf, len, IPSTR, IP2STR(&info.ip));
    return ESP_OK;
}

/* --------------------------------------------------------------- console */

static struct {
    struct arg_lit *start;
    struct arg_lit *stop;
    struct arg_end *end;
} wifi_args;

static void print_status(void)
{
    if (!s_inited) {
        printf("Wi-Fi not initialised - see the boot log for why\n");
        return;
    }
    printf("access point %s\n", s_up ? "UP" : "down");
    printf("SSID %s   channel %d   %s   up to %d station(s)\n", s_ssid,
           CONFIG_NET_WIFI_CHANNEL,
           strlen(CONFIG_NET_WIFI_PASSWORD) ? "WPA2" : "OPEN (no key!)",
           CONFIG_NET_WIFI_MAX_STA);
    if (!s_up) {
        return;
    }
    char ip[16] = "?";
    net_wifi_ip_str(ip, sizeof(ip));
    printf("address %s   UDP link port %d, console port %d\n", ip,
           CONFIG_NET_LINK_PORT, CONFIG_NET_CONSOLE_PORT);

    wifi_sta_list_t list;
    if (esp_wifi_ap_get_sta_list(&list) == ESP_OK) {
        printf("%d station(s) associated\n", list.num);
        for (int i = 0; i < list.num; i++) {
            printf("  " MACSTR "   rssi %d dBm\n", MAC2STR(list.sta[i].mac), list.sta[i].rssi);
        }
    }
}

static int do_wifi_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&wifi_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, wifi_args.end, argv[0]);
        return 1;
    }
    if (wifi_args.start->count) {
        esp_err_t err = net_wifi_init();
        if (err == ESP_OK) {
            err = net_wifi_start();
        }
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "start failed: %s", esp_err_to_name(err));
            return 1;
        }
        printf("access point starting\n");
        return 0;
    }
    if (wifi_args.stop->count) {
        if (net_wifi_stop() != ESP_OK) {
            return 1;
        }
        printf("access point stopping - this console goes with it if you are on it\n");
        return 0;
    }
    print_status();
    return 0;
}

void register_wifi(void)
{
    wifi_args.start = arg_lit0(NULL, "start", "Bring the access point up");
    wifi_args.stop  = arg_lit0(NULL, "stop", "Radio off");
    wifi_args.end   = arg_end(2);

    const esp_console_cmd_t cmd = {
        .command = "wifi",
        .help = "Status of the blimp's Wi-Fi access point; --start / --stop",
        .hint = NULL,
        .func = &do_wifi_cmd,
        .argtable = &wifi_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
