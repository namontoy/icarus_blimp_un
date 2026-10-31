/*
 * motor_map.c - the motor map kept in NVS, and the 'motormap' command.
 *
 * The map itself (which name, position and sign each driver output carries)
 * lives in cmd_motor.c; cmd_motor.h explains why the name and the position
 * are kept apart. This file makes it a console line to change and survive a
 * reboot, so that renaming a motor, moving one or turning a propeller round
 * never needs a rebuild (asked for on 2026-10-30, when the rope rig found the
 * team's names mirrored against core/'s frame and the team chose to keep
 * them for now).
 *
 * Every change is checked (names and positions stay permutations, signs are
 * + or -), refused while a motor turns or the control loop holds the motors,
 * applied at once and saved at once: a map is a fact about the airframe, not
 * a tuning value to try and forget, so there is no RAM-only stage as 'param'
 * has. 'motormap reset' goes back to Kconfig's defaults and forgets the saved
 * one.
 */
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include "esp_console.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "cmd_motor.h"
#include "motor_map.h"

static const char *TAG = "motormap";

#define MAP_NVS_NAMESPACE   "motormap"
#define MAP_NVS_KEY         "map"
#define MAP_BLOB_VERSION    1

typedef struct {
    uint8_t     version;
    motor_map_t map;
} map_blob_t;

static const char *const k_labels[MOTOR_COUNT] = { "FL", "RL", "FR", "RR" };

/* NVS is initialised by the Wi-Fi bring-up (net_wifi.c); without the radio,
   do it here, but never erase: a partition that needs erasing is the Wi-Fi
   code's decision, and the map then simply stays at its defaults. */
static esp_err_t open_nvs(nvs_open_mode_t mode, nvs_handle_t *h)
{
    esp_err_t err = nvs_open(MAP_NVS_NAMESPACE, mode, h);
    if (err == ESP_ERR_NVS_NOT_INITIALIZED) {
        err = nvs_flash_init();
        if (err == ESP_OK) {
            err = nvs_open(MAP_NVS_NAMESPACE, mode, h);
        }
    }
    return err;
}

static esp_err_t save_map(const motor_map_t *map)
{
    nvs_handle_t h;
    esp_err_t err = open_nvs(NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    map_blob_t blob = { .version = MAP_BLOB_VERSION, .map = *map };
    err = nvs_set_blob(h, MAP_NVS_KEY, &blob, sizeof(blob));
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

static esp_err_t forget_map(void)
{
    nvs_handle_t h;
    esp_err_t err = open_nvs(NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_erase_key(h, MAP_NVS_KEY);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        err = ESP_OK;
    }
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

void motor_map_load(void)
{
    nvs_handle_t h;
    esp_err_t err = open_nvs(NVS_READONLY, &h);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "no saved motor map: Kconfig's defaults");
        return;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS unavailable (%s): Kconfig's motor map", esp_err_to_name(err));
        return;
    }
    map_blob_t blob;
    size_t len = sizeof(blob);
    err = nvs_get_blob(h, MAP_NVS_KEY, &blob, &len);
    nvs_close(h);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "no saved motor map: Kconfig's defaults");
        return;
    }
    if (err != ESP_OK || len != sizeof(blob) || blob.version != MAP_BLOB_VERSION) {
        ESP_LOGE(TAG, "saved motor map unreadable (%s, %u bytes, version %u): Kconfig's defaults",
                 esp_err_to_name(err), (unsigned)len, (unsigned)blob.version);
        return;
    }
    err = motors_set_map(&blob.map);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "saved motor map refused (%s): Kconfig's defaults", esp_err_to_name(err));
        return;
    }
    ESP_LOGW(TAG, "motor map from NVS, not Kconfig's: 'motormap' shows it, 'motormap reset' restores");
}

static int parse_label(const char *s)
{
    for (int m = 0; m < MOTOR_COUNT; m++) {
        if (strcasecmp(s, k_labels[m]) == 0) {
            return m;
        }
    }
    return -1;
}

static void print_map(void)
{
    motor_map_t map, def;
    motors_get_map(&map);
    motors_default_map(&def);
    printf("%-7s %-12s %-6s %-9s %-5s\n", "output", "driver", "name", "position", "sign");
    for (int o = 0; o < MOTOR_COUNT; o++) {
        const bool changed = map.name[o] != def.name[o] || map.position[o] != def.position[o] ||
                             map.sign[o] != def.sign[o];
        printf("%-7d %-12s %-6s %-9s %-5c%s\n", o + 1, motor_where((motor_id_t)map.name[o]),
               k_labels[map.name[o]], k_labels[map.position[o]], map.sign[o] > 0 ? '+' : '-',
               changed ? "  (not Kconfig's)" : "");
    }
    printf("name: what the link, the console and the ground use. position: where the motor sits in\n"
           "core/'s frame seen from behind the blimp (FL = surge motor on its own left); the control\n"
           "loop drives positions. sign: the command direction that pushes forward or up.\n");
}

static int usage(void)
{
    printf("motormap                          the map\n"
           "motormap name <1-4> <FL|RL|FR|RR>  rename an output (swaps with the output that had the name)\n"
           "motormap position <1-4> <label>    where an output's motor sits, in core/'s frame (swaps too)\n"
           "motormap sign <1-4> <+|->          the command sign that pushes forward (surge) or up (heave)\n"
           "motormap reset                    Kconfig's map, and forget the saved one\n"
           "Every change is saved in NVS at once; refused while a motor turns or 'hold' runs.\n");
    return 1;
}

static int do_motormap_cmd(int argc, char **argv)
{
    if (argc == 1) {
        print_map();
        return 0;
    }
    motor_map_t map;
    motors_get_map(&map);

    if (argc == 2 && strcasecmp(argv[1], "reset") == 0) {
        motors_default_map(&map);
        esp_err_t err = motors_set_map(&map);
        if (err != ESP_OK) {
            printf("refused: %s\n", err == ESP_ERR_INVALID_STATE
                   ? "a motor is driven or the control loop holds the motors" : esp_err_to_name(err));
            return 1;
        }
        err = forget_map();
        if (err != ESP_OK) {
            printf("applied, but the saved map could not be erased: %s\n", esp_err_to_name(err));
            return 1;
        }
        print_map();
        return 0;
    }
    if (argc != 4) {
        return usage();
    }

    const int o = (argv[2][0] >= '1' && argv[2][0] <= '4' && argv[2][1] == '\0') ? argv[2][0] - '1' : -1;
    if (o < 0) {
        printf("the output is 1, 2, 3 or 4 ('motormap' lists them)\n");
        return 1;
    }
    if (strcasecmp(argv[1], "name") == 0 || strcasecmp(argv[1], "position") == 0) {
        const int label = parse_label(argv[3]);
        if (label < 0) {
            printf("'%s' is not FL, RL, FR or RR\n", argv[3]);
            return 1;
        }
        uint8_t *field = (strcasecmp(argv[1], "name") == 0) ? map.name : map.position;
        for (int k = 0; k < MOTOR_COUNT; k++) {
            if (field[k] == label) {
                field[k] = field[o];    /* the output that had it takes this one's */
            }
        }
        field[o] = (uint8_t)label;
    } else if (strcasecmp(argv[1], "sign") == 0) {
        if (strcmp(argv[3], "+") != 0 && strcmp(argv[3], "-") != 0) {
            printf("the sign is + or -\n");
            return 1;
        }
        map.sign[o] = (argv[3][0] == '-') ? -1 : 1;
    } else {
        return usage();
    }

    esp_err_t err = motors_set_map(&map);
    if (err != ESP_OK) {
        printf("refused: %s\n", err == ESP_ERR_INVALID_STATE
               ? "a motor is driven or the control loop holds the motors" : esp_err_to_name(err));
        return 1;
    }
    err = save_map(&map);
    if (err != ESP_OK) {
        printf("applied but NOT saved (%s): a reboot restores the previous map\n", esp_err_to_name(err));
    }
    print_map();
    return err == ESP_OK ? 0 : 1;
}

void register_motormap(void)
{
    const esp_console_cmd_t cmd = {
        .command = "motormap",
        .help = "Which name, position (core/'s frame) and sign each motor output has; saved in NVS",
        .hint = "[name|position <1-4> <FL|RL|FR|RR> | sign <1-4> <+|-> | reset]",
        .func = &do_motormap_cmd,
        .argtable = NULL
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
