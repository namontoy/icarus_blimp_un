/*
 * settings - see settings.h for what and why.
 */
#include "settings.h"

#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#define NS          "targetcam"
#define KEY         "cfg"

/* Bump when tc_settings_t changes meaning. The stored blob carries the version
   and its size, so an old record is rejected instead of misread.
   v2: the int16 'fps' became 'period_ms', so 12.5 FPS (80 ms) is storable.
   v3: 'stage' and 'cnn_k' (the learned detector). They sit in what was the
       struct's tail padding, so all three versions have the same size, and
       only the version tells them apart. */
#define VERSION     3

/* The migration below relies on the size staying 28 B; adding a field that
   grows it makes every older record fail the size check instead. */
_Static_assert(sizeof(tc_settings_t) == 28, "tc_settings_t changed size: "
               "older records will no longer migrate; review settings_load()");

typedef struct {
    uint16_t      version;
    uint16_t      size;
    tc_settings_t s;
} record_t;

esp_err_t settings_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
}

bool settings_load(tc_settings_t *out, bool *migrated)
{
    *migrated = false;
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) {
        return false;                       /* never saved */
    }
    record_t r;
    size_t len = sizeof r;
    esp_err_t err = nvs_get_blob(h, KEY, &r, &len);
    nvs_close(h);
    if (err != ESP_OK || len != sizeof r || r.size != sizeof r.s) {
        return false;
    }
    if (r.version == 1 || r.version == 2) {
        /* v1 and v2 had no learned stages: they ran the HSV detector. Their
           tail bytes were padding, so these two are set rather than read. */
        r.s.stage = 0;
        r.s.cnn_k = 2;
    }
    if (r.version == 1) {
        /* Same layout, but the frame-rate slot held an integer fps. Every v1
           record says 10, the only default that version had, so carrying it
           over would quietly keep the board at 10 FPS after the update. The
           calibration is what is worth keeping; the rate takes the build
           default, and the caller says so. */
        r.s.period_ms = CONFIG_TARGETCAM_FRAME_PERIOD_MS;
        *migrated = true;
    } else if (r.version != 2 && r.version != VERSION) {
        return false;
    }
    *out = r.s;
    return true;
}

esp_err_t settings_save(const tc_settings_t *s)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    record_t r = { .version = VERSION, .size = sizeof r.s, .s = *s };
    err = nvs_set_blob(h, KEY, &r, sizeof r);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

esp_err_t settings_erase(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_erase_key(h, KEY);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        err = ESP_OK;                       /* nothing saved is still "erased" */
    }
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}
