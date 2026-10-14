/*
 * settings - see settings.h for what and why.
 */
#include "settings.h"

#include "nvs.h"
#include "nvs_flash.h"

#define NS          "targetcam"
#define KEY         "cfg"

/* Bump when tc_settings_t changes meaning. The stored blob carries the version
   and its size, so an old record is rejected instead of misread. */
#define VERSION     1

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

bool settings_load(tc_settings_t *out)
{
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) {
        return false;                       /* never saved */
    }
    record_t r;
    size_t len = sizeof r;
    esp_err_t err = nvs_get_blob(h, KEY, &r, &len);
    nvs_close(h);
    if (err != ESP_OK || len != sizeof r ||
        r.version != VERSION || r.size != sizeof r.s) {
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
