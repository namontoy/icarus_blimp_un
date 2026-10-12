/*
 * Platform layer for ST's VL53L1X ULD, on top of the i2c_master driver.
 *
 * Implements the nine VL53L1_* entry points that vl53l1_platform.h declares.
 * The ULD addresses registers with a 16-bit big-endian index, so every
 * transaction is prefixed with the index high byte then low byte.
 *
 * ST's convention is that these return zero on success and non-zero on
 * failure; the ULD ORs the results together, so any non-zero value
 * propagates out as a VL53L1X_ERROR.
 */
#include <string.h>
#include "vl53l1x_port.h"
#include "vl53l1_platform.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "vl53l1x_port";

/* Generous: a stalled ranging cycle should surface as a failed read rather
   than a blocked task. */
#define PORT_TIMEOUT_MS     100

/* The sensor accepts 400 kHz, but it shares this bus with slower parts and
   with breadboard wiring, so it runs at the same 100 kHz as the rest of the
   bench. Raise it here if the wiring justifies it. */
#define PORT_SCL_SPEED_HZ   (100 * 1000)

/* The ULD never calls WriteMulti and its only ReadMulti is 17 bytes, so this
   bounds the transaction buffer well above anything the driver asks for. */
#define PORT_MAX_PAYLOAD    64

/* One sensor is the normal case; two allows a re-addressed second device. */
#define PORT_MAX_DEVICES    2

/* Non-zero, and distinct from the ULD's own error codes, so a platform-level
   failure is recognisable in a status returned to the caller. */
#define PORT_ERR            (-1)

static struct {
    i2c_master_dev_handle_t dev;
    uint16_t addr;
} s_devs[PORT_MAX_DEVICES];

static i2c_master_dev_handle_t port_lookup(uint16_t addr)
{
    for (int i = 0; i < PORT_MAX_DEVICES; i++) {
        if (s_devs[i].dev != NULL && s_devs[i].addr == addr) {
            return s_devs[i].dev;
        }
    }
    /* The ULD has no way to report "not attached", so say it here - otherwise
       a missing attach looks like a sensor that will not answer. */
    ESP_LOGE(TAG, "address 0x%02x is not attached; call vl53l1x_port_attach() first",
             (unsigned)addr);
    return NULL;
}

esp_err_t vl53l1x_port_attach(i2c_master_bus_handle_t bus, uint16_t addr_7bit)
{
    if (bus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    for (int i = 0; i < PORT_MAX_DEVICES; i++) {
        if (s_devs[i].dev != NULL && s_devs[i].addr == addr_7bit) {
            return ESP_ERR_INVALID_STATE;
        }
    }
    for (int i = 0; i < PORT_MAX_DEVICES; i++) {
        if (s_devs[i].dev == NULL) {
            i2c_device_config_t conf = {
                .scl_speed_hz = PORT_SCL_SPEED_HZ,
                .device_address = addr_7bit,
            };
            esp_err_t err = i2c_master_bus_add_device(bus, &conf, &s_devs[i].dev);
            if (err != ESP_OK) {
                s_devs[i].dev = NULL;
                return err;
            }
            s_devs[i].addr = addr_7bit;
            return ESP_OK;
        }
    }
    return ESP_ERR_NO_MEM;
}

void vl53l1x_port_detach(uint16_t addr_7bit)
{
    for (int i = 0; i < PORT_MAX_DEVICES; i++) {
        if (s_devs[i].dev != NULL && s_devs[i].addr == addr_7bit) {
            i2c_master_bus_rm_device(s_devs[i].dev);
            s_devs[i].dev = NULL;
            s_devs[i].addr = 0;
            return;
        }
    }
}

static int8_t port_write(uint16_t dev, uint16_t index, const uint8_t *data, uint32_t count)
{
    i2c_master_dev_handle_t h = port_lookup(dev);
    if (h == NULL) {
        return PORT_ERR;
    }
    if (count > PORT_MAX_PAYLOAD) {
        ESP_LOGE(TAG, "write of %u bytes exceeds the %d byte buffer",
                 (unsigned)count, PORT_MAX_PAYLOAD);
        return PORT_ERR;
    }

    uint8_t buf[2 + PORT_MAX_PAYLOAD];
    buf[0] = (uint8_t)(index >> 8);
    buf[1] = (uint8_t)(index & 0xFF);
    if (count > 0) {
        memcpy(&buf[2], data, count);
    }
    return (i2c_master_transmit(h, buf, 2 + count, PORT_TIMEOUT_MS) == ESP_OK) ? 0 : PORT_ERR;
}

static int8_t port_read(uint16_t dev, uint16_t index, uint8_t *data, uint32_t count)
{
    i2c_master_dev_handle_t h = port_lookup(dev);
    if (h == NULL) {
        return PORT_ERR;
    }
    uint8_t idx[2] = { (uint8_t)(index >> 8), (uint8_t)(index & 0xFF) };
    /* One transaction: the index write and the data read must not be split by
       another master, or the read returns someone else's register. */
    return (i2c_master_transmit_receive(h, idx, sizeof(idx), data, count,
                                        PORT_TIMEOUT_MS) == ESP_OK) ? 0 : PORT_ERR;
}

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
    return port_write(dev, index, pdata, count);
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
    return port_read(dev, index, pdata, count);
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data)
{
    return port_write(dev, index, &data, 1);
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data)
{
    uint8_t buf[2] = { (uint8_t)(data >> 8), (uint8_t)(data & 0xFF) };
    return port_write(dev, index, buf, sizeof(buf));
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data)
{
    uint8_t buf[4] = {
        (uint8_t)(data >> 24), (uint8_t)(data >> 16),
        (uint8_t)(data >> 8),  (uint8_t)(data & 0xFF)
    };
    return port_write(dev, index, buf, sizeof(buf));
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *pdata)
{
    return port_read(dev, index, pdata, 1);
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *pdata)
{
    uint8_t buf[2];
    int8_t status = port_read(dev, index, buf, sizeof(buf));
    if (status == 0) {
        *pdata = (uint16_t)((buf[0] << 8) | buf[1]);
    }
    return status;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *pdata)
{
    uint8_t buf[4];
    int8_t status = port_read(dev, index, buf, sizeof(buf));
    if (status == 0) {
        *pdata = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
                 ((uint32_t)buf[2] << 8)  | (uint32_t)buf[3];
    }
    return status;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms)
{
    (void)dev;
    if (wait_ms <= 0) {
        return 0;
    }
    /* VL53L1X_SensorInit() polls boot completion with WaitMs(1) up to a
       thousand times. Routing that through vTaskDelay at this build's 10 ms
       tick would stretch a boot that really takes a few milliseconds into
       seconds, so sub-tick waits spin and only longer ones yield.

       Longer waits round up and add a tick, because vTaskDelay truncates to
       whole ticks and its first tick can elapse immediately - so the delay is
       never shorter than asked. */
    if (wait_ms <= portTICK_PERIOD_MS) {
        esp_rom_delay_us((uint32_t)wait_ms * 1000);
        return 0;
    }
    TickType_t ticks = ((TickType_t)wait_ms + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS;
    vTaskDelay(ticks + 1);
    return 0;
}
