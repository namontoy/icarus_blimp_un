/*
 * net_console.c - the bench console over UDP.
 *
 * Each datagram is one command line. It runs through esp_console_run(), the
 * same registry the USB REPL uses, so 'imu', 'tof', 'motor --sweep' and
 * everything registered later work over the air with no per-command code.
 * The output goes back to the sender as one or more datagrams, closed by a
 * single 0x04 byte (ASCII End of Transmission) in a datagram of its own, so
 * the host knows when the command has finished.
 *
 * How the output is captured: under ESP-IDF's FreeRTOS, newlib's stdio is
 * per task - 'stdout' resolves to the calling task's own stream. This task
 * points its stdout and stderr at a funopen() stream whose write function
 * buffers into a datagram, and nothing else notices: the USB REPL keeps its
 * console, and other tasks' logs keep going where they went. One consequence
 * worth knowing: a log line printed by another task during a command (the
 * motor failsafe tripping, a station leaving) still goes to USB, not here.
 * The telemetry stream carries the failsafe state for exactly that reason.
 *
 * Output is flushed when the datagram buffer fills, at the end of the
 * command, and on a newline once enough time has passed since the last
 * flush - so a slow command such as 'tof -n 200' streams line by line
 * instead of arriving in one lump 20 seconds later, while 'help' is not
 * split into a hundred tiny datagrams.
 *
 * One console at a time. esp_console_run() splits every line in one static
 * buffer and each command keeps one argtable, so a line from USB and one
 * from here at the same instant would corrupt each other. Nothing enforces
 * this; on a bench with one operator it does not need enforcing, and in
 * flight the USB cable is not there.
 *
 * Anyone on the access point can run commands. The WPA2 key is the gate.
 * One thing is refused over the air: switching the motor failsafe off, which
 * would leave a console-driven motor with no timer (cmd_motor.c asks
 * net_console_is_caller()).
 */
#include <stdio.h>
#include <string.h>
#include "esp_console.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "sdkconfig.h"
#include "net_console.h"

static const char *TAG = "net_console";

/* The REPL's own limit for one line (esp_console's default). */
#define CONSOLE_LINE_MAX        256

/* Below the Wi-Fi MTU with room for the IP and UDP headers. */
#define CONSOLE_OUT_BYTES       1400

/* A newline flushes only if this long has passed since the last flush. */
#define CONSOLE_LINE_FLUSH_US   20000

/* Commands run on this stack: 'tof' keeps a 200-float buffer in static
   storage, but argtable, printf with floats and the ULD are all here. */
#define CONSOLE_TASK_STACK      8192
#define CONSOLE_TASK_PRIO       2       /* same as the USB REPL */

static const char CONSOLE_EOT = '\x04';

typedef struct {
    int                sock;
    struct sockaddr_in peer;
    char               out[CONSOLE_OUT_BYTES];
    size_t             len;
    int64_t            last_flush_us;
} console_ctx_t;

static console_ctx_t s_ctx;
static TaskHandle_t  s_task;    /* the task that runs the commands, for net_console_is_caller() */

static void console_flush(console_ctx_t *ctx)
{
    if (ctx->len > 0) {
        sendto(ctx->sock, ctx->out, ctx->len, 0,
               (struct sockaddr *)&ctx->peer, sizeof(ctx->peer));
        ctx->len = 0;
    }
    ctx->last_flush_us = esp_timer_get_time();
}

/* funopen() write callback: everything the running command prints. */
static int console_write(void *cookie, const char *data, int n)
{
    console_ctx_t *ctx = cookie;
    for (int i = 0; i < n; i++) {
        ctx->out[ctx->len++] = data[i];
        if (ctx->len == sizeof(ctx->out)) {
            console_flush(ctx);
        } else if (data[i] == '\n' &&
                   esp_timer_get_time() - ctx->last_flush_us > CONSOLE_LINE_FLUSH_US) {
            console_flush(ctx);
        }
    }
    return n;
}

static void console_task(void *arg)
{
    console_ctx_t *ctx = arg;

    FILE *out = funopen(ctx, NULL, console_write, NULL, NULL);
    if (out == NULL) {
        ESP_LOGE(TAG, "funopen failed; console over UDP unavailable");
        vTaskDelete(NULL);
        return;
    }
    /* Unbuffered, so console_write sees every printf as it happens and
       does its own batching with the timing above. */
    setvbuf(out, NULL, _IONBF, 0);
    stdout = out;
    stderr = out;   /* argtable prints its usage errors here */

    char line[CONSOLE_LINE_MAX + 1];
    for (;;) {
        struct sockaddr_in from;
        socklen_t from_len = sizeof(from);
        int n = recvfrom(ctx->sock, line, CONSOLE_LINE_MAX, 0,
                         (struct sockaddr *)&from, &from_len);
        if (n <= 0) {
            continue;
        }
        line[n] = '\0';
        while (n > 0 && (line[n - 1] == '\n' || line[n - 1] == '\r' || line[n - 1] == ' ')) {
            line[--n] = '\0';
        }
        if (n == 0) {
            continue;
        }

        ctx->peer = from;
        ctx->len = 0;
        ctx->last_flush_us = esp_timer_get_time();

        /* Mirror the REPL's own wording for the error cases, so a transcript
           reads the same whichever console it came from. */
        int rc = 0;
        esp_err_t err = esp_console_run(line, &rc);
        if (err == ESP_ERR_NOT_FOUND) {
            printf("Unrecognized command\n");
        } else if (err == ESP_ERR_INVALID_ARG) {
            printf("Command line too long or empty (max %d characters)\n", CONSOLE_LINE_MAX);
        } else if (err != ESP_OK) {
            printf("Internal error: %s\n", esp_err_to_name(err));
        } else if (rc != 0) {
            printf("Command returned non-zero error code: 0x%x (%s)\n", rc, esp_err_to_name(rc));
        }

        console_flush(ctx);
        sendto(ctx->sock, &CONSOLE_EOT, 1, 0, (struct sockaddr *)&ctx->peer, sizeof(ctx->peer));
    }
}

esp_err_t net_console_start(void)
{
    if (s_ctx.sock > 0) {
        return ESP_OK;
    }
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket: errno %d", errno);
        return ESP_FAIL;
    }
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(CONFIG_NET_CONSOLE_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind to UDP %d: errno %d", CONFIG_NET_CONSOLE_PORT, errno);
        close(sock);
        return ESP_FAIL;
    }
    s_ctx.sock = sock;

    if (xTaskCreate(console_task, "net_console", CONSOLE_TASK_STACK, &s_ctx,
                    CONSOLE_TASK_PRIO, &s_task) != pdPASS) {
        ESP_LOGE(TAG, "could not start the console task");
        close(sock);
        s_ctx.sock = 0;
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "console over UDP on port %d", CONFIG_NET_CONSOLE_PORT);
    return ESP_OK;
}

/* Commands run on this console's own task, so the task is the caller: unlike
   a flag set around esp_console_run(), it cannot be fooled by a line typed on
   USB at the same moment. */
bool net_console_is_caller(void)
{
    return s_task != NULL && xTaskGetCurrentTaskHandle() == s_task;
}
