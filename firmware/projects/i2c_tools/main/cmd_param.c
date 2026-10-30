/*
 * cmd_param.c - the live blimp_params_t and the 'param' console command;
 * see cmd_param.h.
 *
 *   param                    list all (a * marks a value that is not params.c's)
 *   param alt.               list the ones whose name starts with "alt."
 *   param alt.kp             show one
 *   param alt.kp 0.4         set one
 *   param --reset [name]     back to params.c, all or one
 *
 * The arguments are parsed here rather than by argtable: argtable would
 * take a negative value such as "-0.06" for an option.
 *
 * Output is meant to be read by people and by software/blimp_link.py
 * (parse_params()): the first line of a listing carries the change count,
 * and each row starts with the name and the value.
 */
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "esp_console.h"
#include "freertos/FreeRTOS.h"
#include "cmd_param.h"
#include "param_table.h"

/* Until increment 3 puts blimp_step() in the loop, a set changes a copy
   nothing reads; say so with every change rather than let a bench session
   wonder why the blimp did not react. Delete with increment 3. */
#define PARAM_USE_NOTE  "(the control loop takes it at its next tick; nothing reads it there " \
                        "until blimp_step() runs in the loop, control-loop plan increment 3)\n"

static struct {
    portMUX_TYPE   mux;             /* guards live and changes               */
    bool           ready;
    blimp_params_t live;
    uint32_t       changes;         /* sets and resets applied since boot    */
    blimp_params_t defaults;        /* params.c's, for the marks and --reset */
} s = {
    .mux = portMUX_INITIALIZER_UNLOCKED,
};

bool param_take(blimp_params_t *out, uint32_t *gen)
{
    bool took = false;
    portENTER_CRITICAL(&s.mux);
    if (s.ready && *gen != s.changes) {
        *out = s.live;
        *gen = s.changes;
        took = true;
    }
    portEXIT_CRITICAL(&s.mux);
    return took;
}

uint32_t param_changes(void)
{
    portENTER_CRITICAL(&s.mux);
    const uint32_t n = s.changes;
    portEXIT_CRITICAL(&s.mux);
    return n;
}

/* Compared through the table, not memcmp: the padding after a bool is not
   part of the value. */
static bool differs(const blimp_params_t *a, const blimp_params_t *b, const param_entry_t *e)
{
    return param_get(a, e) != param_get(b, e);
}

static size_t count_differing(const blimp_params_t *a, const blimp_params_t *b)
{
    size_t n = 0;
    for (size_t i = 0; i < param_count(); i++) {
        n += differs(a, b, param_at(i));
    }
    return n;
}

static void snapshot(blimp_params_t *out, uint32_t *changes)
{
    portENTER_CRITICAL(&s.mux);
    *out = s.live;
    *changes = s.changes;
    portEXIT_CRITICAL(&s.mux);
}

static void print_row(const blimp_params_t *p, const param_entry_t *e)
{
    printf("%-22s %12.6g  %-12s [%g, %g]%s\n", e->name, (double)param_get(p, e), e->unit,
           (double)e->min, (double)e->max, differs(p, &s.defaults, e) ? "  *" : "");
}

static int list(const char *prefix)
{
    blimp_params_t p;
    uint32_t changes;
    snapshot(&p, &changes);
    const size_t plen = prefix ? strlen(prefix) : 0;

    size_t shown = 0;
    for (size_t i = 0; i < param_count(); i++) {
        const param_entry_t *e = param_at(i);
        if (plen && strncmp(e->name, prefix, plen) != 0) {
            continue;
        }
        if (shown++ == 0) {
            printf("%u parameters; %" PRIu32 " changes since boot; %u differ from params.c "
                   "(marked *); RAM only, a reboot restores params.c\n",
                   (unsigned)param_count(), changes, (unsigned)count_differing(&p, &s.defaults));
            printf("%-22s %12s  %-12s %s\n", "name", "value", "unit", "accepted");
        }
        print_row(&p, e);
    }
    if (shown == 0) {
        printf("no parameter named or starting with '%s'; 'param' lists them\n", prefix);
        return 1;
    }
    return 0;
}

static int set(const param_entry_t *e, float asked)
{
    blimp_params_t before, after;
    uint32_t changes;
    portENTER_CRITICAL(&s.mux);
    before = s.live;
    const param_result_t r = param_set(&s.live, e, asked);
    after = s.live;
    const bool moved = count_differing(&before, &after) != 0;
    if (moved) {
        s.changes++;
    }
    changes = s.changes;
    portEXIT_CRITICAL(&s.mux);

    if (r != PARAM_OK && r != PARAM_CLAMPED) {
        printf("%s: %g %s; not changed (accepted: [%g, %g] %s)\n", e->name, (double)asked,
               param_result_str(r), (double)e->min, (double)e->max, e->unit);
        if (r == PARAM_ERR_ORDER) {
            printf("to move a clamp past its other end, set the end that widens it first\n");
        }
        return 1;
    }
    if (!moved) {
        printf("%s is already %g %s; nothing changed\n", e->name, (double)param_get(&after, e), e->unit);
        return 0;
    }
    const float now = param_get(&after, e);
    printf("%s = %g %s (was %g)%s; %" PRIu32 " changes since boot\n", e->name, (double)now, e->unit,
           (double)param_get(&before, e), now != asked ? ", clamped by blimp_params_validate()" : "",
           changes);
    for (size_t i = 0; i < param_count(); i++) {
        const param_entry_t *o = param_at(i);
        if (o != e && differs(&before, &after, o)) {
            printf("  also clamped by blimp_params_validate(): %s %g -> %g %s\n", o->name,
                   (double)param_get(&before, o), (double)param_get(&after, o), o->unit);
        }
    }
    fputs(PARAM_USE_NOTE, stdout);
    return 0;
}

static int reset_all(void)
{
    portENTER_CRITICAL(&s.mux);
    const size_t n = count_differing(&s.live, &s.defaults);
    if (n) {
        s.live = s.defaults;
        s.changes++;
    }
    const uint32_t changes = s.changes;
    portEXIT_CRITICAL(&s.mux);

    if (n == 0) {
        printf("every parameter already has params.c's value; nothing changed\n");
        return 0;
    }
    printf("%u parameters back to params.c's values; %" PRIu32 " changes since boot\n",
           (unsigned)n, changes);
    fputs(PARAM_USE_NOTE, stdout);
    return 0;
}

static const param_entry_t *find_or_say(const char *name)
{
    const param_entry_t *e = param_find(name);
    if (e == NULL) {
        printf("no parameter '%s'; 'param' lists them, 'param %.*s' the ones that start "
               "the same\n", name, (int)strcspn(name, "."), name);
    }
    return e;
}

static int do_param_cmd(int argc, char **argv)
{
    if (!s.ready) {
        printf("parameters not loaded - register_param() did not run\n");
        return 1;
    }
    if (argc == 1) {
        return list(NULL);
    }
    if (strcmp(argv[1], "--reset") == 0 || strcmp(argv[1], "-r") == 0) {
        if (argc == 2) {
            return reset_all();
        }
        const param_entry_t *e = find_or_say(argv[2]);
        return e ? set(e, param_get(&s.defaults, e)) : 1;
    }
    if (argv[1][0] == '-') {
        printf("usage: param [name|prefix [value]] | param --reset [name]\n");
        return 1;
    }
    if (argc == 2) {
        const param_entry_t *e = param_find(argv[1]);
        return list(e ? e->name : argv[1]);
    }
    if (argc == 3) {
        const param_entry_t *e = find_or_say(argv[1]);
        if (e == NULL) {
            return 1;
        }
        float v;
        if (param_parse(e, argv[2], &v) != PARAM_OK) {
            printf("%s: '%s' is not %s; not changed\n", e->name, argv[2],
                   e->type == PARAM_BOOL ? "0/1, true/false or on/off" : "a number");
            return 1;
        }
        return set(e, v);
    }
    printf("usage: param [name|prefix [value]] | param --reset [name]\n");
    return 1;
}

void register_param(void)
{
    blimp_params_defaults(&s.defaults);
    (void)blimp_params_validate(&s.defaults);
    portENTER_CRITICAL(&s.mux);
    s.live = s.defaults;
    s.changes = 0;
    s.ready = true;
    portEXIT_CRITICAL(&s.mux);

    const esp_console_cmd_t cmd = {
        .command = "param",
        .help = "The controller's parameters (core/'s blimp_params_t), live in RAM: "
                "'param' lists them, 'param alt.' those starting so, 'param alt.kp 0.4' sets one, "
                "'param --reset [name]' returns to params.c. A reboot restores params.c",
        .hint = "[name|prefix [value]] | --reset [name]",
        .func = &do_param_cmd,
        .argtable = NULL
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
