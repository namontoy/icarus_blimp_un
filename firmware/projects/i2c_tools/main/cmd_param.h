/*
 * cmd_param.h - the live copy of core/'s blimp_params_t and the 'param'
 * console command that reads and changes it, over USB or over the air.
 *
 * Pulled forward from increment 5 of docs/control-loop-plan.md (2026-10-30),
 * as a console command rather than the PARAM_GET/PARAM_SET packets of D9:
 * the UDP console (net_console.c) already carries any console command over
 * Wi-Fi, so 'param' works from the ground station's console dock and from
 * 'blimp_link.py console' with no new packet. The packets can come later
 * over the same table (param_table.h) if a ground-station panel wants them.
 *
 * The values live in RAM only. A reboot returns to core/src/params.c, which
 * stays the one place a gain is kept: a tune worth keeping is committed
 * there, never left on one chip's flash where git cannot see it.
 *
 * Hand-over to the control task: the console edits the live copy under a
 * spinlock, and the control task takes a copy at the top of a tick when the
 * change count moved (param_take()), so the loop never runs on a set that
 * is half written, and the console never waits on the loop. Until increment
 * 3 runs blimp_step() in the loop, nothing reads the copy the loop holds.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "blimp/params.h"

#ifdef __cplusplus
extern "C" {
#endif

/* A taker's count before its first take: no live set has this count. */
#define PARAM_GEN_NONE UINT32_MAX

/**
 * For the control task, once per tick: copies the live set into *out when
 * *gen differs from its change count and updates *gen. A spinlock and a
 * compare when nothing changed; nothing that waits. Returns true on a copy.
 */
bool param_take(blimp_params_t *out, uint32_t *gen);

/** Sets and resets applied since boot; 0 means params.c's values as built. */
uint32_t param_changes(void);

/**
 * Loads params.c's defaults into the live copy and registers 'param'. Call
 * before ctl_task_start(): until then param_take() copies nothing.
 */
void register_param(void);

#ifdef __cplusplus
}
#endif
