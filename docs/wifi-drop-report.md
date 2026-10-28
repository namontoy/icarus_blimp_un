# Wi-Fi drops after the control task landed: esp_timer ISR dispatch (esp-idf #18808)

**Target:** Waveshare ESP32-S3-Zero, `firmware/projects/i2c_tools/`, as the Wi-Fi access point
**Framework:** ESP-IDF v5.5.5
**Branch:** `s3-wireless-link`, on top of 8d4d99b (the control task skeleton and the capture, a697cd8)
**Date of this report:** 2026-10-28
**Status:** cause found and fixed; the fix held for 2 × 600 s of soak. Not yet confirmed by a long
operator session with the ground station and motors running.

---

## 1. Summary

After the control task (`main/ctl_task.c`) and the capture (`main/capture.c`) were added, the
S3's access point kept dropping the laptop: about every 14 s the laptop stopped hearing the
AP's beacons, left, re-joined, and lost them again. The previous firmware, 6e1add4, held the
link for minutes with the ground station open and motors running.

The cause was one line of `sdkconfig.defaults`:
`CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD=y`, which the control task used to run its
20 ms timer from the timer interrupt. In ESP-IDF v5.5.5, an ISR-dispatched esp_timer can stall
every task-dispatched esp_timer. This is esp-idf issue
[#18808](https://github.com/espressif/esp-idf/issues/18808), fixed on master and not yet in a
release. Wi-Fi's internal timers are task-dispatched. With the option off, the timer runs from
the esp_timer task as the ESP-FLY shell's did. The link then held, and the loop still keeps
20.00 ms within ±0.3 ms.

PSRAM, the 40 MHz channel, the laptop's power save, the CPU clock and silent reboots were all
suspected, and all ruled out along the way (section 3).

## 2. The symptom (measured)

The laptop's journal shows one cycle as follows. The laptop is a QCA6174, ath10k driver,
interface `wlp5s0`, NetworkManager profile `ICARUS-EB03F9`; the AP is WPA2 on channel 1.

```
00:03:05.539  wpa_supplicant: CTRL-EVENT-CONNECTED - Connection to a0:f2:62:eb:03:f9 completed
00:03:45.024  wpa_supplicant: CTRL-EVENT-BEACON-LOSS
00:03:48.072  kernel: wlp5s0: Connection to AP a0:f2:62:eb:03:f9 lost
00:03:48.120  wpa_supplicant: CTRL-EVENT-DISCONNECTED bssid=a0:f2:62:eb:03:f9 reason=4 locally_generated=1
00:03:53.171  wpa_supplicant: CTRL-EVENT-CONNECTED - Connection to a0:f2:62:eb:03:f9 completed
```

The S3's USB console shows the same cycle from the other side, with times taken from the
soak harness's log (section 4):

```
 19.64  net_wifi: station 5c:3a:45:4c:d3:51 joined (aid 1)
 21.64  esp_netif_lwip: DHCP server assigned IP to a client, IP is: 192.168.4.2
 28.85  net_wifi: station 5c:3a:45:4c:d3:51 left (aid 1, reason 4)
 34.06  net_wifi: station 5c:3a:45:4c:d3:51 joined (aid 1)
```

The cycle repeats:

- The laptop joins and gets DHCP 2 s later.
- About 7 s after DHCP it loses the beacons.
- 3 s later it gives up, and 5 s after that it joins again.
- The period is about 14.2 s.

The first session after a boot sometimes lasted 60-160 s before the cycle began; the operator
reported "it starts after 2 or 2 and half minutes". Telemetry stops while the laptop is gone.
In the baseline soak, 91 of 300 s had no telemetry at all.

The direction matters. The S3 always heard the laptop: it logged every join and handed out
DHCP every time. The laptop stopped hearing the S3. Reason 4 on both sides is an inactivity
timeout, raised locally by the laptop.

## 3. What was suspected, and what ruled each out

| Suspect | Test | Result |
|---|---|---|
| Laptop Wi-Fi power save, PMF | Profile set to `802-11-wireless.powersave 2` (off) and `pmf 1` (disable) | Still dropped. Measured. |
| Ground-station traffic | Drops with only `blimp_link.py status` running | Still dropped. Measured. |
| A laptop software update | `apt` and boot history | None since boot. Measured. |
| Silent S3 reboots | `rst:0x15 USB_UART_CHIP_RESET` in the logs | Each one came from opening the USB port, which resets the chip. No reboots in any soak. Measured. |
| CPU clock (Wi-Fi shares core 0) | Effective `sdkconfig` of both builds | 160 MHz in both. Wi-Fi task on core 0 at priority 23 in both. Wi-Fi buffer settings identical. Read from the files. |
| 40 MHz channel in a crowded band (neighbours on 1, 3, 4 and 6) | `esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20)`; the log then read "bgn, 20" | Operator: "new firmware but same behavior". Reverted. Measured. |
| PSRAM (new in this firmware; it also raised `INT_WDT` from 300 to 800 ms) | A build with `CONFIG_SPIRAM` off | Still dropped (operator, then soak A below). Measured. |

That left what was new in tonight's firmware and active without any traffic:

- the control task (core 1, priority 20, waking every 20 ms);
- its esp_timer with ISR dispatch;
- the capture buffer;
- the sensor stream publishing raw counts under its spinlock.

Everything else in the diff from 6e1add4 was capture readout code that runs only on request.
Nothing in the new code touches GPIO, the Wi-Fi driver, the PHY or power management.

## 4. The bisect (measured)

Watching for drops by eye took minutes per build and needed someone at the laptop. The
comparison was made mechanical with a soak harness, now `tools/wifi_soak.py`:

- it opens the USB console and logs it;
- it re-joins the AP with `nmcli`;
- it sends link heartbeats and counts TELEMETRY packets per second;
- it counts `CTRL-EVENT-BEACON-LOSS` in the journal and the S3's "left" lines.

Each variant was a copy of the project in a scratch directory, built and flashed from Linux.

| Run | Build | Soak | Beacon losses | S3 "left" | Seconds without telemetry |
|---|---|---|---|---|---|
| A | tonight's firmware, PSRAM off, ISR dispatch | 300 s | 21 | 22 | 91 (first at 16 s) |
| B | A without `ctl_task_start()` | 300 s | 0 | 1 † | 0 |
| C | A with the task, ISR dispatch off (task dispatch) | 300 s | 0 | 1 † | 0 |
| E | tonight's firmware, PSRAM on, task dispatch | 600 s | 0 | 0 | 0 |
| F | the committed defaults after the fix (section 6) | 600 s | 0 | 1 † | 0 |

† The re-join the harness makes at the start.

- A reproduces the fault.
- B clears it, so the control task is involved.
- C keeps the task and clears it too, so the task's work is not the cause. Its work is a
  struct copy and a record push, 4-40 µs per tick.
- E and F put PSRAM back on with no change.

The cause is the ISR-dispatched timer, with nothing else needed. 6e1add4 itself was not run
through the harness; its "stable" rests on the operator's 5-minute session.

## 5. The root cause

### What the bug is

Espressif's issue [#18808](https://github.com/espressif/esp-idf/issues/18808), *"esp_timer:
TASK-dispatch callbacks permanently stop while ISR dispatch keeps running"*, was opened
2026-07-09. It was reproduced on an ESP32-S3 on v5.5.3, v5.5.4 and master: "From a random
instant on (typically within minutes under interrupt load), all TASK-dispatch esp_timer
callbacks permanently stop being invoked. ISR-dispatch timers continue to fire normally."

The fix is commit
[958dc8a](https://github.com/espressif/esp-idf/commit/958dc8afcf4a5e345bad601265c2a04eb3bf5b1a),
on master, 2026-07-15. Its message: "The task dispatch method for the esp_timer could stall
even if it is armed if ther ISR dispatch alarm triggers close to the task dispatch. This MR
fixes a bug where the esp_timer cleared the incorrect cached array timer index and
subsequently the timer task is never woken up."

As of 2026-10-28 neither `release/v5.5` (v5.5.6, released 2026-10-16) nor `release/v6.0` has a
commit to `esp_timer_impl_common.c` after it, so no release carries the fix.

### How it works in the v5.5.5 source (read, not traced on target)

`components/esp_timer/src/esp_timer.c` and `esp_timer_impl_common.c` in the installed v5.5.5:

- The hardware has one alarm. esp_timer caches two deadlines, one for the task-dispatch list
  and one for the ISR-dispatch list, and programs the earlier one.
- On an interrupt, `esp_timer_impl_try_to_set_next_alarm()` assumes the earlier cached
  deadline is the one that fired, and removes it from the cache.
- `timer_alarm_handler()` then runs the due ISR-dispatch timers. **If any ISR timer ran, it
  does not notify the esp_timer task** (`if (isr_timers_processed == false)
  vTaskNotifyGiveFromISR(s_timer_task, ...)`).
- Suppose both deadlines are due in the same interrupt and the task deadline is the earlier
  one. Then the task deadline is dropped from the cache and the ISR timer runs. The task is
  not woken, and the hardware is re-armed only for the next ISR deadline.
- From then on, every interrupt is an ISR one and the task-dispatch timers wait. They wait
  until something re-arms the task deadline, which happens when a newly started task timer
  becomes the head of its list.

A 20 ms ISR timer gives this coincidence 50 chances a second. Wi-Fi arms task-dispatch
timers continually, so it meets one often.

### Why it showed up as Wi-Fi drops (reasoned)

Wi-Fi's internal timers run through esp_timer with task dispatch. That a stall of those
timers is what silenced the AP towards the laptop is reasoned from three things:

- the symptom: the laptop stopped hearing the AP, while the AP still heard the laptop;
- the bisect: the drops followed the dispatch method and nothing else;
- the bug's mechanism.

It was not traced: no sniffer capture of the beacons, and no esp_timer dump during a drop.

Why the link here recovered every cycle, instead of stalling for good as in #18808, is also
reasoned. Wi-Fi keeps starting task timers of its own, and one of them becoming the head of
the list re-arms the lost deadline; the station's disconnect and re-join also arms new ones.

### A second, smaller fault in our callback

The control task's callback also called `portYIELD_FROM_ISR()` on the ISR path. esp_timer's
documentation forbids that for ISR-dispatched callbacks and asks for
`esp_timer_isr_dispatch_need_yield()` instead. Here it was a no-op: the woken task is pinned
to core 1, the interrupt runs on core 0, so `woken` stays false and FreeRTOS raises the
cross-core interrupt itself. It is corrected anyway, in case ISR dispatch comes back (section 6).

## 6. The fix in this repository

- `firmware/projects/i2c_tools/sdkconfig.defaults`: `CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD`
  is no longer set, with a comment naming the issue and the fixing commit. An existing
  `sdkconfig` keeps the old value: delete it and run `set-target esp32s3` again.
- `firmware/projects/i2c_tools/main/ctl_task.c`:
  - The header's "Clock" paragraph now explains the choice.
  - The ISR path of `tick_cb()` uses `esp_timer_isr_dispatch_need_yield()`.
  - `ctl_task_start()` logs a warning at boot if ISR dispatch is turned back on.
  - `s.isr_dispatch` is set before the task starts, so the boot line names the right dispatch;
    it used to race.
- `docs/control-loop-plan.md`: decision D1 changes from ISR dispatch to task dispatch; section
  6 has the bench record.
- `tools/wifi_soak.py`: the harness of section 4.
- The 20 MHz (HT20) change was reverted: it did not help, and nothing measured supports it.

## 7. What task dispatch costs (measured)

The reason for ISR dispatch was jitter. A task-dispatched tick can queue behind Wi-Fi
(priority 23) on core 0. After run F's 600 s, with the laptop associated:

| Measurement | ISR dispatch, 2026-10-27 | Task dispatch, 2026-10-28 |
|---|---|---|
| `ctl` window period, min/max/mean | 19.99 / 20.01 / 20.00 ms | 19.98 / 20.02 / 20.00 ms |
| Overruns, missed ticks | 0, 0 | 0, 0 |
| 60 s capture over the air, step min/mean/max | (not taken over the air) | 19.70 / 20.00 / 20.30 ms, σ 0.016 ms, 0 gaps in 3000 records |
| Execution per tick | 4-58 µs | 4-40 µs |

A worst case of ±0.3 ms against a 20 ms period is negligible for a blimp's control loop.

## 8. Not verified

- That Wi-Fi's own timers are the ones that stalled, and that the AP stopped transmitting
  beacons rather than the laptop missing them. Both are reasoned (section 5); a sniffer on
  channel 1 or an `esp_timer_dump()` during a drop would settle them.
- 6e1add4's stability through the harness. It rests on the operator's observation.
- Any laptop other than this QCA6174 under Ubuntu 22.04, and the Windows laptop in particular.
  The bug is in the S3, so the fix should hold for any station, but that was not run.
- A long session with the ground station and motors running on the fixed build. This is the
  operator's next check.

## 9. Follow-ups

- **When an ESP-IDF release carries 958dc8a**, ISR dispatch can come back if the jitter of
  section 7 ever matters:
  1. Upgrade ESP-IDF.
  2. Set the option in `sdkconfig.defaults`.
  3. Run `python tools/wifi_soak.py <label> --connection ICARUS-xxxxxx --seconds 600` on both
     dispatch methods.
  4. Compare beacon losses and silent seconds.

  Until then, any new `ESP_TIMER_ISR` timer is a Wi-Fi bug.
- **A 600 s calibration capture does not fit in PSRAM.** 600 s × 50 Hz × 72 B is 2.16 MB, and
  1594 KB is free, so the buffer tops out near 440 s. With the link now stable, streaming each
  record to the laptop as it is made (about 3.6 KB/s) removes the limit. Not implemented.
- **The laptop's `ICARUS-EB03F9` profile** still has power save and PMF off from the
  investigation. Neither was the cause. To undo:
  `nmcli con modify ICARUS-EB03F9 802-11-wireless-security.pmf default 802-11-wireless.powersave 0`.

## 10. Sources

- esp-idf issue #18808, *esp_timer: TASK-dispatch callbacks permanently stop while ISR dispatch
  keeps running*: <https://github.com/espressif/esp-idf/issues/18808>
- esp-idf commit 958dc8a, *fix(esp_timer): Fix esp_timer task dispatch stall*:
  <https://github.com/espressif/esp-idf/commit/958dc8afcf4a5e345bad601265c2a04eb3bf5b1a>
- ESP Timer documentation (ESP32-S3), the ISR-dispatch rules for callbacks:
  <https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/esp_timer.html>
- esp-idf issue #11636, a related report of task-dispatch and ISR-dispatch timers holding each
  other up: <https://github.com/espressif/esp-idf/issues/11636>
- ESP-IDF v5.5.5 source as installed: `components/esp_timer/src/esp_timer.c`
  (`timer_alarm_handler`, `timer_process_alarm`) and `esp_timer_impl_common.c`
  (`esp_timer_impl_try_to_set_next_alarm`).
