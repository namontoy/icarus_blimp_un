# vl53l1x_uld

ST's VL53L1X Ultra Lite Driver with a platform layer that talks to a bus the
caller already owns, so the sensor can share the I2C bus with other devices.

## Provenance

`st/` and the ST headers in `include/` are vendored from the
`vl53l1x_uld_esp_wrapper` component on the ESP Component Registry, whose
`core/` directory is ST's ULD (implementation version 3.5.5). ST's licence is
in `LICENSE.txt` (BSD OPEN SOURCE SLA0103).

## What was changed, and what was left out

Left out: upstream's `core/src/vl53l1_platform.c` and its entire
`esp_wrapper/`. That wrapper creates and owns an I2C bus of its own, which is
the one thing we cannot have - the GY-88 is already on this bus.
`port/vl53l1x_port.c` replaces it.

Two edits to `include/vl53l1_platform.h`, both marked `Removed:` in the file:

1. Dropped `#include "i2c_handler.h"`. It was the only thing tying the ULD
   core to the discarded wrapper.
2. Dropped the `vl53l1x_*` function-pointer typedefs and `g_vl53l1x_*_ptr`
   globals the fork added. The ULD calls the `VL53L1_*` prototypes directly,
   so the indirection served only upstream's wrapper - and once nothing
   assigns those globals, a call through one would be a NULL dereference.

`st/VL53L1X_api.c` and `st/VL53L1X_calibration.c` are byte-for-byte upstream.
The ST files keep their CRLF line endings so diffs against upstream stay clean.

## Use

```c
#include "VL53L1X_api.h"
#include "vl53l1x_port.h"

ESP_ERROR_CHECK(vl53l1x_port_attach(my_bus, VL53L1X_ADDR_DEFAULT));
/* The 7-bit address doubles as the ULD's opaque device token. */
uint16_t dev = VL53L1X_ADDR_DEFAULT;
VL53L1X_SensorInit(dev);
...
vl53l1x_port_detach(dev);
```

See `main/cmd_tof.c` for a worked example including boot-state polling,
distance mode, timing budget and range-status decoding.
