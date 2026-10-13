# Optical-Flow Sensor (ESP32-CAM): Basic Idea

Oct 12, 2026 · @Nerio Andrés Montoya

## The problem

The blimp's existing sensors cannot observe horizontal position, so a new sensor is required. With the measured 70 mg accelerometer bias, dead reckoning drifts 0.34 m in 1 s and 34 m in 10 s. No filter can fix this, because the information is simply absent from the data.

A downward-looking camera is the cheapest sensor that measures horizontal *velocity* directly, instead of integrating acceleration. This is the seed of that sensor: a first working version, not a finished one.

## The basic idea

An ESP32-CAM looks straight down and compares each frame with the previous one. How far the picture shifted, in pixels, times the ground size of a pixel, divided by the frame interval, gives horizontal velocity. The design target is a **2 m hover**, matching the altitude band (1.85–2.25 m) already chosen for the PING))) 28015 altimeter.

&#91;embedded content: optical flow principle · camera geometry and two frames\]

Left: the camera sees a patch of floor below it. Right: when the camera moves, a floor feature shifts by dx pixels between frames, and that shift gives the velocity.

The conversion from pixels to millimetres depends only on height and field of view:

```latex
\text{mm/px} = \frac{2 \cdot h \cdot \tan(\text{hfov}/2)}{W}
```

With the nominal 54° field of view and W = 160 px, the ground footprint and resolution are:

|  | 2.0 m (design) | 1.5 m (bench) |
| --- | --- | --- |
| Ground footprint (m) | 2.04 × 1.53 | 1.53 × 1.15 |
| mm per pixel | 12.74 | 9.55 |
| Max measurable speed (mm/s, 12.5 FPS) | 956 | 717 |

The usable search radius is one pixel less than the configured one, because a minimum on the window edge is rejected.

## How it works

The matcher is sparse block matching on a 5×5 grid of 16×16 px blocks, using integer arithmetic only (no gradients, no matrix inverse). Per block, in this order:

1. **Variance gate.** A block below the variance threshold (default 50) is featureless and is dropped, because matching a blank patch gives a confident but meaningless answer.
2. **Full SAD surface.** Sum of absolute differences over the whole ±7 px search window, with no early exit, so the neighbouring values needed for sub-pixel refinement stay intact.
3. **Boundary rejection.** If the minimum sits on the window edge, the true motion is probably outside it, so the block is rejected instead of being reported pegged at the limit.
4. **Sub-pixel fit** on the three SAD values around the minimum, per axis (the V-fit, see the results below).
5. **Median across surviving blocks**, with the median absolute deviation reported as `spread`. The median keeps a single false match from dragging the result.

Velocity is `vx = -dx · mm_per_px / dt`, sent as a 16-field CSV line on UART0 at 115200 baud.

**Honest failure is the design rule.** Every stage may refuse to answer, and the output says why: `ok = 0` means *no information*, never "stationary", and the counts `n_flat`, `n_edge`, `n_ambig` give the reason. Pointed at a white ceiling, the firmware returned `ok=0, flat=25/25` instead of inventing motion.

## Key design decisions

| Decision | Why |
| --- | --- |
| Grayscale, 8-bit QQVGA (160×120) straight from the sensor | Block matching works on intensity; colour would triple memory traffic and match cost with no usable information. Two 19 200-byte frames fit in internal RAM. |
| Matcher in portable C with no ESP-IDF dependency (234 lines) | It compiles and runs on the PC, so it was verified before any hardware existed. The host suite gives 0.024 px mean error. |
| Sharpening, denoising and extra auto-exposure off; gain capped at 4× | Sharpening invents edges and denoising smears texture; high gain adds noise that causes false matches. |
| Auto-exposure and auto-gain left on (for now) | Floor brightness varies. Pinning them is a queued step. |
| Wi-Fi and Bluetooth never initialised | Heat. Verified: the radio init functions are absent from the linked binary. |
| Frame rate set by the sensor clock alone | 20 MHz XCLK gives exactly 12.5 FPS (80 ms frames); 10 MHz gave 6.25 FPS. |

Existing projects were surveyed first. Only Espressif's `esp32-camera` driver was adopted, because none of the others is a separable sparse matcher that reports its own failures.

## What the seed has shown so far

The sensor runs on real hardware, tells still from moving, and its scale deficit has been traced and largely fixed. Measured at a 1.5 m bench distance unless noted:

- **Noise floor:** stationary velocity sigma is 2.47 mm/s at 12.5 FPS, about 24× below the \~60 mm/s signal of a slow hand slide.
- **Frame budget:** matching takes 47–55 ms of the 80 ms frame (about 65% CPU duty at 240 MHz).
- **Scale deficit:** the original parabolic sub-pixel fit recovered only about 51% of true distance at the \~0.12 px/frame a 2 m hover produces. This is pixel locking: a SAD minimum is a cusp, not a parabola, so the fit pulls estimates toward integer positions.
- **Fix:** a V-fit, the exact inverse of a V-shaped cost, recovers 91% on the host and is now the default.

| Estimator | Recovered distance at 0.12 px/frame (host) | Real-image check, 270 mm reference |
| --- | --- | --- |
| Parabola | 51% | 73.5% (−26.5%) |
| Gaussian | 22% | not tested on hardware |
| **V-fit (default)** | **91%** | **107.7% (+7.7%)** |

The Gaussian did worse than the parabola it was meant to replace: it suits correlation peaks, but at a SAD minimum the centre value tends to zero and its logarithm diverges.

Two more findings shape what comes next. The remaining dominant error is a constant image-axis bias of about 1–4 mm/s. It stays fixed in the image when the camera is rolled, so it comes from the sensor or algorithm, not the room. And one lamp alone shifted drift 20× (−2.257 to −0.114 mm/s), so plain SAD is too sensitive to illumination.

## Known limits and open questions

**Built-in limits of this board**

- **Rotation looks like translation.** Yaw produces about 35 mm/s of fake velocity per 1 °/s at 2 m. The camera board has no IMU, so it reports raw `dx_px`/`dy_px` and the board that owns the gyro must subtract the rotation term.
- **Height is a constant** typed in by the operator, not fused from the altimeter.
- **Flashing needs the BOOT button held**, because the sensor clock sits on GPIO0, a strap pin. The USB bridge also wedges after a 460800-baud flash, so use 115200.
- **Texture is required.** A target must match the geometry: at 1.5 m one block sees 153 mm and block centres are 305 mm apart.

**Not yet established**

- Absolute scale, i.e. the committed field of view. The nominal 54° looks right to within 3%, but that rests on one round trip, which has zero degrees of freedom.
- Why gain reads about 92% on diagonal motion and 108–116% on axis-aligned motion: speed, or cross-coupling in the 2-D basin. If it is diagonality, it affects every real flight.
- The mechanism of the image-axis bias, and whether zero-mean SAD removes it.
- Behaviour at the real 2 m height over a real floor, where the same speed gives 25% less pixel displacement.
- Thermal margin. Nothing has been measured with a probe, and the two main heat measures were spent to reach 12.5 FPS.

## Next steps

In priority order. Done so far: the V-fit estimator, and the analysis showing the out/back spread is a constant bias.

1. **Zero-mean SAD.** Highest value: it targets both the illumination sensitivity and the image-axis bias. Validate on the host sweep first, then on hardware.
2. **Speed or diagonality?** Repeat the 43.5° roll run with the lens at about 700 mm, which doubles the per-frame displacement. Gain staying near 92% means diagonality; climbing toward 115% means speed.
3. **Sum the per-block SAD surfaces** and minimise once, instead of 25 minima plus a median.
4. **Keep the reference frame** when a frame is rejected, so a rejection costs latency instead of displacement.
5. **Two round trips** on the height-adjustable table, then commit the field of view.
6. **Pin exposure and gain** to stop brightness hunting.
7. **Test at 2 m over the real floor**, then **measure can temperature** with an external probe under sustained load.
