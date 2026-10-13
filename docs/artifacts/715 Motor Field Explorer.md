BLIMP – ESP32 · Motors, magnetometer and power

# 715 Motor Field Explorer

A coreless motor drawing current acts like a small magnetic dipole. Two motors were measured with their propellers: a 715 with a 35 mm four-blade propeller and a 720 with a 55 mm two-blade propeller. This page shows where their field would disturb the IMU's magnetometer, compares the two, and uses the measurements to size the BLIMP's buoyancy, motor operating point and battery.

The field plot and the BLIMP calculator use the selected motor. The comparison section always shows both.

Field line, below limit Field line, above limit |B| above limit Recommended distance

Click or drag on the plot to move the IMU. With the plot focused, arrow keys move it 0.5 cm. The plane shown contains the shaft and the winding dipole (copper arrow). Motor body drawn to scale.

### Motor orientation

### Motor and sensor parameters

Current, I

Torque constant, Kt

Choosing a motor sets its estimate. Measure the speed at one voltage to replace it.

Air-gap field, Bgap

Assumed. Typical NdFeB micro-motors: 0.3–0.5 T.

Allowed interference, Blim

0.1 µT ≈ resolution of a low-cost magnetometer.

Recommended distance

Thrust at this current

Winding dipole moment, m

Limit distance, equatorial

Limit distance, axial

---

IMU distance

|B| at the IMU

Share of Earth field (≈ 30 µT)

Worst-case heading error

## Measured motors, propellers mounted

Voltage at the motor terminals, thrust read on a scale, mass averaged over 10 readings. The m and rax columns use each motor's Kt estimate (715: 0.7, 720: 1.0 mNm/A) and the Bgap and Blim sliders above.

| Motor | Rotation | V \[V\] | I \[A\] | Thrust \[g\] | P = V·I \[W\] | Efficiency \[g/W\] | m \[A·m²\] | rax \[cm\] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |

### What the data show

**Thrust grows in a straight line with current** for both motors. 715: T ≈ 11.4 g/A · (I − 0.067 A), every point within 0.45 g. 720: T ≈ 9.6 g/A · I + 0.63 g, every point within 0.2 g.

**Reverse rotation (715) gives about 35 % less thrust** at the same current: 10.3 g against 15.7 g at 1.45 A. The motor works just as hard, but the cambered blades run backwards. The 720's reverse thrust has not been measured yet; the calculator assumes the same 0.65 ratio.

**Efficiency falls as current rises**, because copper loss grows as I²R while thrust grows only as I. Low throttle is where both motors are most efficient.

715 forward 715 reverse 715 reverse, 0.65 × forward 720 forward

## 715 or 720?

Same thrust, compared by interpolating each motor's own measurements. In a blimp the helium carries the weight, so the power needed for a given thrust matters more than the motor's mass.

| Thrust \[g\] | 715 + 35 mm × 4 | 720 + 55 mm × 2 | 720 / 715 power | rax 715 / 720 \[cm\] |
| --- | --- | --- | --- | --- |

### Analysis

**The 720 is much better at low thrust.** It needs a quarter to half the power at 2–4 g, which is where the BLIMP's vertical trim motors work. The bigger disc moves more air more slowly, which costs less power, and the motor runs at a much lower voltage.

**At high thrust they converge.** The power ratio rises to 0.84 at 15 g. Both reach about 15–16.5 g at 1.5 A, the DRV8833's per-bridge limit.

**The IMU distance is about the same in the trim range.** Up to about 4 g the 720 draws less current, which offsets its higher Kt: the distances agree within 8 %. From 5 g up the 720 needs 12–15 % more distance.

**The extra mass is negligible.** Four 720s weigh 2.0 g more than four 715s, about 2.4 L more helium.

715, measured 720, measured

| Property | 715 | 720 |
| --- | --- | --- |
| Mass with propeller | 3.1 g | 3.6 g |
| Propeller | 35 mm, 4 blades | 55 mm, 2 blades |
| Maximum tested | 4.05 V · 1.51 A · 16.5 g | 3.00 V · 1.495 A · 15.0 g |
| Reverse thrust | Measured: 0.65 × forward | Not measured |
| Flight time, calculator settings below |  |  |
| Thrust response | Faster (small propeller) | Slower (more propeller inertia) |

**Recommendation: the 720 with the 55 mm two-blade propeller.** It needs a quarter to two thirds of the 715's power in the BLIMP's working range of 2–6 g, which lengthens flight time, for 2 g of extra mass and about the same magnetometer distance at trim thrust.

Before committing, measure one reverse point for the 720, and retune the controller gains: the larger propeller responds more slowly.

## BLIMP design calculator

Uses the motor selected at the top and the LARC 2026 airship rules: helium only, ≤ about 1 m³, neutral buoyancy within ±10 gf, total thrust ≤ 100 gf, size ≤ 1 × 1 × 1.5 m. Values marked “estimate” are starting points to replace with your own measurements.

### Buoyancy

Electronics and battery, without motors

Measured 62 g with four 715s, minus 4 × 3.1 g.

Envelope and frame

Estimate. Weigh the empty envelope and frame.

Altitude

Air temperature

Helium purity

Party-balloon helium is often 80–95 %.

---

Lift per litre

Four motors with propellers

Total mass

Helium needed

Equivalent sphere diameter

### Vertical motors

Residual weight after trimming

\+ heavy (motors push up), − light (motors push down). The rules allow ±10 g.

Vertical motors

Of the four motors; the rest are horizontal.

### Propeller's strong direction

---

Thrust per vertical motor

Rotation used

Current per motor

Motor voltage

Recommended IMU distance

### Battery and power

2S battery voltage

8.4 V full, 7.4 V nominal, about 7.0 V near empty.

Cruise thrust per horizontal motor

Electronics power

Estimate. ESP32-CAM streaming dominates. Measure the idle battery current.

---

Maximum PWM duty

Duty, vertical / horizontal

Motor power, total

Battery current

Flight time (80 % of 2.2 Wh)

Field of battery lead at 5 cm

## Why a few centimetres matter so much

A dipole field falls as 1/r³. On a log-log plot that is a straight line with slope −3: every time the distance doubles, the field drops by a factor of 8.

The axial curve sits exactly twice as high as the equatorial one. That factor of 2 in field becomes a factor of only 21/3 ≈ 1.26 in distance, which is why one safe radius covers every orientation.

The same cube root explains the current slider: doubling I doubles m, but moves the safe distance out by only 26 %.

The dot marks your IMU. Measuring a few such points on the real BLIMP and checking the −3 slope is the simplest way to validate the model.

## Equations used on this page

### Dipole moment from Kt

m ≈ Kt · I / Bgap

Torque on a current loop is τ = m·B, and the motor torque is τ = Kt·I. Kt already contains the turns and geometry. The 720's estimate is 1.4 × the 715's: at equal thrust a propeller's torque scales with its diameter.

### Dipole field and limit distance

|B| = (µ0/4π)·m·√(1 + 3cos²θ) / r³

θ = 90° (equatorial) gives the minimum, θ = 0° (axial) twice that. The recommended distance is the axial one: rax = 21/3·(µ0m / 4πBlim)1/3.

### Measured fits

715: T ≈ 11.4·(I − 0.067), V ≈ 0.75·I + 2.40·√(I − 0.042)

720: T ≈ 9.6·I + 0.63, V ≈ 0.96·I + 1.29·√(I − 0.024)

T in g, I in A, V in V. Valid within each motor's measured range; the calculator interpolates the measured points directly.

### Helium lift

lift = x·(ρair − ρHe), ρ = p / (Rs·T)

x is the helium purity. Pressure from the standard atmosphere, p = 101 325·(1 − 2.256×10⁻⁵·h)5.256 Pa. At Medellín pure helium lifts about 0.87 g/L.

### PWM from a 2S battery

Vmotor ≈ D · Vbat, Ibat ≈ ΣP / Vbat

Scaling the duty D with the measured Vbat keeps the same command at the same thrust as the battery drains. Driver losses are ignored.

### Heading error and wire field

Δψ ≈ arctan(ΔB / Bh), Bwire = µ0I / 2πr

0.1 µT against 30 µT gives about 0.2°. A single straight lead with 1 A gives 4 µT at 5 cm.

## Design notes and what the model leaves out

- **Battery leads and power traces.** The supply current to all motors flows through them, up to about 3 A at full power. Twist the battery pair and keep the main supply path away from the GY-88.
- **Compassmot with signed current.** In reverse rotation the current and the dipole flip sign but keep their size. A linear correction K·I fitted with signed duty (+ forward, − reverse) handles both directions.
- **The permanent magnet.** Its moment is about 300 times the winding's, but it is static and fixed to the frame, so hard-iron calibration removes it. It must only stay below the sensor's full scale.
- **Several motors.** Each motor adds its own dipole. Fields add as vectors, so the worst case is the sum of magnitudes.
- **The near field.** Closer than about 3 cm the point-dipole model underestimates the real field.
- **Thrust stand and flight.** Thrust was read with the propeller close to the scale. Confirm the trim current with the motor in its final mounting on the BLIMP.
- **Still to measure.** The 720's reverse thrust, speed at one voltage to fix each Kt, the envelope mass, and the electronics' idle current.

Motor V–I, thrust and mass measured on a real 715 (35 mm four-blade propeller) and 720 (55 mm two-blade propeller), 2026-10-09. Rules: IAR LARC 2026 airship rules. Reference data: [Orbray CMS07-17](https://orbray.com/en/product/dccorelessmotor/brushed_ca_cms07-17.html) and a [generic 716 motor](https://scccltd.com/products/716-coreless-dc-motor-3-5v). Bgap, Kt and magnet size are estimates. BLIMP – ESP32 project, Universidad Nacional de Colombia, Sede Medellín.