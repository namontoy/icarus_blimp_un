# Act on these this week

Two blocking items, both resolved by sending an email and asking your friend for files.
Everything else in this repo can wait a few days. These cannot.

---

## A. Email the organizers — today

The pre-registration deadline (14 Sep) has lapsed and the early-bird window closes **28 Sep**.
Send both emails below. Send them separately: the IAR chairs handle the category's technical
rules, the UMNG address handles registration and logistics.

### A1 → `larslarc@unimilitar.edu.co` (registration & logistics, in Spanish)

> **Asunto: Inscripción tardía — categoría IAR (International Airship Racing) — LARC 2026**
>
> Buenos días,
>
> Somos un equipo de la Universidad Militar Nueva Granada interesado en participar en la
> categoría **IAR (International Airship Racing)** de LARC 2026.
>
> Notamos que la fecha de preinscripción indicada en el sitio (14 de septiembre de 2026) ya
> pasó. Quisiéramos confirmar:
>
> 1. ¿Es aún posible inscribir un equipo en la categoría IAR? ¿Existe algún proceso de
>    inscripción tardía?
> 2. ¿Cuál es el procedimiento y el enlace de pago para la inscripción *early bird*
>    (100.000 COP) antes del 28 de septiembre?
> 3. **Confirmación de fechas:** la página `larc.html` indica del 2 al 6 de noviembre de 2026,
>    mientras que la página de contacto indica del 4 al 6 de noviembre. ¿Cuáles son las fechas
>    correctas? Necesitamos confirmarlo antes de reservar desplazamientos.
> 4. ¿Cuál es el formato, la extensión y el lugar de entrega del **Team Description Paper**
>    con fecha límite del 12 de octubre? ¿Existe una plantilla?
> 5. ¿Se exige además un video de clasificación?
> 6. ¿Cuántos integrantes puede tener un equipo en la categoría IAR? ¿Se requiere un tutor
>    o mentor?
> 7. **Helio:** ¿la organización suministra helio en el recinto, o cada equipo debe llevar su
>    propio cilindro? Si lo suministra la organización, ¿qué tipo de regulador y boquilla
>    debemos prever?
>
> Agradecemos su orientación.
>
> Cordialmente,
> [nombre] — Universidad Militar Nueva Granada

### A2 → `dantonio@oakland.edu`, cc `saldana@lehigh.edu` (category rules, in English)

> **Subject: IAR-LARC 2026 — rule clarifications before we commit to an envelope design**
>
> Dear Prof. D'Antonio, Prof. Saldaña,
>
> We are a team from Universidad Militar Nueva Granada preparing an entry for IAR-LARC 2026.
> We have read `IAR_LARC2026.pdf` (27 March 2026) carefully. A few points are load-bearing for
> our design and we would rather ask than guess.
>
> **1. Rule 12 vs Rule 14.** Rule 12 states there are no explicit size limitations; Rule 14
> gives a 1 m × 1 m × 1.5 m bounding box explicitly including the envelope. Which governs?
> We are designing to Rule 14 as the stricter reading.
>
> **2. Rule 10 — for your information, a geometric inconsistency.** Rule 10 caps helium volume at
> that of a 50-inch balloon and states this is "approximately 1 cubic meter". A true 50-inch
> sphere is 1.27 m across and 1.07 m³ — but it could not fit inside Rule 14's 1 m × 1 m
> footprint, and the commercial balloon linked in Rule 10 appears to inflate to roughly 0.17 m³,
> not 1 m³ (the MochiSwarm paper quotes 120 gf of net buoyancy for it, which is consistent with
> ~0.17 m³, not with 1 m³). We are treating Rule 14's bounding box as the binding constraint and
> Rule 10's "any balloon shape or configuration" as permitting a custom envelope. Please correct
> us if that is not the intent.
>
> **3. Timing and loops (Rules 4 and 5).** We read Rule 4 as meaning the clock starts only on the
> first waypoint *touch*, so the launch and approach to waypoint 1 are untimed; and that across
> the three loops of Rule 5, the repositioning between one loop's final touch and the next loop's
> first touch is likewise untimed. Is that correct?
>
> **4. Offboard computation (Rules 1 and 15).** We read this as: all sensors must be on the
> vehicle, but processing may happen on a ground station over a radio link. Is that correct?
> If so:
> - May we bring and operate our own 2.4 GHz access point or ESP-NOW ground node?
> - Are there channel restrictions or an RF coordination process at the venue?
>
> **5. Waypoint order (Rule 2).** How far before a run is the visiting order announced, and in
> what form is it given to the team?
>
> **6. Arena environment.** For vision and for thrust margin, could you tell us: ceiling height,
> whether HVAC/air conditioning will be running during runs, the lighting (daylight, fluorescent,
> LED), and the floor and wall colours? We ask because the waypoints are red balloons and we
> need to know what else in the hall is red.
>
> **7. Practice.** How much dedicated practice time does Rule 23 allocate per team, and on which
> days?
>
> **8. Baseline design.** The rulebook's Appendix A describes the baseline as using *coreless
> brushed* motors, while the BlimpSwarm documentation lists *brushless 1102 13500 kV* motors with
> ESCs. Which reflects the current reference platform?
>
> Thank you — we are looking forward to the first edition.
>
> Best regards,
> [name] — Universidad Militar Nueva Granada

---

## B. Ask your friend for the PCB package

The single number blocking the whole weight budget is **the mass of his populated board**.
Everything else follows from it.

**Minimum ask (one message):**

> Could you send:
> 1. The **schematic PDF** and **board/Gerber files** for the ESP32-S3 PCB.
> 2. The **populated board on a kitchen scale**, photographed — we need its mass in grams.
> 3. Photos of both sides of the board.
> 4. How many boards exist, assembled and bare?
> 5. What firmware, if any, has run on it, and has it ever driven a motor?

The full 19-question audit — module variant, PSRAM, power rails and decoupling, sensors and I2C
addresses, motor-driver topology, antenna keep-out, firmware branch — is in
[02-hardware.md §6](02-hardware.md). Send him that section once he's replied to the five above.

---

## C. The one experiment that removes the biggest unknown

**Buy one balloon and weigh its lift.** Cost: a few dollars and an afternoon.

There is a ~9× discrepancy between the rulebook's "50-inch balloon ≈ 1 m³" and the MochiSwarm
paper's "120 grams of net buoyancy" for the same balloon (see [the README](../README.md#what-50-inch-balloon-really-means)).
Separately, the envelope's film areal density is unknown within a factor of two (42 vs 85 g/m²),
which swings usable payload at Bogotá between ~200 g and ~88 g.

**Procedure:**
1. Weigh the empty balloon and valve. Record grams.
2. Inflate with helium until firm but not stretched. Measure the inflated diameter in three axes.
3. Tape on a light cup. Add small weights (coins, washers) until it neither rises nor sinks.
4. The added mass is the **net lift**. Record it, with the ambient temperature and your altitude.

Do this **before** ordering an envelope or finalising the electronics mass target. It converts
the largest guess in the project into a measurement, and it can be done the same day the
organizers reply.
