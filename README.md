# Intelligent Multi-Zone Classroom Lighting System – MATLAB / Simulink model

Power electronics PBL. A switching-level model of the full system:

```
LDR / PIR / DHT11  ->  ESP32 (decision engine + PWM)  ->  12 V -> 5 V custom MOSFET buck converter
                                   ^   5 V feedback               |
                                   |                              +--> MOSFET -> LED strip Zone 1 (Front)
                                   |                              +--> MOSFET -> LED strip Zone 2 (Middle)
                                   |                              +--> MOSFET -> LED strip Zone 3 (Back)
                                   '-> Monitoring + energy analytics -> dashboard figures
```

## Quick start

1. Put all the files in one folder and set it as MATLAB's **Current Folder**.
2. Pick one:

| Option | Needs | Command |
|---|---|---|
| **A. Simulink model** | MATLAB + Simulink | `build_lighting_model` → `ClassroomLighting.slx` opens → press **Run** |
| **B. Plain MATLAB script** | MATLAB only | `run_lighting_simulation` |

Both options run the same model with the same parameters and open the same 6 dashboard figures, plus an hourly report in the Command Window.

* The build script creates `ClassroomLighting.slx` once. After that you can open the `.slx` directly and press Run.
* The Simulink run takes about 0.5–2 minutes, because it computes 1.1 million steps of 0.2 µs. The figures open automatically when it finishes.
* If Simulink complains about a missing **C compiler** for the MATLAB Function block, rebuild with
  `build_lighting_model('interpreted')`. That version needs no compiler.

## Files

| File | What it is |
|---|---|
| `lighting_params.m` | **All parameters**: buck design, PID gains, LED zones, classroom-day sensor scenario |
| `esp32_controller.m` | ESP32 decision rules: closed-loop daylight harvesting + occupancy + thermal derating (stand-in for the ML model) |
| `controller_config.m` | Controller settings: target 500 lux, thresholds |
| `build_lighting_model.m` | Builds the Simulink model `ClassroomLighting.slx` |
| `run_lighting_simulation.m` | Same simulation as a plain MATLAB script |
| `lighting_dashboard.m` | Feedback/communication layer: hourly report, energy analytics, plots |
| `DESIGN_REVIEW.md` | **Read before building the hardware**: issues found, components to replace or add |
| `esp32_controller_vec.m` | Wrapper used only by the `'interpreted'` build |

## Simulink model structure (`ClassroomLighting.slx`)

The top level matches the block diagram:

* **Input Layer (Sensors)**: LDR per zone (daylight + the zone's own LED light, ADC-averaged), PIR (occupancy per zone), DHT11 temperature and humidity. They are read every 1 ms.
* **Processing Layer (ESP32)**
  * *AI Decision Engine* (MATLAB Function → `esp32_controller.m`) outputs the duty of each zone.
  * Digital buck voltage controller: ADC sample once per PWM period → soft-start → feed-forward `Vref/Vin` + PID (filtered D) → one-period computation delay → 50 kHz PWM.
  * Zone PWM: three 5 kHz PWMs, phase shifted 120° (interleaved).
* **Power & Lighting Layer**
  * *Buck Converter 12V-5V*: MOSFET + Schottky diode + L + C, with conduction losses (Rds_on, DCR, diode drop) and capacitor ESR. Includes DCM at light load.
  * *Zone k MOSFET + LED strip*: low-side MOSFET switch and an LED strip model (knee voltage + series resistance).
* **Feedback Layer (Monitoring + Energy)**: scopes, energy meters (input, each zone, ESP32) and the `sim_log` data log.
* **Communication Layer**: the dashboard figures (`lighting_dashboard.m`, model StopFcn).

## Design values

| Item | Value |
|---|---|
| Input / output | 12 V → 5 V, up to 3.25 A (3 zones × 1 A + ESP32 0.25 A) |
| Switching frequency | 50 kHz |
| Duty cycle | D = Vo/Vin = 0.417 ideal, ≈ 0.44 with losses |
| Inductor | 68 µH → ΔI_L = Vo(1−D)/(f·L) ≈ 0.86 A p-p |
| Capacitor | 1000 µF (2 × 470 µF low-ESR, 30 mΩ total) → LC resonance ≈ 610 Hz |
| Voltage controller | Digital PID sampled at 50 kHz, zeros on the LC resonance, crossover 1.5 kHz (≈ 8–12 dB gain margin) |
| LED dimming PWM | 5 kHz, 3 zones interleaved |
| Simulation | 1 school hour = 25 ms of circuit time, 9 hours (08:00–16:00), step 0.2 µs |

## Results (from the plain-MATLAB run)

* 5 V rail regulated at 5.01 V every hour. Ripple is 42–155 mV (≤ 3%), mainly from the ESR and the 5 kHz LED PWM load steps.
* Every occupied zone is held at 500 ± 2 lux. Daylight is used first, and an empty zone is switched off.
* Converter efficiency is 93–97%. Only conduction losses are modelled, so switching losses would lower this somewhat.
* Energy per day: 52 Wh for LEDs versus 135 Wh for conventional always-on lighting, a **61% saving**. Counting the ESP32 and converter losses from the 12 V input, the saving is **50%**.

## Things to try

* Change the sensor scenario (cloudy day, empty classroom) in `lighting_params.m`.
* Change `L_buck`, `C_buck` or `fsw`, then check the ripple and the startup in figures 3 and 4.
* Set `zone_phase = [0 0 0]` to turn off interleaving and see the 5 V rail ripple grow. In Simulink, rebuild the model after this change.
* Replace the rules in `esp32_controller.m` with your trained ML model. Keep the same inputs and outputs.
