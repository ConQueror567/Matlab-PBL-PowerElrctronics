# Design review – Intelligent Multi-Zone Classroom Lighting System

This review checks the MATLAB/Simulink model and the hardware in the block diagram.
Part A lists problems found in the first version of the model, which are now fixed.
Part B lists hardware changes needed before you build the circuit.

## A. Model problems found and fixed

| # | Problem | Effect | Fix |
|---|---|---|---|
| A1 | The controller assumed the LDR measures **daylight only**. In the room, each LDR also sees its own zone's LEDs. | In real hardware the lights would flicker or oscillate: LEDs on → LDR reads bright → LEDs off → repeat. A minimum-duty rule also left Zone 1 at 480 lux (below target) at 10:00. | Closed-loop (integral) daylight harvesting in `esp32_controller.m`. The plant now feeds the LED light back into the LDR. All occupied zones settle at 498–502 lux. |
| A2 | The buck voltage controller was modelled as **continuous**. The ESP32 is digital: it samples the ADC once per PWM period and applies the new duty one period later. | With that delay, the 3 kHz loop design has only **1.5–6 dB gain margin**, and it is **unstable** if sampled every second period. | The controller is now discrete (one sample per period plus a one-period delay). It is re-tuned to a 1.5 kHz crossover, giving about 8–12 dB gain margin and 35–60° phase margin. |
| A3 | No **ESR** on the output capacitor. | Output ripple was underestimated: 25–77 mV in the model versus about 75–155 mV in reality. | ESR = 30 mΩ added (2 × 470 µF low-ESR). |
| A4 | The parameter file suggested an IRLZ44N as the buck switch. | That is an N-channel MOSFET. It cannot be driven as the **high-side** switch of a buck directly from an ESP32 (see B1). | Comment corrected. |
| A5 | In the `'interpreted'` Simulink build, the controller's memory carried over between runs. | The second run started with old duty values. | The model's InitFcn now runs `clear esp32_controller`. |

Results after the fixes (plain-MATLAB run): 5.01 V rail, 42–155 mV ripple, 500 ± 2 lux in every occupied zone, converter efficiency 93–97%. The saving is still 61% on LED energy and 50% from the 12 V input.

## B. Hardware: components to replace or add

### Critical (the circuit will not work or will be damaged without these)

| # | Issue | What to do |
|---|---|---|
| B1 | **High-side MOSFET gate drive.** In a buck converter the switch sits between +12 V and the inductor. An N-channel MOSFET (IRLZ44N, IRFZ44N) needs its gate about 10 V *above its source*, so the source would swing to 12 V. The ESP32's 3.3 V output cannot drive it. | **Option 1:** P-channel MOSFET (e.g. AOD4185 or IRF9540N) with a small NPN/PNP gate driver. Add a gate pull-up to +12 V so it is off during ESP32 boot. **Option 2:** N-channel MOSFET with a bootstrap driver (IR2104/IR2184). Keep the 90% maximum duty, because the bootstrap needs off-time. |
| B2 | **ESP32 is powered from the rail it controls.** At power-up the 5 V rail is 0 V, so the ESP32 can't start, so no PWM, so no 5 V (a chicken-and-egg problem). | Power the ESP32 from a **separate small regulator** from 12 V, e.g. an MP1584 or LM2596 buck module. Do not use an AMS1117 from 12 V: it would dissipate about 2 W and overheat. |
| B3 | **No overvoltage protection.** If the firmware hangs with the MOSFET on, or the MOSFET fails shorted, the "5 V" rail rises to 12 V and destroys the ESP32 and LED strips. | Add a fuse (5 A) on the 12 V input, a TVS diode (SMBJ5.0A) on the 5 V rail, the ESP32 watchdog, and a pull resistor that holds the buck MOSFET **off** while the ESP32 resets. |
| B4 | **No input capacitor** in the block diagram. The MOSFET draws 0 ↔ 3.7 A pulses at 50 kHz from the adapter. | Add 470 µF / 25 V electrolytic + 10 µF ceramic, placed right at the MOSFET. |
| B5 | **Freewheeling diode type.** A standard rectifier such as the 1N4007 is far too slow at 50 kHz and will overheat. | Use a Schottky: **SS54** (5 A, 40 V) recommended. SS34 (3 A) is at its limit, since average diode current ≈ 1.8 A. |

### Important (it would work, but badly or unreliably)

| # | Issue | What to do |
|---|---|---|
| B6 | **Inductor saturation.** The simulation shows a **4.5 A peak** at start-up, and 3.7 A peak at full load. | 68 µH, **saturation current ≥ 5 A**, DCR ≤ 50 mΩ (e.g. a wound toroid). In firmware, keep the LED zones off until the 5 V rail is up. |
| B7 | **Output capacitor.** | 2 × 470 µF / 16 V **low-ESR** (≤ 60 mΩ each, ripple rating ≥ 1 A) + 10 µF ceramic. Standard electrolytics with 0.2–0.5 Ω ESR would give about 0.5 V ripple. |
| B8 | **Zone MOSFETs driven at 3.3 V.** The IRLZ44N/IRFZ44N is specified at 4–5 V (the IRFZ44N at 10 V). At 3.3 V it is only partly on and heats up. | Use a MOSFET specified at 2.5 V, e.g. **AO3400A**, or IRLB8721. Add a 100 Ω gate resistor and a **10 kΩ gate pull-down**, so the LEDs stay off during ESP32 boot. |
| B9 | **The ESP32 PID must run 50,000 times per second** (one ADC read per PWM period). That is at the edge of what ESP32 firmware can do. | Use the **MCPWM** peripheral for the buck PWM and LEDC for the zones. Read the ADC in a timer interrupt, and keep the PID in integer maths. If the loop can't keep up, sample every 2nd period: set `Ts_v = 2/fsw` and `f_c = 1e3` in `lighting_params.m` (the loop analysis shows this stays stable). Alternative: a TL494/UC3843 analog controller does the regulation, and the ESP32 only enables it. |
| B10 | **Voltage feedback to the ADC.** The ESP32 ADC is nonlinear near 0 V and 3.3 V. | Use a divider of 10 kΩ / 6.8 kΩ (5 V → 2.0 V) with 100 nF to ground, and the ADC calibration API. |
| B11 | **PIR HC-SR501 detects motion, not presence.** Students sitting still "disappear", and the lights go off during class. | Set the PIR hold time to 5–15 min, or use an **LD2410 mmWave** presence sensor. |
| B12 | **LDR accuracy.** An LDR is nonlinear and temperature dependent, so "500 lux" is only approximate. | Calibrate it against a lux meter, or replace it with a **BH1750** digital lux sensor (I²C). Average many ADC readings, because the LDR also sees the 5 kHz LED flicker. |
| B13 | **Energy analytics needs a current measurement.** The model calculates energy exactly, but the real ESP32 has no current sensor. | Add an **INA219/INA226** on the 12 V input (and optionally the 5 V rail). |

### Good to know

* **DHT11:** OK for monitoring (±2 °C, one reading per second). It measures the room, not the LEDs. For LED thermal protection, put an NTC thermistor on the LED strip. A DHT22 or SHT31 is more accurate.
* **Scale:** 3 zones × 5 W makes this a **table-top prototype**. Each zone lights about 1 m² to 500 lux. A real classroom (about 60 m²) needs roughly 300–400 W of LEDs, which means 24/48 V constant-current LED drivers and a power stage about 10× larger. Say so in the report.
* **5 V LED strips:** keep each zone's strip short (below about 1 m at 1 A) or feed it from both ends, otherwise the voltage drop makes the far end dimmer.

## C. Remaining model limitations

* Only conduction losses are modelled (Rds_on, DCR, diode drop, ESR). Switching and gate-drive losses are not, so real efficiency will be a few % lower (about 88–93%).
* The 12 V adapter is ideal (no cable inductance or adapter current limit).
* The Simulink build script was checked for correct wiring (106 blocks and 133 connections, every port connected once). It has not been run in real MATLAB. The plain-MATLAB script was run end-to-end (in GNU Octave).
