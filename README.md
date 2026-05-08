
# MPPT Solar Charge Controller (Arduino + ESP32 + TinyML)

<div align="center">

![Status](https://img.shields.io/badge/Status-Phase%201%20Complete-brightgreen)
![Phase 2](https://img.shields.io/badge/Phase%202-In%20Progress-yellow)
![Platform](https://img.shields.io/badge/Platform-Arduino%20Nano%20%2B%20ESP32-blue)
![License](https://img.shields.io/badge/License-MIT-purple)
![SDG](https://img.shields.io/badge/SDG-7%20%7C%209%20%7C%2013-orange)

**A low-cost solar MPPT charge controller built using a custom buck converter and the Perturb & Observe (P&O) algorithm, with ongoing research into replacing conventional MPPT control using TinyML-based prediction..**

[Hardware](#hardware) · [Circuit Design](#circuit-design) · [Firmware](#firmware) · [Results](#results) · [Roadmap](#roadmap) · [Getting Started](#getting-started)

</div>

---

## What this project is

This project implements a low-cost solar Maximum Power Point Tracking (MPPT) charge controller using a custom buck converter controlled by an Arduino Nano.

Phase 1 (completed): A fully functional MPPT system using the Perturb & Observe (P&O) algorithm, real-time LCD monitoring, and serial CSV data logging. The system has been tested on real hardware using a 5W, 12V solar panel charging a 3S Li-ion battery pack.

Phase 2 (in progress): Development of a TinyML-based MPPT controller using an ESP32 co-processor. The goal is to replace the conventional P&O algorithm with a lightweight neural network trained on real operating data to improve tracking efficiency and response time.

This project combines power electronics, embedded systems, and TinyML for low-cost renewable energy applications.

---


---

## Hardware

### Components

| Component | Value / Part | Role |
|-----------|-------------|------|
| Microcontroller | Arduino Nano (ATmega328) | MPPT algorithm, PWM, sensing 
| Co-processor | ESP32 DevKit | TFLite inference, WiFi dashboard 
| Current sensor | INA219 module (I2C) | Panel current measurement  
| MOSFET | IRLZ44N (55V/47A, logic-level) | Buck converter switch  
| Inductor | 100µH / 3A | Buck converter energy storage 
| Freewheeling diode | 1N5822 Schottky (40V/3A) | Inductor current path when MOSFET off
| Output capacitor | 100µF / 50V electrolytic | Output ripple smoothing
| Input capacitors | 100µF + 100nF | Panel voltage stabilisation
| Voltage divider | R1=18kΩ, R2=3.9kΩ | Scale 21.6V → 4.09V for ADC
| Display | 16×2 LCD with I2C adapter | Live V/I/P/duty readout 
| Solar panel | 5W 12V polycrystalline | Power source (Voc=21.6V, Vmpp=17.2V)  
| Battery | 3S Li-ion (3 × 3.7V = 11.1V nom) | Load / energy storage 
| Misc | Breadboard, terminals, wire, solder | Assembly 


### Key design decisions

**Why IRLZ44N instead of IRF540N?** The IRLZ44N is a logic-level MOSFET — its gate turns fully ON at 5V, so the Arduino drives it directly. No IR2104 gate driver IC needed, saving ₹120 and significant wiring complexity. At our 0.3A operating current the IRLZ44N runs completely cold (Rds_on = 22mΩ vs effective ~200mΩ when IRF540N is partially driven at 5V).

**Why two input capacitors?** The 100µF electrolytic handles bulk energy storage, supplying burst current during each 50kHz switch-on. The 100nF ceramic kills high-frequency spikes the electrolytic is too slow to catch (due to internal inductance). Without these, the 50kHz switching noise corrupts INA219 readings and the MPPT algorithm tracks garbage.

**Why 0.5Ω shunt instead of 0.1Ω?** The standard 0.1Ω precision shunt is hard to source locally. Two 1Ω carbon film resistors in parallel give 0.5Ω — at 0.29A this produces a 145mV drop, actually cleaner for the INA219 to read than 29mV would be. The INA219 calibration is corrected in firmware.

**Why R1=18kΩ, R2=3.9kΩ for the voltage divider?** The panel Voc can reach 23V worst case. The original 10kΩ+4.7kΩ divider would output 6.9V — exceeding the Arduino's 5V ADC limit and damaging the pin. 18kΩ+3.9kΩ scales 23V to 4.09V, safe under all conditions.

---

## Circuit design

### Buck converter operating principle

The buck converter steps down panel voltage (17.2V at MPP) to battery voltage (11.1V nominal) while preserving power with ~90% efficiency. The core relationship is:

```
Vout = D × Vin

where D = duty cycle (0–1)
D = 11.1 / 17.2 = 0.645  →  64.5% starting duty cycle
```

This is derived from volt-second balance on the inductor: in steady state, the flux must return to the same value each cycle, which requires the volt-seconds during ON time to equal the volt-seconds during OFF time.

### Component sizing

```
Switching frequency:  fsw = 50 kHz  (Timer1 reconfigured from default 490Hz)
Duty cycle range:     20% – 90%     (safety clamp in firmware)
Min inductance:       L = (Vin-Vout)×D / (fsw×ΔIL) = 68µH  →  use 100µH ✓
Min capacitance:      C = ΔIL / (8×fsw×ΔVout) = 4µF        →  use 100µF ✓
MOSFET Vds rating:    ≥ 1.5 × Voc = 34.5V  →  IRLZ44N 55V ✓
Diode reverse V:      ≥ 1.5 × Voc = 34.5V  →  1N5822 40V  ✓
```


---

## Firmware

### Arduino Nano (Phase 1)

The firmware is structured in non-blocking layers — all tasks use `millis()` timers, no `delay()` calls, so they interleave without blocking each other.

```
loop()
  ├── mpptStep()      every 100ms  — P&O algorithm, updates PWM duty
  ├── uartLog()       every 250ms  — CSV row to serial (for ESP32 / logging)
  └── updateDisplay() every 600ms  — LCD refresh
```

**P&O algorithm core (10 lines of real logic):**

```cpp
float dP = panelPower - prevPower;
int   dD = dutyCycle  - prevDuty;

if (abs(dP) > 0.005) {       // ignore changes below 5mW noise floor
    if (dP > 0)
        direction = (dD >= 0) ? 1 : -1;   // power up → keep direction
    else
        direction = (dD >= 0) ? -1 : 1;   // power down → reverse
}
setDuty(dutyCycle + direction * PERTURB_STEP);  // PERTURB_STEP = 2
```

**Timer1 reconfiguration for 50kHz:**

```cpp
// Default Arduino PWM is 490Hz — needs a 20mH inductor (impractical)
// 50kHz allows 100µH inductor — small, cheap, available
TCCR1A = _BV(COM1A1) | _BV(WGM11);
TCCR1B = _BV(WGM13)  | _BV(WGM12) | _BV(CS10);
ICR1   = 319;   // 16MHz / 320 = 50kHz
```

**Battery protection:**

```cpp
const float BATT_FULL = 12.6;  // 3S Li-ion full charge
const float BATT_LOW  = 9.0;   // deep discharge protection

if (battVoltage >= BATT_FULL) { setDuty(DUTY_MIN); return; }  // stop charging
if (panelVoltage < 10.0)      { setDuty(DUTY_MIN); return; }  // no panel
```

### Libraries required

```
Adafruit INA219         — install via Arduino Library Manager
LiquidCrystal I2C       — by Frank de Brabander, Library Manager
```

---

## Results

> Measured values from Phase 1 testing. Panel: 5W 12V polycrystalline. Load: 3S Li-ion battery. Environment: outdoor noon sunlight, Delhi, ~900 W/m² irradiance.

| Metric | Value |
|--------|-------|
| Panel voltage at MPP | 17.1 V |
| Panel current at MPP | 0.28 A |
| Peak power measured | 4.79 W |
| P&O MPPT efficiency | ~87% |
| Direct connection efficiency (baseline) | ~68% |
| Efficiency improvement over direct | **+19%** |
| Buck converter efficiency | ~91% |
| Duty cycle at MPP | 64.9% |
| INA219 current resolution (0.5Ω shunt) | 0.1 mA |
| Switching frequency | 50 kHz |
| Total component cost | ₹2,827 |

> **Note:** MPPT efficiency = P_operating / P_mpp × 100%. Measured by sweeping duty cycle manually to find true P_mpp, then comparing to P&O tracking value.

### LCD display output (live)

```
V:17.1 I:0.28A
P:4.79W D:64.9%
```

### Serial CSV output (for data logging / ESP32)

```csv
time_ms,volt_V,curr_A,power_W,duty_pct,batt_V,source
1024,17.134,0.2801,4.799,64.90,11.24,PO
1274,17.128,0.2798,4.791,65.10,11.24,PO
1524,17.141,0.2804,4.806,64.90,11.25,PO
```

---

## Roadmap

### Phase 1 — P&O MPPT ✅ Complete

- [x] Buck converter hardware (IRLZ44N + 100µH + 1N5822 + 100µF)
- [x] INA219 current sensing with 0.5Ω shunt correction
- [x] Voltage divider (18kΩ + 3.9kΩ) with 5V ADC safety margin
- [x] 50kHz PWM via Timer1 reconfiguration
- [x] Non-blocking P&O MPPT algorithm (100ms loop)
- [x] Input capacitors (100µF + 100nF) for noise suppression
- [x] 16×2 LCD I2C display with live V/I/P/duty readout
- [x] 3S Li-ion battery protection (12.6V cutoff, 9.0V low)
- [x] CSV data logging over serial (training data for Phase 2)
- [x] Transfer from breadboard to perf board

### Phase 2 — TinyML Integration 🔄 In Progress

- [ ] Collect 10,000+ rows of varied training data (different weather, times of day)
- [ ] Train MLP model in Python / Google Colab (2 inputs: V, I → 1 output: duty%)
- [ ] Convert to TFLite int8 quantised model using TFLite converter
- [ ] Deploy TFLite Micro on ESP32 (520KB RAM — sufficient for small MLP)
- [ ] UART communication: Nano → ESP32 (sensor CSV), ESP32 → Nano (duty float)
- [ ] Level shift circuit: Nano TX 5V → ESP32 RX 3.3V (10kΩ + 4.7kΩ divider)
- [ ] A/B efficiency comparison: P&O vs TinyML under same conditions
- [ ] Live WiFi dashboard (ESP32 AP mode, accessible from any browser)

### Phase 3 — Possible future upgrades

- [ ] Custom PCB design (EasyEDA → JLCPCB fabrication)
- [ ] Temperature compensation for battery charging (NTC thermistor)
- [ ] Partial shading detection and global MPP tracking
- [ ] Bluetooth BLE logging to mobile app
- [ ] Over-the-air (OTA) firmware updates via ESP32
- [ ] Current sensing on battery side (second INA219) for SoC estimation
- [ ] Publish efficiency comparison results as a paper (IJERT / IRJET)

---

## Getting Started

### Hardware assembly order

> Build and verify in this exact order. Never solder until the previous stage is breadboard-verified.

**Stage 1 — Sensing (verify on breadboard first)**
1. Connect INA219: VCC→5V, GND→GND, SDA→A4, SCL→A5
2. Place 0.5Ω shunt (two 1Ω in parallel) in series on panel+ line
3. Connect INA219 Vin+ before shunt, Vin- after shunt (Node A)
4. Connect voltage divider: R1=18kΩ from Node A, R2=3.9kΩ to GND, midpoint→A0
5. Add 100nF ceramic cap from A0 midpoint to GND (noise filter)
6. Upload sensor test sketch, verify readings on Serial Monitor

**Stage 2 — Display**
1. Connect LCD I2C adapter: VCC→5V, GND→GND, SDA→A4, SCL→A5
2. If LCD address is 0x3F change `lcd(0x27,...)` to `lcd(0x3F,...)`
3. Verify LCD shows test message on startup

**Stage 3 — Power stage (add last, after sensing verified)**
1. Connect IRLZ44N: Gate→D9 via 10Ω, Drain→Node A, Source→inductor pin 1
2. Connect 100µH inductor: pin 1→MOSFET Source, pin 2→Node B
3. Connect 1N5822: cathode→Node B, anode→GND (stripe = cathode)
4. Connect 100µF cap: positive→Node B, negative→GND (check polarity!)
5. Connect input caps: 100µF and 100nF both from Node A to GND
6. Verify no shorts with multimeter before connecting panel
7. Connect panel and battery, verify LCD shows voltage and duty cycling

### Software setup

```bash
# Clone repository
git clone https://github.com/YOUR_USERNAME/mppt-solar-controller.git
cd mppt-solar-controller

# Open in Arduino IDE
# File → Open → nano_main/nano_main.ino
```

Install libraries via Arduino Library Manager:
- `Adafruit INA219`
- `LiquidCrystal I2C` (by Frank de Brabander)

Update these constants in `nano_main.ino` to match your components:

```cpp
// Voltage divider — change if using different resistors
const float R1 = 18000.0;   // top resistor in ohms
const float R2 = 3900.0;    // bottom resistor in ohms

// Shunt — change if using different resistance
const float SHUNT_OHMS = 0.5;  // 0.5 for 2×1Ω parallel, 1.0 for single 1Ω

// Battery — change for your chemistry
const float BATT_FULL = 12.6;  // 3S Li-ion | use 14.4 for 12V SLA
const float BATT_LOW  = 9.0;   // 3S Li-ion | use 11.0 for 12V SLA

// LCD address — try 0x3F if display blank
LiquidCrystal_I2C lcd(0x27, 16, 2);
```

### Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| INA219 not found on startup | Wrong I2C address or wiring | Check SDA/SCL, try address 0x41 |
| LCD blank | Wrong I2C address | Change 0x27 to 0x3F in code |
| Voltage reads ~5× too high | Wrong DIVIDER_RATIO | Recalculate: (R1+R2)/R2 |
| Current reads ~5× too high | SHUNT_CORRECTION wrong | Set SHUNT_OHMS = your actual resistance |
| Duty cycle oscillates wildly | Noise on INA219 readings | Add input caps, check power loop layout |
| MOSFET gets hot | No heat sink or poor gate drive | Clip-on TO-220 heat sink, verify 5V on gate |
| Battery not charging | Duty cycle too low or cutoff triggered | Check BATT_FULL constant vs actual battery V |

---

## Repository structure

```
mppt-solar-controller/
│
├── nano_main/
│   └── nano_main.ino          # Arduino Nano firmware (Phase 1 complete)
│
├── esp32_main/
│   ├── esp32_main.ino         # ESP32 firmware (Phase 2 in progress)
│   └── model_data.h           # TFLite model placeholder
│
├── training/
│   └── train_model.py         # Python script to train TinyML model
│
├── docs/
│   ├── circuit_breadboard.png # Breadboard layout diagram
│   ├── circuit_perfboard.png  # Perf board layout diagram
│   └── schematic.png          # Full circuit schematic
│
├── data/
│   └── sample_log.csv         # Sample logged data from Phase 1
│
└── README.md
```

---

## Theory background

This project covers several interconnected concepts in power electronics and embedded ML. Brief explanations for each:

**Why MPPT?** A solar panel's output is not fixed. Its I-V curve shifts with irradiance and temperature, and the maximum power point moves continuously. A fixed resistive load will only hit the MPP by coincidence. MPPT dynamically adjusts the load impedance to always extract peak power.

**Why a buck converter?** The battery (11.1V) is at a different voltage than the panel MPP (17.2V). Connecting directly forces the panel to operate at battery voltage — away from its MPP. The buck converter decouples the two sides, letting the panel run at 17.2V while the battery sees 11.1V. Efficiency is ~90% vs ~0% for a linear regulator.

**Why TinyML over P&O?** P&O hill-climbs blindly — it perturbs, measures, and reverses if power drops. It never truly settles (±1 step oscillation around MPP) and gets confused during rapid irradiance changes. A neural network trained on historical data predicts the optimal duty cycle directly — no oscillation, instant response to changing conditions.

**Why dual-chip (Nano + ESP32)?** The Nano handles real-time PWM control — a 100ms deterministic loop that must never stall. The ESP32 runs WiFi and TFLite inference, both of which can take unpredictable amounts of time. Mixing them on one chip risks the WiFi stack stalling the PWM loop. Separation gives each chip one job it does reliably.

---

## SDG alignment

This project contributes to three UN Sustainable Development Goals:

**SDG 7 — Affordable and Clean Energy:** MPPT extracts ~19% more power from the same panel vs direct connection. At under ₹3,000, this controller makes efficient solar charging accessible where ₹5,000+ commercial units are not.

**SDG 13 — Climate Action:** A 5W panel at 87% MPPT efficiency vs 68% baseline generates ~12 Wh extra per day. At India's grid carbon intensity (0.71 kg CO2/kWh), this avoids ~3 kg CO2 per panel per year.

**SDG 9 — Industry, Innovation and Infrastructure:** Combining embedded ML with power electronics at sub-₹3,000 cost demonstrates frugal innovation for resource-constrained deployment contexts.

---

## About

Built as a college electronics project (2nd year, B.E. Electronics). Phase 1 implemented and tested on real hardware. Phase 2 (TinyML) actively in development.

**Skills demonstrated:** Power electronics design, embedded C++ firmware, I2C/UART protocols, analog sensing, PCB layout (perf board), Python ML pipeline, TensorFlow Lite, system architecture.

**Open to:** Internship opportunities in embedded systems, power electronics, IoT, or renewable energy. Feel free to reach out or open an issue if you have questions about the design.

---

## License

MIT License — see [LICENSE](LICENSE) for details. Use freely, attribution appreciated.

---

<div align="center">
<sub>If this helped you, a ⭐ on the repo is appreciated.</sub>
</div>
