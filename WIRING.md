# Wiring Diagram and Hardware Setup

## Component Overview

This document provides detailed wiring instructions for connecting all components to the ESP32.

## Power Supply Requirements

### Power Budget
- ESP32: 500mA @ 3.3V (via USB or voltage regulator)
- DC Motor: 500-2000mA @ 12V (depends on motor size)
- TEC Module: 2-6A @ 12V (depends on TEC size)
- PC Fan: 100-500mA @ 12V (depends on fan size)
- Total 12V requirement: 3-8A (use adequate power supply)

### Recommended Setup
- Main power: 12V 5A+ power supply
- ESP32 power: USB cable or buck converter (12V → 5V → ESP32)
- Use separate power rails for logic (3.3V) and motors/TEC (12V)
- **CRITICAL**: Never connect 12V directly to ESP32 pins!

## Pin Connections

### ESP32 GPIO Assignments

```
┌─────────────────────────────────────────────┐
│              ESP32 DevKit                    │
├─────────────────────────────────────────────┤
│                                              │
│  3.3V  ●────────────► VCC (Sensors)         │
│  GND   ●────────────► GND (Common Ground)   │
│                                              │
│  GPIO25 ●───────────► Motor PWM              │
│  GPIO26 ●───────────► Motor Direction        │
│  GPIO27 ●───────────► Photocell Input        │
│  GPIO14 ●───────────► DS18B20 Data (4.7kΩ)  │
│  GPIO32 ●───────────► TEC PWM                │
│  GPIO33 ●───────────► Fan PWM                │
│                                              │
└─────────────────────────────────────────────┘
```

## Component Wiring Details

### 1. DC Motor (Carousel Rotation)

**Using L298N Motor Driver:**

```
ESP32                  L298N                Motor
─────────────────────────────────────────────────
GPIO25 (PWM)  ───────► ENA
GPIO26 (DIR)  ───────► IN1
GND           ───────► IN2
                       OUT1 ──────────────► Motor (+)
                       OUT2 ──────────────► Motor (-)
                       +12V ◄─────────────  12V Supply
                       GND  ◄─────────────  Power GND
```

**Notes:**
- Connect ESP32 GND to L298N GND (common ground)
- Do NOT connect ESP32 power to motor power
- ENA pin controls speed via PWM (0-255)
- IN1/IN2 control direction (IN1=HIGH, IN2=LOW for forward)

**Alternative: Using MOSFET Module (simpler for single direction):**

```
ESP32                  MOSFET              Motor
─────────────────────────────────────────────────
GPIO25 (PWM)  ───────► Signal (S)
                       VCC (+12V) ◄────────  12V Supply
                       GND ◄───────────────  Power GND
                       OUT (+) ────────────► Motor (+)
                                             Motor (-) ◄── Power GND
```

### 2. Photocell Position Sensor

**Option A: Photoresistor (Analog)**

```
                    3.3V
                     │
                    ┌┴┐
                    │ │ Photoresistor
                    │ │ (1kΩ - 10kΩ)
                    └┬┘
                     │
                     ├──────────► GPIO27
                     │
                    ┌┴┐
                    │ │ 10kΩ
                    │ │ (Pull-down)
                    └┬┘
                     │
                    GND
```

**Option B: IR Reflective Sensor (Digital - Recommended)**

```
Sensor Module            ESP32
──────────────────────────────
VCC (+3.3V or 5V) ◄───── 3.3V
GND              ◄───── GND
OUT (Digital)    ───────► GPIO27
```

**Recommended Sensors:**
- TCRT5000 IR reflective sensor module
- LM393 comparator-based photoelectric sensor
- Any digital output photocell module

**Carousel Setup:**
- Mount sensor to detect gaps or slots in carousel rim
- Add reflective tape or cutouts on carousel for reliable detection
- Position sensor 2-5mm from carousel edge

### 3. DS18B20 Temperature Sensor

**Standard 1-Wire Configuration:**

```
DS18B20              Resistor           ESP32
──────────────────────────────────────────────
VDD (Pin 3)  ◄─────────────────────────  3.3V
                     ┌───┐
                     │4.7│ kΩ (pullup)
                     │   │
GND (Pin 1)  ◄───────┴───┴──────────────  GND
DATA (Pin 2) ────────┬──────────────────► GPIO14
                     │
                   (pullup to 3.3V)
```

**Important:**
- 4.7kΩ pullup resistor is REQUIRED between DATA and VDD
- For longer cable runs (>3m), use lower resistance (2.2kΩ)
- Can be powered in "parasite mode" (connect VDD to GND) but not recommended

### 4. TEC (Thermoelectric Cooler) Control

**Using MOSFET Module for PWM Control:**

```
ESP32                MOSFET (IRF520/540)        TEC
───────────────────────────────────────────────────
GPIO32 (PWM) ───────► Signal (S)
                      VCC (+12V) ◄──────────  12V Supply
                      GND ◄─────────────────  Power GND
                      OUT (+) ──────────────► TEC (+)
                                               TEC (-) ◄─ Power GND
```

**TEC Polarity:**
- Red wire (+): Connected to MOSFET output
- Black wire (-): Connected to power ground
- Reversing polarity switches between cooling and heating
- Use heatsink on TEC hot side to dissipate heat

**Important Notes:**
- TEC modules can draw 2-6A - ensure MOSFET can handle current
- Add flyback diode (1N5408 or similar) across TEC for protection
- Monitor TEC temperature to prevent damage (max ~150°C hot side)

### 5. PC Fan Control

**12V Fan with PWM Control:**

```
ESP32                MOSFET                Fan (12V)
─────────────────────────────────────────────────
GPIO33 (PWM) ───────► Signal (S)
                      VCC (+12V) ◄─────────  12V Supply
                      GND ◄────────────────  Power GND
                      OUT (+) ─────────────► Fan (+) Red
                                              Fan (-) Black ◄─ Power GND
```

**4-Pin PWM Fan (Alternative):**

```
Fan Connector           Connection
──────────────────────────────────
Pin 1 (Black)  ───────── Power GND
Pin 2 (Yellow) ───────── +12V
Pin 3 (Green)  ───────── Tachometer (optional, not used)
Pin 4 (Blue)   ───────── ESP32 GPIO33 (PWM signal)
```

**Notes:**
- 2-wire fans: Control speed by varying voltage via MOSFET
- 4-wire fans: Can use PWM signal directly (25kHz preferred)
- For 4-wire fans, keep +12V constant and PWM on blue wire
- Add 1kΩ resistor between ESP32 and PWM pin if using 4-wire fan

## Complete Wiring Diagram (ASCII Art)

```
                           ┌─────────────┐
                           │  12V 5A PSU │
                           └──┬──────┬───┘
                              │      │
                    ┌─────────┤      └─────────────┐
                    │         │                    │
                    │         │                    │
              ┌─────▼─────┐   │              ┌─────▼─────┐
              │  L298N    │   │              │ MOSFET    │
              │  Motor    │   │              │ TEC PWM   │
              │  Driver   │   │              └─────┬─────┘
              └─────┬─────┘   │                    │
                    │         │                    ▼
                    ▼         │              ┌──────────┐
              ┌─────────┐     │              │   TEC    │
              │  Motor  │     │              │  Module  │
              │Carousel │     │              └──────────┘
              └─────────┘     │
                              │
                        ┌─────▼─────┐
                        │ MOSFET    │
                        │ Fan PWM   │
                        └─────┬─────┘
                              │
                              ▼
                        ┌──────────┐
                        │  PC Fan  │
                        └──────────┘

┌──────────────────────────────────────────────┐
│           ESP32 Development Board             │
├──────────────────────────────────────────────┤
│                                               │
│  GPIO25 ──────► Motor Driver PWM              │
│  GPIO26 ──────► Motor Driver DIR              │
│  GPIO27 ◄────── Photocell Sensor              │
│  GPIO14 ◄────── DS18B20 (with pullup)         │
│  GPIO32 ──────► TEC MOSFET PWM                │
│  GPIO33 ──────► Fan MOSFET PWM                │
│                                               │
│  3.3V   ──────► Sensor VCC                    │
│  GND    ──────► Common Ground (All GND)       │
│                                               │
└──────────────────────────────────────────────┘
                    │
                    │ USB Cable
                    │ (Programming & Power)
                    ▼
```

## Safety Considerations

### Electrical Safety
1. **Separate Power Domains**: Never connect 12V directly to ESP32
2. **Common Ground**: Connect all grounds together (ESP32, PSU, motor drivers)
3. **Reverse Polarity Protection**: Add diode on power input
4. **Fusing**: Add appropriate fuses on 12V lines
5. **Heat Management**: Use heatsinks on MOSFETs and TEC modules

### Mechanical Safety
1. **Motor Protection**: Add limit switches or current sensing
2. **Enclosure**: Use proper enclosure to prevent cat access to electronics
3. **Ventilation**: Ensure fan has adequate airflow
4. **Cable Management**: Secure all cables away from moving parts

### Food Safety
1. **Temperature Monitoring**: Ensure TEC can maintain safe temperature (<5°C)
2. **Backup Power**: Consider battery backup for critical functions
3. **Fail-Safe**: Motor should default to OFF if ESP32 crashes

## Testing Procedure

### Initial Power-On Test
1. Connect only ESP32 via USB (no motors)
2. Verify ESP32 boots and creates WiFi AP
3. Check serial output for errors

### Component Testing (one at a time)
1. **Photocell**: Monitor sensor state via web interface
2. **Temperature**: Verify DS18B20 readings are sensible
3. **Motor**: Test rotation via web interface (no load)
4. **TEC**: Test cooling (monitor temperature drop)
5. **Fan**: Verify fan spins at different PWM values

### Integration Testing
1. Mount carousel on motor
2. Align photocell with carousel gaps
3. Test manual feeding cycle
4. Verify position detection
5. Adjust calibration offset as needed
6. Load food and perform full feeding test

## Troubleshooting

### Motor doesn't rotate
- Check 12V power supply voltage
- Verify motor driver connections
- Test motor directly with 12V (bypass driver)
- Check GPIO output with multimeter/oscilloscope

### Photocell not detecting
- Check sensor alignment with carousel
- Verify 3.3V power on sensor
- Test sensor output with multimeter
- Adjust sensor sensitivity (if adjustable)
- Try different sensor position/angle

### Temperature reading -127°C
- DS18B20 not connected or faulty
- Missing 4.7kΩ pullup resistor
- Wrong GPIO pin assignment
- Check wiring continuity

### TEC not cooling
- Check polarity (may be heating instead)
- Verify adequate power supply current
- Ensure heatsink on hot side
- Check MOSFET is switching (scope/LED test)
- Verify PWM signal on GPIO32

### Fan not spinning
- Check 12V supply to fan
- Verify PWM signal reaches fan
- Test fan directly with 12V
- Check MOSFET switching

## Advanced Modifications

### Adding Battery Backup
```
12V Battery ─┬─► Charge Controller ─► System 12V
             │
             └─► ESP32 UPS Module ─► ESP32 5V
```

### Adding Display
```
OLED/LCD Display (I2C)
SDA ──────► GPIO21
SCL ──────► GPIO22
VCC ──────► 3.3V
GND ──────► GND
```

### Adding Additional Sensors
- **Humidity Sensor**: DHT22 on GPIO15
- **Food Level Sensor**: Ultrasonic HC-SR04 on GPIO16/17
- **Door Switch**: Magnetic reed switch on GPIO18

## Parts List and Suppliers

### Essential Components
- ESP32 DevKit (any variant)
- L298N motor driver or equivalent
- 2x MOSFET modules (IRF520/540)
- DS18B20 temperature sensor with cable
- IR reflective sensor (TCRT5000 or similar)
- 4.7kΩ resistor (pullup for DS18B20)
- TEC module (TEC1-12706 or similar)
- 12V PC fan (80mm or 120mm)
- 12V 5A power supply
- Heatsinks for TEC and MOSFETs
- Dupont wires and connectors
- Breadboard or PCB for prototyping

### Optional Components
- Buck converter (12V to 5V for ESP32)
- Fuses and holders
- Terminal blocks
- Project enclosure
- Perfboard or custom PCB

### Recommended Suppliers
- AliExpress (bulk components, long shipping)
- Amazon (faster shipping, higher cost)
- Digikey/Mouser (quality components, best for resistors/capacitors)
- Local electronics store (immediate availability)

## Maintenance

### Regular Checks
- Clean photocell sensor monthly
- Check motor for wear/noise
- Verify TEC cooling performance
- Inspect all connections for corrosion
- Clean fan filter (if present)

### Software Updates
- Monitor serial output for errors
- Update firmware for bug fixes
- Backup configuration before updates
- Test feeding cycles after updates
