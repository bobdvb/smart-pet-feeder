# Quick Start Guide - Smart Cat Feeder

Get your ESP32 cat feeder up and running in 30 minutes!

## What You Need

### Hardware (Minimum)
- [ ] ESP32 development board
- [ ] DC motor (6-12V with gear reduction)
- [ ] Motor driver (L298N or MOSFET module)
- [ ] Photocell or IR sensor
- [ ] DS18B20 temperature sensor
- [ ] 4.7kΩ resistor (for DS18B20)
- [ ] TEC module (optional but recommended)
- [ ] 12V PC fan
- [ ] 2x MOSFET modules (for TEC and fan)
- [ ] 12V 5A power supply
- [ ] USB cable for ESP32 programming
- [ ] Jumper wires

### Software
- [ ] Arduino IDE (1.8.19+ or 2.x) OR PlatformIO
- [ ] ESP32 board support
- [ ] Required libraries (see ARDUINO_LIBRARIES.txt)

## Step-by-Step Setup

### 1. Prepare the Development Environment (15 min)

#### Option A: Arduino IDE
```bash
1. Download Arduino IDE from arduino.cc
2. Install ESP32 board support:
   - File → Preferences
   - Add to "Additional Board Manager URLs":
     https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   - Tools → Board → Boards Manager
   - Search "esp32" and install

3. Install required libraries:
   - Sketch → Include Library → Manage Libraries
   - Install: WiFiManager, OneWire, DallasTemperature, ArduinoJson
```

#### Option B: PlatformIO (Recommended)
```bash
1. Install Visual Studio Code
2. Install PlatformIO extension
3. Open this project folder
4. PlatformIO will auto-install all dependencies
```

### 2. Wire Up the Hardware (10 min)

**Basic Test Setup** (test before full assembly):

```
ESP32           Component
─────────────────────────────────
GPIO25    ──►   Motor PWM (via driver)
GPIO26    ──►   Motor Direction (via driver)
GPIO27    ◄──   Photocell sensor
GPIO14    ◄──   DS18B20 (with 4.7kΩ pullup)
GPIO32    ──►   TEC PWM (via MOSFET)
GPIO33    ──►   Fan PWM (via MOSFET)
3.3V      ──►   Sensor VCC
GND       ──►   All grounds (common ground!)
```

**Important**:
- See WIRING.md for detailed diagrams
- Connect 12V devices to MOSFET/driver outputs, NOT directly to ESP32
- Use common ground for all components

### 3. Upload the Code (5 min)

#### Arduino IDE:
```bash
1. Open CatFeeder.ino
2. Tools → Board → ESP32 Dev Module
3. Tools → Port → [Select your ESP32 port]
4. Click "Upload" button
5. Wait for "Done uploading" message
```

#### PlatformIO:
```bash
1. Open project in VS Code
2. Click "Upload" button (→) in bottom toolbar
   OR run: pio run -t upload
3. Wait for upload to complete
```

### 4. Initial Configuration (5 min)

```bash
1. Open Serial Monitor (115200 baud)
   - Arduino IDE: Tools → Serial Monitor
   - PlatformIO: Click monitor icon

2. Watch for startup messages:
   "Smart Cat Feeder - Starting..."
   "WiFi Manager - AP Mode"

3. Connect to WiFi:
   - On your phone/computer, connect to "CatFeeder-Setup" WiFi network
   - Browser should auto-open to configuration page
   - If not, navigate to: http://192.168.4.1

4. Configure WiFi:
   - Select your WiFi network
   - Enter password
   - Set feeding times (default: 7:00 AM and 6:00 PM)
   - Set target temperature (default: 15°C)
   - Set timezone offset (e.g., -5 for EST)
   - Click "Save"

5. Device will reboot and connect to your WiFi
   - Note the IP address shown in Serial Monitor
   - Example: "IP Address: 192.168.1.100"
```

### 5. Test the System (5 min)

#### Access Web Interface:
1. Open browser to ESP32 IP address (from step 4)
2. You should see the "Smart Cat Feeder" dashboard

#### Test Each Component:

**Temperature Sensor:**
- Status page should show current temperature
- If showing -127°C: Check DS18B20 wiring and pullup resistor

**Photocell Sensor:**
- Cover/uncover sensor
- Check debug output in Serial Monitor
- Should see state changes

**Motor:**
- Click "Manual Feed Now" button
- Motor should rotate until photocell detects next slot
- Check Serial Monitor for "Rotation complete"

**TEC and Fan:**
- If temperature is above target, TEC should activate
- Fan speed should increase with temperature
- Check status page for "TEC Power" and "Fan Speed" values

## Configuration Options

### Web Interface Settings:

| Setting | Description | Default | Range |
|---------|-------------|---------|-------|
| Morning Feed | First feeding time | 7:00 AM | 00:00-23:59 |
| Evening Feed | Second feeding time | 6:00 PM | 00:00-23:59 |
| Target Temp | TEC target temperature | 15°C | 5-30°C |
| Timezone | Hours offset from UTC | 0 | -12 to +14 |
| Calibration | Fine-tune motor stop position | 0 ms | -500 to +500 |
| Enable Cooling | Turn TEC on/off | ON | ON/OFF |

### Calibration Procedure:

If carousel doesn't align perfectly:

1. Run manual feed cycle
2. Observe where motor stops relative to slot
3. Adjust calibration offset:
   - If stops too early: Add positive value (e.g., +100 ms)
   - If stops too late: Add negative value (e.g., -100 ms)
4. Save configuration and test again
5. Repeat until alignment is perfect

## Typical Usage Pattern

### Daily Operation:
```
7:00 AM  → Carousel rotates to fresh food
6:00 PM  → Carousel rotates to fresh food
         → TEC keeps food cool throughout day
         → Fan adjusts speed based on temperature
```

### Maintenance Schedule:
- **Daily**: Check water level, verify feeding occurred
- **Weekly**: Refill food slots, clean bowls
- **Monthly**: Clean photocell sensor, check motor
- **Quarterly**: Deep clean, verify all functions

## Troubleshooting

### Problem: Can't connect to WiFi AP

**Solution:**
```bash
1. Press ESP32 reset button
2. Wait 30 seconds for AP to start
3. Look for "CatFeeder-Setup" network
4. If still not visible, reflash firmware
```

### Problem: Motor doesn't rotate

**Check:**
- [ ] 12V power supply connected and turned on
- [ ] Motor driver wiring correct (see WIRING.md)
- [ ] GPIO25 and GPIO26 pins correct
- [ ] Test motor with 12V directly (bypass ESP32)
- [ ] Check Serial Monitor for error messages

**Quick Test:**
```cpp
// Add to loop() temporarily:
digitalWrite(MOTOR_DIR_PIN, HIGH);
ledcWrite(MOTOR_PWM_CHANNEL, 128);  // 50% speed
delay(2000);
ledcWrite(MOTOR_PWM_CHANNEL, 0);
delay(5000);
```

### Problem: Photocell not detecting slots

**Check:**
- [ ] Sensor powered (3.3V)
- [ ] Sensor aligned with carousel gaps
- [ ] Sensor sensitivity (if adjustable)
- [ ] GPIO27 connection

**Quick Test:**
```cpp
// Add to loop() temporarily:
Serial.print("Photocell: ");
Serial.println(digitalRead(PHOTOCELL_PIN));
delay(100);
```

### Problem: Temperature shows -127°C

**Check:**
- [ ] DS18B20 connected correctly
- [ ] 4.7kΩ pullup resistor between DATA and VCC
- [ ] GPIO14 pin correct
- [ ] Try different DS18B20 (may be faulty)

**Quick Test:**
```cpp
// Check OneWire devices found:
tempSensor.begin();
Serial.print("Devices: ");
Serial.println(tempSensor.getDeviceCount());
```

### Problem: TEC not cooling

**Check:**
- [ ] TEC polarity (red = +, black = -)
- [ ] 12V power adequate (TEC draws high current)
- [ ] MOSFET rated for TEC current (6A+)
- [ ] Heatsink on TEC hot side
- [ ] GPIO32 connection

**Quick Test:**
```cpp
// Add to loop() temporarily:
ledcWrite(TEC_PWM_CHANNEL, 255);  // Full power
delay(30000);  // Run for 30 seconds
ledcWrite(TEC_PWM_CHANNEL, 0);
// Touch TEC - one side should be cool, other hot
```

### Problem: Web interface not accessible

**Check:**
- [ ] ESP32 connected to WiFi (check Serial Monitor)
- [ ] Using correct IP address
- [ ] Same WiFi network as ESP32
- [ ] No firewall blocking connection

**Find IP Address:**
```bash
1. Check Serial Monitor for "IP Address: x.x.x.x"
2. Or use router admin page to find "CatFeeder"
3. Or use network scanner app
```

## Advanced Configuration

### Modify Feeding Times in Code:
```cpp
// In CatFeeder.ino, modify defaults:
config.feedTime1Hour = 6;    // 6:00 AM
config.feedTime1Minute = 30; // 6:30 AM
config.feedTime2Hour = 19;   // 7:00 PM
config.feedTime2Minute = 0;  // 7:00 PM
```

### Adjust PID Parameters:
```cpp
// For faster/slower temperature response:
const float Kp = 2.0;  // Increase for faster response
const float Ki = 0.1;  // Increase to eliminate steady-state error
const float Kd = 1.0;  // Increase to reduce overshoot
```

### Change Motor Speed:
```cpp
#define MOTOR_SPEED 255  // 0-255 (255 = full speed)
```

### Add Third Feeding Time:
```cpp
// Add new config variables:
int feedTime3Hour = 12;
int feedTime3Minute = 0;

// Add check in checkFeedingSchedule():
if (currentHour == config.feedTime3Hour &&
    currentMinute == config.feedTime3Minute) {
  if (!lastFeedTime3Done) {
    rotateCarousel();
    lastFeedTime3Done = true;
  }
} else {
  lastFeedTime3Done = false;
}
```

## Safety Checklist

Before leaving unattended:

- [ ] Test feeding cycle 3+ times successfully
- [ ] Verify photocell reliably detects all slots
- [ ] Confirm temperature stays in safe range (0-20°C)
- [ ] Check motor doesn't stall or overheat
- [ ] Ensure food won't spoil (use ice packs if needed)
- [ ] Verify cat can easily access open slot
- [ ] Check all wiring is secure and insulated
- [ ] Test power loss recovery (unplug and replug)
- [ ] Confirm feeding times are correct for your timezone

## Next Steps

1. **Mechanical Assembly**: Build or modify carousel housing
2. **Enclosure**: 3D print or build case for electronics
3. **Backup Power**: Add battery backup for power outages
4. **Notifications**: Add email/push notifications for feeding events
5. **Food Detection**: Add sensors to detect if food is present
6. **Web Dashboard**: Enhance UI with feeding history graphs

## Additional Resources

- **Detailed Wiring**: See WIRING.md
- **Library Setup**: See ARDUINO_LIBRARIES.txt
- **Project Info**: See README.md
- **Code Comments**: Read inline comments in CatFeeder.ino

## Getting Help

**Serial Monitor Output**: Always check Serial Monitor first!
- Shows startup sequence
- Reports errors and warnings
- Displays feeding events
- Shows temperature readings

**Common Error Messages:**
```
"Motor timeout - stopping" → Photocell not detecting
"Error reading temperature sensor" → DS18B20 problem
"Failed to connect and hit timeout" → WiFi issues
"Rotation failed - timeout" → Motor or sensor problem
```

**Community Support:**
- GitHub Issues: Report bugs and ask questions
- ESP32 Forums: arduino-esp32 on GitHub
- Reddit: r/esp32, r/arduino

## Success Criteria

Your system is working correctly when:
- ✓ Temperature displays correctly (not -127°C)
- ✓ Manual feed rotates carousel to next slot reliably
- ✓ Photocell detects each slot position
- ✓ TEC activates when temp exceeds target
- ✓ Fan speed changes with temperature
- ✓ Web interface accessible and responsive
- ✓ Scheduled feedings occur at correct times
- ✓ System survives power cycle without issues

## Final Notes

- Start simple: Test each component individually first
- Document your calibration values for future reference
- Take photos of your wiring for troubleshooting
- Keep food refrigerated until testing is complete
- Monitor first few days closely before leaving unattended

**Congratulations!** You now have an automated, internet-connected, temperature-controlled cat feeder! 🐱
