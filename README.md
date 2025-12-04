# Smart Cat Feeder - ESP32 Controller

An ESP32-based automatic cat feeder with temperature control, featuring a 5-slot rotating carousel, TEC cooling system, and web-based configuration interface.

## Features

- **Automatic Feeding**: Two configurable feeding times per day (morning and evening)
- **5-Slot Carousel**: Rotates to dispense food from the next slot at feeding time
- **Smart Positioning**: Photocell sensor detects carousel position with calibration offset
- **Temperature Control**: TEC cooler with PID control to maintain set temperature
- **Adaptive Cooling**: Fan speed adjusts proportionally to temperature
- **Web Interface**: WiFiManager-based configuration portal
- **NTP Time Sync**: Accurate timekeeping via internet time servers
- **Status Monitoring**: Real-time status display via web interface

## Hardware Requirements

### Components
- ESP32 Development Board (any variant with sufficient GPIO pins)
- DC Motor with gear reduction for carousel rotation
- Motor driver (e.g., L298N, TB6612FNG, or MOSFET module)
- Photocell or IR sensor for position detection
- DS18B20 1-Wire temperature sensor
- TEC (Thermoelectric Cooler) module
- PC Fan (12V recommended)
- 2x MOSFET modules for PWM control (TEC and fan)
- Power supply (12V recommended for motor, TEC, and fan)
- Voltage regulator if powering ESP32 from same supply

### Pin Connections

```
ESP32 Pin    | Component           | Description
-------------|---------------------|----------------------------------
GPIO 25      | Motor PWM           | PWM output to motor driver
GPIO 26      | Motor Direction     | Motor direction control
GPIO 27      | Photocell Sensor    | Digital input (carousel position)
GPIO 14      | DS18B20 Data        | 1-Wire temperature sensor (4.7kΩ pullup)
GPIO 32      | TEC PWM             | PWM output to TEC MOSFET
GPIO 33      | Fan PWM             | PWM output to fan MOSFET
```

**Note**: Adjust pin assignments in the code as needed for your specific wiring.

## Software Requirements

### PlatformIO Dependencies
The project uses PlatformIO for easier dependency management. See `platformio.ini` for library dependencies.

### Arduino IDE Libraries
If using Arduino IDE, install these libraries:
- WiFiManager by tzapu (v2.0.16-rc.2 or later)
- OneWire (for DS18B20)
- DallasTemperature (for DS18B20)
- ArduinoJson (v6.x)
- ESPAsyncWebServer
- AsyncTCP

## Configuration

### Initial Setup
1. Flash the firmware to your ESP32
2. On first boot, the device creates a WiFi access point named "CatFeeder-Setup"
3. Connect to this AP with your phone/computer
4. Configure your WiFi credentials and feeding schedule
5. Save settings - the device will reboot and connect to your WiFi

### Web Interface
Access the web interface by navigating to the device's IP address in your browser:
- **Configuration**: Set feeding times, temperature targets, calibration
- **Manual Control**: Test carousel rotation, view current temperature
- **Status**: View next feeding time, current slot, temperature readings

### Default Settings
- Morning Feed: 07:00
- Evening Feed: 18:00
- Target Temperature: 15°C (59°F)
- NTP Server: pool.ntp.org
- Timezone Offset: 0 (UTC)

## Operation

### Feeding Cycle
1. At scheduled feeding time, the motor starts rotating the carousel
2. System waits for photocell to detect the current slot edge
3. Continues rotation until the next slot position is reached
4. Applies calibration offset for precise positioning
5. Motor stops, allowing cat to access fresh food

### Temperature Control
- Continuously monitors temperature via DS18B20 sensor
- TEC activates when temperature exceeds target (cooling mode)
- Fan speed increases proportionally with temperature differential
- PID control maintains stable temperature

### Position Calibration
The calibration offset allows fine-tuning the stop position:
- Positive values: Continue rotating slightly past sensor trigger
- Negative values: Stop slightly before sensor trigger
- Range: -500ms to +500ms
- Adjust via web interface for perfect slot alignment

## Safety Features

- **Timeout Protection**: Motor stops after 30 seconds if position not detected
- **Temperature Limits**: TEC and fan limits prevent overheating
- **Manual Override**: Emergency stop and manual feed via web interface
- **Configuration Backup**: Settings stored in EEPROM/Preferences

## Troubleshooting

### Motor doesn't rotate
- Check motor power supply and driver connections
- Verify GPIO pin assignments match your wiring
- Test motor manually via web interface

### Position sensor not working
- Check photocell/sensor power and signal connections
- Verify sensor can detect carousel gaps (check alignment)
- Monitor sensor state in web interface debug mode

### Temperature control issues
- Verify DS18B20 has 4.7kΩ pullup resistor
- Check TEC polarity (heating vs cooling)
- Ensure adequate power supply for TEC
- Verify fan is running (check PWM signal)

### WiFi connection problems
- Press reset to enter AP mode again
- Clear saved credentials via web interface
- Check WiFi signal strength and router settings

## Customization

### Modify Feeding Times
Edit via web interface or modify default values in code:
```cpp
config.feedTime1Hour = 7;   // Morning: 7:00 AM
config.feedTime2Hour = 18;  // Evening: 6:00 PM
```

### Adjust Temperature Control
Modify PID parameters in code for different cooling behavior:
```cpp
float Kp = 2.0;  // Proportional gain
float Ki = 0.1;  // Integral gain
float Kd = 1.0;  // Derivative gain
```

### Change Carousel Slots
If your carousel has a different number of slots, adjust:
```cpp
#define SLOTS_COUNT 5  // Change to match your carousel
```

## License

This project is provided as-is for personal use. Modify and adapt as needed for your specific hardware setup.

## Contributing

Feel free to submit issues and enhancement requests!
