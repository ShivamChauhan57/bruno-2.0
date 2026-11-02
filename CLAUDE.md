# Bruno Robot Workspace

## Environment
- OS: Ubuntu Server (x86_64)
- ESP32 Port: /dev/ttyUSB0
- ESP32 FQBN: esp32:esp32:esp32
- Build Tool: arduino-cli

## ESP32 Hardware Pin Map
- Left Motor: IN1=16, IN2=18, ENA=25 (PWM)
- Right Motor: IN3=19, IN4=23, ENB=26 (PWM)
- Status LEDs: Blue=15, Green=2, Yellow=4, Red=5
- Ultrasonics (3.3V Logic):
  - Left: TRIG=13, ECHO=34
  - Mid:  TRIG=14, ECHO=35
  - Right: TRIG=27, ECHO=39

## How to Flash the ESP32
To compile and upload firmware, run:
arduino-cli compile --upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 <path_to_sketch_dir>

*Rule:* Ensure any serial reader on /dev/ttyUSB0 is closed before flashing.
