Overview:
This system will detect the door state, ambient light and nearby object distances to activate a buzzer alarm whilst an
offboard led will blink as a simulated 'heartbeat'

Core components:
- Ultrasonic sensor
- Door button
- Light dependent resistor
- Buzzer
- LED
- Arduino Uno R3
- Resistors (2 x 220Ohm and 1 x 10kOhm)
- Jump wires

Configurations:
Ultrasonic sensor (3 pin): D8 -> PB0/PCINT0 (Trigger + Echo)
Push Button: D9 -> PB1/PCINT1, create pullup
Buzzer: D6 
LED: D13
LDR: A0
Ultrasonic VCC: 5v Rail
Ultrasonic GND: GND Rail

Full board configurations, including resistor orientation and pin handlings, can be found in attached photos
of this GITHUB repository :D

How to run:
1. Open the .ino file and upload to the Arduino (assuming proper configuration)
2. Open serial monitor at baud rate of 115200 to observe real-time event logs
3. Test different sensor states by applying:
   - Different levels of light to LDR
   - Different distances from Ultrasonic sensor
   - Pushing down pushbutton with different LDR and ultrasonic sensor readings
   - Looking at LED heartbeat functionality
   - Testing alarm conditions