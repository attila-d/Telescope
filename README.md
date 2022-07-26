# Motorized Alt-Azimuth telescope

I built a Newtonian/Dobsonian 8" telescope, with some extra features: two stepper motors can move its altitude and azimuth direction. 
It is controlled by an Arduino Mega2560 board, having two rotary knobs for immediate positioning, and connecting to Android/PC Stellarium software via Bluetooth, talking the Nexstar protocol. 

![alt text](http://url/to/img.png)

I am standing on shoulders of giants: the celestial calculations are not my work, found them on the internet with licences not forbidding to reuse them. I'll add references to the original code. 

## Hardware

- Arduino Mega 2560
- 2 Nema17 stepper motors, with approrpriate drivers (4988)
- Magnetometer - compass, for initial calibration to North pole
- Accelerometer ADXL345 - tilt sensor
- (GPS support removed: Stellarium syncs position and current time when connection established)
- Bluetooth module HC-05
- 2 rotary encoders with push button
- Chargeable battery 6*18650, power socket, power switch (ON-OFF)
- Charge controller board (3S 25A for  18650 Lithium Battery Protection Board)

Alt encoder pins: 45,47 + 44
Azimuth encoder: 46, 48 + 49

Alt stepper step pin 25, dir pin 23
Azi stepper step pin 24, dir pin 22

Bluetooth HC-05 is at Serial3 (RX3/TX3)

ADXL345 && MPU9250 are connected to the default I2C port.

## Control

### ({Alt}|{Azi}) buttons

- Azi button =>  +- azimuth control
- Alt button =>   +- altitude control

- Alt button =>  press: stop motion / cancel finder
- Azi button =>  press: stop motion / cancel finder

- Alt button =>  dbl: attach on/off (fb?)
- Azi button =>  dbl: tracking on/off (feedback?)

- Alt button =>  push + enc: speed rate+-
- Azi button =>  push + enc: -

- Azi button =>  long push + Alt button push: spiral move finder until any button clicked
- Alt button =>  long push + Azi button push: spiral move finder until any button clicked

### Boot:
.Both pressed: compass + (base tilt?) calibration
.B1 pressed: North pole
.B2 pressed: top position?

## TODO:

- .make short AZI moves instead the long way around
- proper range function
- improve north finding
- calibrate tilt (if not placed on very horizontal surface) (Using MPU9250 accelerometer) (MPU9250 gyro(Z) shows rotation around AZIMUTH!)

## Notes

https://learn.sparkfun.com/tutorials/adxl345-hookup-guide/all

Bluetooth: https://askubuntu.com/questions/721954/bluetooth-connection-to-hc-05-paired-but-not-connected 

Connect PC serial to bluetooth

> sudo rfcomm bind 0 98:D3:33:80:7D:ED

https://docs.platformio.org/en/latest//faq.html#platformio-udev-rules

sudo apt-get install lib32z1

https://alberand.com/hc-05-linux.html

adxl 345: xyz 
topdown -10.04, 1.77, 0.94
above -1.02, -10.36, 0.27
bottom 10.36, -1.92, 0.31

__MAGNETO

Bootstrap: 
https://getbootstrap.com/docs/4.0/examples/checkout/

Nexstar protocol is available at: https://docplayer.net/20889250-Nexstar-communication-protocol.html
