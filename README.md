# Two-Axis-Reaction-Wheel-Stick

Arduino mini, Nidec 24H motors, MPU6050, 3S 500 mAh LiPo battery.

Balancing controllers can be tuned remotely over bluetooth.

Example (change K1):

Send p+ (or p+p+p+p+p+p+p+) for increase K1.

Send p- (or p-p-p-p-p-p-p-) for decrease K1.

Send i, s if you need to change K2, K3.

Send c+ from serial monitor for calibrating procedure. Place the stick at the balancing point (as accurately as possible). Send c- from serial monitor. The stick will start balancing and write the offsets values ​​into the EEPROM.

<img src="/pictures/stick1.jpg" alt="Balancing stick pic"/>
<img src="/pictures/stick2.jpg" alt="Balancing stick pic"/>
<img src="/pictures/schematic.png" alt="Schematic"/>

About schematic:

Battery: 3S1P LiPo (11.1V). 

Buzzer: any 5V active buzzer.

More about this:

https://youtu.be/vXHlW2S24GQ

https://youtu.be/4gS2i5fecFE

This video may also help:

https://youtu.be/Nkm9PoihZOI