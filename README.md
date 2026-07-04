# autocar_tb6612_keil

Keil project for MSPM0G3507 using the wiring in `E:\NUEDC\picture`.

Open `keil/autocar_tb6612.uvprojx`, build, then download.

Default behavior:
- press center key to start H problem task 1
- follow the tracking module toward B
- stop after the A-to-B time and beep/blink

Important wiring:
- PA27 -> PWMA, PB25 -> AIN1, PA25 -> AIN2
- PB24 -> PWMB, PA24 -> BIN1, PA22 -> BIN2
- PA00 -> PWMC, PA08 -> CIN1, PA01 -> CIN2
- PA28 -> PWMD, PB04 -> DIN1, PA31 -> DIN2
- PA17 -> tracking SCL, PA16 -> tracking SDA
- PB7 -> ICM42688 SCL, PB8 -> ICM42688 SDA
- PA11 -> key up, PA12 -> key down, PA13 -> key left
- PA18 -> key right, PB06 -> key center
- PB18 -> LED, PA21 -> buzzer
- all boards share GND
