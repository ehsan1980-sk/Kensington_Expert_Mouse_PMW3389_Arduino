# PMW3360
Interfacing PixArt PMW3360 with Arduino boards

# Parts info
* PMW3360 Module: https://www.tindie.com/products/jkicklighter/pmw3360-motion-sensor/ by JACK Enterprises
  * <img src="img/tindie_sensor_image.jpg" width="200" alt="PMW3360 Module Image">
* Base source code: https://github.com/mrjohnk/PMW3360DM-T2QU by mrjohnk
* Teensy LC (https://www.pjrc.com/store/teensy.html) with Teensyduino (https://www.pjrc.com/teensy/teensyduino.html)
  * Arduino Leonardo: https://www.arduino.cc/en/Main/Arduino_BoardLeonardo

# Pin connection
* MI = MISO
* MO = MOSI
* SS = Slave Select / Chip Select
* SC = SPI Clock
* MT = Motion (active low interrupt line)
* RS = Reset
* GD = Ground
* VI = Voltage in up to +5.5V

```
[Module] --- [Teensy]
      RS --- (NONE)
      GD --- GND
      MT --- (NONE)
      SS --- Pin 10
      SC --- SCK
      MO --- MOSI
      MI --- MISO
      VI --- 3.3V

// in Mouse example
[Button] --- [Teensy]
    Left --- Pin 1
   Right --- Pin 3
    Back --- Pin 0
 Forward --- Pin 2
   
* Connect the other pole of a button to GND
```

# Sketch Descriptions
* PMW3360DM-Mouse/
  * Fully functional trackball (w. 1k polling rate) with four buttons (left, right, middle, back)
  * Default CPI and debounce time can be set (see #define sections in the source code)
  * Cycle through CPI with button combo (back + forward)
  * Cycle through button mapping with button combo (back + forward + left + right)
  * Middle click emulation (left + right)
  * Scroll emulation (back or forward + horizontal or vertical ball movement)
  * Commands (newline (\n) should be placed at the end of each command)
    * Q: toggle surface quality report (can be seen thorugh Serial Monitor or Serial Plotter)
    * I: print device signature
    * C[number]: Set cpi level, example) C1600\n   = set CPI to 1600.

# How to use
  * Build the circuit as described.
  * (Arduino only) Copy the /library/AdvMouse/ to your Arduino library folder
  * Load PMW3360DM-Mouse on Arduino IDE.
    * Modify dx, dy for your sensor orientation
 
# Construction
  * ![Image showing bottom shell modification](img/chassis_mod.jpg?raw=true "Sensor inside a modified Logitech Trackman Marble bottom shell")
  * ![Image showing ball holder modification](img/ball_holder_mod.jpg?raw=true "Ball holder modification")
  * ![Image showing bottom shell assembly](img/IMG_20200615_034036.jpg?raw=true "Bottom shell assembly")
  * ![Image showing bottom shell slit](img/IMG_20200615_031907.jpg?raw=true "Bottom shell slit")
  * ![Image showing lens alignment](img/IMG_20200615_031844.jpg?raw=true "Lens alignment")
