# PMW3389 Mouse
This code is for a custom 3D printed mouse that utilizes the PMW3389 sensor.
The video about this project can be found here: https://youtu.be/qmX8vL-GbxU

# Parts info
* PMW3389 Module: https://www.tindie.com/products/citizenjoe/pmw3389-motion-sensor/
  * <img src="img/tindie_sensor_image.jpg" width="200" alt="PMW3360 Module Image">
* Original version of this code: https://github.com/dkao/Kensington_Expert_Mouse_PMW3389_Arduino by dkao
* Base source code: https://github.com/mrjohnk/PMW3389DM by mrjohnk
* Arduino Pro Micro

# Pin connection
* MI = MISO
* MO = MOSI
* SS = Slave Select / Chip Select
* SC = SPI Clock
* MT = Motion (active low interrupt line)
* RS = Reset
* GD = Ground
* VI = Voltage in up to +5.5V

# Sketch Descriptions
* PMW3389DM-Mouse/
  * Mouse with  high polling rate and 4 mian buttons: Left, Right, Middle, Back, plus 2 user-definable Keyboard buttons
  * Default CPI and debounce time can be set (see #define sections in the source code)
  * Cycle through CPI with button combo (back + right click), LED will update to show which mode is selected

# How to use
  * Build the circuit as described.
  * (Arduino only) Copy the /library/AdvMouse/ to your Arduino library folder
  * Load PMW3389DM-Mouse on Arduino IDE.
    * Change dx, dy to positive values if input is reversed
  * Upload code to Arduino
  * Enjoy!
