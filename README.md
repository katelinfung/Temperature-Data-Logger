# Temperature-Data-Logger

This device uses a PIC32 MX130 Microcontroller and an LM335 Temperature Sensor. The temperature is displayed in degrees Celsius on an LCD screen and on the Python stripchart. There are three LEDs that show whether the temperature is less than 20C (blue), between 20C-22C (green), or above 22C (red). There is a reset pushbutton and an on/off push button. When the sensor is off, "OFF" is displayed on the LCD, the LEDs are turned off, and the Python stripchart pauses. When the sensor is turned on again, by pressing the reset or on/off button, the on/off LED turns on and the three LEDs blink. There is also an animation on the LCD screen.
