# HatireSTM
Hat Tracking implementation compatible with 'Hatire' protocol using BNO055 DOF and Maple Mini.

Based on OpenTrack's Hatire plugin and Arduino counterpart; I've implemented this.

Generally other trackers use MPU9250 or similar sensors; but I found them quite hard to calibrate. For some reason they just didn't "track me well" :)

Enter BNO055.. I learned about this IC when looking for alternatives and I'm loving it! So easy to use and very accurate. A tad more pricy but still good.

## How to use

Just flash using Sloeber or Arduino IDE. It requires STM32 Core: https://github.com/stm32duino/Arduino_Core_STM32

Use OpenTrack or similar software and chose Hatire or Hat as 'plugin source'.

The chip will not submit input until it calibrates itself after 'I' command. Button 32 (maple's onboard button) can be used to 'bail' from initial calibration. In normal run; it invokes calibration by hand. Of course; command 'C' does work too.

Enjoy!


## Used Libraries

* Adafruit Unified Sensor Library
* Adafruit BNO055 Library
* EEPROM (for saving calibration)
