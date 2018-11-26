# READ ME


Welcome to the respositry for the Gate Keeper ESP. It's still early in development but to prove its security the entire software is open source.




Note the secure boot signing key is for testing only.




## Programming

Insure you have installed the latest (or latest compatiable) ESP-IDF.

https://docs.espressif.com/projects/esp-idf/en/latest/get-started/

#### ESP32 Based Devices

Use an external USB to UART converter. Plug into 6 pin header on bottom board or connect directly to the top board where indicated.

The 6 pin header matches exactly with the ESP-PROG 1.27mm pitch design.

https://github.com/espressif/esp-iot-solution/blob/master/documents/evaluation_boards/ESP-Prog_guide_en.md

This requires a 6 pin 1.27mm pitch IDC cable. Power is shared with vehicle power and must be 5 Volts (direct from USB). Both vehicle power and USB power can be used simutaneously, although for extra safety you should disconnect 5V power when working with the vehicle.

Once connected, identifiy the correct usb device. Navigate to the project root directory.

~~~~
idf.py menuconfig
~~~~
 This will bring up the menuconfig menu. Set the default usb port as well as any other desired changes.
 
~~~~
idf.py flash monitor
~~~~
This will run cmake, make a binary, flash it to the device, and begin monitoring the ESP32. This method insures you get the boot messages. You can optionally run "idf.py monitor" at any time.
