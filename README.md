# ESP32 3-DoF Bluetooth Headtracker

- emulates a gamepad to work with opentrack.
- strap it to your forehead with a head band :-) great for sim games - MSFS, Nuclear Option, etc
- device must be strapped in the correct orientation, can be checked via the serial console or the "Raw tracker data" in opentrack.
- ./docs folder has various prototype pics, schematics, opentrack setup, etc.
- imho, this is more stable than the IR or webcam based headtrackers, especially for HMCS operation (aiming with the head). i didn't miss the postional output.
- there will eventually be gyro drift affecting the yaw output. in practice, drift does not really affect gameplay. you should bind the "Center" function in opentrack to reset the orientation. the device also performs automatic live calibration if you keep it still.
- feedback, comments and suggestions welcomed!

## features

- esp32-s3 supermini (built-in battery charging circuit)
- bmi160 imu (gyro and accel)
- simple battery voltage measurement circuit, reports battery level over bluetooth.
- emulates gamepad X (roll), Y (pitch) and Z (Yaw) axes for 3 rotational degrees of freedom. no translation (position) output.
- simple serial console to perform basic operations (restart and status).
- live gyro calibration if you keep the device still for about 5 secs (green flash confirms gyro bias update).

## serial console

connect via USB to perform basic operations over the serial console, type one of the following commands and press ENTER:

- `restart`: restart device.
- `stat`: toggle show imu, orientation, gyro bias, battery voltage and approximate level.

## LED indication

- solid white: initialization (likely too brief to notice)
- solid/flashing red: error (probably during initialization).
- solid blue: bluetooth connected and updating IMU.
- green flash: gyro calibration bias update.
- yellow: idle, waiting for bluetooth connection.

## pictures

> ### external view. box is about 40mm x 35mm x 20mm.  
> <img src="docs/external view.jpg" width="800">


> ### internal view.  
> <img src="docs/internal view.jpg" width="800">


> ### front side of protoype pcb showing esp32-s3 supermini and battery switch.  
> <img src="docs/pcb front.jpg" width="800">


> ### back side of protoype pcb showing bmi160, voltage divider and battery wires.  
> <img src="docs/pcb back.jpg" width="800">


> ### schematic diagram. esp32 connected to bmi160, simple voltage divider, battery switch and battery connector.
> <img src="docs/schematic.png" width="800">


> ### windows bluetooth setup showing the device being detected and battery level. last 4 hex digits in the device name is the part of the mac address (ec24 in this case).  
> <img src="docs/bluetooth device.png" width="800">


> ### windows usb game controller setup showing the BT Headtracker gamepad controller.  
> <img src="docs/usb game controller.png" width="800">


> ### opentrack setup. change the input source to "Joystick Input". set the Yaw, Pitch and Roll mapping to the appopriate joystick axis (#4, #3, #2). the Yaw output should also be reversed in the Options/Output/Axis Assignment section.
> <img src="docs/opentrack.png" width="800">
> <img src="docs/opentrack output reverse.png" width="800">

