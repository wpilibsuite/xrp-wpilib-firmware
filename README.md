# WPILib HAL Simulation - XRP Edition
## Introduction
This repository contains a reference implementation of a [XRP robot](https://www.sparkfun.com/products/22230) that can be controlled via the WPILib XRP extension.

The firmware implements a [custom binary protocol](https://github.com/wpilibsuite/allwpilib/tree/main/simulation/halsim_xrp) over UDP to account for the less performant hardware on the XRP.

## Documentation
Official documentation for the XRP and how to use it with WPILib can be found on the [WPILib Docs site](https://docs.wpilib.org/en/latest/docs/xrp-robot/index.html). The documentation below is also reflected in the official WPILib documentation.

## Installation and Usage

### Firmware installation and upgrades
To install the latest firmware on your XRP, do the following:

* Download the latest firmware UF2 file from Releases
* Plug the XRP into your computer with a USB cable. You should see a red power LED that lights up.
* While holding the BOOTSEL button (the white button on the green Pico W, near the USB connector), quickly press the reset button (middle left side of the XRP board), and then release the BOOTSEL button
* The board will temporariloy disconnect from your computer, and then reconnect as a USB storage device named "RPI-RP2"
  * If this drive does not appear, you can also try unplugging the XRP from your computer, holding the BOOTSEL button down, reconnecting the XRP to your computer and then releasing the BOOTSEL button.
* Drag the UF2 firmware file into the "RPI-RP2" drive, and it will automatically update the firmware
* Once complete, the "RPI-RP2" device will disconnect, and the board should automatically reconnect as a serial device running the WPILib firmware
* At this point, you can disconnect the XRP board from your computer and run it off battery power

### Basic usage
The firmware provides an endpoint for the WPILib Simulation layer that allows WPILib robot programs to interact with real hardware on the XRP over UDP. 

Upon boot up, the following will happen:
* The IMU will calibrate itself. This lasts approximately 3-5 seconds, and will be indicated by the green LED rapidly blinking.
* The network will be configured
  * By default, a WiFi Access point will be created
    * The Access Point will have an SSID of the form "XRP-AAAA-BBBB" where "AAAA-BBBB" are hexadecimal digits representing the unique ID of a particular XRP board
    * The password for the access point is set to "xrp-wpilib" (without the quotes)
  * If set as such (see the section on XRP Configuration), the XRP will either start a custom-named AP, or connect to an existing network

For ideal use, the XRP should be placed on a flat surface prior to power up, and if necessary, users can hit the reset button to restart the firmware and IMU calibration process.

If setup as an Access Point, the configured AP name should appear in the list of available WiFi networks. The XRP will be available at the IP address 192.168.42.1.

If setup in STA mode (i.e. connected to an existing network), the IP address can be determined by either using a tool like Angry IP Scanner, or (more easily), by connecting the XRP to a computer, navigating to the PICODISK removable drive and opening the `xrp-status.txt` file. This file contains information about which network the XRP is connected to, as well as the IP address.

### XRP Configuration
The XRP provides a simple web-based configuration screen that allows users to adjust the network settings. This screen is available at `http://<IP ADDRESS OF XRP>:5000`. By default, this will be `http://192.168.42.1:5000`.

Users can manually edit the JSON configuration to change the AP name/password, or provide a list of networks to connect to in STA mode. Note that an AP name and password must always be provided as the XRP will fallback to generating an AP if it cannot connect to any listed networks. The `mode` field can be switched between `AP` or `STA` depending on the user's preference.

After saving changes, make sure the restart the XRP.

#### Note
As of 10/13/2023, you MUST use the [2024 Beta 1 version](https://github.com/wpilibsuite/allwpilib/releases/tag/v2024.1.1-beta-1) (or later) of WPILib to write XRP programs. There are also examples and templates available (look for "XRP" in the examples/templates dropdown when creating a new project).

## Built-in IO Mapping

### Digital I/O Map
| DIO Port # | Function          |
|------------|-------------------|
| 0          | XRP User Button   |
| 1          | XRP Onboard LED   |
| 2          | RESERVED          |
| 3          | RESERVED          |
| 4          | Left Encoder A    |
| 5          | Left Encoder B    |
| 6          | Right Encoder A   |
| 7          | Right Encoder B   |
| 8          | Motor 3 Encoder A |
| 9          | Motor 3 Encoder B |
| 10         | Motor 4 Encoder A |
| 11         | Motor 4 Encoder B |

### Analog I/O Map
| Analog Port # | Function          |
|---------------|-------------------|
| 0             | Left Reflectance  |
| 1             | Right Reflectance |
| 2             | Rangefinder       |

NOTE: The analog I/O mapping assumes that the reflectance sensor and rangefinder are plugged into the XRP as directed in the setup instructions. All analog I/O channels return values in the range 0-5V.

#### Reflectance Sensors
The reflectance sensors return a value from 0.0V (white) to 5.0V (black).

#### Rangefinder
The maximum range of the rangefinder is 4m, and the minimum detectable range is 2cm. Any values outside of this range will cause the rangefinder to saturate and return the maximum value.

The rangefinder will return a value between 0.0V (min distance) to 5.0V (4m).

### Motor and Servo Map

Instead of pure PWM channels, the XRP uses SimDevices, specifically the `XRPMotor` and `XRPServo` devices. 

| Device  # | Device Type | Function    |
|-----------|-------------|-------------|
| 0         | XRPMotor    | Left Motor  |
| 1         | XRPMotor    | Right Motor |
| 2         | XRPMotor    | Motor 3     |
| 3         | XRPMotor    | Motor 4     |
| 4         | XRPServo    | Servo 1     |
| 5         | XRPServo    | Servo 2     |
