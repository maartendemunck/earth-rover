# Earth Rover

This repository contains the documentation, designs, circuit diagrams, PCB
layouts and firmware of my earth rover (RC Volvo C202) project.

## Electronics

The `electronics` subdirectory contains the circuit diagrams and PCB layouts for
the different components, created with KiCad:

- `human-machine-interface`: main human machine interface (HMI) PCB
- `joystick`: joystick breakout boards for the human machine interface
- `vehicle-control-unit`: main vehicle control unit (VCU) PCB
- `position-encoder`: position encoder for the main driveshaft
- `panel`: panel for fabrication of the PCBs
- `kicad-libraries`: libraries with schematic symbols and component footprints

## Software

The `software/remote-control` subdirectory contains the firmware for a manual
remote control:

- `human-machine-interface`: PlatformIO project for the Teensy 3.2-based human
   machine interface
- `human-machine-interface-display`: Nextion project for the Nextion Enhanced
   NX4827K043 HMI touch display
- `vehicle-control-unit`: PlatformIO project for the Teensy 3.2-based vehicle
   control unit

## License

The earth rover hardware, software and accompanying documentation are released
under the [3-Clause BSD License](https://opensource.org/license/bsd-3-clause/).
