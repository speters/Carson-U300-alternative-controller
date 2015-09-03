# Carson U300 alternative controller

This project is an alternative controller for the Carson U300 Unimog RC car.
It replaces the original EM78P540N microcontroller (which was dead on my son's car) and its closed-source firmware with an "Arduino pro mini" Atmel ATmega328p controller and open-source firmware.

## Contents

 * schematics.svg
   Reverse-engineered schematics of the PCB. Made with KiCAD and edited with Inkscape.
   
 * Carson Unimog Steuerplatine Belegung.ods
   LibreOffice/OpenOffice Calc document containing pinouts, etc.
   
 * Carson Unimog Steuerplatine.xcf
   GIMP image file with pictures of the PCB bottom/top. Used while identifying the PCB traces.
   
 * IMG_3559_1280.jpg, IMG_3560_1280.jpg
   Photos of the messy wiring
   
 * rccontroller.ino
   The Arduino based firmware. Uses [EnableInterrupt library](https://github.com/GreyGnome/EnableInterrupt).
   
## What works and what not

Receiving RC channels, and driving the motors and lights work.

## TODOs
* Enabling the sound module
* Output of blinking sequences
* New features
