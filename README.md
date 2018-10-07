# mmu2 (firmware clone project)
          
This code was developed on an Arduino Mega board using three 8825 stepper motors to control the MMU2 multi-color print head. 
It was designed to operate with the PRUSA mk3 printer.  This code was developed to replace the stock MMU2 controller board, I got bored so I decided to create my own design.  Getting it to play nicely with the mk3 was quite a challenge.  This home brew MMU2 only will operate with the MK3 currently.

You will need an Arduino Mega 2560 processor board and a RAMPS 1.5 or RAMPS 1.6 shield (for the stepper motor controllers) - both are available on Amazon (see my BOM for additional details)

A serial cable is required (5 pin) in order to communicate between this board and the Mk3 controller.  (Vcc, Tx, Rx, GND, Reset).  Only 3 pins are required (Tx, Rx and Reset) in order to work properly with the Mk3.


WARNING:  This project is a work in progress and requires a knowledge of Mk3 hardware and software.  If you don't know what you 
          are doing, you can/will PERMANENTLY damage your hardware.  I hope you have been appropriately warned.
