# mmu2 (firmware clone project)
          
This code was developed on an Arduino Mega board using three 8825 stepper motors to control the MMU2 multi-color print head. 
It was designed to operate with the PRUSA mk3 printer.  This code was developed to replace the stock MMU2 controller board, I got bored so I decided to create my own design.  Getting it to play nicely with the mk3 was quite a challenge.  This home brew MMU2 only will operate with the MK3 currently.

You will need an Arduino Mega 2560 processor board and a RAMPS 1.5 or RAMPS 1.6 shield (for the stepper motor controllers) - both are available on Amazon (see my BOM for additional details)

A serial cable is required (5 pin) in order to communicate between this board and the Mk3 controller.  (Vcc, Tx, Rx, GND, Reset).  Only 3 pins are required (Tx, Rx and Reset) in order to work properly with the Mk3.


WARNING:  This project is a work in progress and requires a knowledge of Mk3 hardware and software.  If you don't know what you 
          are doing, you can/will PERMANENTLY damage your hardware.  I hope you have been appropriately warned.
          
ADDITIONAL WARNING (you probably won't read this because it is too far down the page):

           Stepper Motor controllers (8825) are high current devices and they get warm/hot.  You can damage these controller easily   (I have) if you don't adjust the source current.  There is a very small variable resister you will have to adjust, I recommend setting this to 0.6V (details on how to adjust this are on the polulu.com website - look for 8825 stepper motor controller).  If you don't have a multimeter for measureing voltage then STOP - you are in deep waters).
