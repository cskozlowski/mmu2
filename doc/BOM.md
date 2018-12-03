DIY Prusa Multi Material Upgrade (MMU) 2.0
==========================================
Bill of Materials (10.20.2018)
------------------------------
For firmware, please refer to https://github.com/cskozlowski/mmu2

Be warned, this is a very difficult project. 
Everyone wants to know the $$$, the list below is under $200 (US). 
The most expensive parts are:  stepper motors ($30 for the three I purchased), arduino ($14) and RAMPS 1.6 board ($14).
This is a starter list of stuff you will need to complete this MMU2 home brew clone

# Change Notes:
* 10.20.18 Added another stepper motor alternative (w/ proper axles) - from aliexpress.com
* 10.17.18 Added 3 STL files required for build of the home-brew MMU2 (do the NEMA17 holder in PETG, other pieces can be 
                                                                          done in PLA or PETG)
* 10.16.18 Changed the stepper motor recommendation, you can also get them on aliexpress.com)
* 10.10.18  Added references to 3 STL design files that you will need (see STL for additional details)
* 10.9.18   626-ZZ should say 625-ZZ bearings ... 626-ZZ are rarer than 1.85mm PTFE
* 10.9.18   Added 7mm ball bearing (you need this for the FINDA detector)
* 10.9.18   Added 5mm x 8mm coupler (need this for the selector stepper motor to threaded rod)

# BOM

## Prusa Parts
* MMU2 plastic parts - https://www.prusa3d.com/prusa-i3-printable-parts/

## Motors

* ~~3 Stepper Motors~~
  (DO NOT PURCHASE THESE MOTORS, THEY DO NOT HAVE SUFFICIENT STALL TORQUE)(https://www.amazon.com/gp/product/B0716S32G4/ref=oh_aui_detailpage_o03_s00?ie=UTF8&psc=1)
            (these motors are super cheap $10 per motor ... don't buy them because they don't have sufficient torque)
            (I used them for 3 weeks and chased problems that won't be there if you get better motors
            (get the motors I list below)
            
            
* 3   Stepper Motors 
  (PURCHASE THESE STEPPER MOTORS INSTEAD)(https://www.amazon.com/gp/product/B06XSYP24P/ref=oh_aui_detailpage_o01_s00?ie=UTF8&psc=1)

  (these motors have 46 N-cm of stall torque.  They idler stepper motor needs quite a bit of stall torque to stay
            in place when the bearing is being moved against the extruder stepper motor).
            
  Get a motor with a minimum of 45 N-cm of stall torque (you will thank me later)
         
  Another potential alternative is these 3 stepper motors (they have proper shafts for the MMU2)
   https://www.aliexpress.com/item/Prusa-i3-mk3-mk2-5-Multi-materials-2-0-3d-printer-motor-kit-MMU2-0/32923672790.html?spm=a2g0s.9042311.0.0.162b4c4dO5RBTY   (I have them on order but have not received them yet for testing)

## Other Stuff
* 5   mk8 hobbed gears (https://www.amazon.com/gp/product/B07CJPP7R7/ref=oh_aui_detailpage_o03_s00?ie=UTF8&psc=1)
* 2   100mm x 5mm steel shafts (https://www.amazon.com/gp/product/B01B27MJC6/ref=oh_aui_detailpage_o04_s00?ie=UTF8&psc=1)
     (these shafts are used to make your 16mm spindles for the bearings,  you can also order 16mm x 5mm spindles from McMcaster-Carr,
     (however, they are wicked tight on the bearings)
* 2   150mm x 5mm steel shafts (https://www.amazon.com/gp/product/B01B5QTM8I/ref=oh_aui_detailpage_o01_s00?ie=UTF8&psc=1)
      (these shafts are used for the selector head shaft guides, you will have to cut them down to size)
      (if you don't know how to cut steel shafts then please stop and go do something else)
* 9   625-ZZ bearings (https://www.amazon.com/gp/product/B01CUTIQWW/ref=oh_aui_detailpage_o04_s01?ie=UTF8&psc=1)
* 20  3mm thin (1.8mm) square nuts) https://www.amazon.com/gp/product/B073SBCMBM/ref=oh_aui_detailpage_o05_s00?ie=UTF8&psc=1
* 1   5mm x 5mm coupler https://www.amazon.com/gp/product/B0159WO7T8/ref=oh_aui_detailpage_o02_s00?ie=UTF8&psc=1
                     (you need one but buy two because you can) 
* 1   5mm x 8mm coupler  (amazon.com ... too lazy to provide the link at the moment).
* 25' 4mm OD x 2mm ID PTFE Tubing (https://www.amazon.com/gp/product/B073RFQQ3Z/ref=oh_aui_detailpage_o09_s00?ie=UTF8&th=1)
                     Needed for the feeders from the spools to the MMU2 unit, you will learn to hate PTFE tubing
* 1   50mm 4mm OD x 1.85mm ID PTFE Tubing (flurostore.com has 1.80mm ID tubing - it works !!!)
                By far the most difficult part to obtain, it should just be called unobtanium
* 1   150mm x 8mm threaded shaft (https://www.amazon.com/gp/product/B07C8P1DWX/ref=oh_aui_detailpage_o08_s01?ie=UTF8&psc=1)
                (yes, you will have to cut it to size using some of your cool shop tools)
* 10  PC4-10 PTFE connectors (https://www.amazon.com/gp/product/B01IB81IHG/ref=oh_aui_detailpage_o02_s00?ie=UTF8&psc=1)
                (get these, they are awesome)
* 1   12-15V, 2A  Power supply - needed for the Arduino Mega/RAMPS board for initial testing
* 1   Arduino Mega 2560 (https://www.amazon.com/gp/product/B01H4ZDYCE/ref=oh_aui_detailpage_o01_s00?ie=UTF8&psc=1)
* 1   1.x RAMPS Boards (https://www.amazon.com/gp/product/B0794YN8XK/ref=oh_aui_detailpage_o09_s00?ie=UTF8&psc=1)
                    (home for all of your stepper motor controllers)
* 3  8825 Stepper Motor Controller (https://www.amazon.com/gp/product/B00S3Q9YZA/ref=oh_aui_detailpage_o05_s00?ie=UTF8&psc=1)
                    (make certain to tweak the voltage rheostat on the top of this little wonder, goto polulu website)
                    (I currently have mine set to 0.60V - don't go too high you will fry this component)
* 1  Razor Blades (https://www.amazon.com/gp/product/B007QYAJRC/ref=oh_aui_detailpage_o07_s00?ie=UTF8&psc=1)
* 1  microswitch   (https://www.amazon.com/gp/product/B06XSBYQ8H/ref=oh_aui_detailpage_o05_s00?ie=UTF8&psc=1)
* 1  FINDA (aka PINDA) Probe (https://www.amazon.com/URBEST-Detecting-Distance-Inductive-Proximity/dp/B01M1777XK/ref=sr_1_fkmr1_3?s=industrial&ie=UTF8&qid=1538915473&sr=1-3-fkmr1&keywords=prusa+pinda+probe)
                  ( I had a spare probe laying around so I have NOT tried this inductive probe)
                  (they are also available from aliexpress.com)
* 1  7mm ball bearing  (mcmaster.carr or amazon), you only need one but you will have to buy many more
* 2  6mm OD x .45mm thick brass tubing (ksmetals.com - part #982, this comes in 300mm length and you will have to cut it down)

## Custom STL Files
* 1  MMU2 to PTFE Festo Coupler 
* 1  MK3 to PTFE Festo Coupler (w/ microswitch holder)
* 1  Selector Motor Mount offset block

## DigiKey Components (needed for cables and wiring)
(to be completed)

      


## Random Tools
(it's a project, you've got to buy new tools - you live for this stuff)
* 1 Reamer (https://www.amazon.com/gp/product/B01EIH573K/ref=oh_aui_detailpage_o07_s00?ie=UTF8&psc=1)
           (used to make awesome interior bevelled edges in that damn 2mm (and 1.80mm) PTFE
* 50mm cutting jig (https://www.thingiverse.com/thing:3062809)    
           (oh look, you get to make something with your 3d printer)
