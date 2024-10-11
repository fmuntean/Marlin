--------------MFD LowRider v2 CNC Machine -------

This is the configuration for my CNC Machine based on V1 Engineering Low Rider v2

I had to modify the printed parts in order to use EMT pipes from HomeDepot 
EMT 1inch => OD= 29.5mm ($10.4/10ft)for the x axis
EMT 3/4inch => OD=23.5mm ($6.2/10ft)for the Z axis

For the controller I am using the RAMPS 1.4 with the 4 Line Text LCD 
For Power supply currently using a 12V ATX power supply but plan to use a 19V laptop power adapter later on.

The motors are NEMA17 5x17HS5415P1 78oz.in (55N.cm)  ($25.20 on Ebay)

I have tried to use the Z motors in parallel then in series but does not seems to work fine for me at least.
I have decided to use E0 for Y axis second motor and E1 for Z axis second motor

For the stepper drivers I use DRV8825 for X axis and 1/32 micro-stepping giving me 200 steps/mm
For the rest I use A4988:
  Y axis uses 1/16 micro-stepping giving me ~ 100 steps/mm
  Z axis uses 1/4  micro-stepping giving me 400 steps/mm due to the lead screw

For the postprocessing in Fusion 360 I use: https://github.com/guffy1234/mpcnc_posts_processor
https://docs.v1engineering.com/tools/milling-basics/

2020-08-13:
  Added end-stops for X and Y + Y2 axes
  Updated the home procedure to only home the X and Y Axes
  
2020-07-25:
  Added board FAN on D10 to cool the board as now it has a case

2020-07-20:
 Set the STRING_DISTRIBUTION_DATE to current build date
 Disabled all extruders
 Disabled all temperature sensors (this free up 3 analog inputs for future use)
 Disable G0 feed rate: seems that based on MPCNC comment we can't use this (https://www.v1engineering.com/marlin-firmware/)
 