polargraph_serverGP
====================

My project is based on Sandy Noble's super polargraph.co.uk project

This Polargraph server  version is connected to the fantastic GCODE converter of Scott-Cooper!

////////////////////////////////////////////////////////////////////////////////////////////////

My Polargraph server works WITHOUT Polarshield touchscreen, only with standard components!
Arduino Mega
standard LCD 4x20
1 x Rotary Encoder
4 x pushbutton

1 x Datalogger Shield with SD card. 
	Modified for MEGA pinout!!!
	UNO  ----> MEGA
	A4,A5  --> Pin 20,21   (SCL,SDA)
	11,12,13-->50,51,52 (SPI bus)

2 x PicoDrive Stepper driver modul

Extended GCODE command set. Drawing directly from SD card or from PC with Sandy Noble's Controller.
https://github.com/euphy/polargraphcontroller 
//////////////////////////////////////////////////////////////////////////////////////////////////

This Polargraph server is connected with the fantastic GCODE converter of Scott-Cooper!
Original version: https://github.com/Scott-Cooper/Drawbot_image_to_gcode_v2
Adapted for Polargraph queue format: https://github.com/gpeter62/Drawbot_image_to_gcode_v2

///////////////////////////////////////////////////////////////////////////////////////////////

Polargraph
----------
Polargraph is the name of the project, and is a portmanteau word invented by the writer
solely for this purpose. Any machine that runs the Polargraph software is technically a 
polargraph machine. I usually reserve the big-P "Polargraph" for things made by
The Polargraph Company, including the Polargraph software and PolargraphSD machine.

Other hanging-v plotters are probably compatible with Polargraph software, but unless
they run it, they are not even polargraphs with a small P.

Project and software written by Sandy Noble.

Released under GNU License version 3.

http://www.polargraph.co.uk


