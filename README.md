camera-slider
=============

A very basic camera slider control program for Arduino

This is a very basic camera slider control program for Arduino.  It is basically a single axis 
stepper motor controller.  It has a menu driven interface that is acessed via any serial terminal 
including the one built into the Arduino IDE at 9600 baud.  The controller can store up to 24 moves/dwells 
in EEPROM and replay them later with a button push.

This is the menu that is displayed


0 = Set Current Location as 0

S = Stop now!

D = Disable Motor

E = Enable Motor

H = Home (Move to 0)

M = Move to ..(M Dest Speed Accel)

J = Jog until stopped ('J 1' for positive motion, 'J -1' for negative, 'J' to stop )

I = Info (current parameters)


G = Go (start program)

P = Show Program"

C = Clear Program

L = Edit Line.  Format: L Line# Dest Speed Accel)  Ex: L 0 2000 3000 1500

	Use 0 for destination and speed to indicate end of program
	
	Use 0 for speed to indicate a pause.  Dest is pause in milliseconds
      
R = Set Max Speed

A = Set Max Accel

V = SaVe to EEPROM

? = Redisplay this menu


Note:  This is more of a starter program than a polished contrller.  It does have a true timer 
interrupt step generator that can be the basis of a more comprehensive program.

The program is currently compatible with these (2) driver shields

http://www.reactivesubstance.com/shop.html (recommended)
http://imall.iteadstudio.com/im120417015.html (not recommended...overheats)


