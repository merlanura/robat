# RoBat Robot Construction Kit #

RoBat is a great robot kit with which you can build your own robot. It's easy and you can design your robot according to your ideas. The instructions explain how to use the kit.

The instructions are for a wheeled robot. You can either control it yourself with a small joystick or let it find its own way.

In contrast to many other robot kits, Robat does not have a specified chassis. On the one hand, this would make it easier to set up, but on the other hand it would limit the possibilities. With a piece of sturdy cardboard and a hot glue gun you can easily build your own chassis.

Please find the RoBat manual in https://github.com/merlanura/robat/documentation/robat_anleitung_v03.pdf

Have fun!

## Serial commands ##

RoBat can be controlled from te outside via serial commands. Te UART pins TX and RX are used at a speed of 115200 b/s.

The commands are as follows:

sercmd - Wechsel in den seriellen Kommando-Modus. Dieser Befehl wird als einziger auch angenommen, wenn der Roboter im manuellen Modus ist.
mancmd - Wechsel in den manuellen Modus. Serielle Kommandos bleiben ohne Funktion mit Ausnahme von sercmd.
lf - linker Motor vorwärts (right forward)
lb - linker Motor rückwärts (right backwards)
rf - rechter Motor vorwärts (right forward)
rb - rechter Motor rückwärts (right backwards)
st - beide Motoren anhalten (stop)

Driving commands must be repeated at least every 5 seconds. Otherwise, the robot will stop. This is a security measure in case the external controller dies or disconnects.
