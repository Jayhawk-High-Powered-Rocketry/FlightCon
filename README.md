STEP 1 Program esp32:
dont have battery or cable plugged in

Make sure you have platformio installed from the vscode extentions

open the project with platformio

edit the altitude to what is needed

open platformio extension on the left side

build the project after saving file

after a working build plug in esp with cable

now can click upload/upload with monitor

after upload the esp should begin transmitting and 
can be seen doing so in monitor

if nothing is seen tranmitting then click the button on 
the left side of where the cable is plugged in for the esp,
it should say something did not init.

If something did not init then unplug esp

double check if wires are loose then try again

STEP 2 Set up GSC:
Open vscode by clicking top left icon -> programming -> vscode

First terminal:
open a terminal then cd into "/src/moose-bowl-db"

enter command: "spacetime start"

now make a second terminal and leave this first one alone

Only crtl-c out of this terminal when all done, else leave it running the whole time

Second terminal:
