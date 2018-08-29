## Building the firmware in platform.io

This folder contains a platformio.ini file which results in a successful build of the hoverboard firmware.

To use it, you must:

1/ make a new folder.

2/ create a sub-folder called 'src'

3/ move all the files in this repo to that folder.

4/ copy the platformio.ini file to the first folder you created.


Then, you can open the folder in platform.io (e.g. vscode or atom based IDE), and it should build.



There were a couple of changes to the source code to enable build:

b/ 'spoof_init.c' was introduced to provide an empty '_init()' function
(not sure how to include the C++ libs)


In the platformio.ini, src_filter is used to prevent platform.io from building ALL the files in Drivers.

I could not find a way to include the platform.io file inside the repo without moving it.


