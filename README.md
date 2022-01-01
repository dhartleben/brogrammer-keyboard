# Brogrammer Keyboard Firmware
The package contains the firmware that runs on the Brogrammer Keyboard v1.0

![image1](https://i.imgur.com/jK5W1C7.jpg)

![image2](https://i.imgur.com/0NbhUfi.jpg)

![image3](https://i.imgur.com/J1uUYyI.jpg)

![image4](https://i.imgur.com/Mh007PS.jpg)

See https://imgur.com/a/oY5QZ14

This keyboard runs a specially-compiled verison of Micropython that includes a module for HID keyboard/mouse emulation.

## Editing/Loading The Firmware

In order to edit/program the firmware, the [Thonny IDE](https://thonny.org/) is used.

After launching Thonny, cilck the interpreter selector in the bottom right and switch to Raspberry Pi Pico.

Press the stop icon to stop the currently-running firmware and bring up the interpreter.

If the interpreter is not showing up, there's likely a race condition because of the utilization of the second core, and the fact that there is no coordination between the first core receving the stop signal but not the second core.

To solve this, continue to unplug and replug the keyboard, each time attempting to bring up the interpreter with the stop icon. Usually it is best timed by pressing stop right after the keyboard lights up.