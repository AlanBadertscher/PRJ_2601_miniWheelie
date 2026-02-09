Dabble Bluetooth Gamepad (and much more) for Smartphone
=======================================================

Documentation
=============
https://ai.thestempedia.com/docs/dabble-app/
https://ai.thestempedia.com/docs/dabble-app/getting-started-with-dabble/

Android app
===========
https://play.google.com/store/apps/details?id=io.dabbleapp

iPhone app (untested)
=====================
https://apps.apple.com/us/app/dabble-bluetooth-controller/id1472734455

GitHub
======
https://github.com/STEMpedia/DabbleESP32

Note that the SBRobot project uses a modified DabbleESP32 library.
!!! It will not compile with the unmodified library. !!!

Requires ESP32 Arduino core 2.0.17 (not ESP32 3.0.0 or higher)

Usage
=====
- Compile the SBRobotDabble sketch with ESP32 Arduino core 2.0.17 (or lower).
- Upload executable to robot.
- Use Dabble in Gamepad mode.
- Do not connect the smartphone to the 'SBRobot' Bluetooth device, let Dabble do it for you.
- Place the robot upright, keep it up with a hand or a foot so you can release it easily.
- Switch on the robot. Wait for the connect message to appear on its display.
- Connect the Dabble app to the robot.
- When the app is connected (a smiley is shown), the robot starts to balance.
- Up/Down/Left/Right only work separately. Be careful, remote control is confusing for the 
  user and gets easily out of control, making the robot fall over and even jump around.
- The robot works best on a rough surface.
