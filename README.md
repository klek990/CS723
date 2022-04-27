# CS723
 
1. Program the DE2-115 using the Quartus Programmer with the provided freq_relay_controller.sof file. This will initialise the components of the board to be interacted with using NiOS.
2. Connect your power, PS2 keyboard, VGA, and USB to the board.
3. Open NiOS Build Tools and run the program.
4. Switch your monitor to the VGA input to show information such as the loads on, current RoC and frequency graphs, current mode, stability, and timing values.
5. When a frequency value or frequency RoC value is exceeded, the system will become unstable and start turning off the RED led’s from right to left (least significant to most significant load). As these loads are shed, the green LED’s will reciprocate a light showing that load has been shed.
6. Use KEY1 on the board to enter maintenance mode. In this mode, you can manually turn on/off loads for the system using the slide switches.. To exit maintenance mode, press KEY1 again. The current mode will also be displayed on the VGA. You can also turn off loads which are currently on with the switches when it is in the load management mode.
7. If all the loads are on and is stable for 500ms or more,, the system will go into normal mode and stop managing loads.
8. The threshold inputs from the keyboard are split into whole values and decimal value steps. To interact with the frequency threshold using the PS2, press ‘F’ to start recording the whole value of the number you wish to set the threshold to (e.g. 51). Enter a number and then press ‘F’ again to move to the decimal recording stage. Enter your number for the decimal part (e.g. 42) and press ‘D’ to exit the decimal recording. The frequency threshold will now be 51.42 Hz. 
9. Similarly, the RoC threshold can also be set using the key ‘R’ and ‘D’, respectively. Only enter integers for both the RoC and frequency thresholds. If any other character is pressed, the value will be set to 0.
10. Timing values are in respect to how fast the system shed’s the first load. The most recent value is the top value and shifts downward as time passes. All times are in milliseconds (ms).
