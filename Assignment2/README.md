# Instructions to run
1. Run *make clean* in your Ubuntu/Linux terminal in the directory of this project
2. Run *make cruiseController.xes*
3. Lastly, run *./cruiseController.xes* to execute the program
4. This will open the XES GUI which shows the pure signals, inputs, and outputs of the system
5. The **cruiseState** uses an integer for the different states as output. 1 = OFF, 2 = ON, 3 = STDBY, 4 = DISABLE
6. Click pure signals which will highlight RED to enable them for 1 clock tick
7. Click *tick* to cycle the clock. The output values will be displayed depending on the inputs
8. On, Off, Resume, Set, QuickAccel, QuickDecel are *pure* signals (on or off)
9. Accel, brake, speed, cruiseSpeed, and throttleCmd are represented as *floats*
10. Test cases can be seen in **testCases.xlsx** which show the output of our program being verified
11. For more information, refer to the **A2-T10.pdf** or project brief