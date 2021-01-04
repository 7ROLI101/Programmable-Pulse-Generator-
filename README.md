# Programmable-Pulse-Generator-
This is the implementation of the programmable pulse generator that I created using the ATMEGA324A microcontroller.      
There are two modes in this program:  
1) The first one is normal mode where you would input in a numeber from 1-255 and it would generate that many number of pulses at an output pin.  
2) The second one is continuous mode, where you would input in 0 for the number of pulses you want to generate, and it would generate pulses continously, until you press the CLR button or RESET the system.  
  
There are also two implementations of this programmable pulse generator:  
1) The first implements the functionality already discussed above.  
2) The second implements the same functionality as above, but also has two extra parameters for the user to input that will allow different characteristics for the pulses you want to generate.  The first new parameter is t, which is the time it takes for a pulse to stay high (or the on time), and d, which is the delay between pulses (or the off time).
