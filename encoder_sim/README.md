# Encoder Sim
The code is the firmware for the Arduino on the wheelchair simulator. It reads the encoders' counts and then send the data to the computer through serial port.

## How it works
The computer query the data by sending 'd;' to the Arduino and then the Arduino returns the count changes between the current data and the data sent last time. The data is in the format 'ws:left,right;'
