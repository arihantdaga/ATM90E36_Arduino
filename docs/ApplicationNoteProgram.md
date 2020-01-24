- Reset
After calling softReset, calibrartion register will be reset. So its mandatory that in the program we should call calibrate right after calling reset(). 

- How to check if calibration registers have been displaced. 
Check the value of registers - ConfigStart, CalStart, HarmStart, AdjStart. If there values are not 0x8675 means they are either in configuration mode(0x5678), or more probably in the power up state that is 0x6886. So if they are not in the right mode it means we should recalibrate the ic. (It'll also restart the ic automatically)



