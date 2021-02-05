# ESP32-controlling-VESC
rough cut version of a esp32 controlling the VESC on my DPV. 
Based on blatently stolen snippets, plus whatever I managed to throw together without messing up too badly.

Needs cleaning up!

Contains the definitions, the startup and the processes for each core. Note that the "loop" is empty, as each core has it's own handler. At the end is the portion that handles the multi-click from the trigger to initiate cruise-control and boost power

Core 0 handles wifi, over-the-air update (OTA), and the SFTP server so you can read out the data log file via filezilla.

Core 1 handles the serial communication with the VESC, reads the trigger, rotarty magnet status as speed setting, and writes the log file.
