## Stroke Outpatient Rehabilitation: Measuring Progress using Ankle Range of Motion
Run with CircuitPython on Feather nRF52840, with connections to the Adafruit microSD breakout board as well as 2 BNO085 IMUs, one of them have its I2C address jumped.

Brief Explanation of Files:
- BNOSD.py: Main Program, maps to code.py on the circuitPython drive mounted.
- live_analyze.py: testing program that outputs useful metrics in real time, for example ankle angle or the rotation matrix. Useful for debugging.
- pyteapot.py: A visualization from PyTeapot of the two IMU's rotations.
- postprocess.py: Analyze data collected stored on the SD card. Graphs ankle angle vs. time.
