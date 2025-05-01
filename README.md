# Stroke Outpatient Rehabilitation: Measuring Progress using Ankle Range of Motion

Run with CircuitPython on Feather nRF52840, with connections to the Adafruit microSD breakout board as well as 2 BNO085 IMUs, one of them have its I2C address jumped. If you have the Foot PCB these are wired inside.

[Project Document](https://docs.google.com/document/d/1W_u-EETFl5g0VrPbvCA_e4A7lP8_Y1Yrq45xGlHE_Yg/edit#heading=h.p52diav6v98f)

[Fusion Folder](https://mynd483.autodesk360.com/g/projects/20240519767317756/data/dXJuOmFkc2sud2lwcHJvZDpmcy5mb2xkZXI6Y28uY2M0ZkN0QkVSeVc1YURQWVN5Zk9Pdw) If the link doesn't work go to the Wearbale Robotics Lab Fusion Team and select AnkleROM.

### Setup for Code Editing
The project uses CircuitPython as the code platform for the nRF52840. This is because of a lack of support for the BNO08x arduino library for multiple IMUs. I also personally find CircuitPython easier to work with.

To set up your computer to edit CircuitPython efficiently, I highly recommend Pycharm as the code editor. Here is a [Guide](https://learn.adafruit.com/welcome-to-circuitpython/pycharm-and-circuitpython) to setting it up. Click through various tabs on explanations on how to use tools like `screen` to connect to serial output.

The basics of CircuitPython code editing is based on the `code.py` in a mounted drive when connected to the feather. Look through this [Guide](https://learn.adafruit.com/welcome-to-circuitpython/the-circuitpy-drive) for more details.

The data collection code uses vaiours libraries including [`adafruit_bno08x.i2c`](https://docs.circuitpython.org/projects/bno08x/en/latest/api.html#adafruit_bno08x.BNO08X) and [`sdcardio`](https://docs.circuitpython.org/en/latest/shared-bindings/sdcardio/index.html). Look through the documentation for various methods of the library.

### Brief Explanation of Files:

- BNOSD.py: Main Program, maps to code.py on the circuitPython drive mounted.
- live_analyze.py: testing program that outputs useful metrics in real time, for example ankle angle or the rotation matrix. Useful for debugging.
- pyteapot.py: A visualization from PyTeapot of the two IMU's rotations.
- postprocess.py: Analyze data collected stored on the SD card. Graphs ankle angle vs. time.
