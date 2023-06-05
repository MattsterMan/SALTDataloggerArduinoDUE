This program is designed to capture and log in-flight data for a Level 3 Model Rocket using an Arduino DUE.<br>
Boards:
- Arduino DUE
- Adafruit BME280 I2C or SPI Temperature Humidity Pressure Sensor
- Adafruit Ultimate GPS Breakout - 66 channel w/10 Hz updates - PA1616S
- Adafruit ADXL345 - Triple-Axis Accelerometer (+-2g/4g/8g/16g) w/ I2C/SPI
- MicroSD Card Breakout Board

1. [Installation](#Installation)
2. [Technical Details](#Technical-Details)

#Installation 

I am using Jetbrains CLion for development, therefore installation instructions follow that process. 

1. Navigate to the home screen of CLion to open a project. Select **Get from VCS**.
	1. Version Control: Git
	2. URL: https://github.com/MattsterMan/SALTDataloggerArduinoDUE.git
	3. Directory: PATH TO PROJECTS FOLDER
	4. Click **Clone**.
2. Once the project is opened, the "open project wizard" will show up. Click OK for now.
3. On the Project File Explorer on the left, right click "platformio.ini", then PlatformIO > Re-Init.
		This will download and unpack all necessary libraries defined in "platformio.ini".
4. After all operations in the Run tab have been completed:
	1. File > Settings > Build, Execution, Deployment > CMake
	2. Select Debug, and then click the minus button. This will remove the current CMake configuration.
	3. Click the plus button to add a new CMake Configuration with the correct target. The target should be "dueUSB".
	4. Click **Apply** and then **OK**.
5. Click the dropdown to the left of the green play button at the top of the main window. The current dropdown selected is "Z_DUMMY_TARGET | dueUSB".
6. Click Edit current configurations.
	1. In the new window that pops up, click the plus button at the top left.
	2. Navigate to PlatformIO, hit the dropdown and select PlatformIO Upload.
	3. Click **Apply** and then **OK**.
7. You are now correctly setup to build and upload the current project to an Arduino DUE using the programming USB Port.<br>

#Technical-Details

The purpose of this program is to efficiently and safely communicate with a multitude of sensors, temporarily store their readings in separate arrays, and then write the contents of each array to a microsd card.

Rather than writing to the microsd card every time data is received from the sensors, it is more efficient to temporarily store those values and write them to the microsd card in batches. Because the arduino will be inside a rocket, the faster the data can be recorded, the better.

The values are written to a .csv file with the correct formatting to make data analysis simpler after the rocket is retrieved. A header for each column is written at the start of the program.  Data files and values are never overwritten, but instead new data is appended to the next available line.
