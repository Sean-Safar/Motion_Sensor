# Motion_Sensor
Created for Purdue
Created By Sean Safar

Hardware Connections:

	ESP32 Feather Board V2 Pin 14 to MPU-6050 SCL Pin
	ESP32 Feather Board V2 SDA Pin to MPU-6050 SDA Pin
	ESP32 Feather Board 3V Pin to MPU-6050 Vin Pin
	ESP32 Feather Board GND Pin to MPU-6050 GND Pin
	ESP32 USB-C to Computer

Brief Description:

	This program is set up to run as a motion detector. 

Steps:

	Script Runs

	Person Activates Motion Detector with voice command through IFTTT application.

	Motion Sensor is Armed with indication from Green LED. Red LED indicates if motion is present.
	Additionally, if motion is present a notification of the position of the MPU is sent to the person device
	via a notification. 

	Motion Sensor is Disamred with voice command through IFTTT application

Video Demonstration:

https://youtu.be/uOxHamc-1WU
