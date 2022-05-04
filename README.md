# RobotArmCrane_RTES

Gesture Controlled Display using Bluetooth protocol on EFR32BG13 Blue Gecko.

## Table of Contents
* [Description](#description)
* [Author](#author)

## Description
Objective: A motion controlled robotic arm which moves based on the movement of a hand on a FreeRTOS system. This Robotic Arm can be used in many applications. According to the robotic arm in this proof of concept, it can be used as a robotic crane based on the input steering from the user. To mimic
the motion of the hand with a robotic arm, we plan to use a MPU6050, a gyroscope and accelerometer sensor as the sensory device for sensing the motion of hand and a robotic arm with
micro servo motors. The MPU6050 sensor gives the data of position and angle of the hand. This sensor works on I2C communication protocol. The write-read-read process is used to receive
data through I2C communication. Using this value, we calibrate the system to a temporary origin. Using the origin as the reference point, we get the hand movement and read the present
values from the sensor. By using the computation algorithm, the direction is identified and updated in a message queue. We will use the Tiva TM4C123 for our processor, the MPU6050 as our accelerometer, and the
SNAM1500 4 DOF Wood Robotic Mechanical Arm for our robot arm.

#### Design Information and Report:

Refer to the documentation at : https://github.com/PradyumnaGudluru/RobotArmCrane_RTES/blob/main/Project%20Report%20ARM%20CRANE.pdf


#### Presentation Reference:

https://github.com/PradyumnaGudluru/RobotArmCrane_RTES/blob/main/RTES.pdf

#### GitHub URL
URL: https://github.com/PradyumnaGudluru/RobotArmCrane_RTES
  
 ## Author
 * Pradyumna Gudluru