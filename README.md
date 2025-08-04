# 7MRI0110-MSc-Individual-Research-Project-Ultrasound-Scanning-with-a-Dual-Arm-Robotic-System
This project provides C++ code for controlling a dual-arm robotic system for ultrasound scanning, implemented in MainWindow.cpp and RobotControl.cpp. The system supports manual pose adjustments and automated scanning sequences using Qt for event handling and precise robotic control.
7MRI0110-MSc-Individual-Research-Project-Ultrasound-Scanning-with-a-Dual-Arm-Robotic-System
Overview
This project provides C++ code for controlling a dual-arm robotic system designed for ultrasound scanning. Implemented in MainWindow.cpp and RobotControl.cpp, the system enables manual pose adjustments and automated scanning sequences, leveraging the Qt framework for event handling and precise robotic control. The code facilitates accurate manipulation of robotic arms, with the right arm (ID=1) used for pose capture and the left arm (ID=0) for executing scanning sequences.
Files
MainWindow.cpp

Purpose: Manages user input through keyboard events to control the robotic system manually.
Key Features:
'P' Key: Enters manual mode, pauses the current sequence, captures the right arm's (ID=1) pose, rotates it 180° around the z-axis, and saves it to p_point.txt. Additionally, generates four points with y-offsets (100mm initial, +5mm increments) saved to Prediction_Point.txt.
'S' Key: Initiates an automated scanning sequence using the left arm (ID=0).
Arrow Keys: 'Left' steps backward, 'Right' steps forward in the sequence.
'C' Key: Resumes the paused sequence.


Dependencies: Qt for event handling and file I/O; RobotControl.h for classes like TRTrans3D, Point3D, and TransMatrix3D.

RobotControl.cpp

Purpose: Executes automated scanning sequences for the left arm (ID=0).
Key Features:
Offsets the left arm’s initial pose by 200mm along the y-axis.
Reads poses from Prediction_Point.txt, skipping the first pose, and aligns them to a scan surface using _scanSpace->getOffsetSurfaceAlignedPose.
Executes the sequence, moving the left arm to each aligned pose, with support for cancellation.
Returns the left arm to its initial pose after sequence completion.
Includes error handling for file access and pose validation (requires at least 4 valid poses).


Dependencies: Qt for event processing; standard C++ libraries for file operations.

Usage

Manual Mode: Use 'P' to capture and save poses, 'Left'/'Right' to navigate steps, and 'C' to resume sequences.
Automated Scan: Press 'S' to run the scanning sequence with the left arm.
Output Files:
p_point.txt: Stores the rotated pose of the right arm.
Prediction_Point.txt: Contains the rotated pose and four additional points with incremental y-offsets.



Requirements

Qt Framework: For GUI, event handling, and file operations.
RobotControl.h: Defines necessary classes (TRTrans3D, TransMatrix3D, Point3D, etc.).
C++ Compiler: Must support C++11 or later.
File Paths: Assumes /Users/shichaozhang/USRobot/ for file I/O; adjust as needed.

Installation

Clone the repository.
Ensure Qt and a compatible C++ compiler are installed.
Include RobotControl.h with required class definitions.
Compile and run using a Qt-supported IDE or build system (e.g., qmake).

Notes

The system assumes a dual-arm robotic setup with IDs 0 (left) and 1 (right).
Ensure _scanSpace is properly initialized for pose alignment in RobotControl.cpp.
File paths in the code are hardcoded; modify for different environments.
