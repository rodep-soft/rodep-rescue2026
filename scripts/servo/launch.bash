#!/bin/bash

# Before using this script, you must install dynamixelwizard from the official website
# https://www.robotis.com/service/download.php?no=1671

# This is a script for launching Dynamixel Wizard2 in wayland 
# Set an environmental variable to force it to launch by using X11

QT_QPA_PLATFORM=xcb ~/ROBOTIS/DynamixelWizard2/DynamixelWizard2.sh
