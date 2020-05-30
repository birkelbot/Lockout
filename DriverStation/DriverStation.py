#!/usr/bin/env python

# This program was built on Python 3.8.1.

from enum import Enum
from numpy import interp
import pygame
import time
import os
import serial
import array
import sys
import math

# To check what serial ports are available in Linux, use the bash command: dmesg | grep tty
# To check what serial ports are available in Windows, use the cmd command: wmic path Win32_SerialPort
#    OR go to Device Manager > Ports (COM & LPT)
comPort = 'COM6'
ser = serial.Serial(comPort, 57600, timeout=1)

### CONTROL SCHEME ###
# Drive:
#   Arcade Drive, i.e.
#     Left joystick Y-axis -- forward/reverse
#     Right joystick X-axis -- turn/arc
#
# Arm:
#    Right trigger -- arm up (analog control)
#    Left trigger -- arm down (analog control)
#######################

# Set the channel numbers for various controls
AXIS_ID_DRIVE_VELOCITY = 1  # Y-axis translation comes from the left joystick Y axis
AXIS_ID_DRIVE_ROTATION = 4  # Rotation comes from the right joystick X axis
AXIS_ID_ARM = 2  # Analog triggers for arm control
BUTTON_ID_STOP_PROGRAM = 1

def main():

    global ser

    # Initialize the gamepad
    pygame.init()
    joysticks = []
    for i in range(0, pygame.joystick.get_count()):
        joysticks.append(pygame.joystick.Joystick(i))
        joysticks[-1].init()
        print("Detected joystick '",joysticks[-1].get_name(),"'")

    # Local variables
    prevDriveMtrCmds = {'left':0, 'right':0}
    prevArmCmd = 127
    prevTimeSent = 0
    done = False
    loopCounter = 0

    try:
        while (done == False):

            pygame.event.pump()  # This line is needed to process the gamepad packets

            if joystickWatchdog(joysticks[0]):
                sendNeutralCommand()
                continue

            ##### WHEEL COMMANDS #####

            # Get the raw values for drive translation/rotation using the gamepad.
            yRaw = joysticks[0].get_axis(AXIS_ID_DRIVE_VELOCITY)
            rRaw = -joysticks[0].get_axis(AXIS_ID_DRIVE_ROTATION)

            # Get the drive motor commands for Arcade Drive
            driveMtrCmds = arcadeDrive(yRaw, rRaw)
            driveMtrCmds['left'] = driveMtrCmds['left']
            driveMtrCmds['right'] = 254 - driveMtrCmds['right']

            ##########################

            ###### ARM COMMAND #######

            # Get the raw values for the arm using the gamepad
            armRaw = joysticks[0].get_axis(AXIS_ID_ARM)

            # NOTE: Choose linear or exponential drive by changing between
            #       `manualArmLinDrive()` and `manualArmExpDrive()`
            armCmd = manualArmExpDrive(armRaw)

            ##########################

            if joysticks[0].get_button(BUTTON_ID_STOP_PROGRAM):
                cleanup()
                done = True
             # Only send if the commands changed or if 50ms have elapsed
            elif prevDriveMtrCmds['left'] != driveMtrCmds['left'] or \
                 prevDriveMtrCmds['right'] != driveMtrCmds['right'] or \
                 prevArmCmd != armCmd or \
                 time.time()*1000 > prevTimeSent + 50:

                print("Sending... L: ", driveMtrCmds['left'], ", R: ", driveMtrCmds['right'], \
                          ", A: ", armCmd, ", loopCounter: ", loopCounter)
                loopCounter = loopCounter + 1
                ser.write((255).to_bytes(1, byteorder='big'))  # Start byte
                ser.write((driveMtrCmds['left']).to_bytes(1, byteorder='big'))
                ser.write((254-driveMtrCmds['right']).to_bytes(1, byteorder='big'))
                ser.write((254-armCmd).to_bytes(1, byteorder='big'))

                prevDriveMtrCmds = driveMtrCmds
                prevArmCmd = armCmd
                prevTimeSent = time.time()*1000
                time.sleep(0.01)

    except KeyboardInterrupt:
        cleanup()


################################################################################
## @brief  Function to compute the drive motor PWM values for Arcade Drive
## @param  yIn - raw joystick input from -1.0 to 1.0 for the Y-axis translation
## @param  rIn - raw joystick input from -1.0 to 1.0 for the rotation
## @return an array containing left and right motor commands
################################################################################
def arcadeDrive(yIn, rIn):
    
    # Set output command range constants
    zeroCommand = int(127)  # the default value that corresponds to no motor power
    cmdRange = int(127)     # the maximum amount (+/-) that the command can vary from the zero command
    maxCommand = cmdRange
    minCommand = -cmdRange

    # Set constants for the exponential functions for each input (y/r)
    endExpConst = 1.44 # don't change this unless you've really looked over the math

    yExpConst = 1.5  # exponential growth coefficient of the Y-axis translation -- should be between 1.0-4.0
    yEndpoint = 127  # maximum/minumum (+/-) for the Y-axis translation

    rExpConst = 1.5  # exponential growth coefficient of the rotation -- should be between 1.0-4.0
    rEndpoint = 80   # maximum/minimum (+/-) for the rotation

    # Set a deadband for the raw joystick input
    yDeadband = 0.10
    rDeadband = 0.17

    # Set a base command (within the command range above) to overcome gearbox resistance at low drive speeds
    leftMtrBaseCmd = int(2)
    rightMtrBaseCmd = int(3)

    # Save the negative-ness, which will be re-applied after the exponential function is applied
    if yIn < 0:
        yNeg = -1
    else:
        yNeg = 1

    if rIn < 0:
        rNeg = -1
    else:
        rNeg = 1

    # Apply a deadband
    if abs(yIn) < yDeadband:
        yIn = 0
    if abs(rIn) < rDeadband:
        rIn = 0
    
    # Compute the drive commands using the exponential function (zero-based)
    yCmd = int(yNeg*(math.pow(math.e, math.pow(math.fabs(yIn), yExpConst)/endExpConst)-1)*yEndpoint) # zero-based
    rCmd = int(rNeg*(math.pow(math.e, math.pow(math.fabs(rIn), rExpConst)/endExpConst)-1)*rEndpoint) # zero-based

    # Convert the drive commands into motor comands (zero-based)
    leftMtrCmd = yCmd + rCmd   # zero-based
    rightMtrCmd = yCmd - rCmd  # zero-based

    # Add an offset for the minimum command to overcome the gearboxes
    if leftMtrCmd > 0:
        leftMtrCmd = leftMtrCmd + leftMtrBaseCmd
    elif leftMtrCmd < 0:
        leftMtrCmd = leftMtrCmd - leftMtrBaseCmd
    if rightMtrCmd > 0:
        rightMtrCmd = rightMtrCmd + rightMtrBaseCmd
    elif rightMtrCmd < 0:
        rightMtrCmd = rightMtrCmd - rightMtrBaseCmd

    # print("L: ", leftMtrCmd, " R: ", rightMtrCmd)

    # If the commands are greater than the maximum or less than the minimum, scale them back
    maxMtrCmd = max(leftMtrCmd, rightMtrCmd)
    minMtrCmd = min(leftMtrCmd, rightMtrCmd)
    scaleFactor = 1.0
    if maxMtrCmd > maxCommand or minMtrCmd < minCommand:
        if maxMtrCmd > abs(minMtrCmd):
            scaleFactor = float(maxCommand) / float(maxMtrCmd)
        else:
            scaleFactor = float(minCommand) / float(minMtrCmd)

    leftdriveMtrCmdScaled = leftMtrCmd * scaleFactor
    rightdriveMtrCmdScaled = rightMtrCmd * scaleFactor

    # Shift the commands to be based on the zeroCommand (above)
    leftMtrCmdFinal = int(leftdriveMtrCmdScaled + zeroCommand)
    rightMtrCmdFinal = int(rightdriveMtrCmdScaled + zeroCommand)

    return {'left':leftMtrCmdFinal, 'right':rightMtrCmdFinal}


############################################################
## @brief  Function to compute the manual arm drive command
##         following a linear control curve
## @param  aIn - raw input from -1.0 to 1.0
## @return the arm command (0 to 254)
############################################################
def manualArmLinDrive(aIn):
    deadband = 0.01
    if (-deadband < aIn and aIn < deadband):
        aIn = 0.0

    return int(interp(aIn, [-1, 1], [0, 254]))


############################################################
## @brief  Function to compute the manual arm drive command
##         following an exponential control curve
## @param  aIn - raw input from -1.0 to 1.0
## @return the arm command (0 to 254)
############################################################
def manualArmExpDrive(aIn):

    # Set output command range constants
    zeroCommand = int(127)  # the default value that corresponds to no motor power
    cmdRange = int(127)     # the maximum amount (+/-) that the command can vary from the zero command
    maxCommand = cmdRange
    minCommand = -cmdRange

    # Set constants for the exponential function
    endExpConst = 1.44 # don't change this unless you've really looked over the math

    expConst = 1.5  # exponential growth coefficient of the Y-axis translation -- should be between 1.0-4.0
    endpoint = 127  # maximum/minumum (+/-) for the Y-axis translation

    # Set a deadband for the raw joystick input
    deadband = 0.0

    # Set a base command (within the command range above) to overcome gearbox resistance at low drive speeds
    baseCmd = int(5)

    # Save the negative-ness, which will be re-applied after the exponential function is applied
    if aIn < 0:
        neg = -1
    else:
        neg = 1

    # Apply a deadband
    if abs(aIn) < deadband:
        aIn = 0
    
    # Compute the motor command using the exponential function (zero-based)
    aCmd = int(neg*(math.pow(math.e, math.pow(math.fabs(aIn), expConst)/endExpConst)-1)*endpoint) # zero-based

    # Add an offset for the minimum command to overcome the gearboxes
    if aCmd > 0:
        aCmd = aCmd + baseCmd
    elif aCmd < 0:
        aCmd = aCmd - baseCmd

    # If the command is greater than the maximum or less than the minimum, scale it back
    if aCmd > maxCommand:
        aCmd = maxCommand
    elif aCmd < minCommand:
        aCmd = minCommand

    # Shift the command to be based on the zeroCommand (above)
    aCmd = aCmd + zeroCommand

    return aCmd

############################################################
## @brief Run a watchdog check on the joystick
## @param joystick - the pygame joystick object
## @return true if the watchdog thinks the joystick died
############################################################
lastChangeDetected = time.time()*1000
prevAxes = []
prevBtns = []

def joystickWatchdog(joystick):
    global lastChangeDetected
    global prevAxes
    global prevBtns

    if not prevAxes:
        for i in range(0, joystick.get_numaxes()):
            prevAxes.append(joystick.get_axis(i))
    else:
        for i in range(0, joystick.get_numaxes()):
            if prevAxes[i] != joystick.get_axis(i):
                lastChangeDetected = time.time()*1000
            prevAxes[i] = joystick.get_axis(i)

    if not prevBtns:
        for i in range(0, joystick.get_numbuttons()):
            prevBtns.append(joystick.get_button(i))
    else:
        for i in range(0, joystick.get_numbuttons()):
            if prevBtns[i] != joystick.get_button(i):
                lastChangeDetected = time.time()*1000
            prevBtns[i] = joystick.get_button(i)

    # If no change happens in 7000ms, consider the joystick dead
    if time.time()*1000 > lastChangeDetected + 7000:
        return True
    else:
        return False


############################################################
## @brief Zero all the commands to the robot
############################################################
def sendNeutralCommand():

    global ser

    for i in range (0, 3):
        ser.write((255).to_bytes(1, byteorder='big'))
        ser.write((127).to_bytes(1, byteorder='big'))
        ser.write((127).to_bytes(1, byteorder='big'))
        ser.write((127).to_bytes(1, byteorder='big'))


############################################################
## @brief Zero all the commands to the robot and exit
############################################################
def cleanup():

    global ser

    print("Cleaning up and exiting")
    sendNeutralCommand()
    ser.close()
    pygame.quit()
    exit()


if __name__ == '__main__':
    sys.exit(int(main() or 0))
