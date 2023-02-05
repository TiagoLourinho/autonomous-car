#
# testing the comm from python to the EPOS 70/10
#
# this is an adaptation of the code in
# https://khorasani.medium.com/interfacing-and-controlling-maxon-motors-using-python-21ba8e860afe
#
# Note: this only works if the EPOS configuration software is not connected
#

import serial
import time
import numpy as np
import os
from ctypes import *


class MotorController:
    """Communicates with the motor controller"""

    def __init__(self, frequency: float, simulated: bool) -> None:
        self.frequency = frequency
        self.simulated = simulated

        if not self.simulated:
            # EPOS Command Library path
            self.path = os.path.join("windows_dlls", "EposCmd64.dll")

            # Load library
            cdll.LoadLibrary(self.path)
            self.epos = CDLL(self.path)

            # Defining return variables from Library Functions
            self.ret = 0
            self.pErrorCode = c_uint()
            self.pDeviceErrorCode = c_uint()

            # Defining a variable NodeID and configuring connection
            self.nodeID = 1
            self.baudrate = 19200
            self.timeout = 500

            # Configure desired motion profile
            self.acceleration = 30000  # rpm/s, up to 1e7 would be possible
            self.deceleration = 30000  # rpm/s

            # Initiating connection and setting motion profile
            #
            # keyHandle = epos.VCS_OpenDevice(b'EPOS2', b'MAXON SERIAL V2', b'USB', b'USB0', byref(pErrorCode)) # specify EPOS version and interface
            self.keyHandle = self.epos.VCS_OpenDevice(
                b"EPOS", b"MAXON_RS232", b"RS232", b"COM9", byref(self.pErrorCode)
            )  # specify EPOS version and interface
            print("device open\n")

            self.epos.VCS_SetProtocolStackSettings(
                self.keyHandle, self.baudrate, self.timeout, byref(self.pErrorCode)
            )  # set baudrate
            print("protocol set\n")

            self.epos.VCS_ClearFault(
                self.keyHandle, self.nodeID, byref(self.pErrorCode)
            )  # clear all faults
            print("faults cleared\n")

            self.epos.VCS_ActivateProfilePositionMode(
                self.keyHandle, self.nodeID, byref(self.pErrorCode)
            )  # activate profile position mode
            print("profile position activated\n")

            self.epos.VCS_SetEnableState(
                self.keyHandle, self.nodeID, byref(self.pErrorCode)
            )  # enable device
            print("enable state\n")

    def send_control(self, current_phi, control):

        if not self.simulated:
            qc = self.deg_to_qc(
                np.rad2deg(current_phi + control[1] * (1 / self.frequency))
            )
            rpm = self.rad_s_to_rpm(control[1])

            # Making it move
            #
            # stuff under assessment (the Hall sensor is not that precise)

            # gearbox = 0.533 Nm/30 Nm = 1/56.5  max  0.533/4 = 1/7.7
            # optical enc = 6400 cpt
            # Hall sensor = 72 cpt
            # experimental:
            # 1000 qc = 2 3/4 tours approx 1 qc = 1 deg
            # 820 qc approx 2 tours
            # 410 qc it does not move
            # 2050 qc approx 7 tours
            # print(f"qc: {qc}\t rpm: {rpm}")
            # print("going in")
            qc = self.MoveToPositionSpeed(
                round(qc), round(rpm)
            )  # move to position  qc  at rpm
            # print("going out")

            # print("Motor position: %s" % (self.GetPositionIs()))
            # time.sleep(5) # FIXME: Should we sleep?
            # print("move 1 completed\n")

            return np.deg2rad(self.qc_to_deg(qc))
        else:
            return None

    # Query motor position
    def GetPositionIs(self):
        pPositionIs = c_long()
        pErrorCode = c_uint()
        ret = self.epos.VCS_GetPositionIs(
            self.keyHandle, self.nodeID, byref(pPositionIs), byref(pErrorCode)
        )
        # print('got position\n')
        return pPositionIs.value  # motor steps

    # Move to position at speed
    # In general the positioning is not fully accurate so when it reaches a close neighborhood
    # of the target point it stops. Stoping for more than 10s terminates the function
    def MoveToPositionSpeed(self, target_position, target_speed):
        previous_position = self.GetPositionIs()
        flag = 0
        delta_t = 0
        initial_time = time.time()
        while True:
            if target_speed != 0:
                self.epos.VCS_SetPositionProfile(
                    self.keyHandle,
                    self.nodeID,
                    target_speed,
                    self.acceleration,
                    self.deceleration,
                    byref(self.pErrorCode),
                )  # set profile parameters
                # print('position profile set\n')

                self.epos.VCS_MoveToPosition(
                    self.keyHandle,
                    self.nodeID,
                    target_position,
                    True,
                    True,
                    byref(self.pErrorCode),
                )  # move to position
                # print('position command sent\n')

            elif target_speed == 0:
                self.epos.VCS_HaltPositionMovement(
                    self.keyHandle, self.nodeID, byref(self.pErrorCode)
                )  # halt motor
                # print("motion halted\n")

            true_position = self.GetPositionIs()
            delta_pos = true_position - previous_position
            previous_position = true_position

            # print("position {0}   {1}   {2}\n".format(true_position, delta_t, delta_pos))
            if abs(true_position - target_position) < 200:
                break
            elif abs(delta_pos) < 10:
                if flag == 0:
                    # initial_time = time.time()
                    flag = 1
                else:
                    delta_t = time.time() - initial_time
                    if delta_t > (1 / self.frequency) / 2:
                        # print("bailing out {0}\n".format(delta_t))
                        break
            elif abs(delta_pos) > 10:
                initial_time = time.time()

        return self.GetPositionIs()

    def housekeeping(self):

        if not self.simulated:
            # Housekeeping
            self.epos.VCS_SetDisableState(
                self.keyHandle, self.nodeID, byref(self.pErrorCode)
            )  # disable device
            print("device disabled\n")

            self.epos.VCS_CloseDevice(
                self.keyHandle, byref(self.pErrorCode)
            )  # close device
            print("device closed\n")

    def qc_to_deg(self, qc):

        return (3 / 500) * qc

    def deg_to_qc(self, deg):

        return deg / (3 / 500)

    def rad_s_to_rpm(self, omega):
        return omega * 9.549296585513721
