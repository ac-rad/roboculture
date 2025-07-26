import py_trees
import py_trees_ros
import py_trees.console as console
import rospy
import numpy as np
import actionlib
import serial
import time
import pyvisa
import pyvisa.constants as pv_const
from dynamic_reconfigure.client import Client

class DeviceController():
    def __init__(self):

        self.pipette_actuator_pos = 1850 # current position of the pipette actuator from dyn reconfig
        self.shaker_active = False

        # Open a serial port
        # ls /dev/ttyACM*

        # Pipette
        self.pipette_ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1) ## TODO make param
        time.sleep(1)
        print(self.pipette_ser.readline())
        rospy.loginfo("[PipetteController] Serial port opened successfully")

        # Shaker
        rm = pyvisa.ResourceManager()
        port = '/dev/ttyUSB1'
        res = f'ASRL{port}::INSTR'
        com_settings = {
            'baud_rate': 1200,
            'parity'   : pv_const.Parity.none,
            'stop_bits': pv_const.StopBits.one,
            'data_bits': 8,
            'read_termination': '\r',
            'write_termination': '\r',
            'timeout': 150000,
        }
        '''
        https://www.richmondscientific.com/wp-content/uploads/2023/10/Ohaus-19mm-Orbital-Shaker-SHEX1619DG-RS-232.pdf

        1. Input speed set point and start motor: 'S' + 4 DIGITS, 4 digits are ASCII speed in RPM
        2. Stop motor: 'O'
        3. Dump all parameters: 'U' (may have to run `minicom -con` in seperate terminal to see result)
        4. Clear index number: 'I=*' (may have to run at start)
        '''
        shaker = rm.open_resource(res, com_settings)
        self.shaker_ser = serial.Serial(port, 1200, timeout=100, parity="N", stopbits=1)
        self.shaker_ser.write(bytes(str('U\r'), 'ascii')) # Dump all parameters
        self.shaker_ser.write(bytes(str('O\r'), 'ascii')) # Stop motor

        self.global_reconfigure_client = Client("cell_culture_global_reconfigure", timeout=30, config_callback=self.dyn_reconfig_callback)

    def dyn_reconfig_callback(self, config):
        rospy.loginfo("[PipetteController] Config set to: {pipette_actuator_pos}, {shaker_active}")
        # self.pipette_actuator_pos = config["pipette_actuator_pos"]
        # self.shaker_active = config["shaker_active"]

        # If pipette_actuator_pos changes, send the new position to the arduino
        if self.pipette_actuator_pos != config["pipette_actuator_pos"]:
            self.pipette_actuator_pos = config["pipette_actuator_pos"]
            self.send_pipette_command(self.pipette_actuator_pos)

        # If shaker_active changes, send the new command to the shaker
        if self.shaker_active != config["shaker_active"]:
            self.shaker_active = config["shaker_active"]
            if self.shaker_active:
                # self.shaker_ser.write(bytes(str('S0070\r'), 'ascii')) # Set motor at 70 rpm
                self.shaker_ser.write(bytes(str('S0180\r'), 'ascii')) # Set motor at 70 rpm
            else:
                self.shaker_ser.write(bytes(str('O\r'), 'ascii')) # Stop motor
                rospy.sleep(3)
    
    def send_pipette_command(self, position):
        self.pipette_ser.write(bytes(str(position), 'ascii'))

    def send_shaker_command(self, command):
        self.shaker_ser.write(bytes(str(command), 'ascii'))

if __name__ == "__main__":
    rospy.init_node("device_controller", anonymous=False)
    dc = DeviceController()
    rospy.spin()