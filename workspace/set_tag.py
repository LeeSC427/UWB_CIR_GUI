#!/usr/bin/env_python

from pypozyx.tools.version_check import perform_latest_version_check
from pypozyx import (PozyxSerial, PozyxConstants, version, SingleRegister, DeviceRange, POZYX_SUCCESS, POZYX_FAILURE, get_first_pozyx_serial_port)
from connect import CONNECT

class SET_TAGS:
    def __init__(self, dest_id_1, dest_id_2, dest_id_3, dest_id_4, dest_id_5, dest_id_6, remote_id_0):
        self.dest_id_1 = dest_id_1
        self.dest_id_2 = dest_id_2
        self.dest_id_3 = dest_id_3
        self.dest_id_4 = dest_id_4
        self.dest_id_5 = dest_id_5
        self.dest_id_6 = dest_id_6

        self.remote_id_0 = remote_id_0

        self.serial_port = None

        self.range_step_mm = 1000
        self.protocol = PozyxConstants.RANGE_PROTOCOL_PRECISION

        self.pozyx = None

        self.r_0 = None
        self.r_1 = None
        self.r_2 = None
        self.r_3 = None
        self.r_4 = None
        self.r_5 = None

        self.data = None

        self.set_serial_port()
        self.set_r()

    def set_serial_port(self):
        check_pypozyx_version = True
        if check_pypozyx_version:
            perform_latest_version_check()

        self.serial_port = get_first_pozyx_serial_port()

        if self.serial_port is None:
            print("No Pozyx connected. Check your USB cable or your driver!")
            quit()
    
    def set_r(self):
        pozyx = PozyxSerial(self.serial_port)
        self.r_0 = CONNECT(pozyx, self.dest_id_1, self.range_step_mm, self.protocol, self.remote_id_0)
        self.r_1 = CONNECT(pozyx, self.dest_id_2, self.range_step_mm, self.protocol, self.remote_id_0)
        self.r_2 = CONNECT(pozyx, self.dest_id_3, self.range_step_mm, self.protocol, self.remote_id_0)
        self.r_3 = CONNECT(pozyx, self.dest_id_4, self.range_step_mm, self.protocol, self.remote_id_0)
        self.r_4 = CONNECT(pozyx, self.dest_id_5, self.range_step_mm, self.protocol, self.remote_id_0)
        self.r_5 = CONNECT(pozyx, self.dest_id_6, self.range_step_mm, self.protocol, self.remote_id_0)
