#!/usr/bin/env_python

from pypozyx.tools.version_check import perform_latest_version_check
from pypozyx import (PozyxSerial, PozyxConstants, version, SingleRegister, DeviceRange, POZYX_SUCCESS, POZYX_FAILURE, get_first_pozyx_serial_port)

class CONNECT(object):
    def __init__(self, pozyx, dest_id, range_step_mm, protocol, remote_id):
        self.pypozyx = pozyx
        self.dest_id = dest_id
        self.range_step_mm = range_step_mm
        self.protocol = protocol
        self.remote_id = remote_id
    
    def setup_pozyx(self):
        print("------------POZYX RANGING V{} -------------".format(version))
        print("NOTES: ")
        print(" - Change the parameters: ")
        print("\tdestination_id(target device)")
        print("\trange_step(mm)")
        print("")
        print("- Approach target device to see range and")
        print("led control")
        print("")
        if self.remote_id is None:
            for device_id in [self.remote_id, self.destination_id]:
                self.pozyx.printDeviceInfo(device_id)
        else:
            for device_id in [None, self.remote_id, self.destination_id]:
                self.pozyx.printDeviceInfo(device_id)
        print("")
        print("- -----------POZYX RANGING V{} -------------".format(version))
        print("")
        print("START Ranging: ")

        # make sure the local/remote pozyx system has no control over the LEDs.
        led_config = 0x0
        self.pozyx.setLedConfig(led_config, self.remote_id)
        # do the same for the destination.
        self.pozyx.setLedConfig(led_config, self.destination_id)
        # set the ranging protocol
        self.pozyx.setRangingProtocol(self.protocol, self.remote_id)

    def pozyx_loop(self):
        device_range = DeviceRange()
        status = self.pozyx.doRanging(
            self.destination_id, device_range, self.remote_id)
        dis = device_range.distance
        if status == POZYX_SUCCESS:
            # print(device_range)
            if self.ledControl(device_range.distance) == POZYX_FAILURE:
                kkkkk=1
                # print("ERROR: setting (remote) leds")
                # print("")
        else:
            error_code = SingleRegister()
            status = self.pozyx.getErrorCode(error_code)
            if status == POZYX_SUCCESS:
                print("ERROR Ranging, local %s" %
                      self.pozyx.getErrorMessage(error_code))
            else:
                print("ERROR Ranging, couldn't retrieve local error")
        # print(dis)
        return dis
    
    def ledControl(self, distance):
        status = POZYX_SUCCESS
        ids = [self.remote_id, self.destination_id]
        # set the leds of both local/remote and destination pozyx device
        for id in ids:
            status &= self.pozyx.setLed(4, (distance < self.range_step_mm), id)
            status &= self.pozyx.setLed(3, (distance < 2 * self.range_step_mm), id)
            status &= self.pozyx.setLed(2, (distance < 3 * self.range_step_mm), id)
            status &= self.pozyx.setLed(1, (distance < 4 * self.range_step_mm), id)
        return status
