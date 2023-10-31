#!/usr/bin/env python
"""
The Pozyx ready to range tutorial (c) Pozyx Labs
Please read the tutorial that accompanies this sketch: https://www.pozyx.io/Documentation/Tutorials/ready_to_range/Python

This demo requires two Pozyx devices. It demonstrates the ranging capabilities and the functionality to
to remotely control a Pozyx device. Move around with the other Pozyx device.

This demo measures the range between the two devices. The closer the devices are to each other, the more LEDs will
light up on both devices.
"""
from pypozyx import (PozyxSerial, PozyxConstants, version,
                     SingleRegister, DeviceRange, POZYX_SUCCESS, POZYX_FAILURE, get_first_pozyx_serial_port)

from pypozyx.tools.version_check import perform_latest_version_check
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from multiprocessing import Process
import time
from pytictoc import TicToc
import psutil
import csv
ttt = TicToc()
data_history=[]
# Tag1 = np.array([3640, 6325]).reshape(2,1)
# # print(Test)
# Tag2 = np.array([3640, 9730]).reshape(2,1)
# Tag3 = np.array([3640,3100]).reshape(2,1)
# est1 = np.array([1900, 7300]).reshape(2,1)
# est2 = np.array([2000, 5300]).reshape(2,1)
# est3 = np.array([3600, 7500]).reshape(2,1)
# est1 = np.array([100, 140]).reshape(2,1)
# est2 = np.array([200, 320]).reshape(2,1)
# est3 = np.array([250, 250]).reshape(2,1)
# ap1 = np.array([650, 11125]).reshape(2,1)
# ap2 = np.array([650, 3700]).reshape(2,1)
# ap3 = np.array([6950, 3100]).reshape(2,1)
# ap4 = np.array([6100, 11405]).reshape(2,1)

est1 = np.array([100, 140]).reshape(2,1)
est2 = np.array([200, 320]).reshape(2,1)
est3 = np.array([250, 250]).reshape(2,1)
# ap1 = np.array([0, 0]).reshape(2,1)
# ap2 = np.array([3600, 0]).reshape(2,1)
# ap3 = np.array([0, 3600]).reshape(2,1)
# ap4 = np.array([3600, 3600]).reshape(2,1)
ap1 = np.array([0, 0]).reshape(2,1)
ap2 = np.array([1800, 0]).reshape(2,1)
ap3 = np.array([0, 1200]).reshape(2,1)
ap4 = np.array([1800, 1200]).reshape(2,1)

t1_comp1=np.array([0,0]).reshape(2,1)
t1_comp2=np.array([0,0]).reshape(2,1)
t1_compAP1=np.array([0,0]).reshape(2,1)
t1_compAP2=np.array([0,0]).reshape(2,1)
t1_compAP3=np.array([0,0]).reshape(2,1)
t1_compAP4=np.array([0,0]).reshape(2,1)

t2_comp1=np.array([0,0]).reshape(2,1)
t2_comp2=np.array([0,0]).reshape(2,1)
t2_compAP1=np.array([0,0]).reshape(2,1)
t2_compAP2=np.array([0,0]).reshape(2,1)
t2_compAP3=np.array([0,0]).reshape(2,1)
t2_compAP4=np.array([0,0]).reshape(2,1)

t3_comp1=np.array([0,0]).reshape(2,1)
t3_comp2=np.array([0,0]).reshape(2,1)
t3_compAP1=np.array([0,0]).reshape(2,1)
t3_compAP2=np.array([0,0]).reshape(2,1)
t3_compAP3=np.array([0,0]).reshape(2,1)
t3_compAP4=np.array([0,0]).reshape(2,1)

kk =0

dff1=[]
dff2=[]
dff3=[]
sigma_d = 0.1

J=6
T=10

a = np.zeros((2,T+1))
b = np.zeros((2,T+1))
c = np.zeros((2,T+1))

aa = np.zeros((2,T))
bb = np.zeros((2,T))
cc = np.zeros((2,T))

# print(bb)
JJ = 1/(J+1)

def dis(x1, x2):
    d = x2 - x1
    dx = d[0]
    dy = d[1]
    d = d[0] ** 2 + d[1] ** 2
    d = np.sqrt(d)
    return d, dx, dy


def diff(x1, x2):
    dd = x2 - x1
    dx = dd[0]
    dy = dd[1]

    return dx, dy


def func1(x, y, p, sigma_d, est, comp, z):
    r = np.sqrt(x ** 2 + y ** 2)
    rx = x / r
    ry = y / r

    xx = sigma_d ** 2 / ((1 / p) * rx ** 2 + sigma_d ** 2)
    xxx = ((1 / p) * rx) / ((1 / p) * rx ** 2 + sigma_d ** 2)

    yy = sigma_d ** 2 / ((1 / p) * ry ** 2 + sigma_d ** 2)
    yyy = ((1 / p) * ry) / ((1 / p) * ry ** 2 + sigma_d ** 2)

    p0 = xx * (est[0] - comp[0]) - xxx * (z - r - rx * est[0])
    p1 = yy * (est[1] - comp[1]) - yyy * (z - r - ry * est[1])

    predic = np.array([p0, p1]).reshape(2, 1)

    # print(p0,p1)
    comp0 = comp[0] + p0 - est[0]
    comp1 = comp[1] + p1 - est[1]

    comp = np.array([comp0, comp1]).reshape(2, 1)
    # return predic, comp
    return predic, comp


class ReadyToRange(object):
    """Continuously performs ranging between the Pozyx and a destination and sets their LEDs"""

    def __init__(self, pozyx, destination_id, range_step_mm=1000, protocol=PozyxConstants.RANGE_PROTOCOL_PRECISION,
                 remote_id=None):
        self.pozyx = pozyx
        self.destination_id = destination_id
        self.range_step_mm = range_step_mm
        self.remote_id = remote_id
        self.protocol = protocol

    def setup(self):
        """Sets up both the ranging and destination Pozyx's LED configuration"""
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

    def loop(self):
        """Performs ranging and sets the LEDs accordingly"""
        device_range = DeviceRange()
        status = self.pozyx.doRanging(
            self.destination_id, device_range, self.remote_id)
        dis = device_range.distance
        if status == POZYX_SUCCESS:
            # print(device_range)
            if self.ledControl(device_range.distance) == POZYX_FAILURE:
                print("ERROR: setting (remote) leds")
        else:
            error_code = SingleRegister()
            status = self.pozyx.getErrorCode(error_code)
            if status == POZYX_SUCCESS:
                print("ERROR Ranging, local %s" %
                      self.pozyx.getErrorMessage(error_code))
            else:
                print("ERROR Ranging, couldn't retrieve local error")
        return dis
    def ledControl(self, distance):
        """Sets LEDs according to the distance between two devices"""
        status = POZYX_SUCCESS
        ids = [self.remote_id, self.destination_id]
        # set the leds of both local/remote and destination pozyx device
        for id in ids:
            status &= self.pozyx.setLed(4, (distance < range_step_mm), id)
            status &= self.pozyx.setLed(3, (distance < 2 * range_step_mm), id)
            status &= self.pozyx.setLed(2, (distance < 3 * range_step_mm), id)
            status &= self.pozyx.setLed(1, (distance < 4 * range_step_mm), id)
        return status

def dis(x1, x2):
    d = x2 - x1
    dx = d[0]
    dy = d[1]
    d = d[0] ** 2 + d[1] ** 2
    d = np.sqrt(d)
    return d, dx, dy


def diff(x1, x2):
    dd = x2 - x1
    dx = dd[0]
    dy = dd[1]

    return dx, dy


def func1(x, y, p, sigma_d, est, comp, z):
    r = np.sqrt(x ** 2 + y ** 2)
    rx = x / r
    ry = y / r

    xx = sigma_d ** 2 / ((1 / p) * rx ** 2 + sigma_d ** 2)
    xxx = ((1 / p) * rx) / ((1 / p) * rx ** 2 + sigma_d ** 2)

    yy = sigma_d ** 2 / ((1 / p) * ry ** 2 + sigma_d ** 2)
    yyy = ((1 / p) * ry) / ((1 / p) * ry ** 2 + sigma_d ** 2)

    p0 = xx * (est[0] - comp[0]) - xxx * (z - r - rx * est[0])
    p1 = yy * (est[1] - comp[1]) - yyy * (z - r - ry * est[1])

    predic = np.array([p0, p1]).reshape(2, 1)

    # print(p0,p1)
    comp0 = comp[0] + p0 - est[0]
    comp1 = comp[1] + p1 - est[1]

    comp = np.array([comp0, comp1]).reshape(2, 1)
    # return predic, comp
    return predic, comp

def fig(est1,est2,est3,ap1,ap2,ap3,ap4):

    plt.close('all')
    plt.plot(Tag1[0],Tag1[1],'ro',label="Veh1")
    plt.plot(Tag2[0],Tag2[1],'bo',label="Veh2")
    plt.plot(Tag3[0],Tag3[1],'go',label='Veh3')
    # plt.plot(a[0, :], a[1, :], 'ro', label="Veh1")
    # plt.plot(b[0, :], b[1, :], 'go', label="Veh2")
    # plt.plot(c[0, :], c[1, :], 'bo', label='Veh3')
    # plt.text(Tag1[0],Tag1[1],'True1')
    # plt.text(Tag2[0],Tag2[1],'True2')
    # plt.plot(aa[0, :], aa[1, :], 'r-x', label="Est1")
    # plt.plot(bb[0, :], bb[1, :], 'g-x', label="Est2")
    # plt.plot(cc[0, :], cc[1, :], 'b-x', label="Est3")
    plt.plot(est1[0], est1[1], 'r-x', label="Est1")
    plt.plot(est2[0], est2[1], 'g-x', label="Est2")
    plt.plot(est3[0], est3[1], 'b-x', label="Est3")
    # plt.plot(ori_est1[0],ori_est1[1],'rd',label="Init1")
    # plt.plot(ori_est2[0],ori_est2[1],'bd',label="Init2")
    # plt.plot(ori_est3[0],ori_est3[1],'gd',label="Init3")
    plt.plot(ap1[0], ap1[1], 'kd', label="AP1")
    plt.plot(ap2[0], ap2[1], 'kd', label="AP2")
    plt.plot(ap3[0], ap3[1], 'kd', label="AP3")
    plt.plot(ap4[0], ap4[1], 'kd', label="AP4")
    plt.legend(loc=1, ncol=3, fontsize=8)
    plt.xlabel('X-axis [mm]')
    plt.ylabel('Y-axis [mm]')
    plt.title('Python language programming')
    plt.grid(True)
    plt.show()
    # time.sleep(1)
    plt.close()

if __name__ == "__main__":
    # Check for the latest PyPozyx version. Skip if this takes too long or is not needed by setting to False.
    check_pypozyx_version = True
    if check_pypozyx_version:
        perform_latest_version_check()

    # hardcoded way to assign a serial port of the Pozyx
    serial_port = 'COM12'

    # the easier way
    serial_port = get_first_pozyx_serial_port()
    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()

    remote_id = 0x6824           # the network ID of the remote device
    remote = True               # whether to use the given remote device for ranging
    if not remote:
        remote_id = None

    remote_id1 = 0x6846
    remote = True
    if not remote:
        remote_id = None

    remote_id2 = 0x6862
    remote = True
    if not remote:
        remote_id = None

    # destination_id1 = 0x0D5D
    # destination_id2 = 0x687B
    # destination_id3 = 0x684E
    # destination_id4 = 0x684F

    destination_id1 = 0x1162
    destination_id2 = 0x114A
    destination_id3 = 0x1126
    destination_id4 = 0x1136

    # distance that separates the amount of LEDs lighting up.
    range_step_mm = 1000

    # the ranging protocol, other one is PozyxConstants.RANGE_PROTOCOL_PRECISION
    ranging_protocol = PozyxConstants.RANGE_PROTOCOL_PRECISION

    pozyx = PozyxSerial(serial_port)
    r = ReadyToRange(pozyx, destination_id1, range_step_mm, ranging_protocol, remote_id)
    r1 = ReadyToRange(pozyx, destination_id2, range_step_mm, ranging_protocol, remote_id)
    r2 = ReadyToRange(pozyx, destination_id3, range_step_mm, ranging_protocol, remote_id)
    r3 = ReadyToRange(pozyx, destination_id4, range_step_mm, ranging_protocol, remote_id)
    r4 = ReadyToRange(pozyx, remote_id1, range_step_mm, ranging_protocol, remote_id)
    r5 = ReadyToRange(pozyx, remote_id2, range_step_mm, ranging_protocol, remote_id)

    rr = ReadyToRange(pozyx, destination_id1, range_step_mm, ranging_protocol, remote_id1)
    rr1 = ReadyToRange(pozyx, destination_id2, range_step_mm, ranging_protocol, remote_id1)
    rr2 = ReadyToRange(pozyx, destination_id3, range_step_mm, ranging_protocol, remote_id1)
    rr3 = ReadyToRange(pozyx, destination_id4, range_step_mm, ranging_protocol, remote_id1)
    rr4 = ReadyToRange(pozyx, remote_id, range_step_mm, ranging_protocol, remote_id1)
    rr5 = ReadyToRange(pozyx, remote_id2, range_step_mm, ranging_protocol, remote_id1)

    rrr = ReadyToRange(pozyx, destination_id1, range_step_mm, ranging_protocol, remote_id2)
    rrr1 = ReadyToRange(pozyx, destination_id2, range_step_mm, ranging_protocol, remote_id2)
    rrr2 = ReadyToRange(pozyx, destination_id3, range_step_mm, ranging_protocol, remote_id2)
    rrr3 = ReadyToRange(pozyx, destination_id4, range_step_mm, ranging_protocol, remote_id2)
    rrr4 = ReadyToRange(pozyx, remote_id, range_step_mm, ranging_protocol, remote_id2)
    rrr5 = ReadyToRange(pozyx, remote_id1, range_step_mm, ranging_protocol, remote_id2)



    r.setup()
    while True:
        r.loop()
        # print(destination_id)
        # data_history = data_history + [[r]]
        # print(r)
        # ddd = pd.DataFrame(data_history)
        # filename = time.strftime('%m%d%H%M'+'.csv')
        # data_path = ''
        # ddd.to_csv('ddd.csv', index=False)
        # ttt.tic()
        d1= np.array([r.loop(), r1.loop(), r2.loop(), r3.loop(), r4.loop(), r5.loop()])
        d2= np.array([rr.loop(), rr1.loop(), rr2.loop(), rr3.loop(), rr4.loop(), rr5.loop()])
        d3= np.array([rrr.loop(), rrr1.loop(), rrr2.loop(), rrr3.loop(), rrr4.loop(), rrr5.loop()])
        # d1 = np.array([[r.loop()], [r1.loop()], [r2.loop()], [r3.loop()], [r4.loop()], [r5.loop()]])
        # d2 = np.array([[rr.loop()], [rr1.loop()], [rr2.loop()], [rr3.loop()], [rr4.loop()], [rr5.loop()]])
        # d3 = np.array([[rrr.loop()], [rrr1.loop()], [rrr2.loop()], [rrr3.loop()], [rrr4.loop()], [rrr5.loop()]])

        kk=kk+1
        print(kk)

        dff1 = pd.DataFrame([d1])
        print(dff1)
        dff1.to_csv('data_cp8_t1.csv',mode='a', header=False, index=False)
        dff2 = pd.DataFrame([d2])
        dff2.to_csv('data_cp8_t2.csv',mode='a', header=False, index=False)
        dff3 = pd.DataFrame([d3])
        dff3.to_csv('data_cp8_t3.csv',mode='a', header=False, index=False)

        # print(d1,d2,d3)

        # print("r1:", np.array([r.loop(), r1.loop(), r2.loop(), r3.loop(), r4.loop(), r5.loop()]))
        # print("r2:", np.array([rr.loop(), rr1.loop(), rr2.loop(), rr3.loop(), rr4.loop(), rr5.loop()]))
        # print("r3:", np.array([rrr.loop(), rrr1.loop(), rrr2.loop(), rrr3.loop(), rrr4.loop(), rrr5.loop()]))
        t = 0
        
        p = 100
   
        for i in range(0, T):
            # i=i+1
            [t1_x1, t1_y1] = diff(est1, est2)
            [t1_x2, t1_y2] = diff(est1, est3)
            [t1_ap1x, t1_ap1y] = diff(est1, ap1)
            [t1_ap2x, t1_ap2y] = diff(est1, ap2)
            [t1_ap3x, t1_ap3y] = diff(est1, ap3)
            [t1_ap4x, t1_ap4y] = diff(est1, ap4)

            [t2_x1, t2_y1] = diff(est2, est1)
            [t2_x2, t2_y2] = diff(est2, est3)
            [t2_ap1x, t2_ap1y] = diff(est2, ap1)
            [t2_ap2x, t2_ap2y] = diff(est2, ap2)
            [t2_ap3x, t2_ap3y] = diff(est2, ap3)
            [t2_ap4x, t2_ap4y] = diff(est2, ap4)

            [t3_x1, t3_y1] = diff(est3, est1)
            [t3_x2, t3_y2] = diff(est3, est2)
            [t3_ap1x, t3_ap1y] = diff(est3, ap1)
            [t3_ap2x, t3_ap2y] = diff(est3, ap2)
            [t3_ap3x, t3_ap3y] = diff(est3, ap3)
            [t3_ap4x, t3_ap4y] = diff(est3, ap4)
            
            # print(r4.loop())

            # p0,p1,c0,c1 = func1(t1_x1,t1_y1,p,sigma_d,est1,t1_predic1,t1_comp1,t1_a_d)
            [t1_predic1, t1_comp1] = func1(t1_x1, t1_y1, p, sigma_d, est1, t1_comp1, d1[4])
            [t1_predic2, t1_comp2] = func1(t1_x2, t1_y2, p, sigma_d, est1, t1_comp2, d1[5])
            [t1_predicAP1, t1_compAP1] = func1(t1_ap1x, t1_ap1y, p, sigma_d, est1, t1_compAP1, d1[0])
            [t1_predicAP2, t1_compAP2] = func1(t1_ap2x, t1_ap2y, p, sigma_d, est1, t1_compAP2, d1[1])
            [t1_predicAP3, t1_compAP3] = func1(t1_ap3x, t1_ap3y, p, sigma_d, est1, t1_compAP3, d1[2])
            [t1_predicAP4, t1_compAP4] = func1(t1_ap4x, t1_ap4y, p, sigma_d, est1, t1_compAP4, d1[3])
            t1_predic = t1_predic1 + t1_predic2 + t1_predicAP1 + t1_predicAP2 + t1_predicAP3 + t1_predicAP4
            t1_comp = t1_comp1 + t1_comp2 + t1_compAP1 + t1_compAP2 + t1_compAP3 + t1_compAP4

            [t2_predic1, t2_comp1] = func1(t2_x1, t2_y1, p, sigma_d, est2, t2_comp1, d2[4])
            [t2_predic2, t2_comp2] = func1(t2_x2, t2_y2, p, sigma_d, est2, t2_comp2, d2[5])
            [t2_predicAP1, t2_compAP1] = func1(t2_ap1x, t2_ap1y, p, sigma_d, est2, t2_compAP1, d2[0])
            [t2_predicAP2, t2_compAP2] = func1(t2_ap2x, t2_ap2y, p, sigma_d, est2, t2_compAP2, d2[1])
            [t2_predicAP3, t2_compAP3] = func1(t2_ap3x, t2_ap3y, p, sigma_d, est2, t2_compAP3, d2[2])
            [t2_predicAP4, t2_compAP4] = func1(t2_ap4x, t2_ap4y, p, sigma_d, est2, t2_compAP4, d2[3])

            t2_predic = t2_predic1 + t2_predic2 + t2_predicAP1 + t2_predicAP2 + t2_predicAP3 + t2_predicAP4
            t2_comp = t2_comp1 + t2_comp2 + t2_compAP1 + t2_compAP2 + t2_compAP3 + t2_compAP4

            [t3_predic1, t3_comp1] = func1(t3_x1, t3_y1, p, sigma_d, est3, t3_comp1, d3[4])
            [t3_predic2, t3_comp2] = func1(t3_x2, t3_y2, p, sigma_d, est3, t3_comp2, d3[5])
            [t3_predicAP1, t3_compAP1] = func1(t3_ap1x, t3_ap1y, p, sigma_d, est3, t3_compAP1, d3[0])
            [t3_predicAP2, t3_compAP2] = func1(t3_ap2x, t3_ap2y, p, sigma_d, est3, t3_compAP2, d3[1])
            [t3_predicAP3, t3_compAP3] = func1(t3_ap3x, t3_ap3y, p, sigma_d, est3, t3_compAP3, d3[2])
            [t3_predicAP4, t3_compAP4] = func1(t3_ap4x, t3_ap4y, p, sigma_d, est3, t3_compAP4, d3[3])

            t3_predic = t3_predic1 + t3_predic2 + t3_predicAP1 + t3_predicAP2 + t3_predicAP3 + t3_predicAP4
            t3_comp = t3_comp1 + t3_comp2 + t3_compAP1 + t3_compAP2 + t3_compAP3 + t3_compAP4

            est1 = JJ * (est1 + t1_predic + t1_comp)
            est2 = JJ * (est2 + t2_predic + t2_comp)
            est3 = JJ * (est3 + t3_predic + t3_comp)


            # a= np.append(a, np.array([Tag1[0],Tag1[1]]))
            # a[0, t] = Tag1[0]
            # a[1, t] = Tag1[1]
            aa[0, i] = est1[0]
            aa[1, i] = est1[1]
            # b = np.append(b,np.array([Tag2[0],Tag2[1]]))
            # print(t,Tag2[0])
            # b[0, t] = Tag2[0]
            # b[1, t] = Tag2[1]
            bb[0, i] = est2[0]
            bb[1, i] = est2[1]

            # print(bb)
            # c = np.append(c,np.array([Tag3[0],Tag3[1]]))
            # c[0, t] = Tag3[0]
            # c[1, t] = Tag3[1]
            cc[0, i] = est3[0]
            cc[1, i] = est3[1]
        

        print("Est1:", est1)
        print("Est2:", est2)
        print("Est3:", est3)
        # err1=dis(Tag1,est1)
        # err2=dis(Tag2,est2)
        # err3=dis(Tag3,est3)
        # print(err1[0],err2[0],err3[0])



        ttt.toc()
        # p = Process(target=fig, args=(est1,est2,est3,ap1,ap2,ap3,ap4))
        # p.start()
        # p.join

        # print("aa",aa)
        # print("bb",bb)
        # print("cc",cc)


        # plt.figure
        # plt.plot(Tag1[0]/1000,Tag1[1]/1000,'mo',markerfacecolor='#e377c2',mec='k',label="Tag1(True1)")
        # plt.plot(Tag2[0]/1000,Tag2[1]/1000,'yo',markerfacecolor='#bcbd22',mec='k',label="Tag2(True2)")
        # plt.plot(Tag3[0]/1000,Tag3[1]/1000,'co',markerfacecolor='#17becf',mec='k',label='Tag3(True3)')
        # # plt.plot(a[0, :], a[1, :], 'ro', label="Veh1")
        # # plt.plot(b[0, :], b[1, :], 'go', label="Veh2")
        # # plt.plot(c[0, :], c[1, :], 'bo', label='Veh3')
        # # plt.text(Tag1[0],Tag1[1],'True1')
        # # plt.text(Tag2[0],Tag2[1],'True2')
        # plt.plot(aa[0, :]/1000, aa[1, :]/1000, 'r-x', label="Tag1(Est1)")
        # plt.plot(bb[0, :]/1000, bb[1, :]/1000, 'g-x', label="Tag2(Est2)")
        # plt.plot(cc[0, :]/1000, cc[1, :]/1000, 'b-x', label="Tag3(Est3)")
        # # plt.plot(est1[0], est1[1], 'r-x', label="Est1")
        # # plt.plot(est2[0], est2[1], 'g-x', label="Est2")
        # # plt.plot(est3[0], est3[1], 'b-x', label="Est3")
        # # plt.plot(ori_est1[0],ori_est1[1],'rd',label="Init1")
        # # plt.plot(ori_est2[0],ori_est2[1],'bd',label="Init2")
        # # plt.plot(ori_est3[0],ori_est3[1],'gd',label="Init3")
        # plt.plot(ap1[0]/1000, ap1[1]/1000, 'kd')
        # plt.plot(ap2[0]/1000, ap2[1]/1000, 'kd')
        # plt.plot(ap3[0]/1000, ap3[1]/1000, 'kd')
        # plt.plot(ap4[0]/1000, ap4[1]/1000, 'kd')
        # plt.text(ap1[0]/1000,ap1[1]/1000+.1,'AP1',ha='center',va='bottom')
        # plt.text(ap2[0]/1000,ap2[1]/1000+.1,'AP2',ha='center',va='bottom')
        # plt.text(ap3[0]/1000,ap3[1]/1000+.1,'AP3',ha='center',va='bottom')
        # plt.text(ap4[0]/1000,ap4[1]/1000+.1,'AP4',ha='center',va='bottom')
        # plt.axis('equal')
        # plt.axis('square')
        # # plt.axis([0, 9000/1000, -1000/1000, 13000/1000])
        # # plt.axis('equal')
        # plt.box
        # # plt.plot([0, 8000], [0, 12000])
        # plt.legend(loc=4, ncol=2, fontsize=7)
        # # plt.legend(loc='upper center', ncol=2, fontsize=8,bbox_to_anchor=(0,1.02,1,0.2))
        # plt.xlabel('X-axis [m]')
        # plt.ylabel('Y-axis [m]')
        # plt.axis('equal')
        # # plt.xlim([700, 7500])
        # # plt.ylim([1380, 12000])
        # plt.grid(True)
        # plt.show()


