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

import matplotlib.pyplot as plt
import pandas as pd
import math
import pyqtgraph as pg
import time
from PyQt5 import QtWidgets, QtCore
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
import sys  # We need sys so that we can pass argv to QApplication
import os
import random

data_history=[]

# est1 = np.array([1900, 7300]).reshape(2,1)
# est2 = np.array([2000, 5300]).reshape(2,1)
# est3 = np.array([3600, 7500]).reshape(2,1)
est1 = np.array([100, 140]).reshape(2,1)
est2 = np.array([200, 320]).reshape(2,1)
est3 = np.array([250, 250]).reshape(2,1)
AP1 = np.array([0, 0]).reshape(2,1)
AP2 = np.array([1800, 0]).reshape(2,1)
AP3 = np.array([0, 1200]).reshape(2,1)
AP4 = np.array([1800, 1200]).reshape(2,1)

t1_comp1=np.array([0,0]).reshape(2,1)
t1_comp2=np.array([0,0]).reshape(2,1)
t1_compAP1=np.array([0,0]).reshape(2,1)
t1_compAP2=np.array([0,0]).reshape(2,1)
t1_compAP3=np.array([0,0]).reshape(2,1)
t1_compAP4=np.array([0,0]).reshape(2,1)
t1_comp_group=[t1_comp1, t1_comp2, t1_compAP1, t1_compAP2, t1_compAP3, t1_compAP4]

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


sigma_d = 0.1

J=6
T=10

a = np.zeros((2,T+1))
b = np.zeros((2,T+1))
c = np.zeros((2,T+1))

aa = np.zeros((2,T))
bb = np.zeros((2,T))
cc = np.zeros((2,T))
JJ = 1/(J+1)


class ADMM:
    def dis(x1, x2):
        d = x2-x1
        dx = d[0]
        dy = d[1]
        d = d[0]**2+d[1]**2
        d = np.sqrt(d)
        return d, dx, dy

    def diff(x1,x2):
        dd=x2-x1
        dx = dd[0]
        dy = dd[1]

        return dx, dy

    def func1(x, y, p, sigma_d,est,comp,z):
        r = np.sqrt(x**2+y**2)
        rx = x/r
        ry = y/r

        xx = sigma_d**2/((1/p)*rx**2+sigma_d**2)
        xxx = ((1/p)*rx)/((1/p)*rx**2+sigma_d**2)

        yy = sigma_d**2/((1/p)*ry**2+sigma_d**2)
        yyy = ((1/p)*ry)/((1/p)*ry**2+sigma_d**2)
    
        p0 = xx*(est[0]-comp[0])-xxx*(z-r-rx*est[0])
        p1 = yy*(est[1]-comp[1])-yyy*(z-r-ry*est[1])

        predic = np.array([p0, p1]).reshape(2,1)

        # print(p0,p1)
        comp0 = comp[0]+p0-est[0]
        comp1 = comp[1]+p1-est[1]

        comp = np.array([comp0, comp1]).reshape(2,1)
        # return predic, comp
        return predic, comp

    def cal(est1,est2,est3,ap1,ap2,ap3,ap4):
        t_a =ADMM.dis(est1,est2)
        t_b =ADMM.dis(est1,est3)
        t_ap1 =ADMM.dis(est1,ap1)
        t_ap2 =ADMM.dis(est1,ap2)
        t_ap3 =ADMM.dis(est1,ap3)
        t_ap4 =ADMM.dis(est1,ap4) 

        return t_a, t_b, t_ap1, t_ap2, t_ap3, t_ap4

    def cal1(est1,est2,est3,ap1,ap2,ap3,ap4):
        [t_x1, t_y1] = ADMM.diff(est1,est2)
        [t_x2, t_y2] = ADMM.diff(est1,est3)
        [t_ap1x, t_ap1y] = ADMM.diff(est1,ap1)
        [t_ap2x, t_ap2y] = ADMM.diff(est1,ap2)
        [t_ap3x, t_ap3y] = ADMM.diff(est1,ap3)
        [t_ap4x, t_ap4y] = ADMM.diff(est1,ap4)

        return t_x1, t_y1, t_x2, t_y2, t_ap1x, t_ap1y, t_ap2x, t_ap2y, t_ap3x, t_ap3y, t_ap4x, t_ap4y

    def admm1(est1,est2,est3,ap1,ap2,ap3,ap4,d1):
        global t1_comp1, t1_comp2, t1_compAP1, t1_compAP2, t1_compAP3, t1_compAP4

        [t1_x1, t1_y1, t1_x2, t1_y2, t1_ap1x, t1_ap1y, t1_ap2x, t1_ap2y, t1_ap3x, t1_ap3y, t1_ap4x, t1_ap4y] = ADMM.cal1(est1,est2,est3,ap1,ap2,ap3,ap4)
        
        t1_a_d=t1_a[0]+np.random.randn(1)*sigma_d
        t1_b_d=t1_b[0]+np.random.randn(1)*sigma_d
        
        [t1_predic1,t1_comp1] = ADMM.func1(t1_x1,t1_y1,p,sigma_d,est1,t1_comp1,d1[4])
        [t1_predic2,t1_comp2] = ADMM.func1(t1_x2,t1_y2,p,sigma_d,est1,t1_comp2,d1[5])
        [t1_predicAP1,t1_compAP1] = ADMM.func1(t1_ap1x,t1_ap1y,p,sigma_d,est1,t1_compAP1,d1[0])
        [t1_predicAP2,t1_compAP2] = ADMM.func1(t1_ap2x,t1_ap2y,p,sigma_d,est1,t1_compAP2,d1[1])
        [t1_predicAP3, t1_compAP3] = ADMM.func1(t1_ap3x, t1_ap3y, p, sigma_d, est1, t1_compAP3, d1[2])
        [t1_predicAP4, t1_compAP4] = ADMM.func1(t1_ap4x, t1_ap4y, p, sigma_d, est1, t1_compAP4, d1[3])

        t1_predic = t1_predic1+t1_predic2+t1_predicAP1+t1_predicAP2+t1_predicAP3+t1_predicAP4
        t1_comp = t1_comp1+t1_comp2+t1_compAP1+t1_compAP2+t1_compAP3+t1_compAP4

        return t1_predic, t1_comp
        
    def admm2(est2,est1,est3,ap1,ap2,ap3,ap4,d2):
        global t2_comp1, t2_comp2, t2_compAP1, t2_compAP2, t2_compAP3, t2_compAP4

        [t2_x1, t2_y1, t2_x2, t2_y2, t2_ap1x, t2_ap1y, t2_ap2x, t2_ap2y, t2_ap3x, t2_ap3y, t2_ap4x, t2_ap4y] = ADMM.cal1(est2,est1,est3,ap1,ap2,ap3,ap4)

        t2_a_d=t2_a[0]+np.random.randn(1)*sigma_d
        t2_b_d=t2_b[0]+np.random.randn(1)*sigma_d

        [t2_predic1,t2_comp1] = ADMM.func1(t2_x1,t2_y1,p,sigma_d,est2,t2_comp1,d2[4])
        [t2_predic2,t2_comp2] = ADMM.func1(t2_x2,t2_y2,p,sigma_d,est2,t2_comp2,d2[5])
        [t2_predicAP1,t2_compAP1] = ADMM.func1(t2_ap1x,t2_ap1y,p,sigma_d,est2,t2_compAP1,d2[0])
        [t2_predicAP2,t2_compAP2] = ADMM.func1(t2_ap2x,t2_ap2y,p,sigma_d,est2,t2_compAP2,d2[1])
        [t2_predicAP3,t2_compAP3] = ADMM.func1(t2_ap3x,t2_ap3y,p,sigma_d,est2,t2_compAP3,d2[2])
        [t2_predicAP4,t2_compAP4] = ADMM.func1(t2_ap4x,t2_ap4y,p,sigma_d,est2,t2_compAP4,d2[3])

        t2_predic = t2_predic1+t2_predic2+t2_predicAP1+t2_predicAP2+t2_predicAP3+t2_predicAP4
        t2_comp = t2_comp1+t2_comp2+t2_compAP1+t2_compAP2+t2_compAP3+t2_compAP4
        
        return t2_predic, t2_comp

    def admm3(est3,est1,est2,ap1,ap2,ap3,ap4,d3):
        global t3_comp1, t3_comp2, t3_compAP1, t3_compAP2, t3_compAP3, t3_compAP4

        [t3_x1, t3_y1, t3_x2, t3_y2, t3_ap1x, t3_ap1y, t3_ap2x, t3_ap2y, t3_ap3x, t3_ap3y, t3_ap4x, t3_ap4y] = ADMM.cal1(est3,est1,est2,ap1,ap2,ap3,ap4)

        t3_a_d=t3_a[0]+np.random.randn(1)*sigma_d
        t3_b_d=t3_b[0]+np.random.randn(1)*sigma_d

        [t3_predic1,t3_comp1] = ADMM.func1(t3_x1,t3_y1,p,sigma_d,est3,t3_comp1,d3[4])
        [t3_predic2,t3_comp2] = ADMM.func1(t3_x2,t3_y2,p,sigma_d,est3,t3_comp2,d3[5])
        [t3_predicAP1,t3_compAP1] = ADMM.func1(t3_ap1x,t3_ap1y,p,sigma_d,est3,t3_compAP1,d3[0])
        [t3_predicAP2,t3_compAP2] = ADMM.func1(t3_ap2x,t3_ap2y,p,sigma_d,est3,t3_compAP2,d3[1])
        [t3_predicAP3,t3_compAP3] = ADMM.func1(t3_ap3x,t3_ap3y,p,sigma_d,est3,t3_compAP1,d3[2])
        [t3_predicAP4,t3_compAP4] = ADMM.func1(t3_ap4x,t3_ap4y,p,sigma_d,est3,t3_compAP2,d3[3])

        t3_predic = t3_predic1+t3_predic2+t3_predicAP1+t3_predicAP2+t3_predicAP3+t3_predicAP4
        t3_comp = t3_comp1+t3_comp2+t3_compAP1+t3_compAP2+t3_compAP3+t3_compAP4

        return t3_predic, t3_comp

    def estimate(est1, est2, est3, ap1, ap2, ap3, ap4,d1,d2,d3):
        global JJ
        
        [t1_predic, t1_comp] = ADMM.admm1(est1,est2,est3,ap1,ap2,ap3,ap4,d1)
        [t2_predic, t2_comp] = ADMM.admm2(est2,est1,est3,ap1,ap2,ap3,ap4,d2)
        [t3_predic, t3_comp] = ADMM.admm3(est3,est1,est2,ap1,ap2,ap3,ap4,d3)

        est1 = JJ*(est1+t1_predic+t1_comp)
        est2 = JJ*(est2+t2_predic+t2_comp)
        est3 = JJ*(est3+t3_predic+t3_comp)

        return est1, est2, est3

    def save(Tag1, est1, Tag2, est2, Tag3, est3, t):

        global a, aa, b, bb, c ,cc

        a[0,t] = Tag1[0]
        a[1,t] = Tag1[1]

        aa[0,t] = est1[0]
        aa[1,t] = est1[1]

        b[0,t] = Tag2[0]
        b[1,t] = Tag2[1]
        bb[0,t] = est2[0]
        bb[1,t] = est2[1]

        c[0,t] = Tag3[0]
        c[1,t] = Tag3[1]
        cc[0,t] = est3[0]
        cc[1,t] = est3[1]

        return a, aa, b, bb, c, cc


def run(est1,est2,est3,AP1,AP2,AP3,AP4,d1,d2,d3):
    global t1_a, t1_b, t1_ap1, t1_ap2, t1_ap3, t1_ap4
    global t2_a, t2_b, t2_ap1, t2_ap2, t2_ap3, t2_ap4
    global t3_a, t3_b, t3_ap1, t3_ap2, t3_ap3, t3_ap4
    global p, i, a, aa, b, bb, c, cc


    
    t1_a, t1_b, t1_ap1, t1_ap2, t1_ap3, t1_ap4 = ADMM.cal(est1,est2,est3,AP1,AP2,AP3,AP4)
    t2_a, t2_b, t2_ap1, t2_ap2, t2_ap3, t2_ap4 = ADMM.cal(est2,est1,est3,AP1,AP2,AP3,AP4)
    t3_a, t3_b, t3_ap1, t3_ap2, t3_ap3, t3_ap4 = ADMM.cal(est3,est1,est2,AP1,AP2,AP3,AP4)

    p =90
    for i in range(0,10):
        est1, est2, est3 = ADMM.estimate(est1,est2,est3,AP1,AP2,AP3,AP4,d1,d2,d3)

    return est1, est2, est3

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, *args, **kwargs):
        global x, y, x1, y1, x2, y2
        super(MainWindow, self).__init__(*args, **kwargs)

        self.graphWidget = pg.PlotWidget()
        self.setCentralWidget(self.graphWidget)

        # self.graphWidget.setXRange(0,10)
        # self.graphWidget.setYRange(0,10)
        

        # plot data: x, y values
        # self.plot(x, y,"Sensor1",'r')
        # self.plot(x1,y1,"Sensor2",'b')

        x = [0]  # 100 time points
        y = [0]  # 100 data points
    
        x1 = [1]
        y1 = [2]

        x2 = [2]
        y2 = [3]

        self.graphWidget.setBackground('w')
        self.graphWidget.setXRange(-300/1000,4000/1000,padding=0)
        self.graphWidget.setYRange(-300/1000,4000/1000,padding=0)
        self.graphWidget.setLabel("left","Y-axis [m]")
        self.graphWidget.setLabel("bottom","X-axis [m]")
        self.graphWidget.addLegend()

        self.graphWidget.showGrid(x=True,y=True)
        # print(self.x, self.y)
        pen = pg.mkPen(color=(0,0,0))
        pen1=pg.mkPen(color=(0,0,0))
        pen2=pg.mkPen(color=(0,0,0))
        pen3=pg.mkPen(color=(255,255,255))
        self.data_line =  self.graphWidget.plot(x, y,name="Tag1",pen=pen,symbolBrush = (0,0,200), symbol='o', symbolSize=14)
        self.data_line1 = self.graphWidget.plot(x1, y1,name="Tag2",pen=pen1, symbolBrush =(0,200,0), symbol='o',symbolSize=14)
        self.data_line2 = self.graphWidget.plot(x2, y2,name="Tag3", pen=pen2, symbolBrush = (200,0,0), symbol='o',symbolSize=14)
        self.data_line3 = self.graphWidget.plot(AP1[0]/1000,AP1[1]/1000, name="AP1", pen=pen3, symbol='p')
        self.data_line4 = self.graphWidget.plot(AP2[0]/1000,AP2[1]/1000, name="AP2", pen=pen3, symbol='p')
        self.data_line5 = self.graphWidget.plot(AP3[0]/1000,AP3[1]/1000, name="AP3", pen=pen3, symbol='p')
        self.data_line6 = self.graphWidget.plot(AP4[0]/1000,AP4[1]/1000, name="AP4", pen=pen3, symbol='p')
 

        self.timer = QtCore.QTimer()
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()

    def plot(self, x, y, plotname, color):
        pen = pg.mkPen(color=color)
        self.graphWidget.plot(x, y, name=plotname, pen=pen, symbol='o', symbolSize=10, symbolBrush=(color))

    def update_plot_data(self):
            global est1, est2, est3
            global AP1,AP2,AP3,AP4
            global x, y, x1, y1, x2, y2
            
            d1= np.array([r.loop(), r1.loop(), r2.loop(), r3.loop(), r4.loop(), r5.loop()])
            d2= np.array([rr.loop(), rr1.loop(), rr2.loop(), rr3.loop(), rr4.loop(), rr5.loop()])
            d3= np.array([rrr.loop(), rrr1.loop(), rrr2.loop(), rrr3.loop(), rrr4.loop(), rrr5.loop()])

            print(d1)
            print(d2)
            print(d3)

            est1, est2, est3 = run(est1,est2,est3,AP1,AP2,AP3,AP4,d1,d2,d3)

            est1=est1/1000
            est2=est2/1000
            est3=est3/1000
            print("Tag1:",est1[0], est1[1], "Tag2:", est2[0], est2[1], "Tag3:", est3[0], est3[1])
            x = x[1:]  # Remove the first y element.
            
            x.extend(est1[0])
            # print(x/1000)  # Add a new value 1 higher than the last.
            y = y[1:]  # Remove the first
            y.extend(est1[1])  # Add a new random value.

            x1 = x1[1:]  # Remove the first y element.
            x1.extend(est2[0])  # Add a new value 1 higher than the last.
            y1 = y1[1:]  # Remove the first
            y1.extend(est2[1])  # Add a new random value.

            # print(Tag3)
            x2 = x2[1:]  # Remove the first y element.
            x2.extend(est3[0])  # Add a new value 1 higher than the last.
            y2 = y2[1:]  # Remove the first
            y2.extend(est3[1])  # Add a new random value.
            # print(x1, y1)

            # print(self.x, self.y)
            self.data_line.setData(x, y)  # Update the data.
            self.data_line1.setData(x1, y1)  # Update the data.
            self.data_line2.setData(x2, y2)  # Update the data.


def main():
    app = QtWidgets.QApplication(sys.argv)
    main = MainWindow()
    main.show()
    sys.exit(app.exec_())


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

if __name__ == "__main__":
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

    remote_id = 0x6846           # the network ID of the remote device
    # remote_id = 0x6824   
    remote = True               # whether to use the given remote device for ranging
    if not remote:
        remote_id = None

    # remote_id1 = 0x6838
    remote_id1 = 0x6824
    remote = True
    if not remote:
        remote_id = None

    # remote_id2 = 0x6828
    remote_id2 = 0x6862
    remote = True
    if not remote:
        remote_id = None

    destination_id1 = 0x1162
    destination_id2 = 0x114A
    destination_id3 = 0x1126
    destination_id4 = 0x1136

    # distance that separates the amount of LEDs lighting up.
    range_step_mm = 100

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
        main()
        # print("r1:", np.array([r.loop(), r1.loop(), r2.loop(), r3.loop(), r4.loop(), r5.loop()]))
        # print("r2:", np.array([rr.loop(), rr1.loop(), rr2.loop(), rr3.loop(), rr4.loop(), rr5.loop()]))
        # print("r3:", np.array([rrr.loop(), rrr1.loop(), rrr2.loop(), rrr3.loop(), rrr4.loop(), rrr5.loop()]))


