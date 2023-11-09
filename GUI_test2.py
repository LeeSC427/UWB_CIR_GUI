from pypozyx import (PozyxSerial, PozyxConstants, version,
                     SingleRegister, DeviceRange, POZYX_SUCCESS, POZYX_FAILURE, get_first_pozyx_serial_port)

from pypozyx.tools.version_check import perform_latest_version_check
from multiprocessing import Process, Queue, Pool, managers
import time
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import math
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QGraphicsPixmapItem
from pyqtgraph import PlotWidget, plot
from PyQt5.QtGui import *
from PyQt5.QtGui import QPixmap
import sys  # We need sys so that we can pass argv to QApplication
import os
import random

data_history=[]

est1 = np.array([100, 140]).reshape(2,1)
est2 = np.array([200, 320]).reshape(2,1)
# est3 = np.array([250, 250]).reshape(2,1)
AP1 = np.array([0, 0]).reshape(2,1)
AP2 = np.array([1800, 0]).reshape(2,1)
# AP3 = np.array([0, 1200]).reshape(2,1)
# AP4 = np.array([1800, 1200]).reshape(2,1)

t1_comp1=np.array([0,0]).reshape(2,1)
t1_comp2=np.array([0,0]).reshape(2,1)
t1_compAP1=np.array([0,0]).reshape(2,1)
t1_compAP2=np.array([0,0]).reshape(2,1)
# t1_compAP3=np.array([0,0]).reshape(2,1)
# t1_compAP4=np.array([0,0]).reshape(2,1)
# t1_comp_group=[t1_comp1, t1_comp2, t1_compAP1, t1_compAP2, t1_compAP3, t1_compAP4]

t2_comp1=np.array([0,0]).reshape(2,1)
t2_comp2=np.array([0,0]).reshape(2,1)
t2_compAP1=np.array([0,0]).reshape(2,1)
t2_compAP2=np.array([0,0]).reshape(2,1)
# t2_compAP3=np.array([0,0]).reshape(2,1)
# t2_compAP4=np.array([0,0]).reshape(2,1)

t3_comp1=np.array([0,0]).reshape(2,1)
t3_comp2=np.array([0,0]).reshape(2,1)
t3_compAP1=np.array([0,0]).reshape(2,1)
t3_compAP2=np.array([0,0]).reshape(2,1)
# t3_compAP3=np.array([0,0]).reshape(2,1)
# t3_compAP4=np.array([0,0]).reshape(2,1)

# dst1 = "some initial value"

sigma_d = 0.1

J=3
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

    def cal(est1,est2,ap1,ap2):
        t_a =ADMM.dis(est1,est2)
        t_ap1 =ADMM.dis(est1,ap1)
        t_ap2 =ADMM.dis(est1,ap2)

        return t_a, t_ap1, t_ap2
    
    def cal1(est1,est2,ap1,ap2):
        [t_x1, t_y1] = ADMM.diff(est1,est2)
        [t_ap1x, t_ap1y] = ADMM.diff(est1,ap1)
        [t_ap2x, t_ap2y] = ADMM.diff(est1,ap2)

        return t_x1, t_y1, t_ap1x, t_ap1y, t_ap2x, t_ap2y

    def admm(est1,est2,ap1,ap2,d1):
        global t1_comp1, t1_compAP1, t1_compAP2

        [t1_x1, t1_y1, t1_ap1x, t1_ap1y, t1_ap2x, t1_ap2y] = ADMM.cal1(est1,est2,ap1,ap2)
        
        # t1_a_d=t1_a[0]+np.random.randn(1)*sigma_d
        # t1_b_d=t1_b[0]+np.random.randn(1)*sigma_d
        
        [t1_predic1,t1_comp1] = ADMM.func1(t1_x1,t1_y1,p,sigma_d,est1,t1_comp1,d1[0])
        [t1_predicAP1,t1_compAP1] = ADMM.func1(t1_ap1x,t1_ap1y,p,sigma_d,est1,t1_compAP1,d1[1])
        [t1_predicAP2,t1_compAP2] = ADMM.func1(t1_ap2x,t1_ap2y,p,sigma_d,est1,t1_compAP2,d1[2])

        t_predic = t1_predic1+t1_predicAP1+t1_predicAP2
        t_comp = t1_comp1+t1_compAP1+t1_compAP2

        return t_predic, t_comp

    def estimate(est1, est2, ap1, ap2, d1):
        global JJ
        
        [t_predic, t_comp] = ADMM.admm(est1,est2,ap1,ap2,d1)

        est1 = JJ*(est1+t_predic+t_comp)

        return est1
    
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

def run(est1,est2,AP1,AP2,d1):
    global t1_a, t1_b, t1_ap1, t1_ap2, t1_ap3, t1_ap4
    global p, i, a, aa, b, bb, c, cc
    
    t1_a, t1_ap1, t1_ap2= ADMM.cal(est1,est2,AP1,AP2)
    p =90
    for i in range(0,10):
        est1 = ADMM.estimate(est1,est2,AP1,AP2,d1)
        
    return est1
    # queue.put((est1))

def compute_est(dst1,dst2,dst3,est1, est2, AP1, AP2):
    d = np.array([dst1, dst2, dst3]) # Replace with the actual loops: [dst4.loop(), dst5.loop(), dst6.loop()]
    est1 = run(est1, est2, AP1, AP2, d)
    # queue.put(est2)
    return est1

class MainWindow(QtWidgets.QMainWindow):
    def plot(self, x, y, plotname, color):
        pen = pg.mkPen(color=color)
        self.graphWidget.plot(x, y, name=plotname, pen=pen, symbol='o', symbolSize=10, symbolBrush=(color))
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.initialize_global_variables()
        self.setup_ui()
        self.setup_timer()
        self.setup_plot()

    def initialize_global_variables(self):
        self.x = [0]
        self.y = [0]
        self.x1 = [1]
        self.y1 = [2]
        self.x2 = [2]
        self.y2 = [3]

    def setup_ui(self):
        self.setWindowTitle("Cooperative localization")
        self.setGeometry(1000, 300, 1150, 600)
        self.create_graph_widget()
        self.create_layout()

    def create_graph_widget(self):
        self.graphWidget = pg.PlotWidget(title="Cooperative localization") #그래프 출력
        self.setCentralWidget(self.graphWidget)
        self.graphWidget.setBackground('w')
        self.graphWidget.setXRange(0,3.5,padding=None)
        self.graphWidget.setYRange(0,3.0,padding=None)
        self.graphWidget.setLabel("left","Y-axis [m]")
        self.graphWidget.setLabel("bottom","X-axis [m]")
        self.graphWidget.addLegend()
        # self.graphWidget.showGrid(x=True,y=True)# Create a Pixmap item from your image
        pixmap = QPixmap('track.png')
        pixmap_item = QGraphicsPixmapItem(pixmap)
        
        # Add the Pixmap item to the PlotWidget's scene
        self.graphWidget.addItem(pixmap_item)
        
        # Make sure the image is behind all the other plot elements
        pixmap_item.setZValue(-100)
        
        # Resize the Pixmap item to fit the entire PlotWidget
        pixmap_item.setScale(self.graphWidget.plotItem.vb.viewRange()[0][1] / pixmap.width())
        # Plot the data with the specified pen, symbols, and symbol sizes
  
    def setup_plot(self):
        
        black_pen = pg.mkPen(color=(0, 0, 0))
        white_pen = pg.mkPen(color=(255, 255, 255))
        self.data_line = self.graphWidget.plot(self.x, self.y, name="Tag1", pen=black_pen, symbolBrush=(0, 0, 200), symbol='o', symbolSize=14)
        self.data_line1 = self.graphWidget.plot(self.x1, self.y1, name="Tag2", pen=black_pen, symbolBrush=(0, 200, 0), symbol='o', symbolSize=14)
        self.data_line2 = self.graphWidget.plot(self.x2, self.y2, name="Tag3", pen=black_pen, symbolBrush=(200, 0, 0), symbol='o', symbolSize=14)
        self.data_line3 = self.graphWidget.plot(AP1[0] / 1000, AP1[1] / 1000, name="AP1", pen=white_pen, symbol='p')
        self.data_line4 = self.graphWidget.plot(AP2[0] / 1000, AP2[1] / 1000, name="AP2", pen=white_pen, symbol='p')

    def create_layout(self):
        
        hbox = QtWidgets.QHBoxLayout() #Main layout
        vbox1 = QtWidgets.QVBoxLayout() #Graph layout
        vbox2 = QtWidgets.QVBoxLayout() #Visual layout
        
        #Button
        self.btns = QtWidgets.QHBoxLayout() #Button layout

        self.btn1 = QtWidgets.QPushButton("Function1")
        self.btn2 = QtWidgets.QPushButton("Function2")
        self.btn3 = QtWidgets.QPushButton("Function3")
        self.btns.addWidget(self.btn1)
        self.btns.addWidget(self.btn2)
        self.btns.addWidget(self.btn3)
        #Graph
        self.graph = QtWidgets.QVBoxLayout()
        self.graph.addWidget(self.graphWidget)
        self.graphWidget.setFixedSize(700, 600) # 너비 400, 높이 300으로 고정
        self.graphWidget.setAspectLocked(lock=True, ratio=1)

        # self.graphWidget.resize(400, 300) # 너비 400, 높이 300으로 변경

        # self.graphWidget.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        #Visual
        self.label = QtWidgets.QVBoxLayout()
        
        # new_size = QtCore.QSize(300, 100)

        self.label1 = QtWidgets.QLabel()
        self.label1.setPixmap(QPixmap("logo.png").scaledToWidth(240))

        self.label2 = QtWidgets.QLabel()
        self.label2.setPixmap(QPixmap("logo.png").scaledToWidth(240))

        self.label3 = QtWidgets.QLabel()
        self.label3.setPixmap(QPixmap("logo.png").scaledToWidth(240))

        self.label4 = QtWidgets.QLabel()
        self.label4.setPixmap(QPixmap("logo.png").scaledToWidth(240))
        
        self.label.addWidget(self.label1)
        self.label.addWidget(self.label2)
        self.label.addWidget(self.label3)
        self.label.addWidget(self.label4)

        #Logo
        self.logo = QtWidgets.QHBoxLayout()
        
        size1 = QtCore.QSize(120, 40)
        size2 = QtCore.QSize(120, 60)
        size3 = QtCore.QSize(60, 60)

        self.logo1 = QtWidgets.QLabel()
        # self.logo1.setPixmap(QPixmap("hyundai.png").scaled(size1, QtCore.Qt.KeepAspectRatio))
        self.logo1.setPixmap(QPixmap("hyundai.png").scaledToWidth(140))

        self.logo2 = QtWidgets.QLabel()
        # self.logo2.setPixmap(QPixmap("wsl.png").scaled(size2, QtCore.Qt.KeepAspectRatio))
        self.logo2.setPixmap(QPixmap("wsl.png").scaledToWidth(140))
        
        self.logo3 = QtWidgets.QLabel()
        # self.logo3.setPixmap(QPixmap("hanyang.png").scaled(size3, QtCore.Qt.KeepAspectRatio))
        self.logo3.setPixmap(QPixmap("hanyang.png").scaledToWidth(80))

        self.logo.addWidget(self.logo1)
        self.logo.addWidget(self.logo2)
        self.logo.addWidget(self.logo3)


        #vbox1 Layout
        vbox1.addLayout(self.btns, 1)
        vbox1.addLayout(self.graph, 3)

        #vbox2 Layout
        vbox2.addLayout(self.logo)
        vbox2.addLayout(self.label)
 
        #hbox Layout
        hbox.addLayout(vbox1)
        hbox.addLayout(vbox2)
        # hbox.setContentsMargins(10,100,100,12)
        hbox.setSpacing(10)

        widget = QtWidgets.QWidget()
        widget.setLayout(hbox)
        self.setCentralWidget(widget)
        # ... 레이아웃 설정 코드 ...

    def setup_timer(self):
        self.timer = QtCore.QTimer()
        self.timer.setInterval(10)
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()

    def update_plot_data(self):
            global est1, est2, est3
            global dst1,dst2,dst3,dst4,dst5,dst6
            global AP1,AP2,AP3,AP4
            # global x, y, x1, y1, x2, y2

            # est1, est2 = run_parallel(est1, est2, AP1, AP2)
            with Pool(processes=1) as pool:
                start=time.time()
                # Create argument lists for both functions
                args1 = (dst1.loop(), dst2.loop(), dst3.loop(), est1, est2, AP1, AP2)
                args2 = (dst4.loop(), dst5.loop(), dst6.loop(), est2, est1, AP1, AP2)
                # Use starmap to run functions with the arguments in parallel

                results = pool.starmap(compute_est, [args1, args2])
            (est1), (est2) = results
            end=time.time()
            print("last Time",(end-start))
            # print("results:",est1,est2)
            est1=est1/1000
            est2=est2/1000
            print("Tag1:",est1[0], est1[1], "Tag2:", est2[0], est2[1])
            # print("Tag1:",est1[0], est1[1], "Tag2:", est2[0], est2[1], "Tag3:", est3[0], est3[1])
            x = self.x[1:]  # Remove the first y element.

            x.extend(est1[0])
            # print(x/1000)  # Add a new value 1 higher than the last.
            y = self.y[1:]  # Remove the first
            y.extend(est1[1])  # Add a new random value.

            x1 = self.x1[1:]  # Remove the first y element.
            x1.extend(est2[0])  # Add a new value 1 higher than the last.
            y1 = self.y1[1:]  # Remove the first
            y1.extend(est2[1])  # Add a new random value.

            # print(Tag3)
            # x2 = x2[1:]  # Remove the first y element.
            # x2.extend(est3[0])  # Add a new value 1 higher than the last.
            # y2 = y2[1:]  # Remove the first
            # y2.extend(est3[1])  # Add a new random value.
            # # print(x1, y1)

            # print(self.x, self.y)
            self.data_line.setData(x, y)  # Update the data.
            self.data_line1.setData(x1, y1)  # Update the data.
            # self.data_line2.setData(x2, y2)  # Update the data.

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
                # print("ERROR Ranging, local %s" %
                print("ERROR Ranging, local %s" %
                      self.pozyx.getErrorMessage(error_code))
            else:
                print("ERROR Ranging, couldn't retrieve local error")
        # print(dis)
        return dis
    def ledControl(self, distance):
        range_step_mm = 100
        """Sets LEDs according to the distance between two devices"""
        status = POZYX_SUCCESS
        ids = [self.remote_id, self.destination_id]
        # set the leds of both local/remote and destination pozyx device
        for id in ids:
            # start=time.time
            status &= self.pozyx.setLed(4, (distance < range_step_mm), id)
            status &= self.pozyx.setLed(3, (distance < 2 * range_step_mm), id)
            status &= self.pozyx.setLed(2, (distance < 3 * range_step_mm), id)
            status &= self.pozyx.setLed(1, (distance < 4 * range_step_mm), id)
            # end=time.time()
            # print("Time",(end-start))
        return status

if __name__ == "__main__":
    global dst1,dst2,dst3,dst4,dst5,dst6
    queue1=Queue()
    queue2=Queue()
    check_pypozyx_version = True
    if check_pypozyx_version:
        perform_latest_version_check()

    # hardcoded way to assign a serial port of the Pozyx
    # serial_port = 'COM12'

    S1= get_first_pozyx_serial_port()
    S2='COM4'
    # the easier way
    serial_port = get_first_pozyx_serial_port()
    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()

    remote_id = 0x6846
    remote_id1 = 0x6862
    
    dest1 = 0x684E
    dest2 = 0x0D5D


    pozyx = PozyxSerial(serial_port)
    ranging_protocol = PozyxConstants.RANGE_PROTOCOL_PRECISION
    range_step_mm = 100

    dst1 = ReadyToRange(pozyx, dest1, range_step_mm, ranging_protocol, remote_id)
    dst2 = ReadyToRange(pozyx, dest2, range_step_mm, ranging_protocol, remote_id)
    dst3 = ReadyToRange(pozyx, remote_id1, range_step_mm, ranging_protocol, remote_id)

    
    dst4 = ReadyToRange(pozyx, dest1, range_step_mm, ranging_protocol, remote_id1)
    dst5 = ReadyToRange(pozyx, dest2, range_step_mm, ranging_protocol, remote_id1)
    dst6 = ReadyToRange(pozyx, remote_id, range_step_mm, ranging_protocol, remote_id1)

    main()




# hbox1 = QtWidgets.QHBoxLayout()
# hbox2 = QtWidgets.QHBoxLayout()
# vbox1 = QtWidgets.QVBoxLayout()
# Gbox0 = QtWidgets.QVBoxLayout()
# Gbox1 = QtWidgets.QVBoxLayout()
# Gbox2 = QtWidgets.QVBoxLayout()
# Gbox3 = QtWidgets.QVBoxLayout()

# self.groupbox_Cam = QtWidgets.QGroupBox('Camera')
# self.groupbox_dis = QtWidgets.QGroupBox('distance')
# self.groupbox_est = QtWidgets.QGroupBox('estimate')
# self.groupbox_graph = QtWidgets.QGroupBox('graph')

# self.label_Cam = QtWidgets.QLabel('0',self)
# self.label_dis = QtWidgets.QLabel('0',self)
# self.label_est = QtWidgets.QLabel('0',self)
# self.label_graph = QtWidgets.QLabel('0',self)

# Gbox0.addWidget(self.label_Cam)
# Gbox1.addWidget(self.label_dis)
# Gbox2.addWidget(self.label_est)
# Gbox3.addWidget(self.label_graph)

# hbox1.addWidget(self.data_line)
# hbox1.addWidget(self.data_line1)
# hbox1.addWidget(self.data_line2)
# hbox1.addWidget(self.data_line3)
# hbox1.addWidget(self.data_line4)

# hbox2.addWidget(self.groupbox_Cam)
# hbox2.addWidget(self.groupbox_dis)
# hbox2.addWidget(self.groupbox_est)
# hbox2.addWidget(self.groupbox_graph)

# vbox1.addLayout(hbox1)
# vbox1.addLayout(hbox2)

# self.setLayout(vbox1)