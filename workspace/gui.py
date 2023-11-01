#!/usr/bin/env_python

import matplotlib.pyplot as plt
import matplotlib.pyplot as plt
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore
from pyqtgraph import PlotWidget, plot
import numpy as np

from set_tag import SET_TAGS

class GUI(QtWidgets.QMainWindow):
    def __init__(self, *args, **kwargs):
        super(GUI, self).__init__(*args, **kwargs)

        self.graphWidget = pg.PlotWidget()
        self.setCentralWidget(self.graphWidget)

        self.x_0 = [0]
        self.y_0 = [0]

        self.x_1 = [1]
        self.y_1 = [2]

        self.x_2 = [2]
        self.y_2 = [3]

        self.AP_0 = np.array([0, 0]).reshape(2,1)
        self.AP_1 = np.array([1800, 0]).reshape(2,1)
        self.AP_2 = np.array([0, 1200]).reshape(2,1)
        self.AP_3 = np.array([1800, 1200]).reshape(2,1)

        self.graphWidget.setBackground('w')
        self.graphWidget.setXRange(-300/1000,4000/1000,padding=0)
        self.graphWidget.setYRange(-300/1000,4000/1000,padding=0)
        self.graphWidget.setLabel("left","Y-axis [m]")
        self.graphWidget.setLabel("bottom","X-axis [m]")
        self.graphWidget.addLegend()

        self.graphWidget.showGrid(x=True,y=True)
        # print(self.x, self.y)
        self.data_line =  self.graphWidget.plot(self.x_0, self.y_0, name="Tag1", pen=pg.mkPen(color=(0,0,0)), symbolBrush = (0,0,200), symbol='o', symbolSize=14)
        self.data_line1 = self.graphWidget.plot(self.x_1, self.y_1, name="Tag2", pen=pg.mkPen(color=(0,0,0)), symbolBrush = (0,200,0), symbol='o', symbolSize=14)
        self.data_line2 = self.graphWidget.plot(self.x_2, self.y_2, name="Tag3", pen=pg.mkPen(color=(0,0,0)), symbolBrush = (200,0,0), symbol='o', symbolSize=14)

        self.data_line3 = self.graphWidget.plot(self.AP_0[0]/1000, self.AP_0[1]/1000, name="AP1", pen=pg.mkPen(color=(255,255,255)), symbol='p')
        self.data_line4 = self.graphWidget.plot(self.AP_1[0]/1000, self.AP_1[1]/1000, name="AP2", pen=pg.mkPen(color=(255,255,255)), symbol='p')
        self.data_line5 = self.graphWidget.plot(self.AP_2[0]/1000, self.AP_2[1]/1000, name="AP3", pen=pg.mkPen(color=(255,255,255)), symbol='p')
        self.data_line6 = self.graphWidget.plot(self.AP_3[0]/1000, self.AP_3[1]/1000, name="AP4", pen=pg.mkPen(color=(255,255,255)), symbol='p')

        self.timer = QtCore.QTimer()
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()

    def plot(self, x, y, plotname, color):
        pen = pg.mkPen(color=color)
        self.graphWidget.plot(x, y, name=plotname, pen=pen, symbol='o', symbolSize=10, symbolBrush=(color))
    