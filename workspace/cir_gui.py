import matplotlib.pyplot as plt
import matplotlib.pyplot as plt
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore
from pyqtgraph import PlotWidget, plot
import numpy as np

class GUI(QtWidgets.QMainWindow):
    def __init__(self, *args, **kwargs):
        super(GUI, self).__init__(*args, **kwargs)

        self.est1 = np.array([100, 140]).reshape(2,1)
        self.est2 = np.array([200, 320]).reshape(2,1)
        self.est3 = np.array([250, 250]).reshape(2,1)

        self.AP1 = np.array([0   ,    0]).reshape(2,1)
        self.AP2 = np.array([1800,    0]).reshape(2,1)
        self.AP3 = np.array([0   , 1200]).reshape(2,1)
        self.AP4 = np.array([1800, 1200]).reshape(2,1)

        self.graphWidget = pg.PlotWidget()
        self.setCentralWidget(self.graphWidget)

        x = [0]
        y = [0]

        x_1 = [1]
        y_1 = [2]

        x_2 = [2]
        y_2 = [3]

        self.graphWidget.setBackground('w')
        self.graphWidget.setXRange(-300/1000, 4000/1000, padding=0)
        self.graphWidget.setYRange(-300/1000, 4000/1000, padding=0)
        self.graphWidget.setLabel("left", "Y-axis [m]")
        self.graphWidget.setLabel("bottom", "X-axis [m]")
        self.graphWidget.addLegend()

        self.graphWidget.showGrid(x=True, y=True)

        pen  = pg.mkPen(color=(0,0,0))
        pen1 = pg.mkPen(color=(0,0,0))
        pen2 = pg.mkPen(color=(0,0,0))
        pen3 = pg.mkPen(color=(0,0,0))
        
        self.data_line =  self.graphWidget.plot(x, y,name="Tag1",pen=pen,symbolBrush = (0,0,200), symbol='o', symbolSize=14)
        self.data_line1 = self.graphWidget.plot(x_1, y_1,name="Tag2",pen=pen1, symbolBrush =(0,200,0), symbol='o',symbolSize=14)
        self.data_line2 = self.graphWidget.plot(x_2, y_2,name="Tag3", pen=pen2, symbolBrush = (200,0,0), symbol='o',symbolSize=14)
        self.data_line3 = self.graphWidget.plot(self.AP1[0]/1000, self.AP1[1]/1000, name="AP1", pen=pen3, symbol='p')
        self.data_line4 = self.graphWidget.plot(self.AP2[0]/1000, self.AP2[1]/1000, name="AP2", pen=pen3, symbol='p')
        self.data_line5 = self.graphWidget.plot(self.AP3[0]/1000, self.AP3[1]/1000, name="AP3", pen=pen3, symbol='p')
        self.data_line6 = self.graphWidget.plot(self.AP4[0]/1000, self.AP4[1]/1000, name="AP4", pen=pen3, symbol='p')

        self.timer = QtCore.QTimer()
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()

    def plot(self, x, y, plotname, color):
        pen = pg.mkPen(color=color)
        self.graphWidget.plot(x, y, name=plotname, pen=pen, symbol='o', symbolSize=10, symbolBrush=(color))
 
    def update_plot_data(self):