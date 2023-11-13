#!/usr/bin/env_python

import matplotlib.pyplot as plt
import matplotlib.pyplot as plt
import numpy as np
import pyqtgraph as pg
from pyqtgraph import PlotCurveItem
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QGraphicsPixmapItem, QGraphicsLineItem
from PyQt5.QtCore import Qt, QPointF, QLineF
from pyqtgraph import PlotWidget, plot
from PyQt5.QtGui import *
from PyQt5.QtGui import QPixmap
#from run import RUN_TAG

from set_tag import SET_TAGS

class GUI(QtWidgets.QMainWindow):
    def __init__(self, *args, **kwargs):
        super(GUI, self).__init__(*args, **kwargs)

        self.btn_pushed = [False, False, False]

        self.AP_0 = np.array([0, 0]).reshape(2,1)
        self.AP_1 = np.array([1800, 0]).reshape(2,1)
        self.AP_2 = np.array([0, 1200]).reshape(2,1)
        self.AP_3 = np.array([1800, 1200]).reshape(2,1)

        self.initialize_global_variables()
        self.setup_ui()
        self.setup_plot()

        #RUN_TAG.timer()
        #self.timer = QtCore.QTimer()
        #self.timer.setInterval(100)
        #self.timer.timeout.connect(Run.update_plot_data)
        #self.timer.start()

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
        self.pixmap = QPixmap('track.png')
        self.pixmap_item = QGraphicsPixmapItem(self.pixmap)
        
        # Add the Pixmap item to the PlotWidget's scene
        self.graphWidget.addItem(self.pixmap_item)
        
        # Make sure the image is behind all the other plot elements
        self.pixmap_item.setZValue(-100)
        
        # Resize the Pixmap item to fit the entire PlotWidget
        self.pixmap_item.setScale(self.graphWidget.plotItem.vb.viewRange()[0][1] / self.pixmap.width())
        # Plot the data with the specified pen, symbols, and symbol sizes

    def setup_plot(self):
        black_pen = pg.mkPen(color=(0, 0, 0))
        white_pen = pg.mkPen(color=(255, 255, 255))
        self.data_line = self.graphWidget.plot(self.x, self.y, name="Tag1", pen=black_pen, symbolBrush=(0, 0, 200), symbol='o', symbolSize=14)
        self.data_line1 = self.graphWidget.plot(self.x1, self.y1, name="Tag2", pen=black_pen, symbolBrush=(0, 200, 0), symbol='o', symbolSize=14)
        self.data_line2 = self.graphWidget.plot(self.x2, self.y2, name="Tag3", pen=black_pen, symbolBrush=(200, 0, 0), symbol='o', symbolSize=14)
        self.data_line3 = self.graphWidget.plot(self.AP_1[0] / 1000, self.AP_1[1] / 1000, name="AP1", pen=white_pen, symbol='p')
        self.data_line4 = self.graphWidget.plot(self.AP_2[0] / 1000, self.AP_2[1] / 1000, name="AP2", pen=white_pen, symbol='p')
        #self.data_line5 = self.graphWidget.plot(self.AP_2[0]/1000, self.AP_2[1]/1000, name="AP3", pen=pg.mkPen(color=(255,255,255)), symbol='p')
        #self.data_line6 = self.graphWidget.plot(self.AP_3[0]/1000, self.AP_3[1]/1000, name="AP4", pen=pg.mkPen(color=(255,255,255)), symbol='p')

    def plot(self, x, y, plotname, color):
        pen = pg.mkPen(color=color)
        self.graphWidget.plot(x, y, name=plotname, pen=pen, symbol='o', symbolSize=10, symbolBrush=(color))

    def btn_func_all(self):
        self.btn_pushed = [True, True, True]
    def btn_func_1(self):
        self.btn_pushed = [True, False, False]
    def btn_func_2(self):
        self.btn_pushed = [False, True, False]
    def btn_func_3(self):
        self.btn_pushed = [False, False, True]

    def plot_btn(self):
        if False not in self.btn_pushed:
            self.data_line.show()
            self.data_line1.show()
            self.data_line2.show()
        elif self.btn_pushed[0] is True:
            self.data_line.show()
            self.data_line1.hide()
            self.data_line2.hide()
        elif self.btn_pushed[1] is True:
            self.data_line.hide()
            self.data_line1.show()
            self.data_line2.hide()
        elif self.btn_pushed[2] is True:
            self.data_line.hide()
            self.data_line1.hide()
            self.data_line2.show()
    
    def create_layout(self):
        
        hbox = QtWidgets.QHBoxLayout() #Main layout
        vbox1 = QtWidgets.QVBoxLayout() #Graph layout
        vbox2 = QtWidgets.QVBoxLayout() #Visual layout
        
        #Button
        self.btns = QtWidgets.QHBoxLayout() #Button layout

        self.btn_all = QtWidgets.QPushButton("Show All")
        self.btn1    = QtWidgets.QPushButton("Show Tag 1")
        self.btn2    = QtWidgets.QPushButton("Show Tag 2")
        self.btn3    = QtWidgets.QPushButton("Show Tag 3")
        self.btns.addWidget(self.btn_all)
        self.btns.addWidget(self.btn1)
        self.btns.addWidget(self.btn2)
        self.btns.addWidget(self.btn3)
        
        #connect functions with buttons
        self.btn_all.clicked.connect(self.btn_func_all)
        self.btn1.clicked.connect(self.btn_func_1)
        self.btn2.clicked.connect(self.btn_func_2)
        self.btn3.clicked.connect(self.btn_func_3)

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
