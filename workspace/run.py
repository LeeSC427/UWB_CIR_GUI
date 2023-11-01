#!/usr/bin/env_python
from pypozyx.tools.version_check import perform_latest_version_check
from pypozyx import (PozyxSerial, PozyxConstants, version, SingleRegister, DeviceRange, POZYX_SUCCESS, POZYX_FAILURE, get_first_pozyx_serial_port)
from admm import ADMM
from cal import CAL
from gui import GUI
from set_tag import SET_TAGS

import numpy as np

class RUN_TAG:
    def __init__(self):
        self.dest_id_1   = 0x1162
        self.dest_id_2   = 0x114A
        self.dest_id_3   = 0x1126
        self.dest_id_4   = 0x1136

        self.remote_id_0 = None
        self.remote_id_1 = None
        self.remote_id_2 = None

        self.est_1 = np.array([100, 140]).reshape(2,1)
        self.est_2 = np.array([200, 320]).reshape(2,1)
        self.est_3 = np.array([250, 250]).reshape(2,1)

        #self.tag_admm = ADMM()
        self.tag_1 = SET_TAGS(self.dest_id_1, self.dest_id_2, self.dest_id_3, self.dest_id_4, self.remote_id_1, self.remote_id_2, self.remote_id_0)
        self.tag_2 = SET_TAGS(self.dest_id_1, self.dest_id_2, self.dest_id_3, self.dest_id_4, self.remote_id_0, self.remote_id_2, self.remote_id_1)
        self.tag_3 = SET_TAGS(self.dest_id_1, self.dest_id_2, self.dest_id_3, self.dest_id_4, self.remote_id_0, self.remote_id_1, self.remote_id_2)

        self.gui = GUI()

    def set_remote_id(self):
        self.remote_id_1 = 0x6846           # the network ID of the remote device
        # remote_id = 0x6824   
        remote = True               # whether to use the given remote device for ranging
        if not remote:
            remote_id = None

        # remote_id1 = 0x6838
        set.remote_id_2 = 0x6824
        remote = True
        if not remote:
            remote_id = None

        # remote_id2 = 0x6828
        set.remote_id_3 = 0x6862
        remote = True
        if not remote:
            remote_id = None

    def run(self, est_1, est_2, est_3, d_1, d_2, d_3):
        #[self.tag_admm.tag_a, self.tag_admm.tag_b, self.tag_admm.ap_1, self.tag_admm.ap_2, self.tag_admm.ap_3, self.tag_admm.ap_4] = ADMM.admm(est_1, est_2, est_3, GUI.AP_0, GUI.AP_1, GUI.AP_2, GUI.AP_3)

        for i in range(0,10):
            [est_1, est_2, est_3] = CAL.estimation(est_1, est_2, est_3, GUI.AP_0, GUI.AP_1, GUI.AP_2, GUI.AP_3, d_1, d_2, d_3)

        return [est_1, est_2, est_3]
    
    def update_plot_data(self, ObjSET_TAGS):
        return np.array([ObjSET_TAGS.r_0.pozyx_loop(), ObjSET_TAGS.r_1.pozyx_loop(), ObjSET_TAGS.r_2.pozyx_loop(), ObjSET_TAGS.r_3.pozyx_loop(), ObjSET_TAGS.r_4.pozyx_loop(), ObjSET_TAGS.r_5.pozyx_loop(), ])
    
    def update_plot(self):
        self.tag_1.data = self.update_plot_data(self.tag_1)
        self.tag_2.data = self.update_plot_data(self.tag_2)
        self.tag_3.data = self.update_plot_data(self.tag_3)

        [self.est_1, self.est_2, self.est_3] = self.run(self.est_1, self.est_2, self.est_3)
        
        self.est_1 = self.est_1 / 1000
        self.est_2 = self.est_2 / 1000
        self.est_3 = self.est_3 / 1000

        print("Tag1: ", self.est_1[0], self.est_1[0], "Tag2: ", self.est_2[0], self.est_2[0], "Tag3: ", self.est_3[0], self.est_3[0])

        self.gui.x_0 = self.gui.x_0[1:]
        self.gui.x_0.extend(self.est_1[0])

        self.gui.y_0 = self.gui.y_0[1:]
        self.gui.y_0.extend(self.est_1[1])

        self.gui.x_1 = self.gui.x_0[1:]
        self.gui.x_1.extend(self.est_2[0])

        self.gui.y_1 = self.gui.y_0[1:]
        self.gui.y_1.extend(self.est_2[1])

        self.gui.x_2 = self.gui.x_0[1:]
        self.gui.x_2.extend(self.est_3[0])

        self.gui.y_2 = self.gui.y_0[1:]
        self.gui.y_2.extend(self.est_3[1])

        self.gui.data_line.setData(self.gui.x_0, self.gui.y_0)
        self.gui.data_line1.setData(self.gui.x_1, self.gui.y_1)
        self.gui.data_line2.setData(self.gui.x_2, self.gui.y_2)