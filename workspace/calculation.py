#!/usr/bin/env python
from admm import TAG

import pandas as pd
import numpy as np
import math
import time
import sys  # We need sys so that we can pass argv to QApplication
import os
import random



class Calculation:
    def __init__(self):
        self.sigma_d = 0.1
        self.J = 6
        self.JJ = 1/(self.J + 1)

    def dis(self, x_1, x_2):
        d = x_2 - x_1
        dx = d[0]
        dy = d[1]
        d = dx**2+dy**2
        d = np.sqrt(d)
        
        return [d, dx, dy]

    def diff(self, x_1, x_2):
        d = x_2 -x_1
        dx = d[0]
        dy = d[1]

        return [dx, dy]

    def cal_dis(self, est1, est2, est3, ap1, ap2, ap3, ap4):
        t_a   = self.dis(est1, est2)
        t_b   = self.dis(est1, est3)
        t_ap1 = self.dis(est1, ap1 )
        t_ap2 = self.dis(est1, ap2 )
        t_ap3 = self.dis(est1, ap3 )
        t_ap4 = self.dis(est1, ap4 )

        return [t_a, t_b, t_ap1, t_ap2, t_ap3, t_ap4]

    def cal_diff(self, est1, est2, est3, ap1, ap2, ap3, ap4):
        [t_x1, t_y1]     = self.diff(est1, est2)
        [t_x2, t_y2]     = self.diff(est1, est3)
        [t_ap1x, t_ap1y] = self.diff(est1, ap1 )
        [t_ap2x, t_ap2y] = self.diff(est1, ap2 )
        [t_ap3x, t_ap3y] = self.diff(est1, ap3 )
        [t_ap4x, t_ap4y] = self.diff(est1, ap4 )

        return [t_x1, t_y1, t_x2, t_y2, t_ap1x, t_ap1y, t_ap2x, t_ap2y, t_ap3x, t_ap3y, t_ap4x, t_ap4y]

    def predic_comp(self, x, y, p, est, comp, z):
        r = np.sqrt(x**2 + y**2)
        r_x = x / r
        r_y = y / r

        xx = self.sigma_d**2 / ((1/p)*r_x**2 + self.sigma_d**2)
        yy = self.sigma_d**2 / ((1/p)*r_y**2 + self.sigma_d**2)

        xxx = ((1/p)*r_x) / ((1/p)*r_x**2 + self.sigma_d**2)
        yyy = ((1/p)*r_y) / ((1/p)*r_y**2 + self.sigma_d**2)

        p0 = xx * (est[0] - comp[0]) - xxx * (z - r - r_x * est[0])
        p1 = yy * (est[1] - comp[1]) - yyy * (z - r - r_y * est[1])

        predic = np.array([p0, p1]).reshape(2,1)

        comp0 = comp[0] + p0 - est[0]
        comp1 = comp[1] + p0 - est[1]

        comp = np.array([comp0, comp1]).reshape(2,1)

        return [predic, comp]

    def estimate(self, est1, est2, est3, ap1, ap2, ap3, ap4, d1, d2, d3):
        [t1_predic, t1_comp] = TAG.admm(est1, est2, est3, ap1, ap2, ap3, ap4, d1)

        est1 = self.JJ * (est1 + t1_predic + t1_comp)

        return est1
