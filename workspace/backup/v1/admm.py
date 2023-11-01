#!/usr/bin/env python

import numpy as np
from calculation import Calculation

class TAG:
    def __init__(self, Calculation):
        self.tag_comp1   = np.array([0,0]).reshape(2,1)
        self.tag_comp2   = np.array([0,0]).reshape(2,1)
        self.tag_compAP1 = np.array([0,0]).reshape(2,1)
        self.tag_compAP2 = np.array([0,0]).reshape(2,1)
        self.tag_compAP3 = np.array([0,0]).reshape(2,1)
        self.tag_compAP4 = np.array([0,0]).reshape(2,1)

        self.p = 90

        self.tag_a
        self.tag_b
        self.tag_ap1
        self.tag_ap2
        self.tag_ap3
        self.tag_ap4

    def admm(self, est1, est2, est3, ap1, ap2, ap3, ap4, d):
        [tag_x1, tag_y1, tag_x2, tag_y2, tag_ap1x, tag_ap1y, tag_ap2x, tag_ap2y, tag_ap3x, tag_ap3y, tag_ap4x, tag_ap4y] = Calculation.cal_diff(est1,est2,est3,ap1,ap2,ap3,ap4)
        
        tag_a_d = self.tag_a[0] + np.random.randn(1) * Calculation.sigma_d
        tag_b_d = self.tag_b[0] + np.random.randn(1) * Calculation.sigma_d

        [tag_predic1  ,tag_comp1  ] = Calculation.predic_comp(tag_x1,   tag_y1,   self.p, Calculation.sigma_d, est1, tag_comp1,   d[4])
        [tag_predic2  ,tag_comp2  ] = Calculation.predic_comp(tag_x2,   tag_y2,   self.p, Calculation.sigma_d, est1, tag_comp2,   d[5])
        [tag_predicAP1,tag_compAP1] = Calculation.predic_comp(tag_ap1x, tag_ap1y, self.p, Calculation.sigma_d, est1, tag_compAP1, d[0])
        [tag_predicAP2,tag_compAP2] = Calculation.predic_comp(tag_ap2x, tag_ap2y, self.p, Calculation.sigma_d, est1, tag_compAP2, d[1])
        [tag_predicAP3,tag_compAP3] = Calculation.predic_comp(tag_ap3x, tag_ap3y, self.p, Calculation.sigma_d, est1, tag_compAP3, d[2])
        [tag_predicAP4,tag_compAP4] = Calculation.predic_comp(tag_ap4x, tag_ap4y, self.p, Calculation.sigma_d, est1, tag_compAP4, d[3])

        tag_predic = tag_predic1 + tag_predic2 + tag_predicAP1 + tag_predicAP2 + tag_predicAP3 + tag_predicAP4
        tag_comp = tag_comp1+tag_comp2+tag_compAP1+tag_compAP2+tag_compAP3+tag_compAP4

        return [tag_predic, tag_comp]
