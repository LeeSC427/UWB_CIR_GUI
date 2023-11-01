#!/usr/bin/env_python

import pandas as pd
import numpy as np

class CAL:
    def __init__(self):
        self.sigma_d = 0.1
        self.J = 6
        self.JJ = 1 / (self.J + 1)

        #self.tag = ADMM()

        self.tag_comp_1   = np.array([0,0]).reshape(2,1)
        self.tag_comp_2   = np.array([0,0]).reshape(2,1)
        self.tag_comp_AP1 = np.array([0,0]).reshape(2,1)
        self.tag_comp_AP2 = np.array([0,0]).reshape(2,1)
        self.tag_comp_AP3 = np.array([0,0]).reshape(2,1)
        self.tag_comp_AP4 = np.array([0,0]).reshape(2,1)

        self.p = 90

        self.tag_predict = None
        self.tag_comp = None

    def dis(self, x_1, x_2):
        d = x_2 - x_1
        dx = d[0]
        dy = d[1]
        d = dx**2 + dy**2
        d = np.sqrt(d)

        return [d, dx, dy]
    
    def diff(self, x_1, x_2):
        d = x_2 - x_1
        dx = d[0]
        dy = d[1]

        return [dx, dy]
    
    def predic_n_comp(self, x, y, p, est, comp, z):
        dist_pow2 = x**2 + y**2
        dist = np.sqrt(dist_pow2)
        dist_x = x / dist
        dist_y = y / dist

        xx = self.sigma_d**2 / ((1 / p) * dist_x**2 + self.sigma_d**2)
        yy = self.sigma_d**2 / ((1 / p) * dist_y**2 + self.sigma_d**2)

        xxx = ((1 / p) * dist_x) / ((1 / p) * dist_x**2 + self.sigma_d**2)
        yyy = ((1 / p) * dist_y) / ((1 / p) * dist_y**2 + self.sigma_d**2)

        predic_0 = xx * (est[0] - comp[0]) - xxx * (z - dist - dist_x * est[0])
        predic_1 = yy * (est[1] - comp[1]) - yyy * (z - dist - dist_x * est[1])

        predic = np.array([predic_0, predic_1]).reshape(2,1)

        comp_0 = comp[0] + predic_0 - est[0]
        comp_1 = comp[1] + predic_1 - est[1]

        comp = np.array([comp_0, comp_1]).reshape(2,1)

        return [predic, comp]
    
    def estimation(tag_predict_1, tag_predict_2, tag_predict_3, tag_comp_1, tag_comp_2, tag_comp_3, self, est_1, est_2, est_3, ap_1, ap_2, ap_3, ap_4, d_1, d_2, d_3):
        [tag_predict_1, tag_comp_1] = self.admm(est_1, est_2, est_3, ap_1, ap_2, ap_3, ap_4, d_1)
        [tag_predict_2, tag_comp_2] = self.admm(est_2, est_1, est_3, ap_1, ap_2, ap_3, ap_4, d_2)
        [tag_predict_3, tag_comp_3] = self.admm(est_3, est_1, est_2, ap_1, ap_2, ap_3, ap_4, d_3)

        est_1 = self.JJ * (est_1 + tag_predict_1 + tag_comp_1)
        est_2 = self.JJ * (est_2 + tag_predict_2 + tag_comp_2)
        est_3 = self.JJ * (est_3 + tag_predict_3 + tag_comp_3)

        return [est_1, est_2, est_3]
    
    def admm(self, tag_a, tag_b, est_1, est_2, est_3, ap_1, ap_2, ap_3, ap_4, d):
        [tag_x1    , tag_y1    ] = self.diff(est_1, est_2)
        [tag_x2    , tag_y2    ] = self.diff(est_1, est_3)
        [tag_ap_x_1, tag_ap_y_1] = self.diff(est_1, ap_1 )
        [tag_ap_x_2, tag_ap_y_2] = self.diff(est_1, ap_2 )
        [tag_ap_x_3, tag_ap_y_3] = self.diff(est_1, ap_3 )
        [tag_ap_x_4, tag_ap_y_4] = self.diff(est_1, ap_4 )

        #tag_a_d = self.tag_a[0] + np.random.randn(1) * self.calculator.sigma_d
        #tag_b_d = self.tag_b[0] + np.random.randn(1) * self.calculator.sigma_d

        [tag_predic1  ,tag_comp1  ] = self.predic_n_comp(tag_x1    , tag_y1    , self.p, self.sigma_d, est_1, tag_comp1  , d[4])
        [tag_predic2  ,tag_comp2  ] = self.predic_n_comp(tag_x2    , tag_y2    , self.p, self.sigma_d, est_1, tag_comp2  , d[5])
        [tag_predicAP1,tag_compAP1] = self.predic_n_comp(tag_ap_x_1, tag_ap_y_1, self.p, self.sigma_d, est_1, tag_compAP1, d[0])
        [tag_predicAP2,tag_compAP2] = self.predic_n_comp(tag_ap_x_2, tag_ap_y_2, self.p, self.sigma_d, est_1, tag_compAP2, d[1])
        [tag_predicAP3,tag_compAP3] = self.predic_n_comp(tag_ap_x_3, tag_ap_y_3, self.p, self.sigma_d, est_1, tag_compAP3, d[2])
        [tag_predicAP4,tag_compAP4] = self.predic_n_comp(tag_ap_x_4, tag_ap_y_4, self.p, self.sigma_d, est_1, tag_compAP4, d[3])

        self.tag_predict = tag_predic1 + tag_predic2 + tag_predicAP1 + tag_predicAP2 + tag_predicAP3 + tag_predicAP4
        self.tag_comp    = tag_comp1 + tag_comp2 + tag_compAP1 + tag_compAP2 + tag_compAP3 + tag_compAP4