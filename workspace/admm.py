#!/usr/bin/env python

import numpy as np
from cal import CAL

class ADMM:
    def __init__(self):
        self.tag_comp_1   = np.array([0,0]).reshape(2,1)
        self.tag_comp_2   = np.array([0,0]).reshape(2,1)
        self.tag_comp_AP1 = np.array([0,0]).reshape(2,1)
        self.tag_comp_AP2 = np.array([0,0]).reshape(2,1)
        self.tag_comp_AP3 = np.array([0,0]).reshape(2,1)
        self.tag_comp_AP4 = np.array([0,0]).reshape(2,1)

        self.p = 90

        self.tag_predict
        self.tag_comp

        self.calculator = CAL()

    def admm(self, tag_a, tag_b, est_1, est_2, est_3, ap_1, ap_2, ap_3, ap_4, d):
        [tag_x1    , tag_y1    ] = self.calculator.diff(est_1, est_2)
        [tag_x2    , tag_y2    ] = self.calculator.diff(est_1, est_3)
        [tag_ap_x_1, tag_ap_y_1] = self.calculator.diff(est_1, ap_1 )
        [tag_ap_x_2, tag_ap_y_2] = self.calculator.diff(est_1, ap_2 )
        [tag_ap_x_3, tag_ap_y_3] = self.calculator.diff(est_1, ap_3 )
        [tag_ap_x_4, tag_ap_y_4] = self.calculator.diff(est_1, ap_4 )

        #tag_a_d = self.tag_a[0] + np.random.randn(1) * self.calculator.sigma_d
        #tag_b_d = self.tag_b[0] + np.random.randn(1) * self.calculator.sigma_d

        [tag_predic1  ,tag_comp1  ] = self.calculator.predic_n_comp(tag_x1    , tag_y1    , self.p, self.calculator.sigma_d, est_1, tag_comp1  , d[4])
        [tag_predic2  ,tag_comp2  ] = self.calculator.predic_n_comp(tag_x2    , tag_y2    , self.p, self.calculator.sigma_d, est_1, tag_comp2  , d[5])
        [tag_predicAP1,tag_compAP1] = self.calculator.predic_n_comp(tag_ap_x_1, tag_ap_y_1, self.p, self.calculator.sigma_d, est_1, tag_compAP1, d[0])
        [tag_predicAP2,tag_compAP2] = self.calculator.predic_n_comp(tag_ap_x_2, tag_ap_y_2, self.p, self.calculator.sigma_d, est_1, tag_compAP2, d[1])
        [tag_predicAP3,tag_compAP3] = self.calculator.predic_n_comp(tag_ap_x_3, tag_ap_y_3, self.p, self.calculator.sigma_d, est_1, tag_compAP3, d[2])
        [tag_predicAP4,tag_compAP4] = self.calculator.predic_n_comp(tag_ap_x_4, tag_ap_y_4, self.p, self.calculator.sigma_d, est_1, tag_compAP4, d[3])

        self.tag_predict = tag_predic1 + tag_predic2 + tag_predicAP1 + tag_predicAP2 + tag_predicAP3 + tag_predicAP4
        self.tag_comp    = tag_comp1 + tag_comp2 + tag_compAP1 + tag_compAP2 + tag_compAP3 + tag_compAP4