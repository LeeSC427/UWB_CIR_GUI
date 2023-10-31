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
from multiprocessing import Process
import time
from pytictoc import TicToc
import psutil
import csv

data_history=[] 

Tag1 = np.array([1500, 1500]).reshape(2,1) #태그 실제 위치
# print(Test)
Tag2 = np.array([1500, 1500]).reshape(2,1) #태그 실제 위치
Tag3 = np.array([1500,1500]).reshape(2,1) #태그 실제 위치

est1 = np.array([100, 140]).reshape(2,1) #추정 초기값
est2 = np.array([200, 320]).reshape(2,1) #추정 초기값
est3 = np.array([250, 250]).reshape(2,1) #추정 초기값
ap1 = np.array([0, 0]).reshape(2,1) #1번 앵커 위치
ap2 = np.array([3600, 0]).reshape(2,1) #2번 앵커 위치
ap3 = np.array([1200, 2400]).reshape(2,1) #3번 앵커 위치
ap4 = np.array([3000, 2400]).reshape(2,1) #4번 앵커 위치

t1_comp1=np.array([0,0]).reshape(2,1) #Compensation 초기값
t1_comp2=np.array([0,0]).reshape(2,1) #Compensation 초기값
t1_compAP1=np.array([0,0]).reshape(2,1) #Compensation 초기값
t1_compAP2=np.array([0,0]).reshape(2,1) #Compensation 초기값
t1_compAP3=np.array([0,0]).reshape(2,1) #Compensation 초기값
t1_compAP4=np.array([0,0]).reshape(2,1) #Compensation 초기값

t2_comp1=np.array([0,0]).reshape(2,1) #Compensation 초기값
t2_comp2=np.array([0,0]).reshape(2,1) #Compensation 초기값
t2_compAP1=np.array([0,0]).reshape(2,1) #Compensation 초기값
t2_compAP2=np.array([0,0]).reshape(2,1) #Compensation 초기값
t2_compAP3=np.array([0,0]).reshape(2,1) #Compensation 초기값
t2_compAP4=np.array([0,0]).reshape(2,1) #Compensation 초기값

t3_comp1=np.array([0,0]).reshape(2,1) #Compensation 초기값
t3_comp2=np.array([0,0]).reshape(2,1) #Compensation 초기값
t3_compAP1=np.array([0,0]).reshape(2,1) #Compensation 초기값
t3_compAP2=np.array([0,0]).reshape(2,1) #Compensation 초기값
t3_compAP3=np.array([0,0]).reshape(2,1) #Compensation 초기값
t3_compAP4=np.array([0,0]).reshape(2,1) #Compensation 초기값

kk =0

dff1=[] #데이터 저장 리스트
dff2=[] #데이터 저장 리스트
dff3=[] #데이터 저장 리스트
sigma_d = 0.1 

J=6 # 인접 단말 수
T=10

a = np.zeros((2,T+1)) #실시간 태그 위치 저장 변수
b = np.zeros((2,T+1)) #실시간 태그 위치 저장 변수
c = np.zeros((2,T+1)) #실시간 태그 위치 저장 변수

aa = np.zeros((2,T)) #실시간 태그 위치 저장 변수
bb = np.zeros((2,T)) #실시간 태그 위치 저장 변수
cc = np.zeros((2,T)) #실시간 태그 위치 저장 변수

JJ = 1/(J+1)


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
                print("ERROR: setting (remote) leds")
        else:
            error_code = SingleRegister()
            status = self.pozyx.getErrorCode(error_code)
            if status == POZYX_SUCCESS:
                print("ERROR Ranging, local %s" %
                      self.pozyx.getErrorMessage(error_code))
            else:
                print("ERROR Ranging, couldn't retrieve local error")
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

def dis(x1, x2): #거리 계산 함수
    d = x2 - x1
    dx = d[0]
    dy = d[1]
    d = d[0] ** 2 + d[1] ** 2
    d = np.sqrt(d)
    return d, dx, dy


def diff(x1, x2): #상대거리 계산 함수
    dd = x2 - x1
    dx = dd[0]
    dy = dd[1]

    return dx, dy


def func1(x, y, p, sigma_d, est, comp, z): #ADMM 알고리즘 예측 단계, 보상 단계 함수
    r = np.sqrt(x ** 2 + y ** 2)
    rx = x / r
    ry = y / r

    xx = sigma_d ** 2 / ((1 / p) * rx ** 2 + sigma_d ** 2)
    xxx = ((1 / p) * rx) / ((1 / p) * rx ** 2 + sigma_d ** 2)

    yy = sigma_d ** 2 / ((1 / p) * ry ** 2 + sigma_d ** 2)
    yyy = ((1 / p) * ry) / ((1 / p) * ry ** 2 + sigma_d ** 2)

    p0 = xx * (est[0] - comp[0]) - xxx * (z - r - rx * est[0])
    p1 = yy * (est[1] - comp[1]) - yyy * (z - r - ry * est[1])

    predic = np.array([p0, p1]).reshape(2, 1)

    # print(p0,p1)
    comp0 = comp[0] + p0 - est[0]
    comp1 = comp[1] + p1 - est[1]

    comp = np.array([comp0, comp1]).reshape(2, 1)
    # return predic, comp
    return predic, comp

def fig(est1,est2,est3,ap1,ap2,ap3,ap4): #Plot 함수

    plt.close('all')
    plt.plot(Tag1[0],Tag1[1],'ro',label="Veh1")
    plt.plot(Tag2[0],Tag2[1],'bo',label="Veh2")
    plt.plot(Tag3[0],Tag3[1],'go',label='Veh3')
    plt.plot(est1[0], est1[1], 'r-x', label="Est1")
    plt.plot(est2[0], est2[1], 'g-x', label="Est2")
    plt.plot(est3[0], est3[1], 'b-x', label="Est3")
    plt.plot(ap1[0], ap1[1], 'kd', label="AP1")
    plt.plot(ap2[0], ap2[1], 'kd', label="AP2")
    plt.plot(ap3[0], ap3[1], 'kd', label="AP3")
    plt.plot(ap4[0], ap4[1], 'kd', label="AP4")
    plt.legend(loc=1, ncol=3, fontsize=8)
    plt.xlabel('X-axis [mm]')
    plt.ylabel('Y-axis [mm]')
    plt.title('Python language programming')
    plt.grid(True)
    plt.show()
    # time.sleep(1)
    plt.close()

if __name__ == "__main__": ## 코드 실행 구간
    # Check for the latest PyPozyx version. Skip if this takes too long or is not needed by setting to False.
    check_pypozyx_version = True
    if check_pypozyx_version: ##Pozyx UWB 버전확인
        perform_latest_version_check()

    # hardcoded way to assign a serial port of the Pozyx
    serial_port = 'COM12' ##Pozyx UWB 연결 포트

    # the easier way
    serial_port = get_first_pozyx_serial_port() #Pozyx 연결 상태 확인 
    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()

    remote_id = 0x6824           # UWB tag1 ID
    remote = True               # Remote 사용 여부 결정 (True or False)
    if not remote:
        remote_id = None

    remote_id1 = 0x6846  #UWB tag2 ID
    remote = True
    if not remote:
        remote_id = None

    remote_id2 = 0x6862 #UWB tag3 ID
    remote = True
    if not remote:
        remote_id = None

    # destination_id1 = 0x0D5D
    # destination_id2 = 0x687B
    # destination_id3 = 0x684E
    # destination_id4 = 0x684F

    destination_id1 = 0x1162  #UWB anchor1 ID
    destination_id2 = 0x114A #UWB anchor2 ID
    destination_id3 = 0x1126 #UWB anchor3 ID
    destination_id4 = 0x1136 #UWB anchor4 ID

    # distance that separates the amount of LEDs lighting up.
    range_step_mm = 1000 #UWB 동작 관련 파라미터 (조정안함)

    # the ranging protocol, other one is PozyxConstants.RANGE_PROTOCOL_PRECISION
    ranging_protocol = PozyxConstants.RANGE_PROTOCOL_PRECISION #UWB 동작 관련 파라미터 (조정안함)


    pozyx = PozyxSerial(serial_port)
    r = ReadyToRange(pozyx, destination_id1, range_step_mm, ranging_protocol, remote_id) #Remote id와 destination1 사이 거리측정
    r1 = ReadyToRange(pozyx, destination_id2, range_step_mm, ranging_protocol, remote_id) #Remote id와 destination2 사이 거리측정
    r2 = ReadyToRange(pozyx, destination_id3, range_step_mm, ranging_protocol, remote_id) #Remote id와 destination3 사이 거리측정
    r3 = ReadyToRange(pozyx, destination_id4, range_step_mm, ranging_protocol, remote_id) #Remote id와 destination4 사이 거리측정
    r4 = ReadyToRange(pozyx, remote_id1, range_step_mm, ranging_protocol, remote_id) #Remote id와 remote id1 사이 거리측정
    r5 = ReadyToRange(pozyx, remote_id2, range_step_mm, ranging_protocol, remote_id) #Remote id와 remote id2 사이 거리측정

    rr = ReadyToRange(pozyx, destination_id1, range_step_mm, ranging_protocol, remote_id1) #Remote id1와 destination1 사이 거리측정
    rr1 = ReadyToRange(pozyx, destination_id2, range_step_mm, ranging_protocol, remote_id1) #Remote id1와 destination2 사이 거리측정
    rr2 = ReadyToRange(pozyx, destination_id3, range_step_mm, ranging_protocol, remote_id1) #Remote id1와 destination3 사이 거리측정
    rr3 = ReadyToRange(pozyx, destination_id4, range_step_mm, ranging_protocol, remote_id1) #Remote id1와 destination4 사이 거리측정
    rr4 = ReadyToRange(pozyx, remote_id, range_step_mm, ranging_protocol, remote_id1) #Remote id1와 remote id 사이 거리측정
    rr5 = ReadyToRange(pozyx, remote_id2, range_step_mm, ranging_protocol, remote_id1) #Remote id1와 remote id2 사이 거리측정

    rrr = ReadyToRange(pozyx, destination_id1, range_step_mm, ranging_protocol, remote_id2) #Remote id2와 destination1 사이 거리측정
    rrr1 = ReadyToRange(pozyx, destination_id2, range_step_mm, ranging_protocol, remote_id2) #Remote id2와 destination2 사이 거리측정
    rrr2 = ReadyToRange(pozyx, destination_id3, range_step_mm, ranging_protocol, remote_id2) #Remote id2와 destination3 사이 거리측정
    rrr3 = ReadyToRange(pozyx, destination_id4, range_step_mm, ranging_protocol, remote_id2) #Remote id2와 destination4 사이 거리측정
    rrr4 = ReadyToRange(pozyx, remote_id, range_step_mm, ranging_protocol, remote_id2) #Remote id2와 remote id 사이 거리측정
    rrr5 = ReadyToRange(pozyx, remote_id1, range_step_mm, ranging_protocol, remote_id2) #Remote id2와 remote id1 사이 거리측정



    r.setup()
    while True:
        r.loop()
        # print(destination_id)
        # data_history = data_history + [[r]]
        # print(r)
        # ddd = pd.DataFrame(data_history)
        # filename = time.strftime('%m%d%H%M'+'.csv')
        # data_path = ''
        # ddd.to_csv('ddd.csv', index=False)
        # ttt.tic()
        d1= np.array([r.loop(), r1.loop(), r2.loop(), r3.loop(), r4.loop(), r5.loop()]) # 태그 1번 거리 측정값 저장
        d2= np.array([rr.loop(), rr1.loop(), rr2.loop(), rr3.loop(), rr4.loop(), rr5.loop()]) # 태그 2번 거리 측정값 저장
        d3= np.array([rrr.loop(), rrr1.loop(), rrr2.loop(), rrr3.loop(), rrr4.loop(), rrr5.loop()]) # 태그 3번 거리 측정값 저장

        kk=kk+1
        print(kk)
        #각 태그의 거리 측정값 엑셀 저장
        dff1 = pd.DataFrame([d1])
        print(dff1)
        dff1.to_csv('data_cp8_t1.csv',mode='a', header=False, index=False)
        dff2 = pd.DataFrame([d2])
        dff2.to_csv('data_cp8_t2.csv',mode='a', header=False, index=False)
        dff3 = pd.DataFrame([d3])
        dff3.to_csv('data_cp8_t3.csv',mode='a', header=False, index=False)
        t = 0
        
        p = 100 # ADMM 알고리즘 패널티 계수 파라미터
   
        for i in range(0, T):
            # i=i+1
            [t1_x1, t1_y1] = diff(est1, est2)
            [t1_x2, t1_y2] = diff(est1, est3)
            [t1_ap1x, t1_ap1y] = diff(est1, ap1)
            [t1_ap2x, t1_ap2y] = diff(est1, ap2)
            [t1_ap3x, t1_ap3y] = diff(est1, ap3)
            [t1_ap4x, t1_ap4y] = diff(est1, ap4)

            [t2_x1, t2_y1] = diff(est2, est1)
            [t2_x2, t2_y2] = diff(est2, est3)
            [t2_ap1x, t2_ap1y] = diff(est2, ap1)
            [t2_ap2x, t2_ap2y] = diff(est2, ap2)
            [t2_ap3x, t2_ap3y] = diff(est2, ap3)
            [t2_ap4x, t2_ap4y] = diff(est2, ap4)

            [t3_x1, t3_y1] = diff(est3, est1)
            [t3_x2, t3_y2] = diff(est3, est2)
            [t3_ap1x, t3_ap1y] = diff(est3, ap1)
            [t3_ap2x, t3_ap2y] = diff(est3, ap2)
            [t3_ap3x, t3_ap3y] = diff(est3, ap3)
            [t3_ap4x, t3_ap4y] = diff(est3, ap4)
            
            [t1_predic1, t1_comp1] = func1(t1_x1, t1_y1, p, sigma_d, est1, t1_comp1, d1[4]) #태그1와 태그2 사이 거리값을 이용한 태그1 위치 예측, 보상값 계산
            [t1_predic2, t1_comp2] = func1(t1_x2, t1_y2, p, sigma_d, est1, t1_comp2, d1[5]) #태그1와 태그3 사이 거리값을 이용한 태그1 위치 예측, 보상값 계산
            [t1_predicAP1, t1_compAP1] = func1(t1_ap1x, t1_ap1y, p, sigma_d, est1, t1_compAP1, d1[0]) #태그1와 앵커1 사이 거리값을 이용한 태그1 위치 예측, 보상값 계산
            [t1_predicAP2, t1_compAP2] = func1(t1_ap2x, t1_ap2y, p, sigma_d, est1, t1_compAP2, d1[1]) #태그1와 앵커2 사이 거리값을 이용한 태그1 위치 예측, 보상값 계산
            [t1_predicAP3, t1_compAP3] = func1(t1_ap3x, t1_ap3y, p, sigma_d, est1, t1_compAP3, d1[2]) #태그1와 앵커3 사이 거리값을 이용한 태그1 위치 예측, 보상값 계산
            [t1_predicAP4, t1_compAP4] = func1(t1_ap4x, t1_ap4y, p, sigma_d, est1, t1_compAP4, d1[3]) #태그1와 앵커4 사이 거리값을 이용한 태그1 위치 예측, 보상값 계산

            t1_predic = t1_predic1 + t1_predic2 + t1_predicAP1 + t1_predicAP2 + t1_predicAP3 + t1_predicAP4 #태그1의 모든 예측값 덧셈
            t1_comp = t1_comp1 + t1_comp2 + t1_compAP1 + t1_compAP2 + t1_compAP3 + t1_compAP4 #태그1의 모든 보상값 덧셈

            [t2_predic1, t2_comp1] = func1(t2_x1, t2_y1, p, sigma_d, est2, t2_comp1, d2[4]) #태그2와 태그1 사이 거리값을 이용한 태그2 위치 예측, 보상값 계산
            [t2_predic2, t2_comp2] = func1(t2_x2, t2_y2, p, sigma_d, est2, t2_comp2, d2[5]) #태그2와 태그3 사이 거리값을 이용한 태그2 위치 예측, 보상값 계산
            [t2_predicAP1, t2_compAP1] = func1(t2_ap1x, t2_ap1y, p, sigma_d, est2, t2_compAP1, d2[0]) #태그2와 앵커1 사이 거리값을 이용한 태그2 위치 예측, 보상값 계산
            [t2_predicAP2, t2_compAP2] = func1(t2_ap2x, t2_ap2y, p, sigma_d, est2, t2_compAP2, d2[1]) #태그2와 앵커2 사이 거리값을 이용한 태그2 위치 예측, 보상값 계산
            [t2_predicAP3, t2_compAP3] = func1(t2_ap3x, t2_ap3y, p, sigma_d, est2, t2_compAP3, d2[2]) #태그2와 앵커3 사이 거리값을 이용한 태그2 위치 예측, 보상값 계산
            [t2_predicAP4, t2_compAP4] = func1(t2_ap4x, t2_ap4y, p, sigma_d, est2, t2_compAP4, d2[3]) #태그2와 앵커4 사이 거리값을 이용한 태그2 위치 예측, 보상값 계산

            t2_predic = t2_predic1 + t2_predic2 + t2_predicAP1 + t2_predicAP2 + t2_predicAP3 + t2_predicAP4 #태그2의 모든 예측값 덧셈
            t2_comp = t2_comp1 + t2_comp2 + t2_compAP1 + t2_compAP2 + t2_compAP3 + t2_compAP4 #태그2의 모든 보상값 덧셈

            [t3_predic1, t3_comp1] = func1(t3_x1, t3_y1, p, sigma_d, est3, t3_comp1, d3[4]) #태그3와 태그1 사이 거리값을 이용한 태그3 위치 예측, 보상값 계산
            [t3_predic2, t3_comp2] = func1(t3_x2, t3_y2, p, sigma_d, est3, t3_comp2, d3[5]) #태그3와 태그2 사이 거리값을 이용한 태그3 위치 예측, 보상값 계산
            [t3_predicAP1, t3_compAP1] = func1(t3_ap1x, t3_ap1y, p, sigma_d, est3, t3_compAP1, d3[0]) #태그3와 앵커2 사이 거리값을 이용한 태그3 위치 예측, 보상값 계산
            [t3_predicAP2, t3_compAP2] = func1(t3_ap2x, t3_ap2y, p, sigma_d, est3, t3_compAP2, d3[1]) #태그3와 앵커2 사이 거리값을 이용한 태그3 위치 예측, 보상값 계산
            [t3_predicAP3, t3_compAP3] = func1(t3_ap3x, t3_ap3y, p, sigma_d, est3, t3_compAP3, d3[2]) #태그3와 앵커2 사이 거리값을 이용한 태그3 위치 예측, 보상값 계산
            [t3_predicAP4, t3_compAP4] = func1(t3_ap4x, t3_ap4y, p, sigma_d, est3, t3_compAP4, d3[3]) #태그3와 앵커2 사이 거리값을 이용한 태그3 위치 예측, 보상값 계산

            t3_predic = t3_predic1 + t3_predic2 + t3_predicAP1 + t3_predicAP2 + t3_predicAP3 + t3_predicAP4 #태그3의 모든 예측값 덧셈
            t3_comp = t3_comp1 + t3_comp2 + t3_compAP1 + t3_compAP2 + t3_compAP3 + t3_compAP4 #태그3의 모든 예측값 덧셈

            est1 = JJ * (est1 + t1_predic + t1_comp) ## 태그1 expectation 
            est2 = JJ * (est2 + t2_predic + t2_comp) ## 태그2 expectation
            est3 = JJ * (est3 + t3_predic + t3_comp) ## 태그3 expectation


            aa[0, i] = est1[0]
            aa[1, i] = est1[1]

            bb[0, i] = est2[0]
            bb[1, i] = est2[1]

            cc[0, i] = est3[0]
            cc[1, i] = est3[1]
        

        print("Est1:", est1)
        print("Est2:", est2)
        print("Est3:", est3)


