# -*- coding: utf-8 -*-
"""
Created on Wed Nov 24 10:50:14 2021

@author: Thanh_Hai
"""

import cv2
import numpy as np
from PyQt5 import QtWidgets
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot, QThread, Qt
from PyQt5.QtWidgets import QApplication, QMessageBox
from math import cos
from math import sin
from math import pi
from math import atan2
from math import sqrt
import time
import serial, serial.tools.list_ports
from threading import Thread, Event

class UI_init(QObject):
    def __init__(self):
        super().__init__()
        
    def Window_Init(self):
        # robot para
        self.ui.l0 = 0.03
        self.ui.l1 = 0.094
        self.ui.l2 = 0.094
        self.ui.l3 = 0.094
        self.ui.l4 = 0.094
        self.ui.l5 = 0.094
        self.ui.l6 = 0.094
        self.ui.l7 = 0.03
        m_o = 0.21
        self.ui.m = np.array([[m_o],[m_o],[m_o],[m_o],[m_o],[0.25],[0.1]])
        
        # Calc
        ## Kine
        self.ui.the_calc = np.array([0,0,0,0,0,pi/2,0])
        self.ui.IK_in = np.array([0.65,0,0,90,0,180])
        ## DYN
        self.ui.the_dyn_d = np.array([[0],[0],[0],[0],[0],[pi/2],[0]])
        self.ui.the_dyn_t = np.array([[0],[0],[0],[0],[0],[pi/2],[0]])
        self.ui.the_dot_d = np.array([[0],[0],[0],[0],[0],[0],[0]])
        self.ui.M_dyn = np.zeros((7,7))
        self.ui.the_dyn = np.array([[0],[0],[0],[0],[0],[pi/2],[0]])
        self.ui.the_dot_dyn = np.array([[0],[0],[0],[0],[0],[0],[0]])
        
        # Plot
        self.ui.the_plot = np.array([0,0,0,0,0,pi/2,0])
        self.ui.the = np.array([0,0,0,0,0,pi/2,0])
        self.ui.Ex = np.array([])
        self.ui.Ey = np.array([])
        self.ui.Ez = np.array([])
        
        # Disp
        ## the deg
        self.ui.the_deg = np.round(self.ui.the*180/pi,3)
        ## pos & ori
        self.ui.X = self.ui.tb_X_init.text()
        self.ui.Y = self.ui.tb_Y_init.text()
        self.ui.Z = self.ui.tb_Z_init.text()
        self.ui.alp = self.ui.tb_alp_init.text()
        self.ui.bet = self.ui.tb_bet_init.text()
        self.ui.gam = self.ui.tb_gam_init.text()
        ## Control para
        self.ui.Kp = np.zeros((7,1))
        self.ui.Ki = np.zeros((7,1))
        self.ui.Kd = np.zeros((7,1))
        self.ui.pos_BC = np.array([[0],[0],[0]])
        ## The max min
        self.ui.the1_max = self.ui.tb_the1_max.text()
        self.ui.the2_max = self.ui.tb_the2_max.text()
        self.ui.the3_max = self.ui.tb_the3_max.text()
        self.ui.the4_max = self.ui.tb_the4_max.text()
        self.ui.the5_max = self.ui.tb_the5_max.text()
        self.ui.the6_max = self.ui.tb_the6_max.text()
        self.ui.the7_max = self.ui.tb_the7_max.text()
        self.ui.the1_min = self.ui.tb_the1_min.text()
        self.ui.the2_min = self.ui.tb_the2_min.text()
        self.ui.the3_min = self.ui.tb_the3_min.text()
        self.ui.the4_min = self.ui.tb_the4_min.text()
        self.ui.the5_min = self.ui.tb_the5_min.text()
        self.ui.the6_min = self.ui.tb_the6_min.text()
        self.ui.the7_min = self.ui.tb_the7_min.text()
        ## Times
        self.ui.timed = self.ui.tb_time_set.text()
        self.ui.time_dyn = 5
        ## Error
        self.ui.error = self.ui.tb_err_set.text()
        
        # Control
        self.ui.time_s = 0
        self.ui.the1_BC = 0
        self.ui.the3_BC = 0
        self.ui.the4_BC = 0
        self.ui.the5_BC = 0
        self.ui.the6_BC = 0
        self.ui.the7_BC = 0
        self.ui.X_BC = 0
        self.ui.Y_BC = 0
        self.ui.Z_BC = 0
        self.ui.alp_BC = 0
        self.ui.bet_BC = 0
        self.ui.gam_BC = 0
        self.ui.sim = 0
        self.ui.run = 1
        self.ui.inp_the = 1
        self.ui.inp_pos = 0
        
        self.ui.tab_plot = 1
        
        # Camera
        self.ui.x_obj = []
        self.ui.y_obj = []
        self.ui.z_obj = []
        
        self.ui.x_obs = []
        self.ui.y_obs = []
        self.ui.z_obs = []
        
        # CAMERA PARA
        '''
        parameter cam1 - cam left
        '''
        #Rotation vector - Tranlation vector
        self.ui.R_vec1 = np.array([-0.562634291082027,	-0.0439455024996611,	-0.0393144889422307])
        self.ui.tb_R1_1.setText(str(round(self.ui.R_vec1[0],3)))
        self.ui.tb_R1_2.setText(str(round(self.ui.R_vec1[1],3)))
        self.ui.tb_R1_3.setText(str(round(self.ui.R_vec1[2],3)))
        self.ui.T_vec1 = np.array([-59.8296872885722,	-96.3783093600323,	619.992477974895])
        self.ui.tb_T1_1.setText(str(round(self.ui.T_vec1[0],3)))
        self.ui.tb_T1_2.setText(str(round(self.ui.T_vec1[1],3)))
        self.ui.tb_T1_3.setText(str(round(self.ui.T_vec1[2],3)))
        #Pinhole and Focus
        self.ui.Ox1 = 665.626065281391	
        self.ui.tb_Ox1.setText(str(round(self.ui.Ox1,3)))
        self.ui.Oy1 = 425.005266399569
        self.ui.tb_Oy1.setText(str(round(self.ui.Oy1,3)))
        self.ui.Fx1 = 851.345834716972	
        self.ui.tb_Fx1.setText(str(round(self.ui.Fx1,3)))
        self.ui.Fy1 = 896.579325754774
        self.ui.tb_Fy1.setText(str(round(self.ui.Fy1,3)))
        #Distortion coefficients
        self.ui.RadialDistorsion1 = np.array([-0.502664239970006,	1.45374226026835,	-4.66904369299741])
        self.ui.TangentialDistorsion1 = np.array([-0.0370434222560384,	-0.00411059230640124])
        '''
        parameter cam2 - cam right
        '''
        #Rotation vector - Tranlation vector
        self.ui.R_vec2 = np.array([-0.789314814260095,	-0.0291648202768128,	0.0161979607030139])
        self.ui.tb_R2_1.setText(str(round(self.ui.R_vec2[0],3)))
        self.ui.tb_R2_2.setText(str(round(self.ui.R_vec2[1],3)))
        self.ui.tb_R2_3.setText(str(round(self.ui.R_vec2[2],3)))
        self.ui.T_vec2 = np.array([-169.164847659025,	-2.95103378148096,	846.765975027944])
        self.ui.tb_T2_1.setText(str(round(self.ui.T_vec2[0],3)))
        self.ui.tb_T2_2.setText(str(round(self.ui.T_vec2[1],3)))
        self.ui.tb_T2_3.setText(str(round(self.ui.T_vec2[2],3)))
        #Pinhole and Focus
        self.ui.Ox2 = 679.414484101639		 #NO CHANGE
        self.ui.tb_Ox2.setText(str(round(self.ui.Ox2,3)))
        self.ui.Oy2 = 327.507621149549 #NO CHANGE
        self.ui.tb_Oy2.setText(str(round(self.ui.Oy2,3)))
        self.ui.Fx2 = 1156.22175715016		 #NO CHANGE
        self.ui.tb_Fx2.setText(str(round(self.ui.Fx2,3)))
        self.ui.Fy2 = 1312.28434279010 #NO CHANGE
        self.ui.tb_Fy2.setText(str(round(self.ui.Fy2,3)))
        #Distortion coefficients
        self.ui.RadialDistorsion2 = np.array([-0.914275600965703,	1.75997599313322,	-7.61374031923349]) #NO CHANGE
        self.ui.TangentialDistorsion2 = np.array([0.0449371408571962,	-0.0193403204959928]) #NO CHANGE
        ## Dist Matrix init
        self.ui.Mi1 = np.zeros((3,3))
        self.ui.Me1 = np.zeros((4,4))
        self.ui.dist1 = np.zeros((5,1))
        self.ui.Mi2 = np.zeros((3,3))
        self.ui.Me2 = np.zeros((4,4))        
        self.ui.dist2 = np.zeros((5,1))
        
class UI_serial(QObject):
    data_available = pyqtSignal(str)
    theta_sent = pyqtSignal(str)
    def __init__(self):
        super().__init__()
        self.sim = 0
        self.run = 1
        self.serialPort = serial.Serial()
        self.serialPort.timeout = 0.5
        self.portList = []
        self.thread1 = None
        self.alive1 = Event()
        self.alive2 = Event()
        
    def update_ports(self):
        self.portList = [port.device for port in serial.tools.list_ports.comports()]
        # print(self.portList)
        
    def connect_serial(self):
        try:
            self.serialPort.open()
        except:
            QMessageBox.critical(None, "Error", "Please choose COM Port first. Or try to connect to device again.",QMessageBox.Cancel) 
            print("ERROR")
            
        if(self.serialPort.is_open and self.sim == True):
            self.start_thread1()
            print('thread1_start')
            
        if(self.serialPort.is_open and self.run == True):
            self.start_thread2()
            print('thread2_start')
    
    def disconnect_serial(self):
        if(self.alive1.isSet()):
            self.stop_thread1()
        if(self.alive2.isSet()):
            self.stop_thread2()
        self.serialPort.close()
    
    def read_serial(self):
        while (self.alive1.isSet() and self.serialPort.is_open):
            data = self.serialPort.readline().decode("utf-8").strip()
            if (len(data)>1):
                self.data_available.emit(data)
                
    def start_stream_data(self):
        while (self.alive2.isSet() and self.serialPort.is_open):
            strdata = self.serialPort.readline().decode().strip()
            print(len(strdata))
            if(len(strdata)==62):
                self.theta_sent.emit(strdata)
    
    def send_data(self,data):
        if(self.serialPort.is_open):
            message = str(data) + "\n"
            self.serialPort.write(message.encode())
        
    def start_thread1(self):
        self.thread1 = Thread(target = self.read_serial)
        self.thread1.setDaemon(1)
        self.alive1.set()
        self.thread1.start()
        
    def stop_thread1(self):
        if(self.thread1 is not None):
            self.alive1.clear()
            self.thread1.join()
            self.thread1 = None
            
    def start_thread2(self):
        self.thread2 = Thread(target = self.start_stream_data)
        self.thread2.setDaemon(1)
        self.alive2.set()
        self.thread2.start()
        
    def stop_thread2(self):
        if(self.thread2 is not None):
            self.alive2.clear()
            self.thread2.join()
            self.thread2 = None
        
class UI_kinematics(QObject):
    def __init__(self):
        super().__init__()
        
    def TransMatrix(a, alp, d, the):
        T = np.array([(         cos(the),         -sin(the),         0,           a),
                      (sin(the)*cos(alp), cos(the)*cos(alp), -sin(alp), -sin(alp)*d),
                      (sin(the)*sin(alp), cos(the)*sin(alp),  cos(alp),  cos(alp)*d),
                      (                0,                 0,         0,          1)])
        return T
    
    def Fcn_FK(the, mode):
        # EXTRACT DATA
        l0 = 0.03
        l1 = 0.094
        l2 = 0.094
        l3 = 0.094
        l4 = 0.094
        l5 = 0.094
        l6 = 0.094
        l7 = 0.03
        the1 = the[0]; the2 = the[1]; the3 = the[2]; the4 = the[3]
        the5 = the[4]; the6 = the[5]; the7 = the[6]
        
        # DH table
        T01  = UI_kinematics.TransMatrix(l0+l1, pi/2,    0,the1)
        T12  = UI_kinematics.TransMatrix(    0,-pi/2,    0,the2)
        T23  = UI_kinematics.TransMatrix(l2+l3, pi/2,    0,the3)
        T34  = UI_kinematics.TransMatrix(    0,-pi/2,    0,the4)
        T45  = UI_kinematics.TransMatrix(l4+l5, pi/2,    0,the5)
        T56  = UI_kinematics.TransMatrix(    0,-pi/2,    0,the6)
        T67  = UI_kinematics.TransMatrix(    0, pi/2,    0,the7)
        T7EE = UI_kinematics.TransMatrix(    0,    0,l6+l7,   0)
        
        # FK 
        T02  = T01@T12
        T03  = T02@T23
        T04  = T03@T34
        T05  = T04@T45
        T06  = T05@T56
        T07  = T06@T67
        T0EE = T07@T7EE
        
        # Pose
        P1  = T01[0:3,3]
        P2  = T02[0:3,3]
        P3  = T03[0:3,3]
        P4  = T04[0:3,3]
        P5  = T05[0:3,3]
        P6  = T06[0:3,3]
        P7  = T07[0:3,3]
        Pee = T0EE[0:3,3]
        
        Tee = np.array([T0EE[0,3], T0EE[1,3], T0EE[2,3], T0EE[0,0], T0EE[0,1], T0EE[0,2], T0EE[1,0], T0EE[1,1], T0EE[1,2], T0EE[2,0], T0EE[2,1], T0EE[2,2]])
        
        Q1 = np.array([0, P1[0], P2[0], P3[0], P4[0], P5[0], P6[0], P7[0], Pee[0]])
        Q2 = np.array([0, P1[1], P2[1], P3[1], P4[1], P5[1], P6[1], P7[1], Pee[1]])
        Q3 = np.array([0, P1[2], P2[2], P3[2], P4[2], P5[2], P6[2], P7[2], Pee[2]])
        
        if mode == 0:
            return T0EE
        if mode == 1:
            return Q1, Q2, Q3
        if mode == 2:
            return [T0EE, Pee, Tee, Q1, Q2, Q3]
        
    def Orientation(T0EE):
        alp1 = atan2(T0EE[1,0],T0EE[0,0])
        bet1 = atan2(-T0EE[2,0],sqrt(T0EE[2,1]**2+T0EE[2,2]**2))
        gam1 = atan2(T0EE[2,1],T0EE[2,2])
        return [alp1, bet1, gam1]
    
    def CalcFK(the):
        T0EE = UI_kinematics.Fcn_FK(the, 0)
        X = T0EE[0,3]; Y = T0EE[1,3]; Z = T0EE[2,3];
        [alp, bet, gam] = UI_kinematics.Orientation(T0EE)        
        return [X, Y, Z, alp, bet, gam]
    
    def sim_FK(self, time):
        the1 = self.ui.tb_the1_FK.text()
        the2 = self.ui.tb_the2_FK.text()
        the3 = self.ui.tb_the3_FK.text()
        the4 = self.ui.tb_the4_FK.text()
        the5 = self.ui.tb_the5_FK.text()
        the6 = self.ui.tb_the6_FK.text()
        the7 = self.ui.tb_the7_FK.text()
        timed = float(time)
        vtd = np.array([the1, the2, the3, the4, the5, the6, the7]).astype(np.float64())*np.pi/180
        vtt = self.ui.the_plot
        self.ui.a, self.ui.b, self.ui.c, self.ui.d = UI_kinematics.calc_abcd(vtt,vtd,time)
        self.ui.timed = timed
        self.ui.timec = 0.1
        self.ui.timer2.start(10)
    
    def sim_IK(self, time):
        the1 = self.ui.tb_the1_IK.text()
        the2 = self.ui.tb_the2_IK.text()
        the3 = self.ui.tb_the3_IK.text()
        the4 = self.ui.tb_the4_IK.text()
        the5 = self.ui.tb_the5_IK.text()
        the6 = self.ui.tb_the6_IK.text()
        the7 = self.ui.tb_the7_IK.text()
        timed = float(time)
        vtd = np.array([the1, the2, the3, the4, the5, the6, the7]).astype(np.float64())*np.pi/180
        vtt = self.ui.the_plot
        self.ui.a, self.ui.b, self.ui.c, self.ui.d = UI_kinematics.calc_abcd(vtt,vtd,time)
        self.ui.timed = timed
        self.ui.timec = 0.1
        self.ui.timer2.start(10)
        
    def calc_abcd(vtt,vtd,time):
        timed = float(time)
        a = vtt
        b = np.array([0,0,0,0,0,0,0])
        c = 3*(vtd - vtt)/(timed*timed)
        d = -2*(vtd - vtt)/(timed*timed*timed)
        return a,b,c,d
    
    def sim_traject(self):
        T = self.ui.timec
        self.ui.the_plot = self.ui.a + self.ui.b*T + self.ui.c*(T*T) + self.ui.d*(T*T*T)
        self.update_ee_2(self.ui.the_plot)
        if T > self.ui.timed - 0.1:
            self.ui.timer2.stop()
        else:
            self.ui.timec += 0.1
    
    def Fcn_J_Inv(the, L):
        the1 = the[0]; the2 = the[1]; the3 = the[2]; the4 = the[3]
        the5 = the[4]; the6 = the[5]; the7 = the[6]
        
        l0 = float(L[0]); l1 = float(L[1]); l2 = float(L[2]); l3 = float(L[3])
        l4 = float(L[4]); l5 = float(L[5]); l6 = float(L[6]); l7 = float(L[7])
        
        Jee = np.array([[- (l6 + l7)*(cos(the6)*(sin(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) + cos(the4)*sin(the1)*sin(the2)) + sin(the6)*(cos(the5)*(cos(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) - sin(the1)*sin(the2)*sin(the4)) + sin(the5)*(cos(the1)*cos(the3) - cos(the2)*sin(the1)*sin(the3)))) - (cos(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) - sin(the1)*sin(the2)*sin(the4))*(l4 + l5) - cos(the2)*sin(the1)*(l2 + l3),   (l6 + l7)*(cos(the6)*(cos(the1)*cos(the2)*cos(the4) - cos(the1)*cos(the3)*sin(the2)*sin(the4)) - sin(the6)*(cos(the5)*(cos(the1)*cos(the2)*sin(the4) + cos(the1)*cos(the3)*cos(the4)*sin(the2)) - cos(the1)*sin(the2)*sin(the3)*sin(the5))) - (cos(the1)*cos(the2)*sin(the4) + cos(the1)*cos(the3)*cos(the4)*sin(the2))*(l4 + l5) - cos(the1)*sin(the2)*(l2 + l3), (l6 + l7)*(sin(the6)*(sin(the5)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) - cos(the4)*cos(the5)*(cos(the3)*sin(the1) + cos(the1)*cos(the2)*sin(the3))) - cos(the6)*sin(the4)*(cos(the3)*sin(the1) + cos(the1)*cos(the2)*sin(the3))) - cos(the4)*(l4 + l5)*(cos(the3)*sin(the1) + cos(the1)*cos(the2)*sin(the3)), (sin(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) - cos(the1)*cos(the4)*sin(the2))*(l4 + l5) - (l6 + l7)*(cos(the6)*(cos(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) + cos(the1)*sin(the2)*sin(the4)) - cos(the5)*sin(the6)*(sin(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) - cos(the1)*cos(the4)*sin(the2))),  sin(the6)*(l6 + l7)*(sin(the5)*(cos(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) + cos(the1)*sin(the2)*sin(the4)) - cos(the5)*(cos(the3)*sin(the1) + cos(the1)*cos(the2)*sin(the3))),  (l6 + l7)*(sin(the6)*(sin(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) - cos(the1)*cos(the4)*sin(the2)) - cos(the6)*(cos(the5)*(cos(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) + cos(the1)*sin(the2)*sin(the4)) + sin(the5)*(cos(the3)*sin(the1) + cos(the1)*cos(the2)*sin(the3)))), 0],
                        [                                                                                                                                                                                                                                                                                                                                                                                                                                                                  0,                                                                                   cos(the2)*(l2 + l3) - (l4 + l5)*(sin(the2)*sin(the4) - cos(the2)*cos(the3)*cos(the4)) - (l6 + l7)*(sin(the6)*(cos(the5)*(sin(the2)*sin(the4) - cos(the2)*cos(the3)*cos(the4)) + cos(the2)*sin(the3)*sin(the5)) - cos(the6)*(cos(the4)*sin(the2) + cos(the2)*cos(the3)*sin(the4))),                                                                                                                                       - (l6 + l7)*(sin(the6)*(cos(the3)*sin(the2)*sin(the5) + cos(the4)*cos(the5)*sin(the2)*sin(the3)) + cos(the6)*sin(the2)*sin(the3)*sin(the4)) - cos(the4)*sin(the2)*sin(the3)*(l4 + l5),                                                                                                                                     (l4 + l5)*(cos(the2)*cos(the4) - cos(the3)*sin(the2)*sin(the4)) + (l6 + l7)*(cos(the6)*(cos(the2)*sin(the4) + cos(the3)*cos(the4)*sin(the2)) + cos(the5)*sin(the6)*(cos(the2)*cos(the4) - cos(the3)*sin(the2)*sin(the4))),                                                                               -sin(the6)*(sin(the5)*(cos(the2)*sin(the4) + cos(the3)*cos(the4)*sin(the2)) + cos(the5)*sin(the2)*sin(the3))*(l6 + l7),                                                                                                                            (l6 + l7)*(cos(the6)*(cos(the5)*(cos(the2)*sin(the4) + cos(the3)*cos(the4)*sin(the2)) - sin(the2)*sin(the3)*sin(the5)) + sin(the6)*(cos(the2)*cos(the4) - cos(the3)*sin(the2)*sin(the4))), 0],
                        [  cos(the1)*cos(the2)*(l2 + l3) - (l6 + l7)*(cos(the6)*(sin(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) - cos(the1)*cos(the4)*sin(the2)) + sin(the6)*(cos(the5)*(cos(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) + cos(the1)*sin(the2)*sin(the4)) + sin(the5)*(cos(the3)*sin(the1) + cos(the1)*cos(the2)*sin(the3)))) - (cos(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) + cos(the1)*sin(the2)*sin(the4))*(l4 + l5), - (l6 + l7)*(sin(the6)*(cos(the5)*(cos(the2)*sin(the1)*sin(the4) + cos(the3)*cos(the4)*sin(the1)*sin(the2)) - sin(the1)*sin(the2)*sin(the3)*sin(the5)) - cos(the6)*(cos(the2)*cos(the4)*sin(the1) - cos(the3)*sin(the1)*sin(the2)*sin(the4))) - (l4 + l5)*(cos(the2)*sin(the1)*sin(the4) + cos(the3)*cos(the4)*sin(the1)*sin(the2)) - sin(the1)*sin(the2)*(l2 + l3), cos(the4)*(l4 + l5)*(cos(the1)*cos(the3) - cos(the2)*sin(the1)*sin(the3)) - (l6 + l7)*(sin(the6)*(sin(the5)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) - cos(the4)*cos(the5)*(cos(the1)*cos(the3) - cos(the2)*sin(the1)*sin(the3))) - cos(the6)*sin(the4)*(cos(the1)*cos(the3) - cos(the2)*sin(the1)*sin(the3))), (l6 + l7)*(cos(the6)*(cos(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) - sin(the1)*sin(the2)*sin(the4)) - cos(the5)*sin(the6)*(sin(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) + cos(the4)*sin(the1)*sin(the2))) - (sin(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) + cos(the4)*sin(the1)*sin(the2))*(l4 + l5), -sin(the6)*(l6 + l7)*(sin(the5)*(cos(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) - sin(the1)*sin(the2)*sin(the4)) - cos(the5)*(cos(the1)*cos(the3) - cos(the2)*sin(the1)*sin(the3))), -(l6 + l7)*(sin(the6)*(sin(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) + cos(the4)*sin(the1)*sin(the2)) - cos(the6)*(cos(the5)*(cos(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) - sin(the1)*sin(the2)*sin(the4)) + sin(the5)*(cos(the1)*cos(the3) - cos(the2)*sin(the1)*sin(the3)))), 0],
                        [cos(the7)*(sin(the6)*(sin(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) + cos(the4)*sin(the1)*sin(the2)) - cos(the6)*(cos(the5)*(cos(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) - sin(the1)*sin(the2)*sin(the4)) + sin(the5)*(cos(the1)*cos(the3) - cos(the2)*sin(the1)*sin(the3)))) + sin(the7)*(sin(the5)*(cos(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) - sin(the1)*sin(the2)*sin(the4)) - cos(the5)*(cos(the1)*cos(the3) - cos(the2)*sin(the1)*sin(the3))), sin(the7)*(sin(the5)*(cos(the1)*cos(the2)*sin(the4) + cos(the1)*cos(the3)*cos(the4)*sin(the2)) + cos(the1)*cos(the5)*sin(the2)*sin(the3)) - cos(the7)*(sin(the6)*(cos(the1)*cos(the2)*cos(the4) - cos(the1)*cos(the3)*sin(the2)*sin(the4)) + cos(the6)*(cos(the5)*(cos(the1)*cos(the2)*sin(the4) + cos(the1)*cos(the3)*cos(the4)*sin(the2)) - cos(the1)*sin(the2)*sin(the3)*sin(the5))), cos(the7)*(cos(the6)*(sin(the5)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) - cos(the4)*cos(the5)*(cos(the3)*sin(the1) + cos(the1)*cos(the2)*sin(the3))) + sin(the4)*sin(the6)*(cos(the3)*sin(the1) + cos(the1)*cos(the2)*sin(the3))) + sin(the7)*(cos(the5)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) + cos(the4)*sin(the5)*(cos(the3)*sin(the1) + cos(the1)*cos(the2)*sin(the3))), cos(the7)*(sin(the6)*(cos(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) + cos(the1)*sin(the2)*sin(the4)) + cos(the5)*cos(the6)*(sin(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) - cos(the1)*cos(the4)*sin(the2))) - sin(the5)*sin(the7)*(sin(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) - cos(the1)*cos(the4)*sin(the2)), sin(the7)*(cos(the5)*(cos(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) + cos(the1)*sin(the2)*sin(the4)) + sin(the5)*(cos(the3)*sin(the1) + cos(the1)*cos(the2)*sin(the3))) + cos(the6)*cos(the7)*(sin(the5)*(cos(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) + cos(the1)*sin(the2)*sin(the4)) - cos(the5)*(cos(the3)*sin(the1) + cos(the1)*cos(the2)*sin(the3))), cos(the7)*(cos(the6)*(sin(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) - cos(the1)*cos(the4)*sin(the2)) + sin(the6)*(cos(the5)*(cos(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) + cos(the1)*sin(the2)*sin(the4)) + sin(the5)*(cos(the3)*sin(the1) + cos(the1)*cos(the2)*sin(the3)))), cos(the7)*(sin(the5)*(cos(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) + cos(the1)*sin(the2)*sin(the4)) - cos(the5)*(cos(the3)*sin(the1) + cos(the1)*cos(the2)*sin(the3))) - sin(the7)*(sin(the6)*(sin(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) - cos(the1)*cos(the4)*sin(the2)) - cos(the6)*(cos(the5)*(cos(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) + cos(the1)*sin(the2)*sin(the4)) + sin(the5)*(cos(the3)*sin(the1) + cos(the1)*cos(the2)*sin(the3))))],
                        [cos(the7)*(sin(the5)*(cos(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) - sin(the1)*sin(the2)*sin(the4)) - cos(the5)*(cos(the1)*cos(the3) - cos(the2)*sin(the1)*sin(the3))) - sin(the7)*(sin(the6)*(sin(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) + cos(the4)*sin(the1)*sin(the2)) - cos(the6)*(cos(the5)*(cos(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) - sin(the1)*sin(the2)*sin(the4)) + sin(the5)*(cos(the1)*cos(the3) - cos(the2)*sin(the1)*sin(the3)))), cos(the7)*(sin(the5)*(cos(the1)*cos(the2)*sin(the4) + cos(the1)*cos(the3)*cos(the4)*sin(the2)) + cos(the1)*cos(the5)*sin(the2)*sin(the3)) + sin(the7)*(sin(the6)*(cos(the1)*cos(the2)*cos(the4) - cos(the1)*cos(the3)*sin(the2)*sin(the4)) + cos(the6)*(cos(the5)*(cos(the1)*cos(the2)*sin(the4) + cos(the1)*cos(the3)*cos(the4)*sin(the2)) - cos(the1)*sin(the2)*sin(the3)*sin(the5))), cos(the7)*(cos(the5)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) + cos(the4)*sin(the5)*(cos(the3)*sin(the1) + cos(the1)*cos(the2)*sin(the3))) - sin(the7)*(cos(the6)*(sin(the5)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) - cos(the4)*cos(the5)*(cos(the3)*sin(the1) + cos(the1)*cos(the2)*sin(the3))) + sin(the4)*sin(the6)*(cos(the3)*sin(the1) + cos(the1)*cos(the2)*sin(the3))), - sin(the7)*(sin(the6)*(cos(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) + cos(the1)*sin(the2)*sin(the4)) + cos(the5)*cos(the6)*(sin(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) - cos(the1)*cos(the4)*sin(the2))) - cos(the7)*sin(the5)*(sin(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) - cos(the1)*cos(the4)*sin(the2)), cos(the7)*(cos(the5)*(cos(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) + cos(the1)*sin(the2)*sin(the4)) + sin(the5)*(cos(the3)*sin(the1) + cos(the1)*cos(the2)*sin(the3))) - cos(the6)*sin(the7)*(sin(the5)*(cos(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) + cos(the1)*sin(the2)*sin(the4)) - cos(the5)*(cos(the3)*sin(the1) + cos(the1)*cos(the2)*sin(the3))), -sin(the7)*(cos(the6)*(sin(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) - cos(the1)*cos(the4)*sin(the2)) + sin(the6)*(cos(the5)*(cos(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) + cos(the1)*sin(the2)*sin(the4)) + sin(the5)*(cos(the3)*sin(the1) + cos(the1)*cos(the2)*sin(the3)))), - cos(the7)*(sin(the6)*(sin(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) - cos(the1)*cos(the4)*sin(the2)) - cos(the6)*(cos(the5)*(cos(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) + cos(the1)*sin(the2)*sin(the4)) + sin(the5)*(cos(the3)*sin(the1) + cos(the1)*cos(the2)*sin(the3)))) - sin(the7)*(sin(the5)*(cos(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) + cos(the1)*sin(the2)*sin(the4)) - cos(the5)*(cos(the3)*sin(the1) + cos(the1)*cos(the2)*sin(the3)))],
                        [- cos(the6)*(sin(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) + cos(the4)*sin(the1)*sin(the2)) - sin(the6)*(cos(the5)*(cos(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) - sin(the1)*sin(the2)*sin(the4)) + sin(the5)*(cos(the1)*cos(the3) - cos(the2)*sin(the1)*sin(the3))), cos(the6)*(cos(the1)*cos(the2)*cos(the4) - cos(the1)*cos(the3)*sin(the2)*sin(the4)) - sin(the6)*(cos(the5)*(cos(the1)*cos(the2)*sin(the4) + cos(the1)*cos(the3)*cos(the4)*sin(the2)) - cos(the1)*sin(the2)*sin(the3)*sin(the5)), sin(the6)*(sin(the5)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) - cos(the4)*cos(the5)*(cos(the3)*sin(the1) + cos(the1)*cos(the2)*sin(the3))) - cos(the6)*sin(the4)*(cos(the3)*sin(the1) + cos(the1)*cos(the2)*sin(the3)), cos(the5)*sin(the6)*(sin(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) - cos(the1)*cos(the4)*sin(the2)) - cos(the6)*(cos(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) + cos(the1)*sin(the2)*sin(the4)), sin(the6)*(sin(the5)*(cos(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) + cos(the1)*sin(the2)*sin(the4)) - cos(the5)*(cos(the3)*sin(the1) + cos(the1)*cos(the2)*sin(the3))), sin(the6)*(sin(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) - cos(the1)*cos(the4)*sin(the2)) - cos(the6)*(cos(the5)*(cos(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) + cos(the1)*sin(the2)*sin(the4)) + sin(the5)*(cos(the3)*sin(the1) + cos(the1)*cos(the2)*sin(the3))), 0],
                        [0, sin(the7)*(sin(the5)*(sin(the2)*sin(the4) - cos(the2)*cos(the3)*cos(the4)) - cos(the2)*cos(the5)*sin(the3)) - cos(the7)*(cos(the6)*(cos(the5)*(sin(the2)*sin(the4) - cos(the2)*cos(the3)*cos(the4)) + cos(the2)*sin(the3)*sin(the5)) + sin(the6)*(cos(the4)*sin(the2) + cos(the2)*cos(the3)*sin(the4))), - cos(the7)*(cos(the6)*(cos(the3)*sin(the2)*sin(the5) + cos(the4)*cos(the5)*sin(the2)*sin(the3)) - sin(the2)*sin(the3)*sin(the4)*sin(the6)) - sin(the7)*(cos(the3)*cos(the5)*sin(the2) - cos(the4)*sin(the2)*sin(the3)*sin(the5)), - cos(the7)*(sin(the6)*(cos(the2)*sin(the4) + cos(the3)*cos(the4)*sin(the2)) - cos(the5)*cos(the6)*(cos(the2)*cos(the4) - cos(the3)*sin(the2)*sin(the4))) - sin(the5)*sin(the7)*(cos(the2)*cos(the4) - cos(the3)*sin(the2)*sin(the4)), - sin(the7)*(cos(the5)*(cos(the2)*sin(the4) + cos(the3)*cos(the4)*sin(the2)) - sin(the2)*sin(the3)*sin(the5)) - cos(the6)*cos(the7)*(sin(the5)*(cos(the2)*sin(the4) + cos(the3)*cos(the4)*sin(the2)) + cos(the5)*sin(the2)*sin(the3)), -cos(the7)*(sin(the6)*(cos(the5)*(cos(the2)*sin(the4) + cos(the3)*cos(the4)*sin(the2)) - sin(the2)*sin(the3)*sin(the5)) - cos(the6)*(cos(the2)*cos(the4) - cos(the3)*sin(the2)*sin(the4))), - cos(the7)*(sin(the5)*(cos(the2)*sin(the4) + cos(the3)*cos(the4)*sin(the2)) + cos(the5)*sin(the2)*sin(the3)) - sin(the7)*(cos(the6)*(cos(the5)*(cos(the2)*sin(the4) + cos(the3)*cos(the4)*sin(the2)) - sin(the2)*sin(the3)*sin(the5)) + sin(the6)*(cos(the2)*cos(the4) - cos(the3)*sin(the2)*sin(the4)))],
                        [0, cos(the7)*(sin(the5)*(sin(the2)*sin(the4) - cos(the2)*cos(the3)*cos(the4)) - cos(the2)*cos(the5)*sin(the3)) + sin(the7)*(cos(the6)*(cos(the5)*(sin(the2)*sin(the4) - cos(the2)*cos(the3)*cos(the4)) + cos(the2)*sin(the3)*sin(the5)) + sin(the6)*(cos(the4)*sin(the2) + cos(the2)*cos(the3)*sin(the4))), sin(the7)*(cos(the6)*(cos(the3)*sin(the2)*sin(the5) + cos(the4)*cos(the5)*sin(the2)*sin(the3)) - sin(the2)*sin(the3)*sin(the4)*sin(the6)) - cos(the7)*(cos(the3)*cos(the5)*sin(the2) - cos(the4)*sin(the2)*sin(the3)*sin(the5)), sin(the7)*(sin(the6)*(cos(the2)*sin(the4) + cos(the3)*cos(the4)*sin(the2)) - cos(the5)*cos(the6)*(cos(the2)*cos(the4) - cos(the3)*sin(the2)*sin(the4))) - cos(the7)*sin(the5)*(cos(the2)*cos(the4) - cos(the3)*sin(the2)*sin(the4)), cos(the6)*sin(the7)*(sin(the5)*(cos(the2)*sin(the4) + cos(the3)*cos(the4)*sin(the2)) + cos(the5)*sin(the2)*sin(the3)) - cos(the7)*(cos(the5)*(cos(the2)*sin(the4) + cos(the3)*cos(the4)*sin(the2)) - sin(the2)*sin(the3)*sin(the5)), sin(the7)*(sin(the6)*(cos(the5)*(cos(the2)*sin(the4) + cos(the3)*cos(the4)*sin(the2)) - sin(the2)*sin(the3)*sin(the5)) - cos(the6)*(cos(the2)*cos(the4) - cos(the3)*sin(the2)*sin(the4))), sin(the7)*(sin(the5)*(cos(the2)*sin(the4) + cos(the3)*cos(the4)*sin(the2)) + cos(the5)*sin(the2)*sin(the3)) - cos(the7)*(cos(the6)*(cos(the5)*(cos(the2)*sin(the4) + cos(the3)*cos(the4)*sin(the2)) - sin(the2)*sin(the3)*sin(the5)) + sin(the6)*(cos(the2)*cos(the4) - cos(the3)*sin(the2)*sin(the4)))],
                        [0, cos(the6)*(cos(the4)*sin(the2) + cos(the2)*cos(the3)*sin(the4)) - sin(the6)*(cos(the5)*(sin(the2)*sin(the4) - cos(the2)*cos(the3)*cos(the4)) + cos(the2)*sin(the3)*sin(the5)), - sin(the6)*(cos(the3)*sin(the2)*sin(the5) + cos(the4)*cos(the5)*sin(the2)*sin(the3)) - cos(the6)*sin(the2)*sin(the3)*sin(the4), cos(the6)*(cos(the2)*sin(the4) + cos(the3)*cos(the4)*sin(the2)) + cos(the5)*sin(the6)*(cos(the2)*cos(the4) - cos(the3)*sin(the2)*sin(the4)), -sin(the6)*(sin(the5)*(cos(the2)*sin(the4) + cos(the3)*cos(the4)*sin(the2)) + cos(the5)*sin(the2)*sin(the3)), cos(the6)*(cos(the5)*(cos(the2)*sin(the4) + cos(the3)*cos(the4)*sin(the2)) - sin(the2)*sin(the3)*sin(the5)) + sin(the6)*(cos(the2)*cos(the4) - cos(the3)*sin(the2)*sin(the4)), 0],
                        [cos(the7)*(sin(the6)*(sin(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) - cos(the1)*cos(the4)*sin(the2)) - cos(the6)*(cos(the5)*(cos(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) + cos(the1)*sin(the2)*sin(the4)) + sin(the5)*(cos(the3)*sin(the1) + cos(the1)*cos(the2)*sin(the3)))) + sin(the7)*(sin(the5)*(cos(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) + cos(the1)*sin(the2)*sin(the4)) - cos(the5)*(cos(the3)*sin(the1) + cos(the1)*cos(the2)*sin(the3))), sin(the7)*(sin(the5)*(cos(the2)*sin(the1)*sin(the4) + cos(the3)*cos(the4)*sin(the1)*sin(the2)) + cos(the5)*sin(the1)*sin(the2)*sin(the3)) - cos(the7)*(cos(the6)*(cos(the5)*(cos(the2)*sin(the1)*sin(the4) + cos(the3)*cos(the4)*sin(the1)*sin(the2)) - sin(the1)*sin(the2)*sin(the3)*sin(the5)) + sin(the6)*(cos(the2)*cos(the4)*sin(the1) - cos(the3)*sin(the1)*sin(the2)*sin(the4))), - cos(the7)*(cos(the6)*(sin(the5)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) - cos(the4)*cos(the5)*(cos(the1)*cos(the3) - cos(the2)*sin(the1)*sin(the3))) + sin(the4)*sin(the6)*(cos(the1)*cos(the3) - cos(the2)*sin(the1)*sin(the3))) - sin(the7)*(cos(the5)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) + cos(the4)*sin(the5)*(cos(the1)*cos(the3) - cos(the2)*sin(the1)*sin(the3))), sin(the5)*sin(the7)*(sin(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) + cos(the4)*sin(the1)*sin(the2)) - cos(the7)*(sin(the6)*(cos(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) - sin(the1)*sin(the2)*sin(the4)) + cos(the5)*cos(the6)*(sin(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) + cos(the4)*sin(the1)*sin(the2))), - sin(the7)*(cos(the5)*(cos(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) - sin(the1)*sin(the2)*sin(the4)) + sin(the5)*(cos(the1)*cos(the3) - cos(the2)*sin(the1)*sin(the3))) - cos(the6)*cos(the7)*(sin(the5)*(cos(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) - sin(the1)*sin(the2)*sin(the4)) - cos(the5)*(cos(the1)*cos(the3) - cos(the2)*sin(the1)*sin(the3))), -cos(the7)*(cos(the6)*(sin(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) + cos(the4)*sin(the1)*sin(the2)) + sin(the6)*(cos(the5)*(cos(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) - sin(the1)*sin(the2)*sin(the4)) + sin(the5)*(cos(the1)*cos(the3) - cos(the2)*sin(the1)*sin(the3)))), sin(the7)*(sin(the6)*(sin(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) + cos(the4)*sin(the1)*sin(the2)) - cos(the6)*(cos(the5)*(cos(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) - sin(the1)*sin(the2)*sin(the4)) + sin(the5)*(cos(the1)*cos(the3) - cos(the2)*sin(the1)*sin(the3)))) - cos(the7)*(sin(the5)*(cos(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) - sin(the1)*sin(the2)*sin(the4)) - cos(the5)*(cos(the1)*cos(the3) - cos(the2)*sin(the1)*sin(the3)))],
                        [cos(the7)*(sin(the5)*(cos(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) + cos(the1)*sin(the2)*sin(the4)) - cos(the5)*(cos(the3)*sin(the1) + cos(the1)*cos(the2)*sin(the3))) - sin(the7)*(sin(the6)*(sin(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) - cos(the1)*cos(the4)*sin(the2)) - cos(the6)*(cos(the5)*(cos(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) + cos(the1)*sin(the2)*sin(the4)) + sin(the5)*(cos(the3)*sin(the1) + cos(the1)*cos(the2)*sin(the3)))), cos(the7)*(sin(the5)*(cos(the2)*sin(the1)*sin(the4) + cos(the3)*cos(the4)*sin(the1)*sin(the2)) + cos(the5)*sin(the1)*sin(the2)*sin(the3)) + sin(the7)*(cos(the6)*(cos(the5)*(cos(the2)*sin(the1)*sin(the4) + cos(the3)*cos(the4)*sin(the1)*sin(the2)) - sin(the1)*sin(the2)*sin(the3)*sin(the5)) + sin(the6)*(cos(the2)*cos(the4)*sin(the1) - cos(the3)*sin(the1)*sin(the2)*sin(the4))), sin(the7)*(cos(the6)*(sin(the5)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) - cos(the4)*cos(the5)*(cos(the1)*cos(the3) - cos(the2)*sin(the1)*sin(the3))) + sin(the4)*sin(the6)*(cos(the1)*cos(the3) - cos(the2)*sin(the1)*sin(the3))) - cos(the7)*(cos(the5)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) + cos(the4)*sin(the5)*(cos(the1)*cos(the3) - cos(the2)*sin(the1)*sin(the3))), sin(the7)*(sin(the6)*(cos(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) - sin(the1)*sin(the2)*sin(the4)) + cos(the5)*cos(the6)*(sin(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) + cos(the4)*sin(the1)*sin(the2))) + cos(the7)*sin(the5)*(sin(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) + cos(the4)*sin(the1)*sin(the2)), cos(the6)*sin(the7)*(sin(the5)*(cos(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) - sin(the1)*sin(the2)*sin(the4)) - cos(the5)*(cos(the1)*cos(the3) - cos(the2)*sin(the1)*sin(the3))) - cos(the7)*(cos(the5)*(cos(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) - sin(the1)*sin(the2)*sin(the4)) + sin(the5)*(cos(the1)*cos(the3) - cos(the2)*sin(the1)*sin(the3))), sin(the7)*(cos(the6)*(sin(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) + cos(the4)*sin(the1)*sin(the2)) + sin(the6)*(cos(the5)*(cos(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) - sin(the1)*sin(the2)*sin(the4)) + sin(the5)*(cos(the1)*cos(the3) - cos(the2)*sin(the1)*sin(the3)))), cos(the7)*(sin(the6)*(sin(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) + cos(the4)*sin(the1)*sin(the2)) - cos(the6)*(cos(the5)*(cos(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) - sin(the1)*sin(the2)*sin(the4)) + sin(the5)*(cos(the1)*cos(the3) - cos(the2)*sin(the1)*sin(the3)))) + sin(the7)*(sin(the5)*(cos(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) - sin(the1)*sin(the2)*sin(the4)) - cos(the5)*(cos(the1)*cos(the3) - cos(the2)*sin(the1)*sin(the3)))],
                        [- cos(the6)*(sin(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) - cos(the1)*cos(the4)*sin(the2)) - sin(the6)*(cos(the5)*(cos(the4)*(sin(the1)*sin(the3) - cos(the1)*cos(the2)*cos(the3)) + cos(the1)*sin(the2)*sin(the4)) + sin(the5)*(cos(the3)*sin(the1) + cos(the1)*cos(the2)*sin(the3))), cos(the6)*(cos(the2)*cos(the4)*sin(the1) - cos(the3)*sin(the1)*sin(the2)*sin(the4)) - sin(the6)*(cos(the5)*(cos(the2)*sin(the1)*sin(the4) + cos(the3)*cos(the4)*sin(the1)*sin(the2)) - sin(the1)*sin(the2)*sin(the3)*sin(the5)), cos(the6)*sin(the4)*(cos(the1)*cos(the3) - cos(the2)*sin(the1)*sin(the3)) - sin(the6)*(sin(the5)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) - cos(the4)*cos(the5)*(cos(the1)*cos(the3) - cos(the2)*sin(the1)*sin(the3))), cos(the6)*(cos(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) - sin(the1)*sin(the2)*sin(the4)) - cos(the5)*sin(the6)*(sin(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) + cos(the4)*sin(the1)*sin(the2)), -sin(the6)*(sin(the5)*(cos(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) - sin(the1)*sin(the2)*sin(the4)) - cos(the5)*(cos(the1)*cos(the3) - cos(the2)*sin(the1)*sin(the3))), cos(the6)*(cos(the5)*(cos(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) - sin(the1)*sin(the2)*sin(the4)) + sin(the5)*(cos(the1)*cos(the3) - cos(the2)*sin(the1)*sin(the3))) - sin(the6)*(sin(the4)*(cos(the1)*sin(the3) + cos(the2)*cos(the3)*sin(the1)) + cos(the4)*sin(the1)*sin(the2)), 0]])
        J_inv = np.linalg.pinv(Jee)
        # I3 = np.eye(3)         
        # J1 = np.matmul(Jee,Jee.T) + 0.001*I3
        # J1_inv = np.linalg.inv(J1)
        # J_inv = np.matmul(Jee.T,J1_inv)
        return J_inv
     
    def theta_check(the):
        the1 = the[0]; the2 = the[1]; the3 = the[2]; the4 = the[3]
        the5 = the[4]; the6 = the[5]; the7 = the[6] 
        if(abs(the1)>pi):
            the1 = the1 % pi
        else:
            the1 = the1    
        if(abs(the2)>pi):
            the2 = the2 % pi
        else:
            the2 = the2
        if(abs(the3)>pi):
            the3 = the3 % pi
        else:
            the3 = the3
        if(abs(the4)>pi):
            the4 = the4 % pi
        else:
            the4 = the4
        if(abs(the5)>pi):
            the5 = the5 % pi
        else:
            the5 = the5
        if(abs(the6)>pi):
            the6 = the6 % pi
        else:
            the6 = the6
        if(abs(the7)>pi):
            the7 = the7 % pi
        else:
            the7 = the7
        the = np.array([the1,the2,the3,the4,the5,the6,the7])
        print(the)
        return the
    
    def CalcIK(IK_in, error):
        # EXTRACT DATA
        l0 = 0.03; l1 = 0.094; l2 = 0.094; l3 = 0.094
        l4 = 0.094; l5 = 0.094; l6 = 0.094; l7 = 0.03
        L = np.array([[l0],[l1],[l2],[l3],[l4],[l5],[l6],[l7]])
        the = np.array([[0],[0],[0],[0],[0],[pi/2],[0]])
        xd = float(IK_in[0]); yd = float(IK_in[1]); zd = float(IK_in[2])
        alp = IK_in[3]*pi/180; bet = IK_in[4]*pi/180; gam = IK_in[5]*pi/180

        # R0EED
        Ralp = np.array([[cos(alp), -sin(alp),  0],
                         [sin(alp),  cos(alp),  0],
                         [       0,         0,  1]]);
        Rbet = np.array([[ cos(bet),   0,   sin(bet)],
                         [        0,   1,          0],
                         [-sin(bet),   0,   cos(bet)]])
        Rgam = np.array([[        1,        0,           0],
                         [        0, cos(gam),   -sin(gam)],
                         [        0, sin(gam),    cos(gam)]])
        R0EED = Ralp@Rbet@Rgam

        # T0EED
        T0EED = np.array([[R0EED[0,0], R0EED[0,1], R0EED[0,2], xd],
                          [R0EED[1,0], R0EED[1,1], R0EED[1,2], yd],
                          [R0EED[2,0], R0EED[2,1], R0EED[2,2], zd],
                          [         0,          0,          0,  1]])

        Teed = np.array([T0EED[0,3], T0EED[1,3], T0EED[2,3],
                         T0EED[0,0], T0EED[0,1], T0EED[0,2],
                         T0EED[1,0], T0EED[1,1], T0EED[1,2],
                         T0EED[2,0], T0EED[2,1], T0EED[2,2]])
        
        # LOOP PARA
        check = 0
        epoch = 0
        gamma = 0.001
        t_store = 0
        
        # NAG PARA
        v0 = 0.9
        vt = np.array([[v0], [v0], [v0], [v0], [v0], [v0], [v0]])

        while(check == 0):
            t_start = time.time()
            epoch += 1
            
            # FK
            [T0EE, Pee, Tee, Q1, Q2, Q3] = UI_kinematics.Fcn_FK(the,2)
            
            # UPDATE E
            E = Teed.reshape(-1,1) - Tee.reshape(-1,1)
            
            # CALC DTHE BY PSUDO-INV
            # NAG UPDATE THE
            # the = the - gamma*vt
            
            # JACOBI PSUDO-INV
            J_inv = UI_kinematics.Fcn_J_Inv(the, L)
            
            # CALC DTHE
            dthe = J_inv@E
            
            # UPDATE VT
            vt = gamma*vt - dthe
            
            # UPDATE THE
            the = the - vt
            
            # Ori
            [alp1, bet1, gam1] = UI_kinematics.Orientation(T0EE)
            
            # CALC ERROR
            P_err = sum(sum(abs(0.5*np.square(Teed.reshape(-1,1)-Tee.reshape(-1,1)))))
            O_err = sum(abs(np.array([alp1, bet1, gam1])-np.array([alp,bet,gam])));
            
            # print("epoch: %d"%(epoch))
            # print("time: %.4f"%(t_store))
            # print(" ")
            
            if (abs(P_err) <= float(error) and abs(O_err) <= float(error) or epoch > 20):
                check = 1; epoch = 0;
            
            t_end = time.time()
            t_store = t_store + (t_end - t_start)
            
        # the = UI_kinematics.theta_check(the)
        return the
            
    def Plot_robot(self):
        # QApplication.processEvents()
        self.ui.out1 = UI_kinematics.Fcn_FK(self.ui.the_plot, 1)
        self.ui.Q1 = self.ui.out1[0]
        self.ui.Q2 = self.ui.out1[1]
        self.ui.Q3 = self.ui.out1[2]
        self.plot3d.clear_fig3d()
        self.plot3d.plot3r(self.ui.Q1, self.ui.Q2, self.ui.Q3)
        if(len(self.ui.x_obj)>0):
            self.plot3d.plot3o(self.ui.x_obj, self.ui.y_obj, self.ui.z_obj, 'sg')
        if(len(self.ui.x_obs)>0):
            self.plot3d.plot3o(self.ui.x_obs, self.ui.y_obs, self.ui.z_obj, 'sr')
        self.plot3d.draw_plot()
        self.plot2xy.clear_fig2xy()
        self.plot2xy.plot2xy(self.ui.Q1, self.ui.Q2, '-bo')
        self.plot2xy.plot2xy(self.ui.Q1, self.ui.Q2, 'ko')
        if(len(self.ui.x_obj)>0):
            self.plot2xy.plot2oxy(self.ui.x_obj, self.ui.y_obj, 'sg')
        if(len(self.ui.x_obs)>0):
            self.plot2xy.plot2oxy(self.ui.x_obs, self.ui.y_obs, 'sr')
        self.plot2xy.draw_plot()
        self.plot2xz.clear_fig2xz()
        self.plot2xz.plot2xz(self.ui.Q1, self.ui.Q3, '-bo')
        self.plot2xz.plot2xz(self.ui.Q1, self.ui.Q3, 'ko') 
        if(len(self.ui.y_obj)>0):
            self.plot2xz.plot2oxz(self.ui.x_obj, self.ui.z_obj, 'sg')
        if(len(self.ui.y_obs)>0):
            self.plot2xz.plot2oxz(self.ui.x_obs, self.ui.z_obs, 'sr')
        self.plot2xz.draw_plot()
        self.plot2yz.clear_fig2yz()
        self.plot2yz.plot2yz(self.ui.Q2, self.ui.Q3, '-bo')
        self.plot2yz.plot2yz(self.ui.Q2, self.ui.Q3, 'ko')
        if(len(self.ui.x_obj)>0):
            self.plot2yz.plot2oyz(self.ui.y_obj, self.ui.z_obj, 'sg')
        if(len(self.ui.x_obs)>0):
            self.plot2yz.plot2oyz(self.ui.y_obs, self.ui.z_obs, 'sr')
        self.plot2yz.draw_plot()
        
class UI_control(QObject):
    def __init__(self):
        super().__init__()
        self.ser = serial.Serial()
        
    def sim_FK_BC(self, time):
        the1 = self.ui.tb_the1_BC.text()
        the2 = self.ui.tb_the2_BC.text()
        the3 = self.ui.tb_the3_BC.text()
        the4 = self.ui.tb_the4_BC.text()
        the5 = self.ui.tb_the5_BC.text()
        the6 = self.ui.tb_the6_BC.text()
        the7 = self.ui.tb_the7_BC.text()
        timed = time
        vtd = np.array([the1, the2, the3, the4, the5, the6, the7]).astype(np.float64())*np.pi/180
        vtt = self.ui.the_plot
        self.ui.a, self.ui.b, self.ui.c, self.ui.d = UI_kinematics.calc_abcd(vtt,vtd,time)
        self.ui.timed = timed
        self.ui.timec = 0.1
        self.ui.timer2.start(10)
        
    def sim_IK_BC(self, the_sent, time):
        the1 = np.round(float(the_sent[0]),1)
        the2 = np.round(float(the_sent[1]),1)
        the3 = np.round(float(the_sent[2]),1)
        the4 = np.round(float(the_sent[3]),1)
        the5 = np.round(float(the_sent[4]),1)
        the6 = np.round(float(the_sent[5]),1)
        the7 = np.round(float(the_sent[6]),1)
        timed = time
        vtd = np.array([the1, the2, the3, the4, the5, the6, the7]).astype(np.float64())*np.pi/180
        vtt = self.ui.the_plot
        self.ui.a, self.ui.b, self.ui.c, self.ui.d = UI_kinematics.calc_abcd(vtt,vtd,time)
        self.ui.timed = timed
        self.ui.timec = 0.1
        self.ui.timer2.start(10)
        
    def Plot_error(self):
        self.plot2Ex.clear_fig2Ex()
        self.plot2Ex.plot2Ex(self.ui.Ex)
        self.plot2Ex.draw_plot()     
        
        self.plot2Ey.clear_fig2Ey()
        self.plot2Ey.plot2Ey(self.ui.Ey)
        self.plot2Ey.draw_plot() 
        
        self.plot2Ez.clear_fig2Ez()
        self.plot2Ez.plot2Ez(self.ui.Ez)
        self.plot2Ez.draw_plot() 
        
class UI_camera(QObject):
    def __init__(self):
        super().__init__()
        
    def Intrinsic_matrix(Ox,Oy,Fx,Fy):
        Mi = np.array([(Fx,      0,     Ox),
                       ( 0,     Fy,     Oy),
                       ( 0,      0,     1)])
        return Mi
    
    def Tranlation_matrix(R,T):
            a = R[0]
            g = R[1]
            b = R[2]
            Rx = np.matrix([[1             , 0             , 0             ],
                            [0             , np.cos(a)     , -np.sin(a)    ],
                            [0             , np.sin(a)     , np.cos(a)     ]])
            Ry = np.matrix([[np.cos(g)     , 0             , np.sin(g)     ] ,
                            [0             , 1             , 0             ] ,
                            [-np.sin(g)    , 0             , np.cos(g)     ]])
            Rz = np.matrix([[np.cos(b)     , -np.sin(b)    , 0             ] ,
                            [np.sin(b)     , np.cos(b)     , 0             ] ,
                            [0              , 0             , 1             ]])
            R0EE = Rx@Ry@Rz
            T0EE = np.matrix([[1       ,1      ,1      ,T[0]],
                              [1       ,1      ,1      ,T[1]],
                              [1       ,1      ,1      ,T[2]],
                              [0       ,0      ,0      ,1]])
            for i in range(3):
                for j in range(3):
                    T0EE[i,j] = R0EE[i,j]   
            return T0EE
        
    def distorsion_inf(Rd,Td):
        dist = np.array([Rd[0], Rd[1],  Td[0],  Td[1],  Rd[2]])
        return dist
    
    def get_uv_from_img(mask, frame):
        u = 0
        v = 0
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)     
            #Lọc mảng màu theo kich 
            if area > 300:
                cv2.drawContours(frame, [approx], 0, (0, 0, 0), 2)
                M = cv2.moments(cnt)
                u = int(M["m10"]/ M["m00"])
                v = int(M["m01"]/ M["m00"])
                #cv2.circle(frame,(u, v), 5, (0,0,0))
                cv2.circle(frame, (u,v), radius=2, color=(0, 0, 255), thickness=-1)
        return u,v
    
    def get_uv_from_position(position,Mi,Me):
        Mie = Mi@Me
        uv = Mie@position
        u = uv[0,0]/uv[2,0]
        v = uv[1,0]/uv[2,0]
        return u,v
    
    def get_position_from_uv(u1,v1,u2,v2,Mi1,Me1,Mi2,Me2):
        T_0_C1 = np.linalg.inv(Me1) 
        #T_0_C2 = np.linalg.inv(Me2)
        T_C2_C1 = np.dot(Me2,T_0_C1)
        
        m = np.array([(Mi1[0,0],  Mi1[0,1],    Mi1[0,2],    0),
                      (Mi1[1,0],  Mi1[1,1],    Mi1[1,2],    0),
                      (Mi1[2,0],  Mi1[2,1],    Mi1[2,2],    0)])
        
        Mii2 = np.array([(Mi2[0,0],  Mi2[0,1],    Mi2[0,2],    0),
                         (Mi2[1,0],  Mi2[1,1],    Mi2[1,2],    0),
                         (Mi2[2,0],  Mi2[2,1],    Mi2[2,2],    0)])
        p = Mii2@T_C2_C1

        A = np.array([(u1*m[2,0]-m[0,0],   u1*m[2,1]-m[0,1],    u1*m[2,2]-m[0,2]),
                      (v1*m[2,0]-m[1,0],   v1*m[2,1]-m[1,1],    v1*m[2,2]-m[1,2]),
                      (u2*p[2,0]-p[0,0],   u2*p[2,1]-p[0,1],    u2*p[2,2]-p[0,2]),
                      (v2*p[2,0]-p[1,0],   v2*p[2,1]-p[1,1],    v2*p[2,2]-p[1,2])])
        # ma tran cot
        B = np.array([[(m[0,3] - m[2,3])],
                      [(m[1,3] - m[2,3])],
                      [(p[0,3] - p[2,3])],
                      [(p[1,3] - p[2,3])]])
        #X = ((A'*A)^-1)*A'*B
        X = (np.linalg.inv(A.T@A))@(A.T)@B
        '''
        AA = np.dot((A.T),A)
        A_1 = np.linalg.inv(AA)
        XX = np.dot(A_1,A.T)
        X = np.dot(XX,B)
        '''
        position = T_0_C1@np.append(X,[1])
        return position
    
    def set_cam(self):
        if not self.ui.timer5.isActive():
            try:
                self.ui.cam1 = cv2.VideoCapture(0)
                self.ui.cam1.set(3, 1280)
                self.ui.cam1.set(4, 720)
                self.ui.cam2 = cv2.VideoCapture(1)
                self.ui.cam2.set(3, 1280)
                self.ui.cam2.set(4, 720)
                self.ui.Mi1 = UI_camera.Intrinsic_matrix(self.ui.Ox1, self.ui.Oy1, self.ui.Fx1, self.ui.Fy1)
                self.ui.Me1 = UI_camera.Tranlation_matrix(self.ui.R_vec1, self.ui.T_vec1)
                self.ui.dist1 = UI_camera.distorsion_inf( self.ui.RadialDistorsion1, self.ui.TangentialDistorsion1)
                self.ui.Mi2 = UI_camera.Intrinsic_matrix(self.ui.Ox2, self.ui.Oy2, self.ui.Fx2, self.ui.Fy2)
                self.ui.Me2 = UI_camera.Tranlation_matrix(self.ui.R_vec2, self.ui.T_vec2)
                self.ui.dist2 = UI_camera.distorsion_inf( self.ui.RadialDistorsion2, self.ui.TangentialDistorsion2)
                self.ui.timer5.start(100)
                self.ui.btn_start_cam.setText("Stop")
                
            except:
                QMessageBox.information(None, "Error", "Camera not connected",QMessageBox.Cancel)
        else:
            self.ui.timer2.stop()
            self.ui.cam1.release()
            self.ui.cam2.release()
            cv2.destroyAllWindows()
            self.ui.btn_start_cam.setText("Start")
            
    def start_cam(self):
        #Tách ảnh từ video
        ret1, frame1 = self.ui.cam1.read()
        ret2, frame2 = self.ui.cam2.read()
        
        #Trải phẳng ảnh - triệt tiêu độ méo ảnh
        frame1 = cv2.undistort(frame1, self.ui.Mi1, self.ui.dist1, None, self.ui.Mi1)
        frame2 = cv2.undistort(frame2, self.ui.Mi2, self.ui.dist2, None, self.ui.Mi2)
        
        #
        obj1 = []
        obj2 = []
        obs1 = []
        obs2 = []
        
        #
        kernel = np.ones((5, 5), np.uint8)
        
        # 
        if(ret1):
            blur1 = cv2.GaussianBlur(frame1, (11, 11), 0)
            
            #Chuyển đổi BGR sang HSV
            hsv1 = cv2.cvtColor(blur1, cv2.COLOR_BGR2HSV)
            
            #Ngưỡng xác định xanh dương
            lower_green = np.array([35, 81, 0])
            upper_green = np.array([123, 255, 255])
            
            #Lọc nhiễu ảnh
            mask1_obj = cv2.inRange(hsv1, lower_green, upper_green)
            
            #Erode
            mask1_obj = cv2.erode(mask1_obj, kernel)
            
            #Xác định tâm vật ở 2 camera
            u1_obj,v1_obj = UI_camera.get_uv_from_img(mask1_obj, frame1)
            
            obj1.append([u1_obj,v1_obj])
            
            #Ngưỡng xác định xanh lá
            lower_red = np.array([166, 129, 136])
            upper_red = np.array([255, 255, 255])
            
            #Lọc nhiễu ảnh
            mask1_obs = cv2.inRange(hsv1, lower_red, upper_red)
            
            #Erode
            mask1_obs = cv2.erode(mask1_obs, kernel)
            
            #Xác định tâm vật ở 2 camera
            u1_obs,v1_obs = UI_camera.get_uv_from_img(mask1_obs, frame1)
            
            obs1.append([u1_obs,v1_obs])
            
        # 
        if(ret2):
            #
            blur2 = cv2.GaussianBlur(frame2, (11, 11), 0)
            
            #Chuyển đổi BGR sang HSV
            hsv2 = cv2.cvtColor(blur2, cv2.COLOR_BGR2HSV)
            
            #Ngưỡng xác định xanh dương
            lower_green = np.array([35, 81, 0])
            upper_green = np.array([123, 255, 255])
            
            #Lọc nhiễu ảnh
            mask2_obj = cv2.inRange(hsv2, lower_green, upper_green)
            
            #Erode
            mask2_obj = cv2.erode(mask2_obj, kernel)
            
            #Xác định tâm vật ở 2 camera
            u2_obj,v2_obj = UI_camera.get_uv_from_img(mask2_obj, frame2)
            
            obj2.append([u2_obj,v2_obj])
            
            #Ngưỡng xác định xanh lá
            lower_red = np.array([166, 129, 136])
            upper_red = np.array([255, 255, 255])
            
            #Lọc nhiễu ảnh
            mask2_obs = cv2.inRange(hsv2, lower_red, upper_red)
            
            #Erode
            mask2_obs = cv2.erode(mask2_obs, kernel)
            
            #Xác định tâm vật ở 2 camera
            u2_obs,v2_obs = UI_camera.get_uv_from_img(mask2_obs, frame2)
            
            obs2.append([u2_obs,v2_obs])
        
            position = UI_camera.get_position_from_uv(u1_obj, v1_obj, u2_obj, v2_obj, self.ui.Mi1, self.ui.Me1, self.ui.Mi2, self.ui.Me2)
            position2 = UI_camera.get_position_from_uv(u1_obs, v1_obs, u2_obs, v2_obs, self.ui.Mi1, self.ui.Me1, self.ui.Mi2, self.ui.Me2)
            
            self.ui.x_obj = []
            self.ui.y_obj = []
            self.ui.z_obj = []
            
            self.ui.x_obs = []
            self.ui.y_obs = []
            self.ui.z_obs = []
            
            x = (position[0,0]/1000) + 0.62
            y = position[0,1]/1000
            z = (position[0,2]/1000)
            
            x2 = (position2[0,0]/1000) + 0.62
            y2 = position2[0,1]/1000
            z2 = (position2[0,2]/1000)
            
            self.ui.x_obj.append(x); print(self.ui.x_obj)
            self.ui.y_obj.append(y); print(self.ui.y_obj)
            self.ui.z_obj.append(z); print(self.ui.z_obj)
            
            # self.ui.x_obs.append(x2); print(self.ui.x_obs)
            # self.ui.y_obs.append(y2); print(self.ui.y_obs)
            # self.ui.z_obs.append(z2); print(self.ui.z_obs)
            
            # #
            # if(len(obj1) == len(obj2)) and (len(obj1) == 0):
            #     self.ui.x_obj = []
            #     self.ui.y_obj = []
            #     self.ui.z_obj = []
            #     # khi can them dong thi them if vao
            #     self.ui.tab_obj.removeRow(1)
            # elif(len(obj1) == len(obj2)) and (len(obj1) != 0):
            #     x_obj = self.ui.x_obj
            #     y_obj = self.ui.y_obj
            #     z_obj = self.ui.z_obj
            #     self.ui.x_obj = []
            #     self.ui.y_obj = []
            #     self.ui.z_obj = []
            #     self.ui.tab_obj.setRowCount(len(obj1))
            #     position = UI_camera.get_position_from_uv(u1_obj, v1_obj, u2_obj, v2_obj, self.ui.Mi1, self.ui.Me1, self.ui.Mi2, self.ui.Me2)   
            #     x = position[0,0]/1000
            #     y = position[0,1]/1000
            #     z = position[0,2]/1000
            #     # for k in range(len(x_obj)):
            #     #     if (abs(x-x_obj[k]) < 0.002) and (abs(y-y_obj[k]) < 0.002) and (abs(z-z_obj[k]) < 0.002):
            #     #         x = x_obj[k]
            #     #         y = y_obj[k]
            #     #         z = z_obj[k]
            #     #         break
            #     # self.ui.x_obj.append(x); print(self.ui.x_obj)
            #     # self.ui.y_obj.append(y); print(self.ui.y_obj)
            #     # self.ui.z_obj.append(z); print(self.ui.z_obj)
            #     # print('\n')
            #     # Hien thi len tab\
            
            self.ui.tab_obj.setItem(1, 0, QtWidgets.QTableWidgetItem(str(np.round(x,4))))
            self.ui.tab_obj.setItem(1, 1, QtWidgets.QTableWidgetItem(str(np.round(y,4))))
            self.ui.tab_obj.setItem(1, 2, QtWidgets.QTableWidgetItem(str(np.round(z,4))))
            
            # self.ui.tab_obs.setItem(1, 0, QtWidgets.QTableWidgetItem(str(np.round(x2,4))))
            # self.ui.tab_obs.setItem(1, 1, QtWidgets.QTableWidgetItem(str(np.round(y2,4))))
            # self.ui.tab_obs.setItem(1, 2, QtWidgets.QTableWidgetItem(str(np.round(z2,4))))
                
            # #
            # if(len(obs1) == len(obs2)) and (len(obs1) == 0):
            #     self.ui.x_obs = []
            #     self.ui.y_obs = []
            #     self.ui.z_obs = []
            #     # khi can them dong thi them if vao
            #     self.ui.tab_obs.removeRow(1)
            # elif(len(obs1) == len(obs2)) and (len(obs1) != 0):
            #     x_obs = self.ui.x_obs
            #     y_obs = self.ui.y_obs
            #     z_obs = self.ui.z_obs
            #     self.ui.x_obs = []
            #     self.ui.y_obs = []
            #     self.ui.z_obs = []
            #     self.ui.tab_obs.setRowCount(len(obs1))
            #     for i in range(len(obs1)):
            #         x,y,z = UI_camera.get_position_from_uv(obs1[i][1], obs1[i][2], obs2[i][1], obs2[i][2], self.ui.Mi1, self.ui.Me1, self.ui.Mi2, self.ui.Me2)   
            #         for k in range(len(x_obs)):
            #             if (abs(x-x_obs[k]) < 0.002) and (abs(y-y_obs[k]) < 0.002) and (abs(z-z_obs[k]) < 0.002):
            #                 x = x_obs[k]
            #                 y = y_obs[k]
            #                 z = z_obs[k]
            #                 break
            #         self.ui.x_obs.append(x); print(self.ui.x_obs)
            #         self.ui.y_obs.append(y); print(self.ui.y_obs)
            #         self.ui.z_obs.append(z); print(self.ui.z_obs)
            #         # Hien thi len tab
            #         self.ui.tab_obs.setItem(i, 0, QtWidgets.QTableWidgetItem(str(np.round(x,4))))
            #         self.ui.tab_obs.setItem(i, 1, QtWidgets.QTableWidgetItem(str(np.round(y,4))))
            #         self.ui.tab_obs.setItem(i, 2, QtWidgets.QTableWidgetItem(str(np.round(z,4))))
                
        frame1 = cv2.resize(frame1,(320,180),interpolation = cv2.INTER_AREA)
        frame2 = cv2.resize(frame2,(320,180),interpolation = cv2.INTER_AREA)
        frame1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2RGB)
        frame2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2RGB)
        qframe1 = QImage(frame1.data, 320,180, QImage.Format_RGB888)
        qframe2 = QImage(frame2.data, 320,180, QImage.Format_RGB888)
        self.ui.view_cam1.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
        self.ui.view_cam2.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
        self.ui.view_cam1.setPixmap(QPixmap.fromImage(qframe1))
        self.ui.view_cam2.setPixmap(QPixmap.fromImage(qframe2))