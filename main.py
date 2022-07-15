# -*- coding: utf-8 -*-
"""
Created on Wed Nov 24 10:33:08 2021

@author: Thanh_Hai
"""

import sys
import numpy as np
from math import pi
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigQT

from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal, QRunnable, pyqtSlot, QThreadPool
from PyQt5.QtWidgets import QApplication, QMainWindow, QDesktopWidget, QMessageBox, QVBoxLayout
from PyQt5.QtGui import QIcon, QPixmap

from Robot_GUI import Ui_MainWindow
from GUI_Functions import UI_init as init
from GUI_Functions import UI_serial as ser
from GUI_Functions import UI_kinematics as kine
from GUI_Functions import UI_control as ctrl
from GUI_Functions import UI_camera as cam

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.main_win = QMainWindow()
        self.ui = Ui_MainWindow()
        self.serial = ser()
        self.ui.setupUi(self.main_win)
        
        self.thread = {}
        self.threadpool_stream = QThreadPool()
        
        # Hide Window Control Box
        self.main_win.setWindowFlags(Qt.FramelessWindowHint)
        self.center()
        
        # Timers
        self.ui.timer1 = QTimer()
        self.ui.timer1.timeout.connect(lambda: kine.Plot_robot(self))
        # self.ui.timer1.timeout.connect(lambda: ctrl.Plot_error(self))
        self.ui.timer2 = QTimer()
        self.ui.timer2.timeout.connect(lambda: kine.sim_traject(self))
        self.ui.timer3 = QTimer()
        self.ui.timer3.timeout.connect(lambda: ctrl.Plot_error(self))
        self.ui.timer5 = QTimer()
        self.ui.timer5.timeout.connect(lambda: cam.start_cam(self))
        
        # Startup
        self.show_home()
        init.Window_Init(self)
        
        # Plot
        self.plot3d = show_plot(mode = 1)
        QVBoxLayout(self.ui.fr_plot3d).addWidget(self.plot3d)
        self.plot2xy = show_plot(mode = 2)
        QVBoxLayout(self.ui.fr_plot2_xy).addWidget(self.plot2xy)
        self.plot2xz = show_plot(mode = 3)
        QVBoxLayout(self.ui.fr_plot2_xz).addWidget(self.plot2xz)
        self.plot2yz = show_plot(mode = 4)
        QVBoxLayout(self.ui.fr_plot2_yz).addWidget(self.plot2yz)
        self.plot2Ex = show_plot(mode = 5)
        QVBoxLayout(self.ui.fr_plot2_Ex).addWidget(self.plot2Ex)
        self.plot2Ey = show_plot(mode = 6)
        QVBoxLayout(self.ui.fr_plot2_Ey).addWidget(self.plot2Ey)
        self.plot2Ez = show_plot(mode = 7)
        QVBoxLayout(self.ui.fr_plot2_Ez).addWidget(self.plot2Ez)
        
        # Change Menu by Buttons
        self.ui.btn_home.clicked.connect(self.show_home)
        self.ui.btn_Connect.clicked.connect(self.show_connect)
        self.ui.btn_Kine.clicked.connect(self.show_kinematic)
        self.ui.btn_Ctrl.clicked.connect(self.show_control)
        self.ui.btn_Calib.clicked.connect(self.show_calib)
        self.ui.btn_Setting.clicked.connect(self.show_setting)
        
        # Forms
        # Form Connect
        ## Check Mode
        self.ui.btn_sim.clicked.connect(self.check_mode)
        ## Serial
        self.ui.cb_baudrate.setCurrentText('115200')
        self.update_ports()
        ## Events
        self.ui.btn_cnt.clicked.connect(self.connect_serial)
        self.ui.btn_upd.clicked.connect(self.update_ports)
        self.ui.btn_send.clicked.connect(self.send_data)
        self.ui.btn_clear.clicked.connect(self.clear_terminal)
        self.serial.data_available.connect(self.update_terminal)
        self.serial.theta_sent.connect(self.update_ee)
        
        # Form Kinematic
        ## FK
        self.ui.btn_FK.clicked.connect(self.FK)
        ### Calc FK
        self.ui.btn_calc_FK.clicked.connect(self.calc_FK)
        self.ui.btn_copy_FK.clicked.connect(self.copy_FK)
        ### Run Simulation FK
        self.ui.btn_run_FK.clicked.connect(lambda: kine.sim_FK(self, float(self.ui.tb_time_FK.text())))
        ## IK
        self.ui.btn_IK.clicked.connect(self.IK)
        ### Calc IK
        self.ui.btn_calc_IK.clicked.connect(self.calc_IK)
        ### Run Simulation IK
        self.ui.btn_run_IK.clicked.connect(lambda: kine.sim_IK(self, float(self.ui.tb_time_IK.text())))
        
        # Form Control
        self.ui.btn_plot_page.clicked.connect(self.Change_plot)
        ## BC
        self.ui.btn_BCtrl.clicked.connect(self.Basic_Ctrl)
        ### Change Input Event
        self.ui.btn_inp_BC.clicked.connect(self.change_input)
        ###

        ### Btn Run
        self.ui.btn_run_BC.clicked.connect(self.run_BC)
        ### Forward
        self.ui.btn_U_BC.clicked.connect(self.Forward_Signal)
        ### Backward
        self.ui.btn_D_BC.clicked.connect(self.Backward_Signal)
        ### Left
        self.ui.btn_L_BC.clicked.connect(self.Left_Signal)
        ### Right
        self.ui.btn_R_BC.clicked.connect(self.Right_Signal)
        ### Z-Up
        self.ui.btn_zU_BC.clicked.connect(self.Up_Signal)
        ### Z-Down
        self.ui.btn_zD_BC.clicked.connect(self.Down_Signal)
        ### Home btn
        self.ui.btn_home_BC.clicked.connect(self.send_home)
        self.ui.btn_home_BC.clicked.connect(self.set_manual)
        
        ## AC
        self.ui.btn_ACtrl.clicked.connect(self.Advance_Ctrl)
        
        # Form Camera
        self.ui.btn_start_cam.clicked.connect(lambda: cam.set_cam(self))
        
        # Form Setting
        self.ui.btn_set_save.clicked.connect(self.save_setting)
        self.ui.btn_set_clear.clicked.connect(self.clear_setting)
        self.ui.btn_set_return.clicked.connect(self.return_setting)
        
        # Action Buttons
        self.action_btn()
        
    '''
    Form Home
    '''
        
    def show_home(self):
        self.ui.timer1.start(100)
        self.ui.stw_main.setCurrentWidget(self.ui.pg_Home)
        self.ui.stw_plot.setCurrentWidget(self.ui.pg_home)
        self.ui.Header_Frame.setStyleSheet("background-color:rgb(245, 223, 77);")
        self.ui.lb_tittle.setStyleSheet("color:rgb(59,59,59);")
        self.ui.btn_close.setStyleSheet("color:black;")
        self.ui.btn_min.setStyleSheet("color:black;")
        self.ui.LH_Frame.setStyleSheet("background-color:rgb(39, 39, 58);")
        self.ui.btn_home.setStyleSheet("color:lightgray;")
        self.ui.btn_Connect.setStyleSheet("background-color:rgb(51, 51, 76); border:0px; color:white;")
        self.ui.btn_Ctrl.setStyleSheet("background-color:rgb(51, 51, 76); border:0px; color:white;")
        self.ui.btn_Calib.setStyleSheet("background-color:rgb(51, 51, 76); border:0px; color:white;")
        self.ui.btn_Kine.setStyleSheet("background-color:rgb(51, 51, 76); border:0px; color:white;")
        self.ui.btn_Setting.setStyleSheet("background-color:rgb(51, 51, 76); border:0px; color:white;")
        # self.ui.btn_sim.setChecked(False)
    
    def set_change(self):
        self.ui.lb_tittle.setStyleSheet("color:white;")
        self.ui.btn_close.setStyleSheet("color:white;")
        self.ui.btn_min.setStyleSheet("color:white;")
        self.ui.btn_home.setStyleSheet("color:white;")
        
    def center(self):
        qr = self.main_win.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.main_win.move(qr.topLeft())
        
    '''
    Form Connections
    '''
        
    def show_connect(self):
        self.ui.timer1.stop()
        self.ui.stw_main.setCurrentWidget(self.ui.pg_ctrl)
        self.ui.stw_ctrl.setCurrentWidget(self.ui.pg_Connect)
        self.ui.stw_plot.setCurrentWidget(self.ui.pg_plot)
        self.ui.stw_plot3d.setCurrentWidget(self.ui.pg_plot_3d)
        self.ui.lb_tittle.setText("CONNECTION")
        self.ui.Header_Frame.setStyleSheet("background-color:#0094BC;")
        self.ui.LH_Frame.setStyleSheet("background-color:#007696;")
        self.ui.btn_Connect.setStyleSheet("background-color:#0094BC; border:0px; color:white;")
        self.ui.btn_Ctrl.setStyleSheet("background-color:rgb(51, 51, 76); border:0px; color:white;")
        self.ui.btn_Calib.setStyleSheet("background-color:rgb(51, 51, 76); border:0px; color:white;")
        self.ui.btn_Kine.setStyleSheet("background-color:rgb(51, 51, 76); border:0px; color:white;")
        self.ui.btn_Setting.setStyleSheet("background-color:rgb(51, 51, 76); border:0px; color:white;")
        self.ui.label_61.setStyleSheet("color:#007696;")
        self.ui.txt_send.setStyleSheet("color:#007696;")
        self.ui.txt_display.setStyleSheet("color:#007696;")
        self.ui.groupBox.setStyleSheet("color:#0094BC; background-color: rgb(240, 240, 240);")
        self.ui.groupBox_4.setStyleSheet("color:#0094BC; background-color: rgb(240, 240, 240);")
        self.set_change()
        
    def check_mode(self):
        if(self.ui.btn_sim.isChecked()):
            self.ui.sim = 1
            self.ui.run = 0
            self.serial.sim = 1
            self.serial.run = 0
            if(self.ui.sim == 1):
                print('Simulation')
                QMessageBox.question(None, "Notification", "Running Preview Mode",QMessageBox.Ok)
            else:
                self.ui.btn_sim.setChecked(False)
                QMessageBox.question(None, "Notification", "Running Runtime Mode",QMessageBox.Ok)
                print('Runtime')
        else:
            self.ui.sim = 0
            self.ui.run = 1
            self.serial.sim = 0
            self.serial.run = 1
            print('Runtime')
            QMessageBox.question(None, "Notification", "Running Runtime Mode",QMessageBox.Ok)
        
    def connect_serial(self):
        if(self.ui.btn_cnt.isChecked()):
            port = self.ui.cb_port.currentText()
            baud = self.ui.cb_baudrate.currentText()
            self.serial.serialPort.port = port
            self.serial.serialPort.baudrate = baud
            self.serial.connect_serial()
            if(self.serial.serialPort.is_open):
                self.ui.btn_cnt.setText('Disconnect')
                self.ui.btn_cnt.setStyleSheet("background-color:#0094BC;")
                # print("Connect")
            else:
                # print("No Connect")
                self.ui.btn_cnt.setText('Connect')
                self.ui.btn_cnt.setStyleSheet("")
                self.ui.btn_cnt.setChecked(False)
        else:
            self.serial.disconnect_serial()
            self.ui.btn_cnt.setText('Connect')
            self.ui.btn_cnt.setStyleSheet("")
            # print("Disconnected")
        
    def send_data(self):
        data = self.ui.txt_send.text()
        self.serial.send_data(data)
        
    def update_ports(self):
        self.serial.update_ports()
        self.ui.cb_port.addItems(self.serial.portList)
    
    def closeEvent(self, e):
        self.serial.disconnect_serial()
        
    def update_terminal(self, data):
        self.ui.txt_display.append(data)
        
    def clear_terminal(self):
        self.ui.txt_display.clear()
        
    def update_ee(self, theta):
        print(theta)
        self.ui.the_ee = theta.split(',')
        print(self.ui.the_ee)
        self.ui.tb_the1_ee.setText(str(self.ui.the_ee[0]))
        self.ui.tb_the2_ee.setText(str(self.ui.the_ee[1]))
        self.ui.tb_the3_ee.setText(str(self.ui.the_ee[2]))
        self.ui.tb_the4_ee.setText(str(self.ui.the_ee[3]))
        self.ui.tb_the5_ee.setText(str(self.ui.the_ee[4]))
        self.ui.tb_the6_ee.setText(str(self.ui.the_ee[5]))
        self.ui.tb_the7_ee.setText(str(self.ui.the_ee[6]))
        self.ui.the_BC = np.array([float(self.ui.the_ee[0]),float(self.ui.the_ee[1]),float(self.ui.the_ee[2]),float(self.ui.the_ee[3]),float(self.ui.the_ee[4]),float(self.ui.the_ee[5]),float(self.ui.the_ee[6])])
        self.ui.BC_FK_out = kine.CalcFK(self.ui.the_BC)
        self.ui.tb_posx_ee.setText(str(np.round(self.ui.BC_FK_out[0],3)))
        self.ui.tb_posy_ee.setText(str(np.round(self.ui.BC_FK_out[1],3)))
        self.ui.tb_posz_ee.setText(str(np.round(self.ui.BC_FK_out[2],3)))
        
    def update_ee_2(self, theta):
        self.ui.tb_the1_ee.setText(str(np.round(theta[0]*180/pi,2)))
        self.ui.tb_the2_ee.setText(str(np.round(theta[1]*180/pi,2)))
        self.ui.tb_the3_ee.setText(str(np.round(theta[2]*180/pi,2)))
        self.ui.tb_the4_ee.setText(str(np.round(theta[3]*180/pi,2)))
        self.ui.tb_the5_ee.setText(str(np.round(theta[4]*180/pi,2)))
        self.ui.tb_the6_ee.setText(str(np.round(theta[5]*180/pi,2)))
        self.ui.tb_the7_ee.setText(str(np.round(theta[6]*180/pi,2)))
        self.ui.the_BC = np.array([float(theta[0]),float(theta[1]),float(theta[2]),float(theta[3]),float(theta[4]),float(theta[5]),float(theta[6])])
        self.ui.BC_FK_out = kine.CalcFK(self.ui.the_BC)
        self.ui.tb_posx_ee.setText(str(np.round(self.ui.BC_FK_out[0],3)))
        self.ui.tb_posy_ee.setText(str(np.round(self.ui.BC_FK_out[1],3)))
        self.ui.tb_posz_ee.setText(str(np.round(self.ui.BC_FK_out[2],3)))
        
    '''
    Form Kinematics
    '''
        
    def show_kinematic(self):
        self.ui.timer1.start(100)
        self.FK()
        self.ui.stw_main.setCurrentWidget(self.ui.pg_ctrl)
        self.ui.stw_ctrl.setCurrentWidget(self.ui.pg_Kine)
        self.ui.stw_plot.setCurrentWidget(self.ui.pg_plot)
        self.ui.stw_plot3d.setCurrentWidget(self.ui.pg_plot_3d)
        self.ui.lb_tittle.setText("KINEMATIC")
        self.ui.Header_Frame.setStyleSheet("background-color:#FF5722;")
        self.ui.LH_Frame.setStyleSheet("background-color:#CC461B;")
        self.ui.btn_Kine.setStyleSheet("background-color:#FF5722; color:white;")
        self.ui.btn_Connect.setStyleSheet("background-color:rgb(51, 51, 76); border:0px; color:white;")
        self.ui.btn_Ctrl.setStyleSheet("background-color:rgb(51, 51, 76); border:0px; color:white;")
        self.ui.btn_Calib.setStyleSheet("background-color:rgb(51, 51, 76); border:0px; color:white;")
        self.ui.btn_Setting.setStyleSheet("background-color:rgb(51, 51, 76); border:0px; color:white;")
        self.ui.btn_calc_FK.setStyleSheet("color:#FF5722;")
        self.ui.btn_calc_IK.setStyleSheet("color:#FF5722;")
        self.ui.btn_run_FK.setStyleSheet("color:#FF5722;")
        self.ui.btn_run_FK.setStyleSheet("color:#FF5722;")
        self.ui.btn_run_IK.setStyleSheet("color:#FF5722;")
        self.ui.btn_run_IK.setStyleSheet("color:#FF5722;")
        self.set_change()
        
    def FK(self):
        self.ui.stw_IK.setCurrentWidget(self.ui.pg_FK)
        self.ui.btn_FK.setStyleSheet("background-color:#FF5722; border:0px; color:white;")
        self.ui.btn_IK.setStyleSheet("boder:0px;")
        
    def calc_FK(self):
        out = []
        the1 = self.ui.tb_the1_FK.text()
        the2 = self.ui.tb_the2_FK.text()
        the3 = self.ui.tb_the3_FK.text()
        the4 = self.ui.tb_the4_FK.text()
        the5 = self.ui.tb_the5_FK.text()
        the6 = self.ui.tb_the6_FK.text()
        the7 = self.ui.tb_the7_FK.text()
        self.ui.the_calc = np.array([the1, the2, the3, the4, the5, the6, the7]).astype(np.float64())*pi/180
        self.thread[1] = Thread_FK(self.ui.the_calc, out, parent = None)
        self.thread[1].start()
        self.thread[1].signal.connect(self.display_FK)
        
    def display_FK(self, out):
        self.thread[1].quit()
        print(self.ui.the_plot)
        out = np.array(out)
        self.ui.tb_X_FK.setText(str(np.round(out[0],3)))
        self.ui.tb_Y_FK.setText(str(np.round(out[1],3)))
        self.ui.tb_Z_FK.setText(str(np.round(out[2],3)))
        self.ui.tb_alp_FK.setText(str(np.round(out[3]*180/pi,3)))
        self.ui.tb_bet_FK.setText(str(np.round(out[4]*180/pi,3)))
        self.ui.tb_gam_FK.setText(str(np.round(out[5]*180/pi,3)))
        QMessageBox.information(None, "Notification", "Complete the calculating process",QMessageBox.Ok)
        
    def copy_FK(self):
        self.ui.tb_X_IK.setText(self.ui.tb_X_FK.text())
        self.ui.tb_Y_IK.setText(self.ui.tb_Y_FK.text())
        self.ui.tb_Z_IK.setText(self.ui.tb_Z_FK.text())
        self.ui.tb_alp_IK.setText(self.ui.tb_alp_FK.text())
        self.ui.tb_bet_IK.setText(self.ui.tb_bet_FK.text())
        self.ui.tb_gam_IK.setText(self.ui.tb_gam_FK.text())
        
    def IK(self):
        self.ui.stw_IK.setCurrentWidget(self.ui.pg_IK)
        self.ui.btn_IK.setStyleSheet("background-color:#FF5722; border:0px; color:white;")
        self.ui.btn_FK.setStyleSheet("boder:0px;")
        
    def calc_IK(self):
        IK_out = []
        X = self.ui.tb_X_IK.text()
        Y = self.ui.tb_Y_IK.text()
        Z = self.ui.tb_Z_IK.text()
        alp = self.ui.tb_alp_IK.text()
        bet = self.ui.tb_bet_IK.text()
        gam = self.ui.tb_gam_IK.text()
        error = self.ui.error
        self.ui.IK_in = np.array([X, Y, Z, alp, bet, gam]).astype(np.float64())
        self.thread[2] = Thread_IK(self.ui.IK_in, IK_out, error, parent = None)
        self.thread[2].start()
        self.thread[2].signal.connect(self.display_IK)
    
    def display_IK(self, IK_out):
        self.thread[2].quit()
        self.ui.the_calc = np.array(IK_out)
        the_deg = np.round((self.ui.the_calc)*180/pi,3)
        self.ui.tb_the1_IK.setText(str(np.round(float(the_deg[0]),3)))
        self.ui.tb_the2_IK.setText(str(np.round(float(the_deg[1]),3)))
        self.ui.tb_the3_IK.setText(str(np.round(float(the_deg[2]),3)))
        self.ui.tb_the4_IK.setText(str(np.round(float(the_deg[3]),3)))
        self.ui.tb_the5_IK.setText(str(np.round(float(the_deg[4]),3)))
        self.ui.tb_the6_IK.setText(str(np.round(float(the_deg[5]),3)))
        self.ui.tb_the7_IK.setText(str(np.round(float(the_deg[6]),3))) 
        QMessageBox.question(None, "Notification", "Complete calculating process",QMessageBox.Ok)
        
    '''
    Form Control
    '''
        
    def show_control(self):
        self.ui.timer1.start(100)
        # self.ui.timer3.start(10)
        self.Basic_Ctrl()
        self.ui.stw_input_BC.setCurrentWidget(self.ui.pg_ithe)
        self.ui.stw_main.setCurrentWidget(self.ui.pg_ctrl)
        self.ui.stw_ctrl.setCurrentWidget(self.ui.pg_Ctrl)
        self.ui.stw_plot.setCurrentWidget(self.ui.pg_plot)
        self.ui.stw_plot3d.setCurrentWidget(self.ui.pg_plot_3d)
        self.ui.lb_tittle.setText("CONTROL")
        self.ui.Header_Frame.setStyleSheet("background-color:#6600CC;")
        self.ui.LH_Frame.setStyleSheet("background-color:#5100A3;")
        self.ui.btn_Ctrl.setStyleSheet("background-color:#6600CC; color:white;")
        self.ui.btn_Connect.setStyleSheet("background-color:rgb(51, 51, 76); border:0px; color:white;")
        self.ui.btn_Calib.setStyleSheet("background-color:rgb(51, 51, 76); border:0px; color:white;")
        self.ui.btn_Kine.setStyleSheet("background-color:rgb(51, 51, 76); border:0px; color:white;")
        self.ui.btn_Setting.setStyleSheet("background-color:rgb(51, 51, 76); border:0px; color:white;")
        self.ui.label_16.setStyleSheet("color:#5100A3;")
        self.ui.tb_time_BC.setText(str(self.ui.time_dyn))
        self.ui.tb_time_AC.setText(str(self.ui.time_dyn))
        self.set_change()
        
    def Change_plot(self):
        icon1 = QIcon()
        icon1.addPixmap(QPixmap(":/Icons/Icons/activity.svg"), QIcon.Normal, QIcon.Off)
        icon2 = QIcon()
        icon2.addPixmap(QPixmap(":/Icons/Icons/camera.svg"), QIcon.Normal, QIcon.Off)
        icon3 = QIcon()
        icon3.addPixmap(QPixmap(":/Icons/Icons/trello.svg"), QIcon.Normal, QIcon.Off)
        self.ui.tab_plot += 1
        if (self.ui.tab_plot%2 == 1):
            self.ui.btn_plot_page.setIcon(icon1)
            self.ui.stw_plot.setCurrentWidget(self.ui.pg_plot)
        if (self.ui.tab_plot%2 == 2):
            self.ui.btn_plot_page.setIcon(icon2)
            self.ui.stw_plot.setCurrentWidget(self.ui.pg_error)
        if (self.ui.tab_plot%2 == 0):
            self.ui.btn_plot_page.setIcon(icon3)
            self.ui.stw_plot.setCurrentWidget(self.ui.pg_camera)
            
    def Basic_Ctrl(self):
        self.ui.stw_Ctrl.setCurrentWidget(self.ui.pg_BCtrl)
        self.ui.btn_BCtrl.setStyleSheet("background-color:#6600CC; border:0px; color:white;")
        self.ui.btn_ACtrl.setStyleSheet("boder:0px;")
        self.ui.label_111.setStyleSheet("color:#5100A3;")
        self.ui.btn_start_BC.setStyleSheet("color:#5100A3;")
        self.ui.btn_run_BC.setStyleSheet("color:#5100A3;")
        if(self.ui.sim == True):
            self.ui.Prv_Led.setStyleSheet("border: 0px solid; border-radius: 10px; background-color: #6600CC;")
            self.ui.Rnt_Led.setStyleSheet("border: 2px solid; border-radius: 10px;")
        if(self.ui.run == True):
            self.ui.Rnt_Led.setStyleSheet("border: 0px solid; border-radius: 10px; background-color: #6600CC;")
            self.ui.Prv_Led.setStyleSheet("border: 2px solid; border-radius: 10px;")
        
        self.ui.the1_BC = float(self.ui.tb_the1_BC.text())
        self.ui.the2_BC = float(self.ui.tb_the2_BC.text())
        self.ui.the3_BC = float(self.ui.tb_the3_BC.text())
        self.ui.the4_BC = float(self.ui.tb_the4_BC.text())
        self.ui.the5_BC = float(self.ui.tb_the5_BC.text())
        self.ui.the6_BC = float(self.ui.tb_the6_BC.text())
        self.ui.the7_BC = float(self.ui.tb_the7_BC.text())
        self.ui.X_BC = float(self.ui.tb_X_BC.text())
        self.ui.Y_BC = float(self.ui.tb_Y_BC.text())
        self.ui.Z_BC = float(self.ui.tb_Z_BC.text())
        self.ui.alp_BC = float(self.ui.tb_alp_BC.text())
        self.ui.bet_BC = float(self.ui.tb_bet_BC.text())
        self.ui.gam_BC = float(self.ui.tb_gam_BC.text())
        
    def change_input(self):
        if (self.ui.btn_inp_BC.isChecked() == True):
            self.ui.btn_inp_BC.setText('Input (the)')
            self.ui.stw_input_BC.setCurrentWidget(self.ui.pg_ithe)
            self.ui.btn_inp_BC.checkStateSet()
        else:
            self.ui.btn_inp_BC.setText('Input (pos)')
            self.ui.stw_input_BC.setCurrentWidget(self.ui.pg_ipos)
            self.ui.btn_inp_BC.checkStateSet()
                
    def set_manual(self):
        if (self.ui.btn_inp_BC.isChecked() == False):
            self.ui.stw_input_BC.setCurrentWidget(self.ui.pg_ipos)
            print(self.ui.btn_inp_BC.isChecked())
        if (self.ui.btn_inp_BC.isChecked() == True):
            self.ui.btn_inp_BC.setText('Input (pos)')
            self.ui.stw_input_BC.setCurrentWidget(self.ui.pg_ipos)
            self.ui.btn_inp_BC.nextCheckState()
            
    def send_the(self, the):
        self.ui.time_s = float(self.ui.tb_time_BC.text())
        self.ui.the_sent = the*180/pi
        self.ui.data_sent = str(np.round(float(self.ui.the_sent[0]),1)) + ","  + str(np.round(float(self.ui.the_sent[1]),1)) + ","  + str(np.round(float(self.ui.the_sent[2]),1)) + "," + str(np.round(float(self.ui.the_sent[3]),1)) + "," + str(np.round(float(self.ui.the_sent[4]),1)) + "," + str(np.round(float(self.ui.the_sent[5]),1)) + "," + str(np.round(float(self.ui.the_sent[6]),1)) + "," + str(self.ui.time_s)
        if(self.ui.run == 1 and self.serial.serialPort.isOpen):
            try:
                message = self.ui.data_sent + "\n"
                print('mess:')
                print(message)
                self.serial.serialPort.write(message.encode())
            except:
                QMessageBox.critical(None, "Error", "Serial is not open",QMessageBox.Cancel) 
        if(self.ui.sim == 1):
            ctrl.sim_FK_BC(self, self.ui.time_s)
        
    def send(self, x, y, z, alp, bet, gam):
        self.ui.time_s = float(self.ui.tb_time_BC.text())
        self.ui.tb_X_BC.setText(str(np.round(x,2)))
        self.ui.tb_Y_BC.setText(str(np.round(y,2)))
        self.ui.tb_Z_BC.setText(str(np.round(z,2)))
        self.ui.tb_alp_BC.setText(str(np.round(alp,2)))
        self.ui.tb_bet_BC.setText(str(np.round(bet,2)))
        self.ui.tb_gam_BC.setText(str(np.round(gam,2)))
        self.IK_in = np.array([x, y, z, alp, bet, gam]).astype(np.float64())
        self.error = self.ui.error
        self.ui.the_sent = np.round(kine.CalcIK(self.IK_in, self.error)*180/pi,1)
        self.ui.data_sent = str(np.round(float(self.ui.the_sent[0]),1)) + ","  + str(np.round(float(self.ui.the_sent[1]),1)) + ","  + str(np.round(float(self.ui.the_sent[2]),1)) + "," + str(np.round(float(self.ui.the_sent[3]),1)) + "," + str(np.round(float(self.ui.the_sent[4]),1)) + "," + str(np.round(float(self.ui.the_sent[5]),1)) + "," + str(np.round(float(self.ui.the_sent[6]),1)) + "," + str(self.ui.time_s)
        print('the_sent')
        print(self.ui.data_sent)
        if(self.ui.run == 1 and self.serial.serialPort.isOpen):
            try:
                message = self.ui.data_sent + "\n"
                print('mess:')
                print(message)
                self.serial.serialPort.write(message.encode())
            except:
                QMessageBox.critical(None, "Error", "Serial is not open",QMessageBox.Cancel) 
        if(self.ui.sim == 1):
            ctrl.sim_IK_BC(self, self.ui.the_sent, self.ui.time_s)
                   
    def send_home(self):
        x = 0.4496; y = 0; z = -0.1240
        alp = 90; bet = 0; gam = 180
        self.send(x,y,z,alp,bet,gam)
        
    def send_inverse(self):
        set_x = float(self.ui.tb_X_BC.text())
        set_y = float(self.ui.tb_Y_BC.text())
        set_z = float(self.ui.tb_Z_BC.text())
        set_a = float(self.ui.tb_alp_BC.text())
        set_b = float(self.ui.tb_bet_BC.text())
        set_g = float(self.ui.tb_gam_BC.text())
        self.send(set_x,set_y,set_z,set_a,set_b,set_g)
        
    def Forward_Signal(self):
        set_x = float(self.ui.tb_X_BC.text()) + 0.05
        set_y = float(self.ui.tb_Y_BC.text())
        set_z = float(self.ui.tb_Z_BC.text())
        set_a = float(self.ui.tb_alp_BC.text())
        set_b = float(self.ui.tb_bet_BC.text())
        set_g = float(self.ui.tb_gam_BC.text())
        self.send(set_x,set_y,set_z,set_a,set_b,set_g)
        
    def Backward_Signal(self):
        set_x = float(self.ui.tb_X_BC.text()) - 0.05
        set_y = float(self.ui.tb_Y_BC.text())
        set_z = float(self.ui.tb_Z_BC.text())
        set_a = float(self.ui.tb_alp_BC.text())
        set_b = float(self.ui.tb_bet_BC.text())
        set_g = float(self.ui.tb_gam_BC.text())
        self.send(set_x,set_y,set_z,set_a,set_b,set_g)
    
    def Left_Signal(self):
        set_x = float(self.ui.tb_X_BC.text()) 
        set_y = float(self.ui.tb_Y_BC.text()) + 0.05
        set_z = float(self.ui.tb_Z_BC.text())
        set_a = float(self.ui.tb_alp_BC.text())
        set_b = float(self.ui.tb_bet_BC.text())
        set_g = float(self.ui.tb_gam_BC.text())
        self.send(set_x,set_y,set_z,set_a,set_b,set_g)
    
    def Right_Signal(self):
        set_x = float(self.ui.tb_X_BC.text()) 
        set_y = float(self.ui.tb_Y_BC.text()) - 0.05
        set_z = float(self.ui.tb_Z_BC.text())
        set_a = float(self.ui.tb_alp_BC.text())
        set_b = float(self.ui.tb_bet_BC.text())
        set_g = float(self.ui.tb_gam_BC.text())
        self.send(set_x,set_y,set_z,set_a,set_b,set_g)
    
    def Up_Signal(self):
        set_x = float(self.ui.tb_X_BC.text()) 
        set_y = float(self.ui.tb_Y_BC.text())
        set_z = float(self.ui.tb_Z_BC.text()) + 0.05
        set_a = float(self.ui.tb_alp_BC.text())
        set_b = float(self.ui.tb_bet_BC.text())
        set_g = float(self.ui.tb_gam_BC.text())
        self.send(set_x,set_y,set_z,set_a,set_b,set_g)
    
    def Down_Signal(self):
        set_x = float(self.ui.tb_X_BC.text())
        set_y = float(self.ui.tb_Y_BC.text())
        set_z = float(self.ui.tb_Z_BC.text()) - 0.05
        set_a = float(self.ui.tb_alp_BC.text())
        set_b = float(self.ui.tb_bet_BC.text())
        set_g = float(self.ui.tb_gam_BC.text())
        self.send(set_x,set_y,set_z,set_a,set_b,set_g)
        
    def run_BC(self):
        if(self.ui.btn_inp_BC.text() == 'Input (the)'):
            the1 = float(self.ui.tb_the1_BC.text())*pi/180
            the2 = float(self.ui.tb_the2_BC.text())*pi/180
            the3 = float(self.ui.tb_the3_BC.text())*pi/180
            the4 = float(self.ui.tb_the4_BC.text())*pi/180
            the5 = float(self.ui.tb_the5_BC.text())*pi/180
            the6 = float(self.ui.tb_the6_BC.text())*pi/180
            the7 = float(self.ui.tb_the7_BC.text())*pi/180
            self.ui.the_BC = np.array([the1,the2,the3,the4,the5,the6,the7])
            
            self.send_the(self.ui.the_BC)
            
            self.ui.BC_FK_out = kine.CalcFK(self.ui.the_BC)
            self.ui.tb_X_BC.setText(str(np.round(self.ui.BC_FK_out[0],3)))
            self.ui.tb_Y_BC.setText(str(np.round(self.ui.BC_FK_out[1],3)))
            self.ui.tb_Z_BC.setText(str(np.round(self.ui.BC_FK_out[2],3)))
            self.ui.tb_alp_BC.setText(str(np.round(self.ui.BC_FK_out[3]*180/pi,3)))
            self.ui.tb_bet_BC.setText(str(np.round(self.ui.BC_FK_out[4]*180/pi,3)))
            self.ui.tb_gam_BC.setText(str(np.round(self.ui.BC_FK_out[5]*180/pi,3)))
            
        if(self.ui.btn_inp_BC.text() == 'Input (pos)'):
            x = float(self.ui.tb_X_BC.text())
            y = float(self.ui.tb_Y_BC.text())
            z = float(self.ui.tb_Z_BC.text())
            a = float(self.ui.tb_alp_BC.text())
            b = float(self.ui.tb_bet_BC.text())
            g = float(self.ui.tb_gam_BC.text())
            
            self.send(x,y,z,a,b,g)
            
            self.ui.tb_the1_BC.setText(str(round(float(self.ui.the_sent[0]),2)))
            self.ui.tb_the2_BC.setText(str(round(float(self.ui.the_sent[1]),2)))
            self.ui.tb_the3_BC.setText(str(round(float(self.ui.the_sent[2]),2)))
            self.ui.tb_the4_BC.setText(str(round(float(self.ui.the_sent[3]),2)))
            self.ui.tb_the5_BC.setText(str(round(float(self.ui.the_sent[4]),2)))
            self.ui.tb_the6_BC.setText(str(round(float(self.ui.the_sent[5]),2)))
            self.ui.tb_the7_BC.setText(str(round(float(self.ui.the_sent[6]),2)))       
                
    def Advance_Ctrl(self):
        self.ui.stw_Ctrl.setCurrentWidget(self.ui.pg_ACtrl)
        self.ui.btn_ACtrl.setStyleSheet("background-color:#6600CC; border:0px; color:white;")
        self.ui.btn_BCtrl.setStyleSheet("boder:0px;")
        self.ui.label_111.setStyleSheet("color:#5100A3;")
        self.ui.btn_stop_AC.setStyleSheet("color:#5100A3;")
        self.ui.btn_run_AC.setStyleSheet("color:#5100A3;")
        self.ui.label_107.setStyleSheet("color:green;")
        self.ui.label_108.setStyleSheet("color:red;")
        if(self.ui.sim == True):
            self.ui.Prv_Led_AC.setStyleSheet("border: 0px solid; border-radius: 10px; background-color: #6600CC;")
            self.ui.Rnt_Led_AC.setStyleSheet("border: 2px solid; border-radius: 10px;")
        if(self.ui.run == True):
            self.ui.Rnt_Led_AC.setStyleSheet("border: 0px solid; border-radius: 10px; background-color: #6600CC;")
            self.ui.Prv_Led_AC.setStyleSheet("border: 2px solid; border-radius: 10px;")
        
    '''
    Form Camera
    '''
        
    def show_calib(self):
        # self.ui.timer2.start(1000)
        self.ui.timer1.start(100)
        self.ui.stw_main.setCurrentWidget(self.ui.pg_ctrl)
        self.ui.stw_ctrl.setCurrentWidget(self.ui.pg_Calib)
        self.ui.stw_plot.setCurrentWidget(self.ui.pg_camera)
        self.ui.stw_plot3d.setCurrentWidget(self.ui.pg_plot_3d)
        self.ui.lb_tittle.setText("CAMERA OPERATION")
        self.ui.Header_Frame.setStyleSheet("background-color:#FF0000;")
        self.ui.LH_Frame.setStyleSheet("background-color:#CC0000;")
        self.ui.btn_Calib.setStyleSheet("background-color:#FF0000; color:white;")
        self.ui.btn_Connect.setStyleSheet("background-color:rgb(51, 51, 76); border:0px; color:white;")
        self.ui.btn_Ctrl.setStyleSheet("background-color:rgb(51, 51, 76); border:0px; color:white;")
        self.ui.btn_Kine.setStyleSheet("background-color:rgb(51, 51, 76); border:0px; color:white;")
        self.ui.btn_Setting.setStyleSheet("background-color:rgb(51, 51, 76); border:0px; color:white;")
        self.ui.label_92.setStyleSheet("color:#CC0000;")
        self.ui.btn_start_cam.setStyleSheet("color:#CC0000;")
        self.set_change()          
        
    '''
    Form Setting
    '''
        
    def show_setting(self):
        self.ui.timer1.stop()
        self.ui.stw_main.setCurrentWidget(self.ui.pg_ctrl)
        self.ui.stw_ctrl.setCurrentWidget(self.ui.pg_Setting)
        self.ui.stw_plot.setCurrentWidget(self.ui.pg_camera)
        self.ui.stw_plot3d.setCurrentWidget(self.ui.pg_setting)
        self.ui.lb_tittle.setText("SYSTEM SETTING")
        self.ui.Header_Frame.setStyleSheet("background-color:#43B76E;")
        self.ui.LH_Frame.setStyleSheet("background-color:#369258;")
        self.ui.btn_Setting.setStyleSheet("background-color:#43B76E; color:white;")
        self.ui.btn_Connect.setStyleSheet("background-color:rgb(51, 51, 76); border:0px; color:white;")
        self.ui.btn_Ctrl.setStyleSheet("background-color:rgb(51, 51, 76); border:0px; color:white;")
        self.ui.btn_Calib.setStyleSheet("background-color:rgb(51, 51, 76); border:0px; color:white;")
        self.ui.btn_Kine.setStyleSheet("background-color:rgb(51, 51, 76); border:0px; color:white;")
        self.ui.label_95.setStyleSheet("color:#369258;")
        self.ui.label_96.setStyleSheet("color:#369258;")
        self.ui.btn_set_save.setStyleSheet("color:#369258;")
        self.ui.btn_set_clear.setStyleSheet("color:#369258;")
        self.ui.btn_set_return.setStyleSheet("color:#369258;")
        self.set_change()
        
    def save_setting(self):
        self.ui.X = self.ui.tb_X_init.text()
        self.ui.Y = self.ui.tb_Y_init.text()
        self.ui.Z = self.ui.tb_Z_init.text()
        self.ui.alp = self.ui.tb_alp_init.text()
        self.ui.bet = self.ui.tb_bet_init.text()
        self.ui.gam = self.ui.tb_gam_init.text()
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
        self.ui.timed = self.ui.tb_time_set.text()
        self.ui.error = self.ui.tb_err_set.text()
        
        self.ui.R_vec1[0] = self.ui.tb_R1_1.text()
        self.ui.R_vec1[1] = self.ui.tb_R1_2.text()
        self.ui.R_vec1[2] = self.ui.tb_R1_3.text()
        self.ui.T_vec1[0] = self.ui.tb_T1_1.text()
        self.ui.T_vec1[0] = self.ui.tb_T1_2.text()
        self.ui.T_vec1[0] = self.ui.tb_T1_3.text()
        self.ui.Ox1 = self.ui.tb_Ox1.text()
        self.ui.Oy1 = self.ui.tb_Oy1.text()
        self.ui.Fx1 = self.ui.tb_Fx1.text()
        self.ui.Fy1 = self.ui.tb_Fy1.text()
        
        self.ui.R_vec2[0] = self.ui.tb_R2_1.text()
        self.ui.R_vec2[1] = self.ui.tb_R2_2.text()
        self.ui.R_vec2[2] = self.ui.tb_R2_3.text()
        self.ui.T_vec2[0] = self.ui.tb_T2_1.text()
        self.ui.T_vec2[0] = self.ui.tb_T2_2.text()
        self.ui.T_vec2[0] = self.ui.tb_T2_3.text()
        self.ui.Ox2 = self.ui.tb_Ox2.text()
        self.ui.Oy2 = self.ui.tb_Oy2.text()
        self.ui.Fx2 = self.ui.tb_Fx2.text()
        self.ui.Fy2 = self.ui.tb_Fy2.text()
        
    def clear_setting(self):
        self.ui.tb_X_init.clear()
        self.ui.tb_Y_init.clear()
        self.ui.tb_Z_init.clear()
        self.ui.tb_alp_init.clear()
        self.ui.tb_bet_init.clear()
        self.ui.tb_gam_init.clear()
        self.ui.tb_the1_max.clear()
        self.ui.tb_the2_max.clear()
        self.ui.tb_the3_max.clear()
        self.ui.tb_the4_max.clear()
        self.ui.tb_the5_max.clear()
        self.ui.tb_the6_max.clear()
        self.ui.tb_the7_max.clear()
        self.ui.tb_the1_min.clear()
        self.ui.tb_the2_min.clear()
        self.ui.tb_the3_min.clear()
        self.ui.tb_the4_min.clear()
        self.ui.tb_the5_min.clear()
        self.ui.tb_the6_min.clear()
        self.ui.tb_the7_min.clear()
        self.ui.tb_time_set.clear()
        self.ui.tb_err_set.clear()
        
        self.ui.tb_R1_1.clear()
        self.ui.tb_R1_2.clear()
        self.ui.tb_R1_3.clear()
        self.ui.tb_T1_1.clear()
        self.ui.tb_T1_2.clear()
        self.ui.tb_T1_3.clear()
        self.ui.tb_Ox1.clear()
        self.ui.tb_Oy1.clear()
        self.ui.tb_Fx1.clear()
        self.ui.tb_Fy1.clear()

        self.ui.tb_R2_1.clear()
        self.ui.tb_R2_2.clear()
        self.ui.tb_R2_3.clear()
        self.ui.tb_T2_1.clear()
        self.ui.tb_T2_2.clear()
        self.ui.tb_T2_3.clear()
        self.ui.tb_Ox2.clear()
        self.ui.tb_Oy2.clear()
        self.ui.tb_Fx2.clear()
        self.ui.tb_Fy2.clear()
        
    def return_setting(self):
        self.ui.tb_X_init.setText(self.ui.X)
        self.ui.tb_Y_init.setText(self.ui.Y)
        self.ui.tb_Z_init.setText(self.ui.Z)
        self.ui.tb_alp_init.setText(self.ui.alp)
        self.ui.tb_bet_init.setText(self.ui.bet)
        self.ui.tb_gam_init.setText(self.ui.gam)
        self.ui.tb_the1_max.setText(self.ui.the1_max)
        self.ui.tb_the2_max.setText(self.ui.the2_max)
        self.ui.tb_the3_max.setText(self.ui.the3_max)
        self.ui.tb_the4_max.setText(self.ui.the4_max)
        self.ui.tb_the5_max.setText(self.ui.the5_max)
        self.ui.tb_the6_max.setText(self.ui.the6_max)
        self.ui.tb_the7_max.setText(self.ui.the7_max)
        self.ui.tb_the1_min.setText(self.ui.the1_min)
        self.ui.tb_the2_min.setText(self.ui.the2_min)
        self.ui.tb_the3_min.setText(self.ui.the3_min)
        self.ui.tb_the4_min.setText(self.ui.the4_min)
        self.ui.tb_the5_min.setText(self.ui.the5_min)
        self.ui.tb_the6_min.setText(self.ui.the6_min)
        self.ui.tb_the7_min.setText(self.ui.the7_min)
        self.ui.tb_time_set.setText(self.ui.timed)
        self.ui.tb_err_set.setText(self.ui.error)
        
        self.ui.tb_R1_1.setText(str(round(self.ui.R_vec1[0],3)))
        self.ui.tb_R1_2.setText(str(round(self.ui.R_vec1[1],3)))
        self.ui.tb_R1_3.setText(str(round(self.ui.R_vec1[2],3)))
        self.ui.tb_T1_1.setText(str(round(self.ui.T_vec1[0],3)))
        self.ui.tb_T1_2.setText(str(round(self.ui.T_vec1[1],3)))
        self.ui.tb_T1_3.setText(str(round(self.ui.T_vec1[2],3)))
        self.ui.tb_Ox1.setText(str(self.ui.Ox1))
        self.ui.tb_Oy1.setText(str(self.ui.Oy1))
        self.ui.tb_Fx1.setText(str(self.ui.Fx1))
        self.ui.tb_Fy1.setText(str(self.ui.Fy1))
        
        self.ui.tb_R2_1.setText(str(round(self.ui.R_vec2[0],3)))
        self.ui.tb_R2_2.setText(str(round(self.ui.R_vec2[1],3)))
        self.ui.tb_R2_3.setText(str(round(self.ui.R_vec2[2],3)))
        self.ui.tb_T2_1.setText(str(round(self.ui.T_vec2[0],3)))
        self.ui.tb_T2_2.setText(str(round(self.ui.T_vec2[1],3)))
        self.ui.tb_T2_3.setText(str(round(self.ui.T_vec2[2],3)))
        self.ui.tb_Ox2.setText(str(self.ui.Ox2))
        self.ui.tb_Oy2.setText(str(self.ui.Oy2))
        self.ui.tb_Fx2.setText(str(self.ui.Fx2))
        self.ui.tb_Fy2.setText(str(self.ui.Fy2))
        
    '''
    Action Buttons
    '''
        
    def action_btn(self):
        self.ui.btn_close.clicked.connect(self.close_window)
        self.ui.btn_min.clicked.connect(self.main_win.showMinimized)
         
    def close_window(self):
        self.ui.timer1.stop()
        self.ui.timer2.stop()
         # self.ui.timer3.stop()
         # self.ui.timer4.stop()
        self.ui.timer5.stop()
        self.main_win.close()
        
    def show(self):
        self.main_win.show()
        
class Thread_FK(QThread):
    signal = pyqtSignal(list)
    
    def __init__(self, the, out, parent = None):
        super(Thread_FK, self).__init__(parent)
        self.the = the
        self.out = out
        self.is_running = True
    
    def run(self):
        out = kine.CalcFK(self.the)
        out = list(out)
        self.signal.emit(out)
        self.is_running = False
        
class  Thread_IK(QThread):
    signal = pyqtSignal(list)
    
    def __init__(self, IK_in, IK_out, error, parent = None):
        super(Thread_IK, self).__init__(parent)
        self.error = error
        self.IK_in = IK_in
        self.IK_out = IK_out
        self.is_running = True
    
    def run(self):
        IK_out = kine.CalcIK(self.IK_in, self.error)
        IK_out = list(IK_out)
        self.signal.emit(IK_out)
        self.is_running = False 
        
class Worker(QRunnable):

	def __init__(self, function, *args, **kwargs):
		super(Worker, self).__init__()
		self.function = function
		self.args = args
		self.kwargs = kwargs

	@pyqtSlot()
	def run(self):

		self.function(*self.args, **self.kwargs)
               
class show_plot(FigQT):
    def __init__(self,mode):
        self.fig = Figure()
        super().__init__(self.fig)
        if mode == 1:
            self.ax1 = self.fig.add_subplot(111, projection='3d')           
        if mode == 2:
            self.fig.set_dpi(60)
            self.ax2 = self.fig.add_subplot()
        if mode == 3:
            self.fig.set_dpi(60)
            self.ax3 = self.fig.add_subplot()
        if mode == 4:
            self.fig.set_dpi(60)
            self.ax4 = self.fig.add_subplot()
        if mode == 5:
            self.fig.set_dpi(60)
            self.ax5 = self.fig.add_subplot()
        if mode == 6:
            self.fig.set_dpi(60)
            self.ax6 = self.fig.add_subplot()
        if mode == 7:
            self.fig.set_dpi(60)
            self.ax7 = self.fig.add_subplot()
      
    def clear_fig3d(self):
        self.fig.gca().cla()
        self.ax1.set_title('3D VIEW: XYZ', fontweight = 'bold', fontsize = 20)
        self.ax1.set_xlabel('X')
        self.ax1.set_ylabel('Y')
        self.ax1.set_zlabel('Z')
        self.ax1.set_xlim(-0.3, 0.7)
        self.ax1.set_ylim(-0.5, 0.5)
        self.ax1.set_zlim(-0.7, 0.7)
        
    def clear_fig2xy(self):
        self.fig.gca().cla()
        self.ax2.set_title('2D VIEW: XY', fontweight = 'bold')
        self.ax2.set_xlim(-0.3, 0.7)
        self.ax2.set_ylim(-0.5, 0.5)
        self.ax2.grid(b=True, which='both')
        
    def clear_fig2xz(self):
        self.fig.gca().cla()
        self.ax3.set_title('2D VIEW: XZ', fontweight = 'bold')
        self.ax3.set_xlim(-0.3, 0.7)
        self.ax3.set_ylim(-0.7, 0.7)
        self.ax3.grid(b=True, which='both')
        
    def clear_fig2yz(self):
        self.fig.gca().cla()
        self.ax4.set_title('2D VIEW: YZ', fontweight = 'bold')
        self.ax4.set_xlim(-0.5, 0.5)
        self.ax4.set_ylim(-0.7, 0.7)
        self.ax4.grid(b=True, which='both')
        
    def clear_fig2Ex(self):
        self.fig.gca().cla()
        self.ax5.set_title('PLOT ERROR X', fontweight = 'bold')
        # self.ax5.set_xlim(-0.5, 0.5)
        # self.ax5.set_ylim(-0.7, 0.7)
        self.ax5.grid(b=True, which='both')
        
    def clear_fig2Ey(self):
        self.fig.gca().cla()
        self.ax6.set_title('PLOT ERROR Y', fontweight = 'bold')
        # self.ax6.set_xlim(-0.5, 0.5)
        # self.ax6.set_ylim(-0.7, 0.7)
        self.ax6.grid(b=True, which='both')
        
    def clear_fig2Ez(self):
        self.fig.gca().cla()
        self.ax7.set_title('PLOT ERROR Z', fontweight = 'bold')
        # self.ax7.set_xlim(-0.5, 0.5)
        # self.ax7.set_ylim(-0.7, 0.7)
        self.ax7.grid(b=True, which='both')
        
    def plot3r(self, Q1, Q2, Q3):
        self.ax1.plot3D(Q1, Q2, Q3, color='blue', marker='o', markerfacecolor='black', markersize=10, linewidth=5.0)
        # self.ax1.plot3D(Q1, Q2, Q3, marker='o', markerfacecolor='black', markersize=10)
        
    def plot3o(self, X, Y, Z, marker):
        self.ax1.plot3D(X, Y, Z, marker, 3)
        
    def plot2oxy(self, X, Y, marker):
        self.ax2.plot(X, Y, marker, linewidth=1.0)
        
    def plot2oxz(self, X, Z, marker):
        self.ax3.plot(X, Z, marker, linewidth=1.0)
        
    def plot2oyz(self, Y, Z, marker):
        self.ax4.plot(Y, Z, marker, linewidth=1.0)
        
    def plot2xy(self, Qx, Qy, marker):
        self.ax2.plot(Qx, Qy, marker, linewidth=3.0)
        
    def plot2xz(self, Qx, Qz, marker):
        self.ax3.plot(Qx, Qz, marker, linewidth=3.0)
    
    def plot2yz(self, Qy, Qz, marker):
        self.ax4.plot(Qy, Qz, marker, linewidth=3.0)
        
    def plot2Ex(self, X):
        self.ax5.plot(X, '-b')
        
    def plot2Ey(self, X):
        self.ax6.plot(X, '-b')
        
    def plot2Ez(self, X):
        self.ax7.plot(X, '-b')
        
    def draw_plot(self):
        self.draw()
        
if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_win = MainWindow()
    main_win.show()
    sys.exit(app.exec())