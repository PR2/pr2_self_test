import os
import datetime
from get_diagnostic_data import *
from analysis_test import Diagnostic
from yaml import load
import matplotlib.pyplot as plt
import numpy
from scipy.stats import scoreatpercentile

from python_qt_binding import loadUi
from python_qt_binding.QtCore import qWarning, Qt
from python_qt_binding.QtGui import QWidget, QPushButton, QFileDialog, QLabel

class DiagnosticToolWidget(QWidget):
  
    def __init__(self):
        super(DiagnosticToolWidget, self).__init__()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'pr2_diagnostic_widget.ui')  
        loadUi(ui_file, self)
        self.setObjectName('DiagnosticToolWidget')
        self.left_bool = False
        self.right_bool = False
        self.head_bool = False
        self.debug_info_bool = False
        self.bad_results_bool = False
        self.r_arm_actuators = ['r_wrist_r_motor','r_wrist_l_motor','r_forearm_roll_motor','r_upper_arm_roll_motor', 'r_elbow_flex_motor','r_shoulder_lift_motor','r_shoulder_pan_motor'] 
        self.l_arm_actuators = ['l_wrist_r_motor','l_wrist_l_motor','l_forearm_roll_motor','l_upper_arm_roll_motor', 'l_elbow_flex_motor','l_shoulder_lift_motor','l_shoulder_pan_motor'] 
        self.head_actuators = ['head_pan_motor','head_tilt_motor']
        self.joint_list = []
        self.filelist = []
        self.filenames = []
        self.jointnames = []
        self.plots = []

        #getDataBtn = QPushButton('Quit', self) 
        self.leftArm.stateChanged.connect(self.left_arm_selected)
        self.rightArm.stateChanged.connect(self.right_arm_selected)
        self.head.stateChanged.connect(self.head_selected)
        self.debugInfo.stateChanged.connect(self.debug_info_selected)
        self.badResults.stateChanged.connect(self.bad_results_selected)
        self.getData.clicked[bool].connect(self.get_data_pressed)
        self.resetJointlist.clicked[bool].connect(self.reset_jointlist)
        self.analyzeData.clicked[bool].connect(self.analyze_data)
        self.loadFile.clicked[bool].connect(self.load_file)
        self.loadDirectory.clicked[bool].connect(self.load_directory)
        self.resetFilelist.clicked[bool].connect(self.reset_filelist)
        templist = self.r_arm_actuators + self.l_arm_actuators + self.head_actuators
        for joint in templist:
            self.jointWidget.addItem(str(joint))        
        self.jointWidget.itemSelectionChanged.connect(self.joint_widget_changed) 
        
        rospy.Subscriber("joy", Joy, callback)

    def joint_widget_changed(self):
        self.jointnames = []
        for joint in self.jointWidget.selectedItems():
           if joint.text() not in self.jointnames:
               self.jointnames.append(joint.text())
        self.jointnames = [f.encode("ascii") for f in self.jointnames]

        self.jointLabel.setText('Joints: ' + str(self.jointnames))
        self.jointLabel.setWordWrap(True)

    def left_arm_selected(self, state):
        if state == Qt.Checked:
            self.left_bool = True
            self.jointnames.extend(self.l_arm_actuators)    
        else:
            self.left_bool = False 
            self.jointnames = [joint for joint in self.jointnames if joint not in self.l_arm_actuators]
            
        self.jointLabel.setText('Joints: ' + str(self.jointnames))
        self.jointLabel.setWordWrap(True)

    def right_arm_selected(self, state):
        if state == Qt.Checked:
            self.right_bool = True
            self.jointnames.extend(self.r_arm_actuators)    
        else:
            self.right_bool = False 
            self.jointnames = [joint for joint in self.jointnames if joint not in self.r_arm_actuators]
            
        self.jointLabel.setText('Joints: ' + str(self.jointnames))
        self.jointLabel.setWordWrap(True)
         
    def head_selected(self, state):
        if state == Qt.Checked:
            self.head_bool = True 
            self.jointnames.extend(self.head_actuators)    
        else:
            self.head_bool = False 
            self.jointnames = [joint for joint in self.jointnames if joint not in self.head_actuators]
            
        self.jointLabel.setText('Joints: ' + str(self.jointnames))
        self.jointLabel.setWordWrap(True)
    
    def debug_info_selected(self, state):
        if state == Qt.Checked:
            self.debug_info_bool = True
        else:
            self.debug_info_bool = False

    def bad_results_selected(self, state):
        if state == Qt.Checked:
            self.bad_results_bool = True
        else:
            self.bad_results_bool = False        

    def reset_jointlist(self):
        self.left_bool = False
        self.right_bool = False
        self.head_bool = False
        self.leftArm.setChecked(False)
        self.rightArm.setChecked(False)
        self.head.setChecked(False)

        self.joint_list = []
        self.jointnames = []

        self.jointLabel.setText('Joints: ' + str(self.jointnames))
        self.jointLabel.setWordWrap(True)

    def reset_filelist(self):
        self.filelist = []
        self.filenames = []
        self.fileLabel.setText('Files: ')
        for fig_name in self.plots: 
            plt.close()
        self.plots = []

    def get_data_pressed(self):
        self.joint_list = []
        self.jointLabel.setText('Joints: ' + str(self.jointnames))
        self.jointLabel.setWordWrap(True)
        self.joint_list.extend(self.jointnames)
        folder = str(datetime.datetime.now())
        folder = folder.split(" ")
        folder = "_".join(folder)

        os.system("mkdir %s" %(folder))
        os.chdir(os.getcwd() + '/' + folder)

        switch_controller([],['diagnostic_controller'],SwitchControllerRequest.STRICT)
        unload_controller('diagnostic_controller')

        for actuator_name in self.joint_list:
            print("Press X to start or for next joint")
            wait_for_X()
            start_diag_controller(actuator_name)
            print("start moving %s for about 5 seconds" %(actuator_name))
            rospy.sleep(5.0)
            print("press circle when your done")
            #rospy.loginfo("move %s for about 5 seconds and then press circle when done", joint_name)
            wait_for_circle()
            get_diag_data(actuator_name)
        print("Finshed getting data")

    def close_all(self):
        for fig_name in self.plots: 
            plt.close()
        self.plots = []

    def analyze_data(self):
        diagnostic = Diagnostic()
        debug_info = self.debug_info_bool
        bad_results = self.bad_results_bool

        for filename in self.filelist:
            actuator_name  = os.path.basename(filename) 
            if debug_info:
                print("\n" + str(actuator_name))
            stream = file(filename, 'r')
            samples = load(stream)
            velocity = []
            encoder_position = []
            supply_voltage = []
            measured_motor_voltage = []
            executed_current = []
            measured_current = []
            timestamp = []

            for s in samples.sample_buffer:
                velocity.append(s.velocity)
                encoder_position.append(s.encoder_position)
                supply_voltage.append(s.supply_voltage)
                measured_motor_voltage.append(s.measured_motor_voltage)
                executed_current.append(s.executed_current)
                measured_current.append(s.measured_current)
                timestamp.append(s.timestamp)

            velocity = numpy.array(velocity)
            encoder_position = numpy.array(encoder_position)
           
            supply_voltage = numpy.array(supply_voltage)
            measured_motor_voltage = numpy.array(measured_motor_voltage)
            executed_current = numpy.array(executed_current)
            measured_current = numpy.array(measured_current)

            acceleration = diagnostic.get_acceleration(velocity, timestamp)

            spikes = acceleration * (velocity[:-1])

            (result1, outlier_limit_neg, outlier_limit_pos) = diagnostic.check_for_spikes(spikes, actuator_name, debug_info)
            result2 = diagnostic.check_for_unplugged(velocity, measured_motor_voltage, actuator_name, debug_info)
            result3 = diagnostic.check_for_open(velocity, measured_motor_voltage, actuator_name, debug_info)
            result = result1 or result2 or result3

            if bad_results:
                if result: 
                    param = (actuator_name,velocity,spikes,acceleration,outlier_limit_neg,outlier_limit_pos,supply_voltage,measured_motor_voltage,executed_current,measured_current, result1, result2, result3)
                    self.plots.append(actuator_name +'_1')
                    self.plots.append(actuator_name +'_2')
                    diagnostic.plot(param)
            else:
                param = (actuator_name,velocity,spikes,acceleration,outlier_limit_neg,outlier_limit_pos,supply_voltage,measured_motor_voltage,executed_current,measured_current, result1, result2, result3)
                self.plots.append(actuator_name +'_1')
                self.plots.append(actuator_name +'_2')
                diagnostic.plot(param)

        plt.show()    

    def load_file(self):
        filename = QFileDialog.getOpenFileName(self, self.tr("Open File"), "../", self.tr("Yaml (*.yaml)"))
        if os.path.basename(filename[0].encode("ascii")) == "":
            return

        self.filelist.append(filename[0])
        self.filenames.append(os.path.basename(filename[0].encode("ascii")))
        self.fileLabel.setText('Files: ' + str(self.filenames))
        self.fileLabel.setWordWrap(True)        
 
    def load_directory(self):
        directory = QFileDialog.getExistingDirectory(self, self.tr("Open Directory"), "../")
        if directory == '': 
            return 

        temp = [f.encode("ascii") for f in os.listdir(directory)]
        self.filenames.extend(temp)
        temp = [directory + '/' + filepath for filepath in os.listdir(directory)]
        self.filelist.extend(temp)
        self.fileLabel.setText('Files: ' + str(self.filenames))
        self.fileLabel.setWordWrap(True)        
