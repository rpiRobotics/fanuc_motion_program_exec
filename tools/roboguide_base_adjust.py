from PyQt5.QtCore import QDateTime,Qt,QTimer,QDir,QObject,QRunnable,pyqtSignal,pyqtSlot,QThread
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox, QDateTimeEdit,
        QDial, QDialog, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit,
        QProgressBar, QPushButton, QRadioButton, QScrollBar, QSizePolicy,
        QSlider, QSpinBox, QDoubleSpinBox, QStyleFactory, QTableWidget, QTabWidget, QTextEdit,
        QVBoxLayout, QWidget,QFileDialog, QMessageBox)
from PyQt5.QtGui import QPixmap, QFont
import sys,os,time,traceback,platform,subprocess

from general_robotics_toolbox import *
from fanuc_motion_program_exec_client import *

class BaseGUI(QDialog):
    def __init__(self, parent=None):
        super(BaseGUI,self).__init__(parent)
        self.bold=QFont('Arial', 12)
        self.bold.setBold(True)

        self.originalPalette = QApplication.palette()

        self.pose_box_name=['X','Y','Z','W','P','R']

        # R1 Base Box
        self.R1BaseBox=QGroupBox("R1 Base")
        self.R1BaseBox.setFont(self.bold)
        self.R1BaseBox.setStyleSheet("background-color: pink")
        # R1 Base Box Input
        self.r1boxes=[]
        Layout=QVBoxLayout()
        for i in range(2):
            Layout_1=QHBoxLayout()
            for j in range(3):
                base_box=QLineEdit()
                self.r1boxes.append(base_box)
                base_box_label=QLabel(self.pose_box_name[i*3+j])
                base_box_label.setFont(self.bold)
                base_box_label.setBuddy(self.r1boxes[-1])
                Layout_1.addWidget(base_box_label)
                Layout_1.addWidget(self.r1boxes[-1])
            Layout.addLayout(Layout_1)
        for i in range(2):
            Layout_1=QHBoxLayout()
            base_box=QLineEdit()
            self.r1boxes.append(base_box)
            base_box_label=QLabel("R"+str(i+1)+" Base Link Height")
            base_box_label.setFont(self.bold)
            base_box_label.setBuddy(self.r1boxes[-1])
            Layout_1.addWidget(base_box_label)
            Layout_1.addWidget(self.r1boxes[-1])
            Layout.addLayout(Layout_1)
        self.R1BaseBox.setLayout(Layout)

        # TP Box
        self.TPBox=QGroupBox("Teach Pendant")
        self.TPBox.setFont(self.bold)
        self.TPBox.setStyleSheet("background-color: lightgreen")
        self.tpboxes=[]
        Layout=QVBoxLayout()
        for i in range(2):
            Layout_1=QHBoxLayout()
            for j in range(3):
                base_box=QLineEdit()
                self.tpboxes.append(base_box)
                base_box_label=QLabel(self.pose_box_name[i*3+j])
                base_box_label.setFont(self.bold)
                base_box_label.setBuddy(self.tpboxes[-1])
                Layout_1.addWidget(base_box_label)
                Layout_1.addWidget(self.tpboxes[-1])
            Layout.addLayout(Layout_1)
        self.TPBox.setLayout(Layout)
        calbutton=QPushButton('Calculate')
        calbutton.setFont(self.bold)
        calbutton.setDefault(True)
        calbutton.clicked.connect(self.transform_base)
        Layout_1=QHBoxLayout()
        Layout_1.addWidget(calbutton)
        Layout.addLayout(Layout_1)
        self.TPBox.setLayout(Layout)

        # R2 Base Box
        self.R2BaseBox=QGroupBox("R2 Base")
        self.R2BaseBox.setFont(self.bold)
        self.R2BaseBox.setStyleSheet("background-color: lightblue")
        self.r2boxes=[]
        Layout=QVBoxLayout()
        for i in range(2):
            Layout_1=QHBoxLayout()
            for j in range(3):
                base_box=QLabel("  ")
                base_box.setFont(self.bold)
                self.r2boxes.append(base_box)
                base_box_label=QLabel(self.pose_box_name[i*3+j])
                base_box_label.setFont(self.bold)
                base_box_label.setBuddy(self.r2boxes[-1])
                Layout_1.addWidget(base_box_label)
                Layout_1.addWidget(self.r2boxes[-1])
            Layout.addLayout(Layout_1)
        self.R2BaseBox.setLayout(Layout)
        base_box=QLabel("  ")
        base_box.setFont(self.bold)
        self.r2boxes.append(base_box)
        Layout_1=QHBoxLayout()
        Layout_1.addWidget(self.r2boxes[-1])
        Layout.addLayout(Layout_1)
        self.R2BaseBox.setLayout(Layout)

        ## main layout
        self.mainLayout = QGridLayout()
        self.mainLayout.addWidget(self.R1BaseBox,0,0)
        self.mainLayout.addWidget(self.TPBox,0,1)
        self.mainLayout.addWidget(self.R2BaseBox,0,2)
        self.setLayout(self.mainLayout)

        self.setWindowTitle('ROBOGUIDE ROBOT BASE ADJUSTMENT')
        self.changeStyle('Windows')

    def changeStyle(self, styleName):
        QApplication.setStyle(QStyleFactory.create(styleName))
        self.changePalette()
    def changePalette(self):
        QApplication.setPalette(self.originalPalette)
    
    def transform_base(self):
        
        try:
            r1x = float(self.r1boxes[0].text())
            r1y = float(self.r1boxes[1].text())
            r1z = float(self.r1boxes[2].text())
            r1w = float(self.r1boxes[3].text())
            r1p = float(self.r1boxes[4].text())
            r1r = float(self.r1boxes[5].text())
            r1h = float(self.r1boxes[6].text())
            r2h = float(self.r1boxes[7].text())
            r1_T = Transform(wpr2R([r1w,r1p,r1r]),[r1x,r1y,r1z])*Transform(np.eye(3),[0,0,r1h])

            tpx = float(self.tpboxes[0].text())
            tpy = float(self.tpboxes[1].text())
            tpz = float(self.tpboxes[2].text())
            tpw = float(self.tpboxes[3].text())
            tpp = float(self.tpboxes[4].text())
            tpr = float(self.tpboxes[5].text())
            tp_T = Transform(wpr2R([tpw,tpp,tpr]),[tpx,tpy,tpz])

            r2_T = r1_T*tp_T*Transform(np.eye(3),[0,0,-r2h])
            wpr = R2wpr(r2_T.R)
            round_up = 4
            self.r2boxes[0].setText(str(round(r2_T.p[0],round_up)))
            self.r2boxes[1].setText(str(round(r2_T.p[1],round_up)))
            self.r2boxes[2].setText(str(round(r2_T.p[2],round_up)))
            self.r2boxes[3].setText(str(round(wpr[0],round_up)))
            self.r2boxes[4].setText(str(round(wpr[1],round_up)))
            self.r2boxes[5].setText(str(round(wpr[2],round_up)))
        except:
            self.r2boxes[-1].setText("Please enter numbers.")

if __name__ == '__main__':

    app=QApplication(sys.argv)

    window=BaseGUI()
    window.show()
    sys.exit(app.exec())