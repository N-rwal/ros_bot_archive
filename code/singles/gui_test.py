import sys
from PyQt5.QtWidgets import QWidget, QHBoxLayout, QPushButton, QApplication
from PyQt5.QtGui     import QIcon
from PyQt5.QtCore    import pyqtSlot, QSize


class Window(QWidget):

    def __init__(self, *args, **kwargs):
        super(Window, self).__init__(*args, **kwargs)

        self.setGeometry(800, 65, 500, 200)

        layout = QHBoxLayout(self)
        layout.addWidget(QPushButton("red button", self,
                                     objectName="RedButton", minimumHeight=48))
        # ----------------------------------------------------------------------
        button = QPushButton('  \n   PyQt5\n   button\n  ', self, 
                                     objectName="GreenButton", minimumHeight=48)
        button.setIcon(QIcon("E:/_Qt/img/qt-logo.png"))
        button.setIconSize(QSize(48, 48))                                     
        layout.addWidget(button) 
        # ----------------------------------------------------------------------

        layout.addWidget(QPushButton("blue button", self,
                                     objectName="BlueButton", minimumHeight=48))
        layout.addWidget(QPushButton("orange button", self,
                                     objectName="OrangeButton", minimumHeight=48))
        layout.addWidget(QPushButton("purple button", self,
                                     objectName="PurpleButton", minimumHeight=48))


StyleSheet = '''
QPushButton {
    border: none;
}
QPushButton#RedButton {
    background-color: #f44336;
}
#RedButton:hover {
    background-color: #e57373; 
    color: #fff;
}
#RedButton:pressed { 
    background-color: #ffcdd2; 
}
#GreenButton {
    background-color: #4caf50;
    border-radius: 5px;       
}
#GreenButton:hover {
    background-color: #81c784;
    color: #fff;              
}
#GreenButton:pressed {
    background-color: #c8e6c9;
}
#BlueButton {
    background-color: #2196f3;
    min-width:  96px;
    max-width:  96px;
    min-height: 96px;
    max-height: 96px;
    border-radius: 48px;        
}
#BlueButton:hover {
    background-color: #64b5f6;
}
#BlueButton:pressed {
    background-color: #bbdefb;
}
#OrangeButton {
    max-height: 48px;
    border-top-right-radius:   20px;   
    border-bottom-left-radius: 20px;   
    background-color: #ff9800;
}
#OrangeButton:hover {
    background-color: #ffb74d;
}
#OrangeButton:pressed {
    background-color: #ffe0b2;
}

QPushButton[text="purple button"] {
    color: white;                    
    background-color: #9c27b0;
}
'''

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyleSheet(StyleSheet)
    w = Window()
    w.setWindowTitle("Demo color-button")
    w.show()
    sys.exit(app.exec_())