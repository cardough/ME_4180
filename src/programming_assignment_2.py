import sys

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QHBoxLayout,
)


from math import pi

from gui_elements import RobotCanvas, ControlPanel

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        central = QWidget()
        layout = QHBoxLayout(central)

        D_H_table = [
            [0,     0,      0,      1],
            [pi/2,  .2,     0,      1],
            [0,     .3,     0,      1],
            [0,     .4,     0,      0],
        ]

        self.canvas = RobotCanvas(self, dh_table=D_H_table)
        self.controls = ControlPanel(self.canvas)

        layout.addWidget(self.controls, 1)
        layout.addWidget(self.canvas, 4)

        self.setCentralWidget(central)
        self.setWindowTitle("Carter Dougherty Programming Assignment 2")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = MainWindow()
    w.resize(1000, 600)
    w.show()
    sys.exit(app.exec())