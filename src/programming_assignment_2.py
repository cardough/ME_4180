import sys
import numpy as np
import matplotlib
matplotlib.use("QtAgg")
import matplotlib.pyplot as plt
plt.show = lambda *args, **kwargs: None     # prevents pyplot another window from appearing

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QHBoxLayout, QFrame,
    QSlider, QPushButton, QLineEdit, QLabel,
    QFormLayout
)
from PySide6.QtCore import Qt

from matplotlib.backends.backend_qtagg import FigureCanvas
from roboticstoolbox.backends.PyPlot import PyPlot
import roboticstoolbox as rtb
from math import pi


class RobotCanvas(FigureCanvas):
    def __init__(self, parent=None):
        self.backend = PyPlot()
        self.backend.launch()
        self.fig = self.backend.fig
        super().__init__(self.fig)

        # DH table; robot is currently hardcoded with arbitrary link lengths
        L1 = rtb.RevoluteMDH(alpha=0,      a=0,     d=0)
        L2 = rtb.RevoluteMDH(alpha=pi/2,   a=.4,     d=0)
        L3 = rtb.RevoluteMDH(alpha=0,      a=.3,     d=0)
        L4 = rtb.RevoluteMDH(alpha=0,      a=.2,     d=0)

        self.robot = rtb.DHRobot([L1, L2, L3, L4], name="robot")
        self.backend.add(self.robot)

        self.q = [0, 0, 0, 0]
        self.backend.step()

        self.ax = self.backend.ax
        self.trajectory = None
        self.update_trajectory(10)

    def update_trajectory(self, number_steps):
        # Remove old line if it exists
        if self.trajectory is not None:
            self.trajectory.remove()

        # linearly interpolate to find range of joint angles
        angle_ranges = []
        for angle in self.q:
            angle_ranges.append(np.linspace(0, angle, number_steps))
        angle_ranges = np.array(angle_ranges).T

        # use forward kinematics to find trajectory shape
        x, y, z = [0 for _ in range(number_steps)], [0 for _ in range(number_steps)], [0 for _ in range(number_steps)]
        for i, q in enumerate(angle_ranges):
            pos = self.robot.fkine(q).t
            x[i] = pos[0]
            y[i] = pos[1]
            z[i] = pos[2]

        self.trajectory, = self.ax.plot(x, y, z, 'b-')
        self.fig.canvas.draw_idle()

    def update_joints(self, value_rad, number_steps):
        self.q = value_rad
        self.robot.q = self.q

        # re-draw robot
        self.backend.step()

        # Re-plot custom elements AFTER step
        self.update_trajectory(number_steps)
        

class JointSlider(QFrame):
    def __init__(self, joint_index):
        super().__init__()
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet("""
            QFrame {
                background-color: #262626;
                border-radius: 5px;
            }
        """)
        
        main_layout = QVBoxLayout(self)
        

        text_entry_layout = QFormLayout()
        self.joint_val = QLineEdit("0.0")
        text_entry_layout.addRow(f"Joint {joint_index} (rad):", self.joint_val)
        main_layout.addLayout(text_entry_layout)

        self.joint_slider = QSlider(Qt.Horizontal)
        self.joint_slider.setMinimum(-314)   # ~ -pi
        self.joint_slider.setMaximum(314)    # ~ pi
        self.joint_slider.setValue(0)
        self.joint_slider.valueChanged.connect(self.sync_text_box)
        main_layout.addWidget(self.joint_slider)
        
    
    def sync_text_box(self, value):
        rad = value / 100.0
        self.joint_val.setText(f"{rad:.3f}")

    def get_joint_val(self):
        try:
            # retrieve and convert angle value
            val = float(self.joint_val.text())
        except:
            raise ValueError
        
        # clamp to valid range
        return max(-np.pi, min(np.pi, val))



class ControlPanel(QWidget):
    def __init__(self, canvas: RobotCanvas):
        super().__init__()
        self.canvas = canvas

        layout = QVBoxLayout(self)
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)

        layout.addWidget(QLabel("Number of Timesteps:"))
        self.n_steps = QLineEdit("10")
        layout.addWidget(self.n_steps)

        self.joint_controls = [None for _ in range(3)]
        for i in range(3):
            self.joint_controls[i] = JointSlider(i)
            layout.addWidget(self.joint_controls[i])


        self.button = QPushButton("Update")
        layout.addWidget(self.button)

        # make the update button re-render the canvas
        self.button.clicked.connect(self.apply_update)

    def apply_update(self):
        joint_values = [self.joint_controls[i].get_joint_val() for i in range(3)]
        joint_values.append(0)
        self.canvas.update_joints(joint_values, int(self.n_steps.text()))


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        central = QWidget()
        layout = QHBoxLayout(central)

        self.canvas = RobotCanvas(self)
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