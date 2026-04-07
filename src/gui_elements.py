import numpy as np

import matplotlib
matplotlib.use("QtAgg")
import matplotlib.pyplot as plt
plt.show = lambda *args, **kwargs: None     # prevents pyplot another window from appearing

from PySide6.QtCore import (Qt, QSignalBlocker)
from PySide6.QtGui import QDoubleValidator
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QFrame,
    QSlider, QPushButton, QLineEdit, QLabel,
    QFormLayout,
)


from matplotlib.backends.backend_qtagg import FigureCanvas
from roboticstoolbox.backends.PyPlot import PyPlot
import roboticstoolbox as rtb

class RobotCanvas(FigureCanvas):
    def __init__(self, parent=None, dh_table=[[0, 0, 0, 0]]):

        # set up the plotting backend
        self.backend = PyPlot()
        self.backend.launch()
        self.fig = self.backend.fig
        super().__init__(self.fig)

        # create rtb robot from given dh table with self.q for joint controls
        self.dh_table = dh_table
        self.n_links = len(dh_table)
        self.q = [0 for _ in range(self.n_links)]
        bot_desc = []
        for row in dh_table:
            link = rtb.RevoluteMDH(alpha=row[0],    a=row[1],   d=row[2])
            bot_desc.append(link)

        self.robot = rtb.DHRobot(bot_desc, name="robot")
        self.backend.add(self.robot)

        # step backend and pull out axes to use for plotting custom stuff
        self.backend.step()
        self.ax = self.backend.ax
        # initialize trajectory
        self.trajectory = None
        self.update_trajectory(10)

    def update_trajectory(self, number_steps):
        # Remove old trajectory line if it exists
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

        # plot and update canvas
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

    # modular element to control a singular joint

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
        self.joint_id = joint_index

        #create and add text box w/ label
        text_entry_layout = QFormLayout()
        self.joint_val = QLineEdit("0.0") # stores the actual joint angle input
        validator = QDoubleValidator(-3.14, 3.14, 3)
        #validator.setNotation(QDoubleValidator.Notation.StandardNotation)
        self.joint_val.setValidator(validator)
        text_entry_layout.addRow(f"Joint {joint_index} (rad):", self.joint_val)
        main_layout.addLayout(text_entry_layout)

        # create and add slider
        self.joint_slider = QSlider(Qt.Horizontal)
        self.joint_slider.setMinimum(-314)   # ~ -pi
        self.joint_slider.setMaximum(314)    # ~ pi
        self.joint_slider.setValue(0)
        main_layout.addWidget(self.joint_slider)

        # sync text box and slider
        self.joint_slider.valueChanged.connect(self.sync_text_box)
        self.joint_val.textChanged.connect(self.sync_slider)

    def sync_text_box(self, value):
        rad = value / 100.0
        with QSignalBlocker(self.joint_val):
            self.joint_val.setText(f"{rad:.3f}")

    def sync_slider(self, text):
        try:
            val = int(float(text) * 100)
        except:
            val = 0

        if val > 314:
            val = 314
        elif val < -314:
            val = 314

        with QSignalBlocker(self.joint_slider):
            self.joint_slider.setValue(val)

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
        self.n_steps = QLineEdit("20")
        layout.addWidget(self.n_steps)

        # Only creates a joint control slider if the joint has a revolute variable
        self.joint_controls = []
        for i, row in enumerate(self.canvas.dh_table):
            if row[3] != 0:
                self.joint_controls.append(JointSlider(i))
                layout.addWidget(self.joint_controls[-1])

        # "update" button that re-renders the canvas
        self.button = QPushButton("Update")
        layout.addWidget(self.button)
        self.button.clicked.connect(self.apply_update)

    def apply_update(self):
        joint_values = [0 for _ in range(self.canvas.n_links)]
        for control in self.joint_controls: 
            joint_values[control.joint_id] = control.get_joint_val()
        self.canvas.update_joints(joint_values, int(self.n_steps.text()))
