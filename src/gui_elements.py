import numpy as np

import matplotlib
matplotlib.use("QtAgg")
import matplotlib.pyplot as plt
plt.show = lambda *args, **kwargs: None     # prevents another pyplot window from appearing

from PySide6.QtCore import (Qt, QSignalBlocker, QTimer)
from PySide6.QtGui import QDoubleValidator
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QFrame,
    QSlider, QPushButton, QLineEdit, QLabel,
    QFormLayout, QSizePolicy,
)

from matplotlib.backends.backend_qtagg import FigureCanvas
from matplotlib.figure import Figure
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from roboticstoolbox.backends.PyPlot import PyPlot
import roboticstoolbox as rtb

class RobotCanvas(FigureCanvas):
    # Hard-coded trajectory: sinusoidal joint motion, 200 frames at 50 Hz → 4 s loop
    _ANIM_DT_MS = 20          # 20 ms → 50 Hz, matches PyPlot default dt=0.05 s
    _ANIM_STEPS = 200

    def __init__(self, parent=None, dh_table=[[0, 0, 0, 0]]):
        
        # initialize pyplot backend
        self.backend = PyPlot()
        self.backend.launch()
        self.fig = self.backend.fig
        super().__init__(self.fig)

        # built rtb robot
        self.dh_table = dh_table
        self.n_links = len(dh_table)
        self.q_final = [0.0] * self.n_links
        bot_desc = []
        for row in dh_table:
            bot_desc.append(rtb.RevoluteMDH(alpha=row[0], a=row[1], d=row[2]))

        self.robot = rtb.DHRobot(bot_desc, name="robot")
        self.backend.add(self.robot)
        self.backend.step()
        self.ax = self.backend.ax

        self.tip_trajectory = None
        self._anim_traj = None   # (N, n_links) array of joint configs
        self._anim_frame_number = 0

        self.build_anim_trajectory()

        # QTimer drives the animation; no plt.pause() involved
        self._timer = QTimer(self)
        self._timer.setInterval(self._ANIM_DT_MS)
        self._timer.timeout.connect(self.anim_step)
        self._timer.start()

    # ------------------------------------------------------------------
    # Animation
    # ------------------------------------------------------------------

    def build_anim_trajectory(self):
        # saves a given trajectoy 
        t = np.linspace(0, 2 * np.pi, self._ANIM_STEPS, endpoint=False)
        traj = np.zeros((self._ANIM_STEPS, self.n_links))
        for i in range(self.n_links):
            traj[:, i] = (np.pi / 3) * np.sin(t + i * np.pi / self.n_links)
        print(traj)
        self._anim_traj = traj
        self._draw_tip_trajectory(self._ANIM_STEPS)

    def anim_step(self):
        """Advance one frame: set robot.q, redraw via RobotPlot.draw(), update timer text."""
        q = self._anim_traj[self._anim_frame_number]
        self.robot.q = q

        # Mirror what PyPlot.step() does, minus plt.pause()
        for rpl in self.backend.robots:
            rpl.draw()

        self.backend._set_axes_equal()

        sim_time = self._anim_frame_number * self._ANIM_DT_MS / 1000.0
        self.backend.timer.set_text(f"t = {sim_time:.2f}")

        self.fig.canvas.draw_idle()

        self._anim_frame_number = (self._anim_frame_number + 1) % self._ANIM_STEPS

    # ------------------------------------------------------------------
    # Trajectory overlay (unchanged logic, kept for external callers)
    # ------------------------------------------------------------------

    def _draw_tip_trajectory(self, n_steps):
        if self.tip_trajectory is not None:
            self.tip_trajectory.remove()

        angle_ranges = np.array([
            np.linspace(0, q, n_steps) for q in self.q_final
        ]).T

        pts = np.array([self.robot.fkine(q).t for q in angle_ranges])
        print(pts)
        self.tip_trajectory, = self.ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], 'b-')
        self.fig.canvas.draw_idle()

    def update_joints(self, value_rad, number_steps):
        self.q_final = value_rad
        self.robot.q = self.q_final
        self.backend.step()
        self._draw_tip_trajectory(number_steps)
    

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
       
        

class AssignmentTwoControlPanel(QWidget):
    def __init__(self, canvas: RobotCanvas):
        super().__init__()
        self.canvas = canvas

        layout = QVBoxLayout(self)
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)

        layout.addWidget(QLabel("Number of Timesteps:"))
        self.n_steps = QLineEdit("20")
        layout.addWidget(self.n_steps)

        # Only creates a joint control slider if the joint is a revolute variable
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
