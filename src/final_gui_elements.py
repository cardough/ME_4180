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

class FinalRobotCanvas(FigureCanvas):
    # Hard-coded trajectory: sinusoidal joint motion, 200 frames at 50 Hz → 4 s loop
    _ANIM_DT_MS = 20          # 20 ms → 50 Hz, matches PyPlot default dt=0.05 s
    _ANIM_STEPS = 200

    def __init__(self, parent=None, dh_table=[[0, 0, 0, 0]],
                 joint_traj=None):
        
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
        
        self.joint_traj = joint_traj
        self.backend.step()
        self.ax = self.backend.ax
        self.ax.view_init(elev=90, azim=-90)

        self.tip_trajectory = None
        self._anim_traj = None   # (N, n_links) array of joint configs
        self._anim_frame_number = 0

        self.build_anim_trajectory()

        # QTimer drives the animation; no plt.pause() involved
        self._timer = QTimer(self)
        self._timer.setInterval(10)
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
        
        zeros = np.zeros((np.array(self.joint_traj).shape[0], 1))
        self._anim_traj = np.column_stack((self.joint_traj, zeros)) 
        self._draw_tip_trajectory(100)

    def anim_step(self):
        """Advance one frame: set robot.q, redraw via RobotPlot.draw(), update timer text."""
        q = self._anim_traj[self._anim_frame_number]
        self.robot.q = q

        # Mirror what PyPlot.step() does, minus plt.pause()
        for rpl in self.backend.robots:
            rpl.draw()

        self.backend._set_axes_equal()

        sim_time = self._anim_frame_number * 10 / 1000.0
        self.backend.timer.set_text(f"t = {sim_time:.2f}")

        self.fig.canvas.draw_idle()

        self._anim_frame_number = (self._anim_frame_number + 1) % 100

    # ------------------------------------------------------------------
    # Trajectory overlay (unchanged logic, kept for external callers)
    # ------------------------------------------------------------------

    def _draw_tip_trajectory(self, n_steps):
        if self.tip_trajectory is not None:
            self.tip_trajectory.remove()

        angle_ranges = np.array([
            np.linspace(0, q, n_steps) for q in self.q_final
        ]).T


        pts = np.array([self.robot.fkine(q).t for q in self._anim_traj])
        #print(pts)
        self.tip_trajectory, = self.ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], 'b-')
        self.fig.canvas.draw_idle()

    def update_joints(self, value_rad, number_steps):
        self.q_final = value_rad
        self.robot.q = self.q_final
        self.backend.step()
        self._draw_tip_trajectory(number_steps)
        
        
   
class BotInfoPlots(QWidget):
    def __init__(
        self,
        timer: QTimer = None,
        parent=None,
        dh_table = None,
        joint_traj = list,
        joint_vels = list,
        joint_accels = list,
        joint_torques = list,
    ):
        # setup objects
        super().__init__(parent)
        self.timer = timer
        # self.timer.timeout.connect(self.update_frame)
        self.figure = Figure(tight_layout=True)
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.canvas.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.axes = self.figure.subplots(4, 1)
        
        #configure plot functions
        self.dh_table = dh_table
        
        self._artists = self.init_artists(self.axes, joint_traj, joint_vels, joint_accels, joint_torques)
        
        # setup  layout
        root = QVBoxLayout(self)
        root.addWidget(self.canvas)
        
        
        
    def init_artists(self, axes, joint_traj, joint_vels, joint_accels, joint_torques):
        
        self.t_series = np.linspace(0, 1, 100)
        points = []
        
        joint_traj = np.array(joint_traj) * (180/np.pi)
        joint_vels = np.array(joint_vels) * (180/np.pi)
        joint_accels = joint_accels * (180/np.pi)
        joint_torques = np.array(joint_torques)
        
        #print("joint accels: ", joint_accels)
        #print(len(joint_accels))
        
        for i in range(3):
            axes[0].set_ylabel("degrees")
            line, = axes[0].plot(self.t_series, joint_traj[:,i], label=f"Joint {i+1} angle")
            axes[1].set_ylabel("degrees/s")
            line, = axes[1].plot(self.t_series, joint_vels[:,i], label=f"Joint {i+1} velocity")
            axes[2].set_ylabel("degrees/s/s")
            line, = axes[2].plot(self.t_series, joint_accels[:,i], label=f"Joint {i+1} acceleration")
            axes[3].set_ylabel("N*m")
            line, = axes[3].plot(self.t_series, joint_torques[:,i], label=f"Joint {i+1} torques")
            
        for ax in axes:
            ax.legend()
            ax.set_xlabel("t (s)")
            
        return points           
            
    # def update_frame(self, frame):
    #     for point in self._artists:
    #         point.set_data(0, 1)
