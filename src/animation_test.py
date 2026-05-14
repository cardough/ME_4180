import numpy as np

import matplotlib
matplotlib.use("QtAgg")
import matplotlib.pyplot as plt
plt.show = lambda *args, **kwargs: None

from PySide6.QtCore import Qt, QTimer
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
    # Hard-coded trajectory: sinusoidal joint motion, 200 frames at 50 Hz → 4 s loop
    _ANIM_STEPS = 200
    _ANIM_DT_MS = 20          # 20 ms → 50 Hz, matches PyPlot default dt=0.05 s

    def __init__(self, parent=None, dh_table=[[0, 0, 0, 0]]):
        self.backend = PyPlot()
        self.backend.launch()
        self.fig = self.backend.fig
        super().__init__(self.fig)

        self.dh_table = dh_table
        self.n_links = len(dh_table)
        self.q = [0.0] * self.n_links

        bot_desc = []
        for row in dh_table:
            bot_desc.append(rtb.RevoluteMDH(alpha=row[0], a=row[1], d=row[2]))

        self.robot = rtb.DHRobot(bot_desc, name="robot")
        self.backend.add(self.robot)
        self.backend.step()
        self.ax = self.backend.ax

        self.trajectory = None
        self._anim_traj = None   # (N, n_links) array of joint configs
        self._anim_frame = 0

        self.update_trajectory(10)
        self._build_anim_trajectory()

        # QTimer drives the animation; no plt.pause() involved
        self._timer = QTimer(self)
        self._timer.setInterval(self._ANIM_DT_MS)
        self._timer.timeout.connect(self._anim_step)
        self._timer.start()

    # ------------------------------------------------------------------
    # Animation
    # ------------------------------------------------------------------

    def _build_anim_trajectory(self):
        """
        Hard-coded sinusoidal sweep across all joints.
        Each joint i oscillates with amplitude pi/3 and a phase offset.
        """
        t = np.linspace(0, 2 * np.pi, self._ANIM_STEPS, endpoint=False)
        traj = np.zeros((self._ANIM_STEPS, self.n_links))
        for i in range(self.n_links):
            traj[:, i] = (np.pi / 3) * np.sin(t + i * np.pi / self.n_links)
        self._anim_traj = traj

    def _anim_step(self):
        """Advance one frame: set robot.q, redraw via RobotPlot.draw(), update timer text."""
        q = self._anim_traj[self._anim_frame]
        self.robot.q = q

        # Mirror what PyPlot.step() does, minus plt.pause()
        for rpl in self.backend.robots:
            rpl.draw()

        self.backend._set_axes_equal()

        sim_time = self._anim_frame * self._ANIM_DT_MS / 1000.0
        self.backend.timer.set_text(f"t = {sim_time:.2f}")

        self.fig.canvas.draw_idle()

        self._anim_frame = (self._anim_frame + 1) % self._ANIM_STEPS

    # ------------------------------------------------------------------
    # Trajectory overlay (unchanged logic, kept for external callers)
    # ------------------------------------------------------------------

    def update_trajectory(self, number_steps):
        if self.trajectory is not None:
            self.trajectory.remove()

        angle_ranges = np.array([
            np.linspace(0, q, number_steps) for q in self.q
        ]).T

        pts = np.array([self.robot.fkine(q).t for q in angle_ranges])
        self.trajectory, = self.ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], 'b-')
        self.fig.canvas.draw_idle()

    def update_joints(self, value_rad, number_steps):
        self.q = value_rad
        self.robot.q = self.q
        self.backend.step()
        self.update_trajectory(number_steps)
        
        
if __name__ == "__main__":
    import sys
    from PySide6.QtWidgets import QApplication, QMainWindow
    
    app = QApplication(sys.argv)
    
    dh_table = [
        [0,      0,    0.333, 0],
        [-np.pi/2, 0,  0,     0],
        [ np.pi/2, 0,  0.316, 0],
    ]
    
    window = QMainWindow()
    canvas = RobotCanvas(parent=window, dh_table=dh_table)
    window.setCentralWidget(canvas)
    window.setWindowTitle("RobotCanvas Demo")
    window.resize(800, 600)
    window.show()
    
    sys.exit(app.exec())