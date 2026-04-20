import sys
import matplotlib
matplotlib.use("QtAgg")

from PySide6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout
import roboticstoolbox as rtb

from roboticstoolbox.backends.PyPlot import PyPlot
from matplotlib.backends.backend_qtagg import FigureCanvas
from matplotlib.figure import Figure

class RobotCanvas(FigureCanvas):
    def __init__(self, parent=None):
        # Let backend create its own managed figure
        self.backend = PyPlot()
        self.backend.launch()  # creates fig WITH manager

        # Extract that figure
        self.fig = self.backend.fig

        # Now wrap it into Qt canvas
        super().__init__(self.fig)

        # Add robot
        import roboticstoolbox as rtb
        self.robot = rtb.models.DH.Panda()
        self.backend.add(self.robot)

        self.robot.q = self.robot.qz
        self.backend.step()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        central = QWidget()
        layout = QVBoxLayout(central)

        self.canvas = RobotCanvas(self)
        layout.addWidget(self.canvas)

        self.setCentralWidget(central)
        self.setWindowTitle("Robotics Toolbox in PySide6")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = MainWindow()
    w.resize(800, 600)
    w.show()
    sys.exit(app.exec())