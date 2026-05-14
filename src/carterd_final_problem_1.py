import sys

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QHBoxLayout,
)
from final_gui_elements import FinalRobotCanvas, BotInfoPlots
from jacobian_calcs import velocity_jacobian
from dynamics_calcs import lagrangian
from sympy.physics.mechanics import dynamicsymbols
from sympy.physics.vector import vprint
from sympy import Matrix, matrix2numpy, zeros
import numpy as np



class FinalProjectWindow(QMainWindow):
    def __init__(self, dh_table=None):
        super().__init__()

        central = QWidget()
        layout = QHBoxLayout(central)
        
        joint_traj, joint_vels, joint_accels, joint_torques = self.generate_data(dh_table)

        self.canvas = FinalRobotCanvas(self, dh_table=dh_table, joint_traj=joint_traj)
        #self.controls = ControlPanel(self.canvas)
        self.plots = BotInfoPlots(self, dh_table=dh_table, joint_traj=joint_traj, joint_vels=joint_vels, joint_accels=joint_accels, joint_torques=joint_torques)

        layout.addWidget(self.canvas, 4)
        layout.addWidget(self.plots, 3)

        self.setCentralWidget(central)
        self.setWindowTitle("Carter Dougherty ME 4180 Final Programming Assignment")
        
    def generate_data(self, dh_table):
        
        theta1, theta2, theta3 = dynamicsymbols("theta1, theta2, theta3")
        joint_vars = Matrix([theta1, theta2, theta3])
        dh_table[0][3] = theta1
        dh_table[1][3] = theta2
        dh_table[2][3] = theta3
        dh_matrix = Matrix(dh_table)
        J = velocity_jacobian(dh_matrix, joint_vars)
        
        v_h = -np.array([[0.1], [0.3], [0]])
        q = np.array([10, 50, 120]) * (np.pi/180)
        
        dt = 0.01 # seconds
        
        joint_traj = []
        joint_vels = []
        
        for i in range(100):
            J_numbers = J.subs({theta1: q[0], theta2: q[1], theta3: q[2]})
            J_numbers = matrix2numpy(J_numbers, dtype=float)
            J_inv = np.linalg.pinv(J_numbers)
            q_dot = J_inv @ v_h
            
            
            q += q_dot.T[0] * dt
            joint_traj.append(list(q))
            
            joint_vels.append(list(q_dot.T[0]))
            
        joint_accels = np.gradient(joint_vels, dt, axis=0)
        
        masses = Matrix([0, 10, 7, 5])
        P_cs = [Matrix([0,   0,  0]),
                Matrix([0,   4,  0]),
                Matrix([0,   3,  0]),
                Matrix([0,   2,  0]),
                ]
        Is = [zeros(3) for _ in range(dh_matrix.shape[0])]
        taus = lagrangian(dh_matrix, masses, Is, P_cs, joint_vars)
        
        qd1 = theta1.diff()
        qd2 = theta2.diff()
        qd3 = theta3.diff()
        qdd1 = theta1.diff().diff()
        qdd2 = theta2.diff().diff()
        qdd3 = theta3.diff().diff()
        
        joint_torques = []
        for i in range(100):
            print(f"torque iteration {i}")
            q = joint_traj[i]
            q_d = joint_vels[i]
            q_dd = joint_vels[i]

            tor = taus.subs({theta1: q[0], theta2: q[1], theta3: q[2],
                             qd1: q_d[0], qd2: q_d[1], qd3: q_d[2],
                             qdd1: q_dd[0], qdd2: q_dd[1], qdd3: q_dd[2],
                             })
            joint_torques.append(list(tor))
            
        return joint_traj, joint_vels, joint_accels, joint_torques
        
        
if __name__ == "__main__":
    
    dh_table = [
        [0,     0,      0,      1],
        [0,     4,      0,      1],
        [0,     3,      0,      1],
        [0,     2,      0,      0],
    ]
    
    app = QApplication(sys.argv)
    w = FinalProjectWindow(dh_table=dh_table)
    w.resize(1400, 800)
    w.show()
    sys.exit(app.exec())