import gui_elements as gui
import jacobian_calcs
import sys



app = gui.QApplication(sys.argv)
window = gui.DataEntryApp()
window.show()
sys.exit(app.exec())