from PySide6.QtWidgets import (QApplication, QWidget, QFormLayout, 
                             QLineEdit, QPushButton, QVBoxLayout, QSlider,
                             QLabel)
from PySide6.QtCore import Qt

class DataEntryApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Simple Data Entry")
        self.resize(300, 500)

        # Create Layouts
        self.main_layout = QVBoxLayout()
        self.form_layout = QFormLayout()

        # Input Widgets
        self.name_input = QLineEdit()
        self.age_input = QLineEdit()

        # Add to Form
        self.form_layout.addRow("Name:", self.name_input)
        self.form_layout.addRow("Age:", self.age_input)

        # Submit Button
        self.submit_btn = QPushButton("Submit")
        self.submit_btn.clicked.connect(self.handle_submit)

        # Assemble UI
        self.main_layout.addLayout(self.form_layout)
        self.main_layout.addWidget(self.submit_btn)
        self.setLayout(self.main_layout)

        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(0, 100)
        self.label = QLabel("Value: 0")
        
        # Connect signal to update label
        self.slider.valueChanged.connect(lambda v: self.label.setText(f"Value: {v}"))
        
        self.main_layout.addWidget(self.slider)
        self.main_layout.addWidget(self.label)

    def handle_submit(self):
        name = self.name_input.text()
        age = self.age_input.text()
        print(f"Data Received - Name: {name}, Age: {age}")
        # Clear fields after submission
        self.name_input.clear()
        self.age_input.clear()

