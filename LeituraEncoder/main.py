import sys
import random
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QLabel, 
    QLineEdit, QComboBox, QTextEdit, QHBoxLayout, QGridLayout, QFrame, 
)
from PyQt6.QtCore import QTimer
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
from serial_service import SerialService  # Importando a classe SerialService
import serial  # Importando o módulo serial para capturar exceções
from PyQt6.QtCore import QThread, pyqtSignal
from PyQt6.QtGui import QColor
from PyQt6.QtCore import Qt
from graph_updater import GraphUpdater


class SerialReaderThread(QThread):
    data_received = pyqtSignal(str)  # Sinal para enviar dados para a UI

    def __init__(self, serial_connection):
        super().__init__()
        self.serial_connection = serial_connection
        self.running = True

    def run(self):
        """ Executa a leitura da serial continuamente em uma thread separada. """
        while self.running:
            if self.serial_connection and self.serial_connection.is_open:
                try:
                    line = self.serial_connection.readline().decode().strip()
                    if line:
                        self.data_received.emit(line)  # Dispara o sinal com os dados lidos
                except Exception as e:
                    self.data_received.emit(f"Erro: {str(e)}")
    
    def stop(self):
        """ Para a thread de leitura. """
        self.running = False
        self.wait()

class RobotControlApp(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Herbie - Control Panel")
        self.setGeometry(100, 100, 1600, 800)  # Aumenta a largura da janela

        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)

        # === Lado Esquerdo: Controle do Robô ===
        left_frame = QFrame()
        left_frame.setFrameShape(QFrame.Shape.Box)
        left_layout = QVBoxLayout(left_frame)
        # Seleção da Porta Serial
        self.port_label = QLabel("Porta Serial:")
        left_layout.addWidget(self.port_label)

        self.port_dropdown = QComboBox()
        SerialService.update_serial_ports(self.port_dropdown)  # Atualizando a lista de portas seriais
        left_layout.addWidget(self.port_dropdown)

        self.connect_button = QPushButton("Conectar")
        self.connect_button.clicked.connect(self.connect_serial)
        left_layout.addWidget(self.connect_button)

        # Terminal Serial
        self.terminal = QTextEdit()
        self.terminal.setReadOnly(True)
        left_layout.addWidget(self.terminal)

        # Envio de Comandos
        self.command_input = QLineEdit()
        left_layout.addWidget(self.command_input)

        self.send_button = QPushButton("Enviar")
        self.send_button.clicked.connect(self.send_command)
        left_layout.addWidget(self.send_button)

        # Botões de Controle
        button_grid = QGridLayout()
        self.control_buttons = []

        # Array de funções e comandos
        functions = [
            ("RUN", "RUN"),
            ("STOP", "STOP"),
            ("PRINT", "PRINT"),
            ("PRINT ENCODERS", "PRTENC"),
            ("KP:", "KP:"),
            ("KD:", "KD;"),
            ("KI:", "KI:"),
            ("LOC PWM", "KPWM:"),
            ("VENT PWM", "KPSUC:")
        ]

        for i, (name, command) in enumerate(functions):
            btn = QPushButton(name)
            btn.clicked.connect(lambda _, cmd=command: self.robot_action(cmd))
            self.control_buttons.append(btn)
            button_grid.addWidget(btn, i // 6, i % 6)  # Organiza em uma grade de 4 colunas

        left_layout.addLayout(button_grid)
        main_layout.addWidget(left_frame, 3)  # Proporção menor para o left_frame

        # === Lado Direito: Monitoramento do Robô ===
        right_frame = QFrame()
        right_frame.setFrameShape(QFrame.Shape.Box)
        right_layout = QVBoxLayout(right_frame)

        # Informações do Robô
        info_grid = QGridLayout()
        self.info_labels = []
        for i in range(6):
            label = QLabel(f"Info {i+1}: ---")
            self.info_labels.append(label)
            info_grid.addWidget(label, i // 3, i % 3)  # Organiza em duas colunas

        right_layout.addLayout(info_grid)

        # Gráficos
        self.figure1, self.ax1 = plt.subplots()
        self.canvas1 = FigureCanvas(self.figure1)
        right_layout.addWidget(self.canvas1)

        self.figure2, self.ax2 = plt.subplots()
        self.canvas2 = FigureCanvas(self.figure2)
        right_layout.addWidget(self.canvas2)

        main_layout.addWidget(right_frame, 3)  # Proporção maior para o right_frame

        # Timer para atualização dos gráficos
        self.graph_updater = GraphUpdater(self.canvas1, self.canvas2, self.ax1, self.ax2)
        self.graph_updater.data_updated.connect(self.update_info_labels)

        # Serial Connection
        self.serial_connection = None
        self.data_x = []
        self.data_y1 = []
        self.data_y2 = []

    def connect_serial(self):
        """ Conecta à porta serial selecionada. """
        try:
            self.serial_connection = SerialService.connect_serial(self.port_dropdown, self.connect_button)
            self.serial_thread = SerialReaderThread(self.serial_connection)
            self.serial_thread.data_received.connect(self.update_terminal)  # Conecta o sinal à função
            self.serial_thread.start()
        except serial.SerialException:
            self.connect_button.setText("Erro!")
            self.serial_connection = None
            
    def update_terminal(self, data):
        """ Atualiza o terminal com os dados recebidos da serial. """
        self.terminal.setTextColor(QColor(Qt.GlobalColor.green))
        self.terminal.append(f"Recebido: {data}")

    def send_command(self):
        """ Envia um comando para o robô via serial. """
        command = self.command_input.text()
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.write(f"{command}\n".encode())
            self.terminal.setTextColor(QColor(Qt.GlobalColor.blue))
            self.terminal.append(f"Enviado: {command}")

    def read_serial_response(self):
        """ Lê a resposta da conexão serial e atualiza o terminal. """
        SerialService.read_response(self.serial_connection, self.terminal)

    def robot_action(self, command):
        """ Envia um comando específico do botão clicado. """
        self.terminal.setTextColor(QColor(Qt.GlobalColor.blue))
        self.terminal.append(f"Executando: {command}")
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.write(f"{command}\n".encode())

    def update_info_labels(self, total_distance, min_radius, total_time, avg_speed):
        """ Atualiza as informações do robô na interface do usuário. """
        self.info_labels[0].setText(f"Distância Total: {total_distance:.2f} m")
        self.info_labels[1].setText(f"Menor Raio de Curva: {min_radius:.2f} cm")
        self.info_labels[2].setText(f"Tempo Total: {total_time:.2f} s")
        self.info_labels[3].setText(f"Velocidade Média: {(avg_speed/100):.2f} m/s")

    def closeEvent(self, event):
        """ Para a thread de atualização dos gráficos ao fechar a aplicação. """
        self.graph_updater.stop()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RobotControlApp()
    window.show()
    sys.exit(app.exec())