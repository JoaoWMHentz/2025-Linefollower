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
        self.setGeometry(100, 100, 1000, 600)

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
        for i in range(2):  # 2 fileiras
            for j in range(8):  # 8 botões por fileira
                btn = QPushButton(f"Função {i*8 + j + 1}")
                btn.clicked.connect(self.robot_action)
                self.control_buttons.append(btn)
                button_grid.addWidget(btn, i, j)

        left_layout.addLayout(button_grid)
        main_layout.addWidget(left_frame)

        # === Lado Direito: Monitoramento do Robô ===
        right_frame = QFrame()
        right_frame.setFrameShape(QFrame.Shape.Box)
        right_layout = QVBoxLayout(right_frame)

        # Informações do Robô
        self.info_labels = []
        for i in range(6):
            label = QLabel(f"Info {i+1}: ---")
            self.info_labels.append(label)
            right_layout.addWidget(label)

        # Gráficos
        self.figure1, self.ax1 = plt.subplots()
        self.canvas1 = FigureCanvas(self.figure1)
        right_layout.addWidget(self.canvas1)

        self.figure2, self.ax2 = plt.subplots()
        self.canvas2 = FigureCanvas(self.figure2)
        right_layout.addWidget(self.canvas2)

        main_layout.addWidget(right_frame)

        # Timer para atualização dos gráficos
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_graphs)
        self.timer.start(1000)

       

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

    def robot_action(self):
        """ Envia um comando específico do botão clicado. """
        sender = self.sender()
        command = sender.text().replace("Função ", "CMD_")
        self.terminal.setTextColor(QColor(Qt.GlobalColor.blue))
        self.terminal.append(f"Executando: {command}")
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.write(f"{command}\n".encode())

    def update_graphs(self):
        """ Atualiza os gráficos com dados simulados. """
        if len(self.data_x) > 20:
            self.data_x.pop(0)
            self.data_y1.pop(0)
            self.data_y2.pop(0)

        self.data_x.append(len(self.data_x))
        self.data_y1.append(random.uniform(0, 10))
        self.data_y2.append(random.uniform(5, 15))

        self.ax1.clear()
        self.ax1.plot(self.data_x, self.data_y1, marker='o', linestyle='-', color='b')
        self.ax1.set_title("Gráfico 1")

        self.ax2.clear()
        self.ax2.plot(self.data_x, self.data_y2, marker='o', linestyle='-', color='r')
        self.ax2.set_title("Gráfico 2")

        self.canvas1.draw()
        self.canvas2.draw()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RobotControlApp()
    window.show()
    sys.exit(app.exec())