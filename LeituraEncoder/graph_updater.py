import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from PyQt6.QtCore import QTimer, pyqtSignal, QObject

class GraphUpdater(QObject):
    data_updated = pyqtSignal(float, float, float, float)  # Sinal para enviar dados atualizados

    def __init__(self, canvas1, canvas2, ax1, ax2):
        super().__init__()
        self.canvas1 = canvas1
        self.canvas2 = canvas2
        self.ax1 = ax1
        self.ax2 = ax2
        self.data_x = []
        self.data_y1 = []
        self.data_y2 = []
        self.total_distance = 0
        self.min_radius = float('inf')
        self.total_time = 0
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_graphs)
        self.timer.start(1000)

    def desenhar_arco(self, ax, x, y, theta, R, L, cor='r'):
        if np.isinf(R):
            return
        
        cx = x - R * np.sin(theta)
        cy = y + R * np.cos(theta)
        
        angulo_inicial = np.degrees(np.arctan2(y - cy, x - cx))
        angulo_final = angulo_inicial + np.degrees(L / abs(R)) * np.sign(R)
        
        angulos = np.linspace(angulo_inicial, angulo_final, 20)
        arco_x = cx + abs(R) * np.cos(np.radians(angulos))
        arco_y = cy + abs(R) * np.sin(np.radians(angulos))
        
        ax.plot(arco_x, arco_y, linestyle='-', color=cor, alpha=0.6)
        
        # Adicionar texto com o raio se estiver dentro do intervalo especificado
        if -15 < R < 15:
            ax.text(cx, cy, f'R={R:.2f}', color='black', fontsize=8, ha='center')
        
    def processar_dados(self, arquivo, L=12.8, raio_roda=1.02):
        fator_conversao = (2 * np.pi * raio_roda) / 64
        x, y, theta = 0, 0, 0
        trajetoria_x, trajetoria_y = [x], [y]
        lastLeft = 0
        lastRight = 0
        distancia_acumulada = 0
        self.total_distance = 0
        self.min_radius = float('inf')
        self.total_time = 0
        self.ax2.clear()
        with open(arquivo, 'r') as f:
            for linha_serial in f:
                linha_serial = linha_serial.strip()
                dados = linha_serial.split(":")
                if len(dados) == 2:
                    try:
                        pL = int(dados[1]) - lastLeft
                        pR = int(dados[0]) - lastRight
                        lastLeft = int(dados[1])
                        lastRight = int(dados[0])
                        
                        dL = pL * fator_conversao
                        dR = pR * fator_conversao
                        
                        dC = (dL + dR) / 2
                        dTheta = (dR - dL) / L
     
                        x_novo = x + dC * np.cos(theta)
                        y_novo = y + dC * np.sin(theta)
                        theta_novo = theta + dTheta

                        trajetoria_x.append(x_novo)
                        trajetoria_y.append(y_novo)
                        
                        distancia_acumulada += abs(dC)
                        self.total_distance += abs(dC)
                        self.total_time += 0.01  # Cada leitura é feita a cada 10 milissegundos
     
                        if distancia_acumulada >= 5:
                            if abs(dR - dL) > 1e-6:
                                R = (L / 2) * ((dL + dR) / (dR - dL))
                            else:
                                R = 1e6
                            self.desenhar_arco(self.ax2, x, y, theta, R, 5)
                            distancia_acumulada = 0  
                            if abs(R) < self.min_radius:
                                self.min_radius = abs(R)

                        x, y, theta = x_novo, y_novo, theta_novo

                    except ValueError:
                        print("Erro")

        self.ax1.clear()
        self.ax1.plot(trajetoria_x, trajetoria_y, linestyle='-', color='b')
        self.ax1.set_title("Trajetória do Robô")
        self.ax1.set_xlabel("X (cm)")
        self.ax1.set_ylabel("Y (cm)")
        self.canvas1.draw()

        
        self.ax2.set_title("Curvas do Robô")
        self.ax2.set_xlabel("X (cm)")
        self.ax2.set_ylabel("Y (cm)")
        self.canvas2.draw()

        # Calcular a velocidade média
        if self.total_time > 0:
            avg_speed = self.total_distance / self.total_time
        else:
            avg_speed = 0

        # Emitir sinal com os dados atualizados
        self.data_updated.emit(self.total_distance / 100, self.min_radius, self.total_time, avg_speed)

    def update_graphs(self):
        self.processar_dados("dados.txt")
        
    def stop(self):
        self.running = False
        self.wait()