import numpy as np
import matplotlib.pyplot as plt

def desenhar_arco(ax, x, y, theta, R, L, cor='r'):
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

def processar_dados(arquivo, L=12.8, raio_roda=1):
    
    fator_conversao = (2 * np.pi * raio_roda) / 64
    
    x, y, theta = 0, 0, 0
    trajetoria_x, trajetoria_y = [x], [y]
    
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_title("Trajetória do Robô com Curvatura")
    ax.set_xlabel("X (cm)")
    ax.set_ylabel("Y (cm)")
    linha, = ax.plot(trajetoria_x, trajetoria_y, marker='o', linestyle='-', color='b', label="Trajetória")
    lastLeft = 0
    lastRight = 0
    distancia_acumulada = 0

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
 
                    if distancia_acumulada >= 5:
                        if abs(dR - dL) > 1e-6:  
                            R = (L / 2) * ((dL + dR) / (dR - dL))
                        else:
                            R = 1e6
                        print(R)
                        if(R < 20 and R > -20):
                            desenhar_arco(ax, x, y, theta, R, 5)
                        distancia_acumulada = 0  

                    x, y, theta = x_novo, y_novo, theta_novo

                except ValueError:
                    print("Erro")

    ax.plot(trajetoria_x, trajetoria_y, linestyle='-', color='b')
    ax.legend()
    plt.show()

arquivo_dados = "dados.txt"

processar_dados(arquivo_dados)
