import numpy as np
import matplotlib.pyplot as plt

def desenhar_arco(ax, x, y, theta, R, L, cor='r'):
    """Desenha um arco representando a curvatura do movimento do robô."""
    if np.isinf(R):  # Movimento retilíneo, não desenha arco
        return
    
    # Determina o centro de curvatura
    cx = x - R * np.sin(theta)
    cy = y + R * np.cos(theta)
    
    # Define o ângulo inicial e final do arco
    angulo_inicial = np.degrees(np.arctan2(y - cy, x - cx))
    angulo_final = angulo_inicial + np.degrees(L / abs(R)) * np.sign(R)
    
    # Gera pontos para o arco
    angulos = np.linspace(angulo_inicial, angulo_final, 20)
    arco_x = cx + abs(R) * np.cos(np.radians(angulos))
    arco_y = cy + abs(R) * np.sin(np.radians(angulos))
    
    # Plota o arco
    ax.plot(arco_x, arco_y, linestyle='-', color=cor, alpha=0.6)

def processar_dados(arquivo, L=12.5, raio_roda=1.005):
    """Processa os dados de um arquivo, calcula a trajetória do robô e desenha os arcos de curvatura."""
    
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

                    # Atualiza a posição
                    x_novo = x + dC * np.cos(theta)
                    y_novo = y + dC * np.sin(theta)
                    theta_novo = theta + dTheta

                    trajetoria_x.append(x_novo)
                    trajetoria_y.append(y_novo)

                    # Acumula a distância percorrida
                    distancia_acumulada += abs(dC)

                    # Quando atingir 10 cm, calcula e desenha o arco
                    if distancia_acumulada >= 10:
                        if abs(dR - dL) > 1e-6:  # Evita divisão por zero
                            R = (L / 2) * ((dL + dR) / (dR - dL))
                        else:
                            R = float('inf')  # Movimento retilíneo
                        
                        desenhar_arco(ax, x, y, theta, R, 10)
                        distancia_acumulada = 0  # Reseta a distância acumulada

                    x, y, theta = x_novo, y_novo, theta_novo

                except ValueError:
                    print("Erro ao converter dados.")

    # Finaliza a plotagem
    ax.plot(trajetoria_x, trajetoria_y, marker='o', linestyle='-', color='b')
    ax.legend()
    plt.show()

# Nome do arquivo contendo os dados
arquivo_dados = "dados.txt"

# Processar os dados do arquivo
processar_dados(arquivo_dados)
